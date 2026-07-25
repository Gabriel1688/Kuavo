/**
 * @file Composer.cpp
 * @brief Lock-free multi-source sensor data composition at 400 Hz.
 *
 * Follows the proven compose+batch pattern from test_actuator_logger.cpp:
 * read staging buffers → merge into SensorData → write SHM double buffer
 * → accumulate sensor+command pairs → flush batch every BATCH_SIZE samples.
 */

#include "Composer.h"

#include <cerrno>
#include <cstring>
#include <sched.h>
#include <time.h>
#include <spdlog/spdlog.h>

namespace mercury {

// ─────────────────────────────────────────────────────────────────────
// Construction / Destruction
// ─────────────────────────────────────────────────────────────────────

Composer::Composer(SourceDoubleBuffer<ImuStageData>&        imu_stage,
                   SourceDoubleBuffer<MotorGroupStageData>& motor_group_a,
                   SourceDoubleBuffer<MotorGroupStageData>& motor_group_b,
                   MotorParamCache&                         param_cache,
                   SharedMemoryLayout&                      shm,
                   SPSCRingBuffer<BatchLogRecord, BATCH_RING_CAPACITY>& ring)
    : imu_stage_(imu_stage)
    , motor_group_a_(motor_group_a)
    , motor_group_b_(motor_group_b)
    , param_cache_(param_cache)
    , shm_(shm)
    , ring_(ring) {}

Composer::~Composer() {
    shutdown();
}

// ─────────────────────────────────────────────────────────────────────
// Thread lifecycle
// ─────────────────────────────────────────────────────────────────────

void* Composer::thread_entry(void* arg) {
    static_cast<Composer*>(arg)->run();
    return nullptr;
}

void Composer::start() {
    if (thread_created_) return;

    // Initialize staleness timestamps to "now" so the system gets a grace
    // period (equal to the respective timeout) before declaring sources stale.
    // Without this, timestamps start at 0 → check_staleness() immediately
    // returns UINT64_MAX elapsed ms → instant emergency stop on first cycle.
    uint64_t now = get_monotonic_ns();
    last_imu_change_ns_.store(now, std::memory_order_relaxed);
    last_grp_a_change_ns_.store(now, std::memory_order_relaxed);
    last_grp_b_change_ns_.store(now, std::memory_order_relaxed);

    running_.store(true, std::memory_order_release);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, STACK_SIZE);

    int rc = pthread_create(&thread_id_, &attr, &Composer::thread_entry, this);
    pthread_attr_destroy(&attr);

    if (rc != 0) {
        SPDLOG_ERROR("Composer: pthread_create failed: {}", strerror(rc));
        running_.store(false, std::memory_order_release);
        return;
    }
    thread_created_ = true;

    // Set thread name for debugging
    pthread_setname_np(thread_id_, "composer");

    // Apply SCHED_FIFO priority 85
    struct sched_param param{};
    param.sched_priority = SCHED_PRIORITY;
    int ret = pthread_setschedparam(thread_id_, SCHED_FIFO, &param);
    if (ret != 0) {
        SPDLOG_WARN("Composer: failed to set SCHED_FIFO/{}: {}",
                    SCHED_PRIORITY, strerror(ret));
    } else {
        SPDLOG_INFO("Composer: SCHED_FIFO priority {}, 400 Hz, {}KiB stack",
                    SCHED_PRIORITY, STACK_SIZE / 1024);
    }
}

void Composer::shutdown() {
    if (!thread_created_) return;
    running_.store(false, std::memory_order_release);
    pthread_join(thread_id_, nullptr);
    thread_created_ = false;
    SPDLOG_INFO("Composer: shutdown complete (dropped={})", dropped_);
}

// ─────────────────────────────────────────────────────────────────────
// Run loop — 400 Hz via CLOCK_MONOTONIC absolute sleep
// ─────────────────────────────────────────────────────────────────────

void Composer::run() {
    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

    while (running_.load(std::memory_order_acquire)) {
        compose_cycle();

        // Advance to next period
        next_wakeup.tv_nsec += PERIOD_NS;
        if (next_wakeup.tv_nsec >= 1'000'000'000L) {
            next_wakeup.tv_sec += 1;
            next_wakeup.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    }

    // Flush any remaining samples in the partial batch
    if (batch_idx_ > 0) flush_batch();
}

// ─────────────────────────────────────────────────────────────────────
// Single compose cycle (mirrors test_actuator_logger.cpp composer_thread_fn)
// ─────────────────────────────────────────────────────────────────────

void Composer::compose_cycle() {
    // 1. Read all sources (lock-free)
    ImuStageData        imu  = imu_stage_.read();
    MotorGroupStageData grpA = motor_group_a_.read();
    MotorGroupStageData grpB = motor_group_b_.read();

    // 2. Build flat SensorData snapshot
    SensorData sd{};

    // IMU
    std::memcpy(sd.imu_inc,     imu.imu_inc,     sizeof(sd.imu_inc));
    std::memcpy(sd.imu_ang_vel, imu.imu_ang_vel, sizeof(sd.imu_ang_vel));
    std::memcpy(sd.imu_acc,     imu.imu_acc,     sizeof(sd.imu_acc));

    // Motor Group A → indices 0..5
    for (int j = 0; j < MOTORS_PER_GROUP; j++) {
        sd.joint_jpos[j] = grpA.joint_jpos[j];
        sd.joint_jvel[j] = grpA.joint_jvel[j];
        sd.motor_jpos[j] = grpA.motor_jpos[j];
        sd.motor_jvel[j] = grpA.motor_jvel[j];
        sd.bus_current[j] = grpA.bus_current[j];
        sd.bus_voltage[j] = grpA.bus_voltage[j];
        sd.jtorque[j]    = grpA.jtorque[j];
        sd.motor_current[j] = grpA.motor_current[j];
        sd.reflected_rotor_inertia[j] = grpA.reflected_rotor_inertia[j];
    }

    // Motor Group B → indices 6..11
    for (int j = 0; j < MOTORS_PER_GROUP; j++) {
        int idx = MOTORS_PER_GROUP + j;
        sd.joint_jpos[idx] = grpB.joint_jpos[j];
        sd.joint_jvel[idx] = grpB.joint_jvel[j];
        sd.motor_jpos[idx] = grpB.motor_jpos[j];
        sd.motor_jvel[idx] = grpB.motor_jvel[j];
        sd.bus_current[idx] = grpB.bus_current[j];
        sd.bus_voltage[idx] = grpB.bus_voltage[j];
        sd.jtorque[idx]    = grpB.jtorque[j];
        sd.motor_current[idx] = grpB.motor_current[j];
        sd.reflected_rotor_inertia[idx] = grpB.reflected_rotor_inertia[j];
    }

    // Parameter cache — atomic loads (overwrites staging bus_voltage/current
    // with the 10 Hz parameter query values when available)
    for (size_t i = 0; i < MotorParamCache::NUM_MOTORS; ++i) {
        double v = param_cache_.load_bus_voltage(i);
        double c = param_cache_.load_bus_current(i);
        double r = param_cache_.load_reflected_rotor_inertia(i);
        if (v != 0.0) sd.bus_voltage[i] = v;
        if (c != 0.0) sd.bus_current[i] = c;
        if (r != 0.0) sd.reflected_rotor_inertia[i] = r;
    }

    // Per-source timestamps and sequences
    sd.imu_timestamp_ns           = imu.timestamp_ns;
    sd.motor_group_a_timestamp_ns = grpA.timestamp_ns;
    sd.motor_group_b_timestamp_ns = grpB.timestamp_ns;
    sd.compose_timestamp_ns       = get_monotonic_ns();
    sd.imu_sequence               = imu.sequence;
    sd.motor_group_a_sequence     = grpA.sequence;
    sd.motor_group_b_sequence     = grpB.sequence;

    // Per-leg controller timing (piggybacked from staging buffers)
    sd.leg_a_duration_us  = grpA.controller_duration_us;
    sd.leg_a_interval_us  = grpA.controller_interval_us;
    sd.leg_b_duration_us  = grpB.controller_duration_us;
    sd.leg_b_interval_us  = grpB.controller_interval_us;

    // 3. Update staleness tracking
    uint64_t now = sd.compose_timestamp_ns;
    if (imu.sequence != prev_imu_seq_) {
        prev_imu_seq_ = imu.sequence;
        last_imu_change_ns_.store(now, std::memory_order_release);
    }
    if (grpA.sequence != prev_grp_a_seq_) {
        prev_grp_a_seq_ = grpA.sequence;
        last_grp_a_change_ns_.store(now, std::memory_order_release);
    }
    if (grpB.sequence != prev_grp_b_seq_) {
        prev_grp_b_seq_ = grpB.sequence;
        last_grp_b_change_ns_.store(now, std::memory_order_release);
    }

    // 4. Write to shared memory double buffer
    uint32_t wb = 1 - shm_.composed_write_idx.load(std::memory_order_acquire);
    std::memcpy(&shm_.composed_buffers[wb], &sd, sizeof(SensorData));
    shm_.composed_write_idx.store(wb, std::memory_order_release);
    shm_.composed_sequence.fetch_add(1, std::memory_order_release);

    // 5. Read latest command from SHM
    uint32_t crb = shm_.cmd_write_idx.load(std::memory_order_acquire);
    Command cmd;
    std::memcpy(&cmd, &shm_.cmd_buffers[crb], sizeof(Command));

    // 6. Accumulate sensor+command pair into batch
    batch_.samples[batch_idx_].sensor = sd;
    batch_.samples[batch_idx_].cmd    = cmd;
    batch_idx_++;

    if (batch_idx_ >= BATCH_SIZE) {
        flush_batch();
    }
}

void Composer::flush_batch() {
    batch_.header.magic        = PAYLOAD_MAGIC;
    batch_.header.version      = PAYLOAD_VERSION;
    batch_.header.record_type  = static_cast<uint8_t>(RecordType::SENSOR_BATCH);
    batch_.header.robot_id     = shm_.robot_id;
    batch_.header.payload_size = static_cast<uint32_t>(
        sizeof(uint32_t) * 2 + batch_idx_ * sizeof(SensorCommandPair));
    batch_.header.sequence     = batch_seq_++;
    batch_.header.timestamp_ns = get_monotonic_ns();
    batch_.sample_count        = batch_idx_;

    if (!ring_.push(batch_)) ++dropped_;

    batch_idx_ = 0;
}

// ─────────────────────────────────────────────────────────────────────
// Staleness check (called by main loop at 100 Hz)
// ─────────────────────────────────────────────────────────────────────

uint8_t Composer::check_staleness() const {
    uint64_t now = get_monotonic_ns();
    uint8_t mask = 0;

    auto elapsed_ms = [now](std::atomic<uint64_t> const& ts) -> uint64_t {
        uint64_t last = ts.load(std::memory_order_acquire);
        if (last == 0) return UINT64_MAX; // never updated
        return (now - last) / 1'000'000;
    };

    if (elapsed_ms(last_imu_change_ns_)  >= IMU_STALE_TIMEOUT_MS)
        mask |= STALE_IMU;
    if (elapsed_ms(last_grp_a_change_ns_) >= MOTOR_GROUP_STALE_TIMEOUT_MS)
        mask |= STALE_MOTOR_GROUP_A;
    if (elapsed_ms(last_grp_b_change_ns_) >= MOTOR_GROUP_STALE_TIMEOUT_MS)
        mask |= STALE_MOTOR_GROUP_B;

    return mask;
}

} // namespace mercury
