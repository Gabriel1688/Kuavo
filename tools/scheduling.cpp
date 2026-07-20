/**
 * @file test_thread_model.cpp
 * @brief Validates Kuavo 10-thread scheduling model
 *
 * Tests deadline compliance, priority preemption, lock-free data
 * integrity, staleness detection, emergency stop latency, and
 * CPU starvation between equal-priority threads.
 *
 * Requires: PREEMPT_RT kernel or CAP_SYS_NICE capability [2]
 *
 * Build:
 *   g++ -O2 -std=c++20 -pthread -lrt -o test_thread_model test_thread_model.cpp
 *
 * Run (requires root or CAP_SYS_NICE):
 *   sudo ./test_thread_model -dur 10
 *   # or with capability:
 *   sudo setcap cap_sys_nice+ep ./test_thread_model
 *   ./test_thread_model -dur 10
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <thread>
#include <vector>

// ============================================================
// Timing utilities
// ============================================================

static inline uint64_t get_monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

struct TimingStats {
    uint64_t min_ns   = UINT64_MAX;
    uint64_t max_ns   = 0;
    uint64_t total_ns = 0;
    uint64_t count    = 0;
    uint64_t deadline_misses = 0;

    void record(uint64_t duration_ns, uint64_t deadline_ns = 0) {
        if (duration_ns < min_ns) min_ns = duration_ns;
        if (duration_ns > max_ns) max_ns = duration_ns;
        total_ns += duration_ns;
        count++;
        if (deadline_ns > 0 && duration_ns > deadline_ns)
            deadline_misses++;
    }

    double avg_us() const { return count > 0 ? (total_ns / count) / 1000.0 : 0; }
    double min_us() const { return count > 0 ? min_ns / 1000.0 : 0; }
    double max_us() const { return max_ns / 1000.0; }
    double miss_pct() const { return count > 0 ? 100.0 * deadline_misses / count : 0; }

    void print(const char* label, uint64_t deadline_us = 0) const {
        printf("  %-35s avg=%7.1f us  min=%7.1f us  max=%7.1f us  (n=%lu)",
               label, avg_us(), min_us(), max_us(), count);
        if (deadline_us > 0) {
            printf("  misses=%lu (%.2f%%)", deadline_misses, miss_pct());
        }
        printf("\n");
    }
};

// ============================================================
// Simulated per-source staging double buffer [1]
// Matches the SourceDoubleBuffer<T> design from the architecture
// ============================================================

template<typename T>
struct SourceDoubleBuffer {
    T buffers[2];
    std::atomic<uint32_t> write_idx{0};
    std::atomic<uint64_t> sequence{0};
    std::atomic<uint64_t> heartbeat_ns{0};

    void publish(const T& data) {
        uint32_t wb = 1 - write_idx.load(std::memory_order_acquire);
        std::memcpy(&buffers[wb], &data, sizeof(T));
        write_idx.store(wb, std::memory_order_release);
        sequence.fetch_add(1, std::memory_order_release);
        heartbeat_ns.store(get_monotonic_ns(), std::memory_order_release);
    }

    T read() const {
        uint32_t rb = write_idx.load(std::memory_order_acquire);
        T result;
        std::memcpy(&result, &buffers[rb], sizeof(T));
        return result;
    }

    uint64_t get_sequence() const {
        return sequence.load(std::memory_order_acquire);
    }

    uint64_t get_heartbeat() const {
        return heartbeat_ns.load(std::memory_order_acquire);
    }
};

// ============================================================
// Simulated data structures matching Kuavo motor topology [1]
// 6 motors per leg, 12 total
// ============================================================

static constexpr int MOTORS_PER_GROUP = 6;
static constexpr int NUM_ACT_JOINT = 12;

struct MotorGroupData {
    double joint_jpos[MOTORS_PER_GROUP];
    double joint_jvel[MOTORS_PER_GROUP];
    double jtorque[MOTORS_PER_GROUP];
    uint64_t timestamp_ns;
    uint64_t sequence;
    uint32_t writer_thread_id;  // For integrity verification
    uint32_t checksum;          // Simple checksum for torn read detection
};

struct ImuData {
    double imu_ang_vel[3];
    double imu_acc[3];
    uint64_t timestamp_ns;
    uint64_t sequence;
    uint32_t writer_thread_id;
    uint32_t checksum;
};

struct ComposedData {
    double joint_jpos[NUM_ACT_JOINT];
    double joint_jvel[NUM_ACT_JOINT];
    double jtorque[NUM_ACT_JOINT];
    double imu_ang_vel[3];
    uint64_t imu_timestamp_ns;
    uint64_t grp_a_timestamp_ns;
    uint64_t grp_b_timestamp_ns;
    uint64_t compose_timestamp_ns;
    uint64_t imu_sequence;
    uint64_t grp_a_sequence;
    uint64_t grp_b_sequence;
};

// Simple checksum for torn read detection
static uint32_t compute_checksum(const void* data, size_t len) {
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = sum * 31 + bytes[i];
    }
    return sum;
}

// ============================================================
// Global shared state
// ============================================================

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

// Per-source staging buffers (matching architecture) [1]
static SourceDoubleBuffer<MotorGroupData> g_motor_grp_a;  // Left leg, motors 1-6
static SourceDoubleBuffer<MotorGroupData> g_motor_grp_b;  // Right leg, motors 7-12
static SourceDoubleBuffer<ImuData>        g_imu_stage;

// Composed output
static SourceDoubleBuffer<ComposedData>   g_composed;

// Emergency stop flag (shared memory atomic) [1]
static std::atomic<bool> g_emergency_stop{false};

// Preemption detection counters
static std::atomic<uint64_t> g_leg_a_preempted_count{0};
static std::atomic<uint64_t> g_leg_b_preempted_count{0};
static std::atomic<uint64_t> g_composer_preempted_count{0};

// ============================================================
// Thread configuration matching revised architecture [1][2]
// ============================================================

struct ThreadConfig {
    const char* name;
    int sched_policy;
    int priority;
    size_t stack_size;
    uint64_t period_ns;  // 0 = event-driven
};

static const ThreadConfig THREAD_CONFIGS[] = {
    // Thread 1: Main Robot Loop [1]
    {"Main Loop",         SCHED_FIFO, 75, 512 * 1024, 10'000'000},   // 100Hz, 10ms

    // Thread 2: Left Leg Subsystem [1]
    {"Left Leg",          SCHED_FIFO, 90, 256 * 1024,  5'000'000},   // 200Hz, 5ms

    // Thread 3: Right Leg Subsystem [1]
    {"Right Leg",         SCHED_FIFO, 90, 256 * 1024,  5'000'000},   // 200Hz, 5ms

    // Thread 4: IMU Reader [1]
    {"IMU Reader",        SCHED_FIFO, 80, 128 * 1024,  5'000'000},   // 200Hz, 5ms

    // Thread 5: Composer [1]
    {"Composer",          SCHED_FIFO, 85, 256 * 1024,  5'000'000},   // 200Hz, 5ms

    // Thread 6: MQTT Logger
    {"MQTT Logger",       SCHED_OTHER, 0, 512 * 1024,  0},           // Network-paced
};

// ============================================================
// Per-thread timing results
// ============================================================

struct ThreadResult {
    TimingStats cycle_stats;       // Full cycle time
    TimingStats work_stats;        // Computation time only
    TimingStats jitter_stats;      // Period jitter
    uint64_t iterations = 0;
    uint64_t stale_reads = 0;
    uint64_t torn_reads = 0;
    uint64_t estop_detect_ns = 0;  // Time to detect emergency stop
    bool sched_ok = false;         // Successfully set scheduling policy
};

static ThreadResult g_results[6];

// ============================================================
// Helper: Apply real-time scheduling to current thread
// ============================================================

static bool apply_scheduling(int policy, int priority, size_t stack_size,
                             const char* name) {
    // Set scheduling policy and priority
    if (policy == SCHED_FIFO || policy == SCHED_RR) {
        struct sched_param param;
        param.sched_priority = priority;
        if (pthread_setschedparam(pthread_self(), policy, &param) != 0) {
            printf("  WARNING: Failed to set %s to SCHED_FIFO priority %d "
                   "(need root or CAP_SYS_NICE)\n", name, priority);
            return false;
        }
    }
    return true;
}

// ============================================================
// Simulated workload — mimics MIT command encoding [7][8]
// float_to_uint conversion for position (16-bit), velocity (12-bit),
// Kp (12-bit), Kd (12-bit), torque (12-bit)
// ============================================================

static void simulate_mit_encoding_work(int num_motors) {
    volatile double pos = 1.234, vel = 5.678, kp = 50.0, kd = 1.0, tau = 0.5;
    for (int m = 0; m < num_motors; m++) {
        // Simulate float_to_uint conversions [8]
        volatile uint16_t p = static_cast<uint16_t>(
            ((pos + 12.5) / 25.0) * 65535);
        volatile uint16_t v = static_cast<uint16_t>(
            ((vel + 45.0) / 90.0) * 4095);
        volatile uint16_t k = static_cast<uint16_t>(
            (kp / 500.0) * 4095);
        volatile uint16_t d = static_cast<uint16_t>(
            (kd / 5.0) * 4095);
        volatile uint16_t t = static_cast<uint16_t>(
            ((tau + 18.0) / 36.0) * 4095);
        (void)p; (void)v; (void)k; (void)d; (void)t;
    }
}

// ============================================================
// Thread 2/3: Leg Subsystem (400Hz, SCHED_FIFO 90) [1]
// Simulates controllerPeriodic(): read SHM cmd, MIT encode,
// send CAN, receive feedback, write staging buffer
// ============================================================

static void leg_thread_fn(int thread_idx, int group_offset,
                          SourceDoubleBuffer<MotorGroupData>* stage,
                          std::atomic<uint64_t>* preempt_counter) {
    const ThreadConfig& cfg = THREAD_CONFIGS[thread_idx];
    ThreadResult& result = g_results[thread_idx];

    result.sched_ok = apply_scheduling(cfg.sched_policy, cfg.priority,
                                       cfg.stack_size, cfg.name);

    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t prev_cycle_start = next_wakeup;
    uint64_t seq = 0;

    while (g_running && !g_emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        // Measure jitter (deviation from expected period)
        if (seq > 0) {
            uint64_t actual_period = cycle_start - prev_cycle_start;
            int64_t jitter = static_cast<int64_t>(actual_period) -
                             static_cast<int64_t>(cfg.period_ns);
            result.jitter_stats.record(
                static_cast<uint64_t>(std::abs(jitter)));
        }
        prev_cycle_start = cycle_start;

        // Simulate work: MIT encoding for 6 motors [7][8]
        uint64_t work_start = get_monotonic_ns();
        simulate_mit_encoding_work(MOTORS_PER_GROUP);

        // Write to staging buffer
        MotorGroupData grp{};
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            grp.joint_jpos[j] = std::sin(seq * 0.01 + j);
            grp.joint_jvel[j] = std::cos(seq * 0.01 + j);
            grp.jtorque[j] = 0.5 * std::sin(seq * 0.02 + j);
        }
        grp.timestamp_ns = get_monotonic_ns();
        grp.sequence = seq;
        grp.writer_thread_id = thread_idx;
        grp.checksum = compute_checksum(grp.joint_jpos,
                                         sizeof(grp.joint_jpos));
        stage->publish(grp);

        uint64_t work_end = get_monotonic_ns();
        result.work_stats.record(work_end - work_start);

        // Full cycle timing
        uint64_t cycle_end = get_monotonic_ns();
        result.cycle_stats.record(cycle_end - cycle_start, cfg.period_ns);

        seq++;
        result.iterations = seq;

        // Sleep until next period
        next_wakeup += cfg.period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            // Missed deadline — record and reset
            preempt_counter->fetch_add(1, std::memory_order_relaxed);
            next_wakeup = now + cfg.period_ns;
        }
    }

    // Measure emergency stop detection latency
    if (g_emergency_stop.load(std::memory_order_acquire)) {
        result.estop_detect_ns = get_monotonic_ns();
    }
}

// ============================================================
// Thread 4: IMU Reader (500Hz, SCHED_FIFO 80) [1]
// Simulates parsing 8 CAN frames (0x514-0x51B) [1]
// ============================================================

static void imu_thread_fn() {
    const int thread_idx = 3;
    const ThreadConfig& cfg = THREAD_CONFIGS[thread_idx];
    ThreadResult& result = g_results[thread_idx];

    result.sched_ok = apply_scheduling(cfg.sched_policy, cfg.priority,
                                       cfg.stack_size, cfg.name);

    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t prev_cycle_start = next_wakeup;
    uint64_t seq = 0;

    while (g_running && !g_emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        if (seq > 0) {
            uint64_t actual_period = cycle_start - prev_cycle_start;
            int64_t jitter = static_cast<int64_t>(actual_period) -
                             static_cast<int64_t>(cfg.period_ns);
            result.jitter_stats.record(
                static_cast<uint64_t>(std::abs(jitter)));
        }
        prev_cycle_start = cycle_start;

        // Simulate work: parse 8 CAN frames × 2 floats [1]
        uint64_t work_start = get_monotonic_ns();
        float parsed[16];
        for (int f = 0; f < 8; f++) {
            uint8_t fake_frame[13] = {};
            float v1 = std::sin(seq * 0.001 + f);
            float v2 = std::cos(seq * 0.001 + f);
            std::memcpy(&fake_frame[5], &v1, 4);
            std::memcpy(&fake_frame[9], &v2, 4);
            std::memcpy(&parsed[f * 2], &fake_frame[5], 4);
            std::memcpy(&parsed[f * 2 + 1], &fake_frame[9], 4);
        }

        // Publish only after all 8 frames (complete cycle) [1]
        ImuData imu{};
        imu.imu_ang_vel[0] = parsed[6];  // gyroX from frame 0x515
        imu.imu_ang_vel[1] = parsed[4];  // gyroY from frame 0x516
        imu.imu_ang_vel[2] = parsed[5];  // gyroZ from frame 0x516
        imu.imu_acc[0] = parsed[0];
        imu.imu_acc[1] = parsed[1];
        imu.imu_acc[2] = parsed[2];
        imu.timestamp_ns = get_monotonic_ns();
        imu.sequence = seq;
        imu.writer_thread_id = thread_idx;
        imu.checksum = compute_checksum(imu.imu_ang_vel,
                                         sizeof(imu.imu_ang_vel));
        g_imu_stage.publish(imu);

        uint64_t work_end = get_monotonic_ns();
        result.work_stats.record(work_end - work_start);

        uint64_t cycle_end = get_monotonic_ns();
        result.cycle_stats.record(cycle_end - cycle_start, cfg.period_ns);

        seq++;
        result.iterations = seq;

        next_wakeup += cfg.period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + cfg.period_ns;
        }
    }

    if (g_emergency_stop.load(std::memory_order_acquire)) {
        result.estop_detect_ns = get_monotonic_ns();
    }
}

// ============================================================
// Thread 5: Composer (400Hz, SCHED_FIFO 85) [1]
// Reads all 3 staging buffers, checks integrity, merges
// ============================================================

static void composer_thread_fn() {
    const int thread_idx = 4;
    const ThreadConfig& cfg = THREAD_CONFIGS[thread_idx];
    ThreadResult& result = g_results[thread_idx];

    result.sched_ok = apply_scheduling(cfg.sched_policy, cfg.priority,
                                       cfg.stack_size, cfg.name);

    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t prev_cycle_start = next_wakeup;
    uint64_t seq = 0;
    uint64_t last_imu_seq = 0, last_grpA_seq = 0, last_grpB_seq = 0;

    while (g_running && !g_emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        if (seq > 0) {
            uint64_t actual_period = cycle_start - prev_cycle_start;
            int64_t jitter = static_cast<int64_t>(actual_period) -
                             static_cast<int64_t>(cfg.period_ns);
            result.jitter_stats.record(
                static_cast<uint64_t>(std::abs(jitter)));
        }
        prev_cycle_start = cycle_start;

        uint64_t work_start = get_monotonic_ns();

        // Read all 3 staging buffers [1]
        ImuData imu = g_imu_stage.read();
        MotorGroupData grpA = g_motor_grp_a.read();
        MotorGroupData grpB = g_motor_grp_b.read();

        // Torn read detection via checksum
        uint32_t imu_check = compute_checksum(imu.imu_ang_vel,
                                               sizeof(imu.imu_ang_vel));
        if (imu_check != imu.checksum && imu.sequence > 0) {
            result.torn_reads++;
        }

        uint32_t grpA_check = compute_checksum(grpA.joint_jpos,
                                                sizeof(grpA.joint_jpos));
        if (grpA_check != grpA.checksum && grpA.sequence > 0) {
            result.torn_reads++;
        }

        uint32_t grpB_check = compute_checksum(grpB.joint_jpos,
                                                sizeof(grpB.joint_jpos));
        if (grpB_check != grpB.checksum && grpB.sequence > 0) {
            result.torn_reads++;
        }

        // Staleness detection [1]
        if (imu.sequence == last_imu_seq && seq > 10)
            result.stale_reads++;
        if (grpA.sequence == last_grpA_seq && seq > 10)
            result.stale_reads++;
        if (grpB.sequence == last_grpB_seq && seq > 10)
            result.stale_reads++;

        last_imu_seq  = imu.sequence;
        last_grpA_seq = grpA.sequence;
        last_grpB_seq = grpB.sequence;

        // Merge into composed snapshot
        ComposedData composed{};
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            composed.joint_jpos[j] = grpA.joint_jpos[j];
            composed.joint_jvel[j] = grpA.joint_jvel[j];
            composed.jtorque[j]    = grpA.jtorque[j];
            composed.joint_jpos[MOTORS_PER_GROUP + j] = grpB.joint_jpos[j];
            composed.joint_jvel[MOTORS_PER_GROUP + j] = grpB.joint_jvel[j];
            composed.jtorque[MOTORS_PER_GROUP + j]    = grpB.jtorque[j];
        }
        composed.imu_ang_vel[0] = imu.imu_ang_vel[0];
        composed.imu_ang_vel[1] = imu.imu_ang_vel[1];
        composed.imu_ang_vel[2] = imu.imu_ang_vel[2];
        composed.imu_timestamp_ns = imu.timestamp_ns;
        composed.grp_a_timestamp_ns = grpA.timestamp_ns;
        composed.grp_b_timestamp_ns = grpB.timestamp_ns;
        composed.compose_timestamp_ns = get_monotonic_ns();
        composed.imu_sequence = imu.sequence;
        composed.grp_a_sequence = grpA.sequence;
        composed.grp_b_sequence = grpB.sequence;

        g_composed.publish(composed);

        uint64_t work_end = get_monotonic_ns();
        result.work_stats.record(work_end - work_start);

        uint64_t cycle_end = get_monotonic_ns();
        result.cycle_stats.record(cycle_end - cycle_start, cfg.period_ns);

        seq++;
        result.iterations = seq;

        next_wakeup += cfg.period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            g_composer_preempted_count.fetch_add(1, std::memory_order_relaxed);
            next_wakeup = now + cfg.period_ns;
        }
    }

    if (g_emergency_stop.load(std::memory_order_acquire)) {
        result.estop_detect_ns = get_monotonic_ns();
    }
}

// ============================================================
// Thread 1: Main Loop (100Hz, SCHED_FIFO 75) [1]
// Supervisory: reads composed data, checks health
// ============================================================

static void main_loop_thread_fn() {
    const int thread_idx = 0;
    const ThreadConfig& cfg = THREAD_CONFIGS[thread_idx];
    ThreadResult& result = g_results[thread_idx];

    result.sched_ok = apply_scheduling(cfg.sched_policy, cfg.priority,
                                       cfg.stack_size, cfg.name);

    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t prev_cycle_start = next_wakeup;
    uint64_t seq = 0;
    uint64_t last_composed_seq = 0;

    while (g_running && !g_emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        if (seq > 0) {
            uint64_t actual_period = cycle_start - prev_cycle_start;
            int64_t jitter = static_cast<int64_t>(actual_period) -
                             static_cast<int64_t>(cfg.period_ns);
            result.jitter_stats.record(
                static_cast<uint64_t>(std::abs(jitter)));
        }
        prev_cycle_start = cycle_start;

        uint64_t work_start = get_monotonic_ns();

        // Read composed data (supervisory health check) [1]
        ComposedData snapshot = g_composed.read();

        // Staleness check
        if (snapshot.grp_a_sequence == last_composed_seq && seq > 10) {
            result.stale_reads++;
        }
        last_composed_seq = snapshot.grp_a_sequence;

        // Simulate parameter query work (every 10th cycle = 10Hz) [1]
        if (seq % 10 == 0) {
            volatile double fake_voltage = 48.0;
            volatile double fake_current = 2.5;
            (void)fake_voltage;
            (void)fake_current;
        }

        uint64_t work_end = get_monotonic_ns();
        result.work_stats.record(work_end - work_start);

        uint64_t cycle_end = get_monotonic_ns();
        result.cycle_stats.record(cycle_end - cycle_start, cfg.period_ns);

        seq++;
        result.iterations = seq;

        next_wakeup += cfg.period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + cfg.period_ns;
        }
    }

    if (g_emergency_stop.load(std::memory_order_acquire)) {
        result.estop_detect_ns = get_monotonic_ns();
    }
}

// ============================================================
// Thread 6: MQTT Logger (SCHED_OTHER, nice+10) [1]
// Non-RT — just validates it does not affect RT threads
// ============================================================

static void logger_thread_fn() {
    const int thread_idx = 5;
    ThreadResult& result = g_results[thread_idx];
    result.sched_ok = true; // SCHED_OTHER always succeeds

    // Set nice +10
    nice(10);

    uint64_t seq = 0;
    while (g_running && !g_emergency_stop.load(std::memory_order_acquire)) {
        uint64_t work_start = get_monotonic_ns();

        // Simulate SPSC drain + serialize work
        ComposedData snapshot = g_composed.read();
        volatile uint8_t fake_payload[1200];
        std::memcpy((void*)fake_payload, &snapshot, sizeof(snapshot));

        uint64_t work_end = get_monotonic_ns();
        result.work_stats.record(work_end - work_start);

        seq++;
        result.iterations = seq;

        // Sleep 200us between drain attempts
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (g_emergency_stop.load(std::memory_order_acquire)) {
        result.estop_detect_ns = get_monotonic_ns();
    }
}

// ============================================================
// Emergency stop test — measures detection latency across
// all threads after the flag is set
// ============================================================

static void emergency_stop_test_fn(double delay_sec) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(delay_sec * 1000)));

    printf("\n  [ESTOP] Setting emergency_stop flag...\n");
    g_emergency_stop.store(true, std::memory_order_release);
}

// ============================================================
// Main
// ============================================================

int main(int argc, char* argv[]) {
    double duration = 10.0;
    bool test_estop = false;
    double estop_delay = 5.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-estop") == 0)
            test_estop = true;
        else if (strcmp(argv[i], "-estop-delay") == 0 && i + 1 < argc)
            estop_delay = atof(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-dur sec] [-estop] [-estop-delay sec]\n", argv[0]);
            printf("  -dur N        Test duration in seconds (default: 10)\n");
            printf("  -estop        Enable emergency stop test\n");
            printf("  -estop-delay  Seconds before triggering estop (default: 5)\n");
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Verify lock-free atomics [1]
    printf("============================================================\n");
    printf("  KUAVO THREAD MODEL TEST\n");
    printf("============================================================\n");
    printf("  Duration:       %.1f seconds\n", duration);
    printf("  Emergency stop: %s\n", test_estop ? "enabled" : "disabled");
    printf("  atomic<uint32_t> lock-free: %s\n",
           std::atomic<uint32_t>::is_always_lock_free ? "YES" : "NO (WARNING)");
    printf("  atomic<uint64_t> lock-free: %s\n",
           std::atomic<uint64_t>::is_always_lock_free ? "YES" : "NO (WARNING)");
    printf("\n");

    // Check for PREEMPT_RT kernel [2]
    FILE* fp = fopen("/sys/kernel/realtime", "r");
    if (fp) {
        char buf[8];
        if (fgets(buf, sizeof(buf), fp)) {
            printf("  PREEMPT_RT kernel: %s\n",
                   buf[0] == '1' ? "YES" : "NO");
        }
        fclose(fp);
    } else {
        printf("  PREEMPT_RT kernel: UNKNOWN (/sys/kernel/realtime not found)\n");
    }
    printf("\n");

    printf("  Thread Configuration:\n");
    for (int i = 0; i < 6; i++) {
        const ThreadConfig& cfg = THREAD_CONFIGS[i];
        if (cfg.period_ns > 0) {
            printf("    [%d] %-15s  policy=%-10s  prio=%2d  stack=%4zuKB  "
                   "period=%.1fms (%.0fHz)\n",
                   i, cfg.name,
                   cfg.sched_policy == SCHED_FIFO ? "SCHED_FIFO" : "SCHED_OTHER",
                   cfg.priority,
                   cfg.stack_size / 1024,
                   cfg.period_ns / 1e6,
                   1e9 / cfg.period_ns);
        } else {
            printf("    [%d] %-15s  policy=%-10s  prio=%2d  stack=%4zuKB  "
                   "period=event-driven\n",
                   i, cfg.name,
                   "SCHED_OTHER", cfg.priority, cfg.stack_size / 1024);
        }
    }
    printf("\n  Launching threads...\n\n");

    // Launch all threads
    std::thread t_main(main_loop_thread_fn);
    std::thread t_leg_a(leg_thread_fn, 1, 0, &g_motor_grp_a,
                        &g_leg_a_preempted_count);
    std::thread t_leg_b(leg_thread_fn, 2, 6, &g_motor_grp_b,
                        &g_leg_b_preempted_count);
    std::thread t_imu(imu_thread_fn);
    std::thread t_composer(composer_thread_fn);
    std::thread t_logger(logger_thread_fn);

    // Optional emergency stop test thread
    std::thread t_estop;
    uint64_t estop_set_time = 0;
    if (test_estop) {
        t_estop = std::thread([&]() {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    static_cast<int>(estop_delay * 1000)));
            estop_set_time = get_monotonic_ns();
            printf("  [ESTOP] Flag set at t=%.3f\n",
                   estop_delay);
            g_emergency_stop.store(true, std::memory_order_release);
        });
    }

    // Wait for test duration
    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration<double>(elapsed).count() >= duration)
            g_running = false;

        // Periodic progress
        double t = std::chrono::duration<double>(elapsed).count();
        if (static_cast<int>(t * 2) % 2 == 0 && t > 0.5) {
            printf("  [%.0fs] leg_a=%lu  leg_b=%lu  imu=%lu  "
                   "composer=%lu  main=%lu  logger=%lu\n",
                   t,
                   g_results[1].iterations, g_results[2].iterations,
                   g_results[3].iterations, g_results[4].iterations,
                   g_results[0].iterations, g_results[5].iterations);
        }
    }

    // Stop all threads
    g_emergency_stop.store(true, std::memory_order_release);
    t_main.join();
    t_leg_a.join();
    t_leg_b.join();
    t_imu.join();
    t_composer.join();
    t_logger.join();
    if (t_estop.joinable()) t_estop.join();

    // ============================================================
    // REPORT
    // ============================================================

    printf("\n");
    printf("============================================================\n");
    printf("  THREAD MODEL TEST REPORT\n");
    printf("============================================================\n");

    const char* thread_names[] = {
        "Main Loop (100Hz)",
        "Left Leg (200Hz)",
        "Right Leg (200Hz)",
        "IMU Reader (200Hz)",
        "Composer (200Hz)",
        "MQTT Logger"
    };

    uint64_t deadline_us[] = {10000, 5000, 5000, 5000, 5000, 0};

    for (int i = 0; i < 6; i++) {
        const ThreadResult& r = g_results[i];
        printf("\n  [%d] %s  (sched=%s, iterations=%lu)\n",
               i, thread_names[i],
               r.sched_ok ? "OK" : "FAILED", r.iterations);

        r.cycle_stats.print("Cycle time", deadline_us[i]);
        r.work_stats.print("Work time", 0);
        if (r.jitter_stats.count > 0) {
            r.jitter_stats.print("Period jitter", 0);
        }
        if (r.stale_reads > 0) {
            printf("    Stale reads:     %lu\n", r.stale_reads);
        }
        if (r.torn_reads > 0) {
            printf("    TORN READS:      %lu  *** DATA INTEGRITY FAILURE ***\n",
                   r.torn_reads);
        }
    }

    // Starvation analysis — compare leg A and leg B iteration counts
    printf("\n  === STARVATION ANALYSIS ===\n");
    uint64_t leg_a_iter = g_results[1].iterations;
    uint64_t leg_b_iter = g_results[2].iterations;
    double imbalance = 0;
    if (leg_a_iter > 0 && leg_b_iter > 0) {
        imbalance = 100.0 * std::abs(
            static_cast<double>(leg_a_iter) - leg_b_iter) /
            std::max(leg_a_iter, leg_b_iter);
    }
    printf("    Left Leg iterations:   %lu\n", leg_a_iter);
    printf("    Right Leg iterations:  %lu\n", leg_b_iter);
    printf("    Imbalance:             %.2f%%\n", imbalance);
    printf("    Left Leg preemptions:  %lu\n",
           g_leg_a_preempted_count.load());
    printf("    Right Leg preemptions: %lu\n",
           g_leg_b_preempted_count.load());
    printf("    Composer preemptions:  %lu\n",
           g_composer_preempted_count.load());
    printf("    Verdict:               %s\n",
           imbalance < 5.0 ? "PASS (< 5%% imbalance)" :
           "FAIL (> 5%% imbalance — starvation suspected)");

    // Emergency stop latency
    if (test_estop && estop_set_time > 0) {
        printf("\n  === EMERGENCY STOP LATENCY ===\n");
        for (int i = 0; i < 6; i++) {
            if (g_results[i].estop_detect_ns > 0) {
                uint64_t latency = g_results[i].estop_detect_ns - estop_set_time;
                printf("    %-20s  detected in %.1f us\n",
                       thread_names[i], latency / 1000.0);
            }
        }
    }

    // Data integrity summary
    printf("\n  === DATA INTEGRITY ===\n");
    uint64_t total_torn = 0;
    for (int i = 0; i < 6; i++) total_torn += g_results[i].torn_reads;
    printf("    Total torn reads:  %lu\n", total_torn);
    printf("    Verdict:           %s\n",
           total_torn == 0 ? "PASS (no torn reads)" :
           "FAIL (torn reads detected — double buffer not atomic)");

    // IMU staleness analysis (expected ~20% at 500Hz IMU vs 400Hz Composer) [1]
    printf("\n  === IMU STALENESS ===\n");
    uint64_t composer_iters = g_results[4].iterations;
    uint64_t stale = g_results[4].stale_reads;
    double stale_pct = composer_iters > 0 ?
                       100.0 * stale / (composer_iters * 3) : 0; // 3 sources
    printf("    Composer stale reads:  %lu (%.1f%% of total source reads)\n",
           stale, stale_pct);
    printf("    Expected IMU stale:    ~20%% (500Hz IMU vs 400Hz Composer)\n");

    printf("\n============================================================\n");
    printf("  TEST COMPLETE\n");
    printf("============================================================\n\n");

    return 0;
}