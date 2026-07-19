#pragma once
/**
 * @file mercury_shm.hpp
 * @brief Multi-source shared memory IPC for Mercury humanoid robot (12 DOF)
 *
 * 3 writer threads publish independently to per-source staging buffers:
 *   - IMU thread → imu_stage (500Hz, 8 messages per 2ms cycle) [1]
 *   - Motor Thread A → motor_group_a_stage (motors 0-5)
 *   - Motor Thread B → motor_group_b_stage (motors 6-11)
 *
 * A composer merges all 3 sources into one consistent SensorData snapshot.
 * The controller only reads the composed output.
 *
 * Damiao feedback format [2]:
 *   D[0]=ID|ERR<<4, D[1:2]=POS(16-bit), D[3:4]=VEL(12-bit),
 *   D[4:5]=T(12-bit), D[6]=T_MOS, D[7]=T_Rotor
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mercury {

static constexpr int num_act_joint = 12;
static constexpr int MOTORS_PER_GROUP = 6;
static constexpr const char* SHM_NAME = "/mercury_robot_ipc";
static constexpr uint32_t SHM_MAGIC = 0x4D455243;

// ============================================================
// Timing measurement
// ============================================================
struct TimingStats {
    uint64_t min_ns   = UINT64_MAX;
    uint64_t max_ns   = 0;
    uint64_t total_ns = 0;
    uint64_t count    = 0;

    void record(uint64_t duration_ns) {
        if (duration_ns < min_ns) min_ns = duration_ns;
        if (duration_ns > max_ns) max_ns = duration_ns;
        total_ns += duration_ns;
        count++;
    }

    double avg_us() const { return count > 0 ? (total_ns / count) / 1000.0 : 0; }
    double min_us() const { return min_ns / 1000.0; }
    double max_us() const { return max_ns / 1000.0; }

    void print(const char* label) const {
        printf("  %-35s avg=%.2f us  min=%.2f us  max=%.2f us  (n=%lu)\n",
               label, avg_us(), min_us(), max_us(), count);
    }

    void reset() { min_ns = UINT64_MAX; max_ns = 0; total_ns = 0; count = 0; }
};

inline uint64_t get_monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

// ============================================================
// Per-Source Staging Buffers
// ============================================================

/** IMU data — written by IMU thread at 500Hz [1] */
struct alignas(64) ImuStageData {
    double imu_inc[3];
    double imu_ang_vel[3];
    double imu_acc[3];
    uint64_t timestamp_ns;
    uint64_t sequence;
};

/** Motor group — written by one UDP thread handling 6 motors [2] */
struct alignas(64) MotorGroupStageData {
    double joint_jpos[MOTORS_PER_GROUP];
    double joint_jvel[MOTORS_PER_GROUP];
    double motor_jpos[MOTORS_PER_GROUP];
    double motor_jvel[MOTORS_PER_GROUP];
    double bus_current[MOTORS_PER_GROUP];
    double bus_voltage[MOTORS_PER_GROUP];
    double jtorque[MOTORS_PER_GROUP];
    double motor_current[MOTORS_PER_GROUP];
    double reflected_rotor_inertia[MOTORS_PER_GROUP];
    uint8_t motor_status[MOTORS_PER_GROUP];
    int32_t mos_temperature[MOTORS_PER_GROUP];
    int32_t rotor_temperature[MOTORS_PER_GROUP];
    uint64_t timestamp_ns;
    uint64_t sequence;
};

/** Contact sensors */
struct alignas(64) ContactStageData {
    bool rfoot_contact;
    bool lfoot_contact;
    uint64_t timestamp_ns;
    uint64_t sequence;
};

// ============================================================
// Per-Source Double Buffer Template
// ============================================================

template<typename T>
struct alignas(64) SourceDoubleBuffer {
    T buffers[2];
    alignas(64) std::atomic<uint32_t> write_idx{0};
    alignas(64) std::atomic<uint64_t> sequence{0};
    alignas(64) std::atomic<uint64_t> heartbeat_ns{0};

    void publish(const T& data) {
        uint32_t wb = 1 - write_idx.load(std::memory_order_acquire);
        std::memcpy(&buffers[wb], &data, sizeof(T));
        write_idx.store(wb, std::memory_order_release);
        sequence.fetch_add(1, std::memory_order_release);
        heartbeat_ns.store(data.timestamp_ns, std::memory_order_release);
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
// Composed SensorData — what the controller reads
// ============================================================

struct SensorData {
    double imu_inc[3];
    double imu_ang_vel[3];
    double imu_acc[3];

    double joint_jpos[num_act_joint];
    double joint_jvel[num_act_joint];
    double motor_jpos[num_act_joint];
    double motor_jvel[num_act_joint];
    double bus_current[num_act_joint];
    double bus_voltage[num_act_joint];
    double jtorque[num_act_joint];
    double motor_current[num_act_joint];
    double reflected_rotor_inertia[num_act_joint];

    bool rfoot_contact;
    bool lfoot_contact;

    // Per-source timestamps for staleness detection
    uint64_t imu_timestamp_ns;
    uint64_t motor_group_a_timestamp_ns;
    uint64_t motor_group_b_timestamp_ns;
    uint64_t compose_timestamp_ns;

    // Per-source sequence numbers
    uint64_t imu_sequence;
    uint64_t motor_group_a_sequence;
    uint64_t motor_group_b_sequence;
};

// ============================================================
// Command — Controller → Actuator
// ============================================================

struct Command {
    double jtorque_cmd[num_act_joint];
    double jpos_cmd[num_act_joint];
    double jvel_cmd[num_act_joint];
    double kp[num_act_joint];
    double kd[num_act_joint];
    uint8_t control_mode[num_act_joint];
    uint8_t enabled[num_act_joint];
    uint64_t timestamp_ns;
    uint64_t sequence;
};

// ============================================================
// Shared Memory Layout
// ============================================================

struct SharedMemoryLayout {
    uint32_t magic;
    uint32_t version;
    uint32_t num_joints;
    uint32_t control_freq_hz;

    // Per-source staging (3 independent double buffers)
    SourceDoubleBuffer<ImuStageData>        imu_stage;
    SourceDoubleBuffer<MotorGroupStageData> motor_group_a_stage;
    SourceDoubleBuffer<MotorGroupStageData> motor_group_b_stage;
    SourceDoubleBuffer<ContactStageData>    contact_stage;

    // Composed output for controller
    SensorData composed_buffers[2];
    alignas(64) std::atomic<uint32_t> composed_write_idx{0};
    alignas(64) std::atomic<uint64_t> composed_sequence{0};

    // Command (controller → actuator)
    Command cmd_buffers[2];
    alignas(64) std::atomic<uint32_t> cmd_write_idx{0};
    alignas(64) std::atomic<uint64_t> cmd_sequence{0};

    // Watchdog
    alignas(64) std::atomic<uint64_t> controller_heartbeat_ns{0};
    alignas(64) std::atomic<bool> emergency_stop{false};

    uint16_t motor_can_ids[num_act_joint];
};

static_assert(std::is_trivially_copyable_v<SensorData>);
static_assert(std::is_trivially_copyable_v<Command>);
static_assert(std::is_trivially_copyable_v<ImuStageData>);
static_assert(std::is_trivially_copyable_v<MotorGroupStageData>);
static_assert(std::is_trivially_copyable_v<ContactStageData>);

} // namespace mercury