#pragma once
/**
 * @file mercury_shm.h
 * @brief Shared memory IPC for Mercury humanoid robot (12 DOF)
 *
 * Data structures aligned to:
 *   - Mercury_SensorData / Mercury_Command (user-provided)
 *   - Damiao motor feedback frame: POS(16-bit), VEL(12-bit), T(12-bit),
 *     T_MOS, T_Rotor [2]
 *   - Damiao MIT control: p_des, v_des, Kp, Kd, t_ff [2]
 *   - CAN-over-UDP 13-byte frame format [1]
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
static constexpr const char* SHM_NAME = "/mercury_robot_ipc";
static constexpr uint32_t SHM_MAGIC = 0x4D455243; // "MERC"

// ============================================================
// Timing measurement structure
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
    double avg_ns() const { return count > 0 ? static_cast<double>(total_ns) / count : 0; }

    void print(const char* label) const {
        printf("  %-30s  avg=%.2f us  min=%.2f us  max=%.2f us  (n=%lu)\n",
               label, avg_us(), min_us(), max_us(), count);
    }
};

// ============================================================
// SensorData: Actuator → Controller
// Mirrors Mercury_SensorData [user-provided]
// Feedback fields map to Damiao protocol [2]:
//   D[1:2]=POS(16-bit), D[3:4]=VEL(12-bit), D[4:5]=T(12-bit),
//   D[6]=T_MOS, D[7]=T_Rotor
// ============================================================
struct SensorData {
    // IMU data (from IMU mode streaming at 500Hz [1])
    double imu_inc[3];
    double imu_ang_vel[3];
    double imu_acc[3];

    // Joint-level state — maps to Damiao POS/VEL [2]
    double joint_jpos[num_act_joint];
    double joint_jvel[num_act_joint];

    // Motor-level state (raw encoder)
    double motor_jpos[num_act_joint];
    double motor_jvel[num_act_joint];

    // Electrical measurements
    double bus_current[num_act_joint];
    double bus_voltage[num_act_joint];

    // Torque — maps to Damiao T (12-bit) [2]
    double jtorque[num_act_joint];
    double motor_current[num_act_joint];
    double reflected_rotor_inertia[num_act_joint];

    // Contact sensors
    bool rfoot_contact;
    bool lfoot_contact;

    // Damiao-specific per-motor status [2]
    // 0=disabled, 1=enabled, 8=overvoltage, ..., D=comm_lost
    uint8_t motor_status[num_act_joint];

    // Damiao temperature feedback [2]: T_MOS, T_Rotor
    int32_t mos_temperature[num_act_joint];
    int32_t rotor_temperature[num_act_joint];

    // Timestamp for round-trip latency measurement
    uint64_t timestamp_ns;
    uint64_t sequence;
};

// ============================================================
// Command: Controller → Actuator
// Mirrors Mercury_Command [user-provided]
// Maps to Damiao MIT mode: p_des, v_des, Kp, Kd, t_ff [2]
// ============================================================
struct Command {
    // Torque feedforward — maps to Damiao t_ff [2]
    double jtorque_cmd[num_act_joint];
    // Position — maps to Damiao p_des [-PMAX, PMAX] [2]
    double jpos_cmd[num_act_joint];
    // Velocity — maps to Damiao v_des [-VMAX, VMAX] [2]
    double jvel_cmd[num_act_joint];

    // Damiao MIT gains [2]: Kp [0,500], Kd [0,5]
    double kp[num_act_joint];
    double kd[num_act_joint];

    // Per-motor control mode [3]:
    // 0=MIT, 1=PosVel(+0x100), 2=Velocity(+0x200), 3=PosForce(+0x300)
    uint8_t control_mode[num_act_joint];

    // Per-motor enable [2]: maps to 0xFC(enable)/0xFD(disable)
    uint8_t enabled[num_act_joint];

    uint64_t timestamp_ns;
    uint64_t sequence;
};

// ============================================================
// Lock-free double-buffered shared memory layout
// ============================================================
struct SharedMemoryLayout {
    uint32_t magic;
    uint32_t version;
    uint32_t num_joints;
    uint32_t control_freq_hz;

    // Controller → Actuator
    Command cmd_buffers[2];
    alignas(64) std::atomic<uint32_t> cmd_write_idx;
    alignas(64) std::atomic<uint64_t> cmd_sequence;

    // Actuator → Controller
    SensorData state_buffers[2];
    alignas(64) std::atomic<uint32_t> state_write_idx;
    alignas(64) std::atomic<uint64_t> state_sequence;

    // Watchdog
    alignas(64) std::atomic<uint64_t> controller_heartbeat_ns;
    alignas(64) std::atomic<uint64_t> actuator_heartbeat_ns;
    alignas(64) std::atomic<bool> emergency_stop;

    // Motor CAN ID config [2]
    uint16_t motor_can_ids[num_act_joint];
};

static_assert(std::is_trivially_copyable_v<SensorData>);
static_assert(std::is_trivially_copyable_v<Command>);

// ============================================================
// Utility: monotonic clock
// ============================================================
inline uint64_t get_monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

} // namespace mercury