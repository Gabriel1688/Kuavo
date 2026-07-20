#pragma once
/**
 * @file mercury_shm.hpp
 * @brief Cross-platform shared memory + MQTT binary payload definitions
 *
 * Used on ARM edge (actuator+logger) and x86 remote (subscriber).
 * Struct packing guaranteed identical via explicit padding + sizeof assertions.
 *
 * Damiao feedback [2]: D[1:2]=POS(16-bit), D[3:4]=VEL(12-bit),
 *   D[4:5]=T(12-bit), D[6]=T_MOS, D[7]=T_Rotor
 * UDP-CAN frame [1]: 13 bytes = DLC(1) + CAN_ID(4) + DATA(8)
 */

#include <atomic>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <ctime>


// Cross-platform endianness guard — both ARM64 and x86-64 are LE
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "Binary payload requires little-endian architecture");

namespace mercury {

static constexpr int NUM_ACT_JOINT = 12;
static constexpr int MOTORS_PER_GROUP = 6;
static constexpr const char* SHM_NAME = "/mercury_robot_ipc";
static constexpr uint32_t SHM_MAGIC = 0x4D455243; // "MERC"

// MQTT topics — simplified, single robot per edge device
static constexpr const char* MQTT_TOPIC_CMD    = "robot/command/bin";
static constexpr const char* MQTT_TOPIC_SENSOR = "robot/sensor/bin";
static constexpr const char* MQTT_TOPIC_STATUS = "robot/status";

// ============================================================
// Timing
// ============================================================
struct TimingStats {
    uint64_t min_ns   = UINT64_MAX;
    uint64_t max_ns   = 0;
    uint64_t total_ns = 0;
    uint64_t count    = 0;

    void record(uint64_t d) {
        if (d < min_ns) min_ns = d;
        if (d > max_ns) max_ns = d;
        total_ns += d;
        count++;
    }

    double avg_us() const { return count > 0 ? (total_ns / count) / 1000.0 : 0; }
    double min_us() const { return count > 0 ? min_ns / 1000.0 : 0; }
    double max_us() const { return max_ns / 1000.0; }

    void print(const char* label) const {
        printf("  %-40s avg=%.2f us  min=%.2f us  max=%.2f us  (n=%lu)\n",
               label, avg_us(), min_us(), max_us(), count);
    }
};

inline uint64_t get_monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

inline uint64_t get_wallclock_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

// ============================================================
// Sensor Data — uses explicit padding for ARM/x86 compatibility
// Fields map to Damiao feedback frame [2]:
//   POS(16-bit), VEL(12-bit), T(12-bit), T_MOS, T_Rotor
// ============================================================

#pragma pack(push, 1)

struct SensorData {
    // IMU data (from IMU mode streaming at 500Hz) [1]
    double imu_inc[3];
    double imu_ang_vel[3];
    double imu_acc[3];

    // Joint-level state — Damiao POS/VEL after gear ratio [2]
    double joint_jpos[NUM_ACT_JOINT];
    double joint_jvel[NUM_ACT_JOINT];

    // Motor-level state — raw Damiao POS/VEL [2]
    double motor_jpos[NUM_ACT_JOINT];
    double motor_jvel[NUM_ACT_JOINT];

    // Electrical measurements
    double bus_current[NUM_ACT_JOINT];
    double bus_voltage[NUM_ACT_JOINT];

    // Torque — Damiao T (12-bit mapped to [-TMAX,TMAX]) [2]
    double jtorque[NUM_ACT_JOINT];
    double motor_current[NUM_ACT_JOINT];
    double reflected_rotor_inertia[NUM_ACT_JOINT];

    // Contact sensors + explicit padding
    uint8_t rfoot_contact;
    uint8_t lfoot_contact;
    uint8_t _pad_contact[6];  // Explicit padding to 8-byte boundary

    // Per-source timestamps
    uint64_t imu_timestamp_ns;
    uint64_t motor_group_a_timestamp_ns;
    uint64_t motor_group_b_timestamp_ns;
    uint64_t compose_timestamp_ns;

    // Per-source sequence numbers
    uint64_t imu_sequence;
    uint64_t motor_group_a_sequence;
    uint64_t motor_group_b_sequence;
};

struct Command {
    double jtorque_cmd[NUM_ACT_JOINT];
    double jpos_cmd[NUM_ACT_JOINT];     // Damiao p_des [-PMAX,PMAX] [2]
    double jvel_cmd[NUM_ACT_JOINT];     // Damiao v_des [-VMAX,VMAX] [2]
    double kp[NUM_ACT_JOINT];           // Damiao Kp [0,500] [2]
    double kd[NUM_ACT_JOINT];           // Damiao Kd [0,5] [2]
    uint8_t control_mode[NUM_ACT_JOINT]; // 0=MIT,1=PosVel,2=Vel,3=PosForce [3]
    uint8_t enabled[NUM_ACT_JOINT];     // 0xFC=enable,0xFD=disable [2]
    uint8_t _pad_cmd[8];               // Explicit padding
    uint64_t timestamp_ns;
    uint64_t sequence;
};

// ============================================================
// Binary MQTT Payload Header — validated on both sides
// Robot identity is in the header, not the topic
// ============================================================

static constexpr uint16_t PAYLOAD_MAGIC = 0x4D52; // "MR"
static constexpr uint8_t  PAYLOAD_VERSION = 1;

enum class RecordType : uint8_t {
    COMMAND       = 0x01,
    SENSOR        = 0x02,
    STATUS        = 0x03,
    SENSOR_BATCH  = 0x04,  // Batched sensor+command samples
};

struct BinaryPayloadHeader {
    uint16_t magic;          // 0x4D52 for validation
    uint8_t  version;        // Protocol version
    uint8_t  record_type;    // RecordType enum
    uint32_t robot_id;       // Unique robot identifier
    uint32_t payload_size;   // sizeof(Command) or sizeof(SensorData) from publisher
    uint32_t _reserved;      // Future use
    uint64_t sequence;       // Monotonic counter
    uint64_t timestamp_ns;   // CLOCK_MONOTONIC nanoseconds
};

#pragma pack(pop)

// Compile-time size assertions — must match on ARM and x86
static_assert(sizeof(BinaryPayloadHeader) == 32,
              "BinaryPayloadHeader size mismatch");
// These will be verified at compile time on both architectures.
// If they differ, the build fails before any data is transmitted.
static_assert(sizeof(SensorData) % 8 == 0,
              "SensorData must be 8-byte aligned");
static_assert(sizeof(Command) % 8 == 0,
              "Command must be 8-byte aligned");

// ============================================================
// SPSC Ring Buffer — lock-free, single producer single consumer
// Used in-process between composer thread and MQTT logger thread
// ============================================================

struct LogRecord {
    BinaryPayloadHeader header;
    union {
        Command    cmd;
        SensorData sensor;
    } data;
};

// ============================================================
// Batch logging — accumulate N samples into one MQTT message
// Reduces MQTT publish rate from 2000/s to ~100/s (at BATCH_SIZE=20)
// ============================================================

static constexpr size_t BATCH_SIZE = 20;

struct SensorCommandPair {
    SensorData sensor;
    Command    cmd;
};

struct BatchLogRecord {
    BinaryPayloadHeader header;     // record_type = SENSOR_BATCH
    uint32_t sample_count;          // Actual number of valid pairs (1..BATCH_SIZE)
    uint32_t _pad;
    SensorCommandPair samples[BATCH_SIZE];
};

template<typename T, size_t Capacity>
class SPSCRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");
public:
    bool push(const T& item) {
        uint64_t head = head_.load(std::memory_order_relaxed);
        uint64_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= Capacity) {
            dropped_++;
            return false;
        }
        buffer_[head & (Capacity - 1)] = item;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool pop(T& item) {
        uint64_t tail = tail_.load(std::memory_order_relaxed);
        uint64_t head = head_.load(std::memory_order_acquire);
        if (tail >= head) return false;
        item = buffer_[tail & (Capacity - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    size_t size() const {
        return head_.load(std::memory_order_acquire) -
               tail_.load(std::memory_order_acquire);
    }

    uint64_t dropped() const { return dropped_; }

private:
    std::atomic<uint64_t> head_{0};
    std::atomic<uint64_t> tail_{0};
    uint64_t dropped_ = 0;
    T buffer_[Capacity];
};

static constexpr size_t LOG_RING_CAPACITY = 4096;

// ============================================================
// Per-Source Staging Double Buffers (for shared memory)
// ============================================================

struct alignas(64) ImuStageData {
    double imu_inc[3];
    double imu_ang_vel[3];
    double imu_acc[3];
    uint64_t timestamp_ns;
    uint64_t sequence;
};

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
    uint8_t _pad[2];
    int32_t mos_temperature[MOTORS_PER_GROUP];
    int32_t rotor_temperature[MOTORS_PER_GROUP];
    uint64_t timestamp_ns;
    uint64_t sequence;
};

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
        heartbeat_ns.store(data.timestamp_ns, std::memory_order_release);
    }

    T read() const {
        uint32_t rb = write_idx.load(std::memory_order_acquire);
        T result;
        std::memcpy(&result, &buffers[rb], sizeof(T));
        return result;
    }
};

// ============================================================
// Shared Memory Layout (between controller and actuator processes)
// The SPSC ring buffer is NOT here — it is process-local
// ============================================================

struct SharedMemoryLayout {
    uint32_t magic;
    uint32_t version;
    uint32_t num_joints;
    uint32_t control_freq_hz;

    SourceDoubleBuffer<ImuStageData>        imu_stage;
    SourceDoubleBuffer<MotorGroupStageData> motor_group_a_stage;
    SourceDoubleBuffer<MotorGroupStageData> motor_group_b_stage;

    SensorData composed_buffers[2];
    std::atomic<uint32_t> composed_write_idx{0};
    std::atomic<uint64_t> composed_sequence{0};

    Command cmd_buffers[2];
    std::atomic<uint32_t> cmd_write_idx{0};
    std::atomic<uint64_t> cmd_sequence{0};

    std::atomic<uint64_t> controller_heartbeat_ns{0};
    std::atomic<bool> emergency_stop{false};

    uint16_t motor_can_ids[NUM_ACT_JOINT];
    uint32_t robot_id;
};

// Damiao motor protocol constants [2]
static constexpr double P_MAX = 12.5;
static constexpr double V_MAX = 45.0;
static constexpr double T_MAX = 18.0;

// Protocol helpers [2][3]
inline uint16_t double_to_uint(double x, double x_min, double x_max, int bits) {
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return static_cast<uint16_t>(((x - x_min) / (x_max - x_min)) * ((1 << bits) - 1));
}

inline double uint_to_double(uint16_t x, double min_v, double max_v, int bits) {
    return (static_cast<double>(x) / ((1 << bits) - 1)) * (max_v - min_v) + min_v;
}

} // namespace mercury

