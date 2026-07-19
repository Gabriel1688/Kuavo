# Shared Memory Considerations for Multi-Source Sensor Data
When SensorData is populated by 3 independent writer threads (1 IMU thread + 2 motor UDP threads), several critical issues arise compared to the single-writer double-buffer design.

Problem: Multiple Writers Break the Double-Buffer Pattern
The original double-buffer design assumes one writer and one reader. With 3 writer threads updating different fields of the same SensorData struct, the controller (reader) can observe a torn read — a snapshot where IMU data is from time T₁, motors 1-6 from time T₂, and motors 7-12 from time T₃.

The IMU streams at 500Hz with 8 messages per 2ms cycle , while each motor UDP thread handles 6 motors with feedback frames decoded per the Damiao protocol . These 3 sources update at different rates and are not synchronized.

## Key Considerations
### 1. Data Consistency (Torn Read Prevention)
   If Thread A writes imu_acc[2] while Thread B writes joint_jpos[5] and the controller reads the entire struct simultaneously, the controller gets a mix of old and new values from different time instants. This is especially dangerous for whole-body control where IMU orientation and joint positions must be temporally coherent.

### 2. Cache Line False Sharing
   On modern CPUs, a cache line is typically 64 bytes. If the IMU thread writes imu_acc[2] and a motor thread writes joint_jpos[0], and these fields happen to share the same cache line, both CPU cores invalidate each other's cache — destroying performance even though they write to logically independent fields.

### 3. Timestamp Coherence
   Each source has its own update rate. A single timestamp_ns field on SensorData cannot accurately represent when all 3 sources last updated. The controller needs to know the freshness of each source independently.

### 4. Source Availability
   If the IMU thread crashes but motor threads continue, the controller must detect that IMU data is stale while motor data is fresh, rather than treating the entire SensorData as valid or invalid.

## Recommended Solution: Per-Source Staging + Atomic Compose
Split the struct into per-source staging buffers (each with its own double buffer and timestamp), then atomically compose the final SensorData snapshot before publishing to the controller.
'''
#include <atomic>
#include <cstdint>
#include <cstring>

namespace mercury {

static constexpr int num_act_joint = 12;
static constexpr int MOTORS_PER_GROUP = 6;

// ============================================================
// Per-Source Staging Buffers
// Each source has its own double buffer + timestamp + sequence
// ============================================================

/**
* IMU data — written by the IMU thread at 500Hz [1]
* The IMU streams 8 messages per 2ms cycle via
* CAN-over-UDP 13-byte frames [1]
  */
  struct alignas(64) ImuStageData {
  double imu_inc[3];
  double imu_ang_vel[3];
  double imu_acc[3];
  uint64_t timestamp_ns;
  uint64_t sequence;
  };

/**
* Motor group data — written by one UDP thread
* handling 6 Damiao motors.
*
* Feedback decoded per protocol [2]:
*   D[0]=ID|ERR<<4, D[1:2]=POS(16-bit),
*   D[3:4]=VEL(12-bit), D[4:5]=T(12-bit),
*   D[6]=T_MOS, D[7]=T_Rotor
*
* Each motor group uses CAN-over-UDP
* 13-byte frames on its own socket [1]
  */
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

/**
* Contact sensor data — may come from either motor
* thread or a separate sensor
  */
  struct alignas(64) ContactStageData {
  bool rfoot_contact;
  bool lfoot_contact;
  uint64_t timestamp_ns;
  uint64_t sequence;
  };

// ============================================================
// Per-Source Double Buffer
// Each source writer uses its own independent double buffer
// so no writer ever contends with another writer
// ============================================================

template<typename T>
struct alignas(64) SourceDoubleBuffer {
T buffers[2];
alignas(64) std::atomic<uint32_t> write_idx{0};
alignas(64) std::atomic<uint64_t> sequence{0};
alignas(64) std::atomic<uint64_t> heartbeat_ns{0};

    /**
     * Writer: publish new data to the inactive buffer
     * then atomically swap the index.
     * Each source thread calls this independently —
     * no contention with other sources.
     */
    void publish(const T& data) {
        uint32_t wb = 1 - write_idx.load(std::memory_order_acquire);
        std::memcpy(&buffers[wb], &data, sizeof(T));
        write_idx.store(wb, std::memory_order_release);
        sequence.fetch_add(1, std::memory_order_release);
        heartbeat_ns.store(data.timestamp_ns,
                           std::memory_order_release);
    }

    /**
     * Reader: get the latest published data.
     * Returns a complete, consistent snapshot
     * of this source's data.
     */
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
// Same struct as the original Mercury_SensorData
// ============================================================

struct SensorData {
// IMU data
double imu_inc[3];
double imu_ang_vel[3];
double imu_acc[3];

    // Joint-level state [2]
    double joint_jpos[num_act_joint];
    double joint_jvel[num_act_joint];

    // Motor-level state
    double motor_jpos[num_act_joint];
    double motor_jvel[num_act_joint];

    // Electrical measurements
    double bus_current[num_act_joint];
    double bus_voltage[num_act_joint];

    // Torque [2]
    double jtorque[num_act_joint];
    double motor_current[num_act_joint];
    double reflected_rotor_inertia[num_act_joint];

    // Contact sensors
    bool rfoot_contact;
    bool lfoot_contact;

    // Per-source timestamps for staleness detection
    uint64_t imu_timestamp_ns;
    uint64_t motor_group_a_timestamp_ns;  // Motors 0-5
    uint64_t motor_group_b_timestamp_ns;  // Motors 6-11
    uint64_t compose_timestamp_ns;        // When snapshot was composed

    // Per-source sequence numbers
    uint64_t imu_sequence;
    uint64_t motor_group_a_sequence;
    uint64_t motor_group_b_sequence;
};

// ============================================================
// Shared Memory Layout — Multi-Source Architecture
// ============================================================

struct SharedMemoryLayout {
uint32_t magic;
uint32_t version;
uint32_t num_joints;
uint32_t control_freq_hz;

    // ---- Per-Source Staging (3 independent double buffers) ----
    // Each source thread writes ONLY to its own buffer.
    // No cross-thread contention on writes.
    SourceDoubleBuffer<ImuStageData>        imu_stage;
    SourceDoubleBuffer<MotorGroupStageData> motor_group_a_stage; // Motors 0-5
    SourceDoubleBuffer<MotorGroupStageData> motor_group_b_stage; // Motors 6-11
    SourceDoubleBuffer<ContactStageData>    contact_stage;

    // ---- Composed Output (single double buffer for controller) ----
    SensorData composed_buffers[2];
    alignas(64) std::atomic<uint32_t> composed_write_idx{0};
    alignas(64) std::atomic<uint64_t> composed_sequence{0};

    // ---- Command (unchanged — single writer from controller) ----
    // ... Command cmd_buffers[2] etc ...

    // ---- Safety ----
    alignas(64) std::atomic<bool> emergency_stop{false};
    uint16_t motor_can_ids[num_act_joint];
};

// ============================================================
// Composer — runs in a dedicated thread or at the start
// of each actuator cycle. Reads all 3 sources and produces
// one consistent SensorData snapshot.
// ============================================================

class SensorComposer {
public:
SensorComposer(SharedMemoryLayout* layout) : layout_(layout) {}

    /**
     * Compose a single consistent SensorData snapshot
     * from all 3 independent source buffers.
     *
     * This is the ONLY function that writes to
     * composed_buffers. It reads from the 3 per-source
     * staging buffers (each independently double-buffered)
     * and merges them into one flat struct.
     *
     * Call this once per control cycle (e.g., at 1kHz).
     */
    void compose() {
        SensorData snapshot;

        // ---- Read IMU (500Hz source) [1] ----
        ImuStageData imu = layout_->imu_stage.read();
        std::memcpy(snapshot.imu_inc,     imu.imu_inc,     sizeof(imu.imu_inc));
        std::memcpy(snapshot.imu_ang_vel, imu.imu_ang_vel, sizeof(imu.imu_ang_vel));
        std::memcpy(snapshot.imu_acc,     imu.imu_acc,     sizeof(imu.imu_acc));
        snapshot.imu_timestamp_ns = imu.timestamp_ns;
        snapshot.imu_sequence     = imu.sequence;

        // ---- Read Motor Group A: joints 0-5 ----
        // Feedback decoded per Damiao protocol [2]:
        //   POS(16-bit mapped to [-PMAX,PMAX]),
        //   VEL(12-bit), T(12-bit)
        MotorGroupStageData grpA = layout_->motor_group_a_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            snapshot.joint_jpos[j]            = grpA.joint_jpos[j];
            snapshot.joint_jvel[j]            = grpA.joint_jvel[j];
            snapshot.motor_jpos[j]            = grpA.motor_jpos[j];
            snapshot.motor_jvel[j]            = grpA.motor_jvel[j];
            snapshot.bus_current[j]           = grpA.bus_current[j];
            snapshot.bus_voltage[j]           = grpA.bus_voltage[j];
            snapshot.jtorque[j]               = grpA.jtorque[j];
            snapshot.motor_current[j]         = grpA.motor_current[j];
            snapshot.reflected_rotor_inertia[j] = grpA.reflected_rotor_inertia[j];
        }
        snapshot.motor_group_a_timestamp_ns = grpA.timestamp_ns;
        snapshot.motor_group_a_sequence     = grpA.sequence;

        // ---- Read Motor Group B: joints 6-11 ----
        MotorGroupStageData grpB = layout_->motor_group_b_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            int idx = MOTORS_PER_GROUP + j;  // offset 6-11
            snapshot.joint_jpos[idx]            = grpB.joint_jpos[j];
            snapshot.joint_jvel[idx]            = grpB.joint_jvel[j];
            snapshot.motor_jpos[idx]            = grpB.motor_jpos[j];
            snapshot.motor_jvel[idx]            = grpB.motor_jvel[j];
            snapshot.bus_current[idx]           = grpB.bus_current[j];
            snapshot.bus_voltage[idx]           = grpB.bus_voltage[j];
            snapshot.jtorque[idx]               = grpB.jtorque[j];
            snapshot.motor_current[idx]         = grpB.motor_current[j];
            snapshot.reflected_rotor_inertia[idx] = grpB.reflected_rotor_inertia[j];
        }
        snapshot.motor_group_b_timestamp_ns = grpB.timestamp_ns;
        snapshot.motor_group_b_sequence     = grpB.sequence;

        // ---- Read Contact Sensors ----
        ContactStageData contact = layout_->contact_stage.read();
        snapshot.rfoot_contact = contact.rfoot_contact;
        snapshot.lfoot_contact = contact.lfoot_contact;

        // ---- Compose timestamp ----
        snapshot.compose_timestamp_ns = get_monotonic_ns();

        // ---- Publish composed snapshot (double-buffered) ----
        uint32_t wb = 1 - layout_->composed_write_idx.load(
            std::memory_order_acquire);
        std::memcpy(&layout_->composed_buffers[wb],
                     &snapshot, sizeof(SensorData));
        layout_->composed_write_idx.store(wb,
            std::memory_order_release);
        layout_->composed_sequence.fetch_add(1,
            std::memory_order_release);
    }

    /**
     * Check if any source is stale.
     * Returns a bitmask: bit 0=IMU, bit 1=GroupA, bit 2=GroupB
     *
     * Maps to Damiao ERR=0x0D (communication lost) [2]
     */
    uint8_t check_staleness(uint64_t timeout_ns = 50'000'000) const {
        uint8_t stale = 0;
        uint64_t now = get_monotonic_ns();

        if ((now - layout_->imu_stage.get_heartbeat()) > timeout_ns)
            stale |= 0x01;
        if ((now - layout_->motor_group_a_stage.get_heartbeat()) > timeout_ns)
            stale |= 0x02;
        if ((now - layout_->motor_group_b_stage.get_heartbeat()) > timeout_ns)
            stale |= 0x04;

        return stale;
    }

private:
SharedMemoryLayout* layout_;

    static uint64_t get_monotonic_ns() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
               + ts.tv_nsec;
    }
};

} // namespace mercury
'''
## Thread Responsibilities
| Thread | Source | Write Target | Update Rate | Data |
|--------|-------|-------------|:-----------:|------|
| **IMU Thread** | IMU CAN-over-UDP socket [1] | `imu_stage` only | 500Hz [1] | `imu_inc`, `imu_ang_vel`, `imu_acc` |
| **Motor Thread A** | Motor UDP socket (motors 0-5) | `motor_group_a_stage` only | 1kHz | Joints 0-5: position, velocity, torque, temperature [2] |
| **Motor Thread B** | Motor UDP socket (motors 6-11) | `motor_group_b_stage` only | 1kHz | Joints 6-11: position, velocity, torque, temperature [2] |
| **Composer** | Reads all 3 staging buffers | `composed_buffers` | 1kHz | Full `SensorData` snapshot |
| **Controller** | Reads `composed_buffers` | (read-only) | 1kHz | Consumes the composed snapshot |

## Why This Design Solves Each Problem
| Problem | Solution |
|---------|----------|
| **Torn reads** | Each source writes to its own double buffer independently. The composer reads 3 consistent per-source snapshots and merges them into one `SensorData` at a single point in time. The controller only ever reads the composed output. |
| **Cache line false sharing** | Each staging buffer is `alignas(64)`, ensuring the IMU fields, motor group A fields, and motor group B fields are on separate cache lines. No cross-thread cache invalidation. |
| **Timestamp coherence** | Each source carries its own `timestamp_ns`. The composed snapshot includes `imu_timestamp_ns`, `motor_group_a_timestamp_ns`, and `motor_group_b_timestamp_ns` so the controller can detect temporal skew. |
| **Source availability** | `check_staleness()` returns a bitmask indicating which sources are stale. If motor group B crashes, the controller sees `stale & 0x04` and can react (e.g., disable joints 6-11) without losing IMU or motors 0-5. This maps to the Damiao communication-lost error (ERR=0x0D) [2]. |
| **Lock-free** | All operations use `std::atomic` with acquire/release semantics. No mutex, no spinlock, no blocking. |

## Data Flow
IMU Thread (500Hz)          Motor Thread A (1kHz)     Motor Thread B (1kHz)
│                           │                         │
│ recvfrom(13B) [1]         │ recvfrom(13B) [1]       │ recvfrom(13B) [1]
│ decode IMU data           │ decode feedback [2]     │ decode feedback [2]
│                           │ motors 0-5              │ motors 6-11
└───────────────────────────┬────────────┴────────────┬────────────┘
        │                   │                         │
        ▼                   ▼                         ▼
┌─────────────┐       ┌─────────────┐           ┌─────────────┐
│ imu_stage   │       │ motor_grp_a │           │ motor_grp_b │
│ [double buf]│       │ [double buf]│           │ [double buf]│
└──────┬──────┘       └──────┬──────┘           └──────┬──────┘
       └────────────┬────────┴────────────┬────────────┘
                    │
                    ▼                         │
             ┌──────────────┐                 │
             │   Composer   │◄────────────────┘
             │  (1kHz tick) │
             │  Read all 3  │
             │  Merge → flat│
             │  SensorData  │
             └──────┬───────┘
                    │
                    ▼
             ┌──────────────┐
             │ composed_buf │
             │ [double buf] │
             └──────┬───────┘
                    │
                    ▼
             ┌──────────────┐
             │  Controller  │
             │  (1kHz read) │
             └──────────────┘

The compose step ensures the controller always receives a complete, consistent snapshot of all 12 joints plus IMU, even though the underlying data arrives from 3 independent asynchronous sources at different rates .