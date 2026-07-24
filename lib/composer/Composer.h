#pragma once
/**
 * @file Composer.h
 * @brief Lock-free multi-source sensor data composition thread.
 *
 * Reads from three per-source SourceDoubleBuffer instances (IMU, Motor
 * Group A, Motor Group B) plus an atomic MotorParamCache, merges into
 * a flat SensorData snapshot, writes to shared-memory double buffer,
 * and accumulates BatchLogRecords into a process-local SPSC ring buffer.
 *
 * Batching follows the proven pattern from test_actuator_logger.cpp:
 * accumulate BATCH_SIZE (20) sensor+command pairs per ring entry,
 * reducing MQTT messages from 400/s to ~20/s at 400Hz.
 *
 * Thread: SCHED_FIFO priority 85, 256 KiB stack, 400 Hz (2.5 ms).
 */

#include "../../include/mercury_shm.h"
#include "MotorParamCache.h"

#include <atomic>
#include <cstdint>
#include <pthread.h>

namespace mercury {

/// SPSC ring capacity for batched records (256 batches * 20 samples = 5120 samples buffered)
static constexpr size_t BATCH_RING_CAPACITY = 256;

class Composer {
public:
    // Staleness timeout constants
    static constexpr uint64_t IMU_STALE_TIMEOUT_MS          = 50;
    static constexpr uint64_t MOTOR_GROUP_STALE_TIMEOUT_MS  = 100;
    static constexpr uint64_t IMU_CRITICAL_TIMEOUT_MS       = 200;

    // Staleness bitmask bits
    static constexpr uint8_t STALE_IMU           = 0x01;
    static constexpr uint8_t STALE_MOTOR_GROUP_A = 0x02;
    static constexpr uint8_t STALE_MOTOR_GROUP_B = 0x04;

    // Thread scheduling
    static constexpr int    SCHED_PRIORITY  = 85;
    static constexpr size_t STACK_SIZE      = 256 * 1024;  // 256 KiB
    static constexpr uint64_t PERIOD_NS     = 2'500'000;   // 2.5 ms = 400 Hz

    /**
     * @param imu_stage          Per-source staging buffer for IMU data
     * @param motor_group_a      Per-source staging buffer for motors 1-6
     * @param motor_group_b      Per-source staging buffer for motors 7-12
     * @param param_cache        Atomic parameter cache (bus voltage/current)
     * @param shm                Shared memory layout (composed_buffers output)
     * @param ring               Process-local SPSC ring buffer for batched logging
     */
    Composer(SourceDoubleBuffer<ImuStageData>&        imu_stage,
             SourceDoubleBuffer<MotorGroupStageData>& motor_group_a,
             SourceDoubleBuffer<MotorGroupStageData>& motor_group_b,
             MotorParamCache&                         param_cache,
             SharedMemoryLayout&                      shm,
             SPSCRingBuffer<BatchLogRecord, BATCH_RING_CAPACITY>& ring);

    ~Composer();

    Composer(const Composer&) = delete;
    Composer& operator=(const Composer&) = delete;

    /** Start the composer thread. */
    void start();

    /** Signal stop and join the thread. */
    void shutdown();

    /**
     * Returns a bitmask indicating which data sources are stale.
     * Thread-safe — called by the main loop at 100 Hz.
     *
     * Bit 0x01 = IMU stale (> IMU_STALE_TIMEOUT_MS)
     * Bit 0x02 = Motor Group A stale (> MOTOR_GROUP_STALE_TIMEOUT_MS)
     * Bit 0x04 = Motor Group B stale (> MOTOR_GROUP_STALE_TIMEOUT_MS)
     */
    uint8_t check_staleness() const;

    /** Dropped batch count (diagnostic). */
    uint64_t dropped() const { return dropped_; }

private:
    /** Single compose cycle: read sources, merge, write SHM, accumulate batch. */
    void compose_cycle();

    /** Flush the current batch to the SPSC ring. */
    void flush_batch();

    /** Thread run loop: 400 Hz clock_nanosleep. */
    void run();

    static void* thread_entry(void* arg);

    // Data sources (references — lifetime managed externally)
    SourceDoubleBuffer<ImuStageData>&        imu_stage_;
    SourceDoubleBuffer<MotorGroupStageData>& motor_group_a_;
    SourceDoubleBuffer<MotorGroupStageData>& motor_group_b_;
    MotorParamCache&                         param_cache_;
    SharedMemoryLayout&                      shm_;
    SPSCRingBuffer<BatchLogRecord, BATCH_RING_CAPACITY>& ring_;

    // Thread state
    pthread_t thread_id_{};
    std::atomic<bool> running_{false};
    bool thread_created_{false};

    // Per-source staleness tracking (updated each compose cycle)
    // Timestamps initialized in start() to current monotonic time so the system
    // gets a grace period before declaring sources stale on first boot.
    std::atomic<uint64_t> last_imu_change_ns_{0};
    std::atomic<uint64_t> last_grp_a_change_ns_{0};
    std::atomic<uint64_t> last_grp_b_change_ns_{0};
    // Sequence sentinels: UINT64_MAX ensures the first real sequence (0 or 1)
    // always triggers a timestamp update.
    uint64_t prev_imu_seq_{UINT64_MAX};
    uint64_t prev_grp_a_seq_{UINT64_MAX};
    uint64_t prev_grp_b_seq_{UINT64_MAX};

    // Batch accumulator (local to composer thread, no synchronization needed)
    BatchLogRecord batch_{};
    uint32_t batch_idx_{0};
    uint64_t batch_seq_{0};

    // Diagnostic
    uint64_t dropped_{0};
};

} // namespace mercury
