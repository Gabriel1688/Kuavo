#pragma once

#include "../../include/mercury_shm.h"
#include <atomic>
#include <cstdint>
#include <string>

/**
 * LPMS-IG1 Sequential CAN reader (32-bit float mode).
 *
 * Spawns a dedicated thread that receives CAN-over-UDP frames from an
 * external CAN bridge, filters by the configured sequential CAN IDs, and
 * extracts two IEEE-754 float32 values per 8-byte CAN payload.
 *
 * After all 8 frames of a measurement cycle are received, the aggregated
 * data is published to a lock-free SourceDoubleBuffer<ImuStageData>.
 *
 * Base CAN ID = startId + (imuId - 1) * 8
 * Value mode = 32-bit floating point
 *  Start ID = 514h
 *  IMU ID = 1
 * Default frame layout (standard output, 8 frames, 16 floats):
 * CAN ID    Offset    Slot-0             Slot-1
 * 0x514      +0      accX   (g)         accY   (g)
 * 0x515      +1      accZ   (g)         eulerX (rad)   [roll]
 * 0x518      +2      eulerY (rad)       eulerZ (rad)   [pitch, yaw]
 * 0x519      +3      quatW              quatX
 * 0x51a      +4      quatY              quatZ
 */

class ImuReader {
public:
    /// Maximum sequential frames (address space per IMU = 5).
    static constexpr int kMaxFrames = 5;
    /// Two float32 values per CAN frame.
    static constexpr int kMaxFloats = kMaxFrames * 2;
    /// IMU update rate in Hz; used for incremental angle integration.
    static constexpr double kRateHz = 200.0;

    // Named indices into the flat float array.
    enum Idx : int {
        ACC_X = 0,
        ACC_Y = 1,
        ACC_Z = 2,
        EULER_X = 3,
        EULER_Y = 4,
        EULER_Z = 5, // roll, pitch, yaw
        QUAT_W = 6,
        QUAT_X = 7,
        QUAT_Y = 8,
        QUAT_Z = 9,
    };

    ImuReader();
    ~ImuReader();

    ImuReader(const ImuReader &) = delete;
    ImuReader &operator=(const ImuReader &) = delete;

    /// Start the receiver thread.  Reads config from Config::instance().imu().
    void start();

    /// Signal the thread to stop and join it.
    void shutdown();

    /// Inject the lock-free staging buffer before start().
    void setStagingBuffer(mercury::SourceDoubleBuffer<mercury::ImuStageData> *ptr) {
        m_stage = ptr;
    }

private:
    void run();
    static void *threadEntry(void *arg);

    std::string m_serverIp;
    int m_localPort{0};
    int m_numFrames{kMaxFrames};
    // TODO:: ImuType m_imuType{ImuType::LPMS_IG1};
    int m_baseId;

    // Thread
    pthread_t m_threadId{};
    std::atomic<bool> m_shutdown{false};
    bool m_threadCreated{false};

    // Staging buffer (set before start(), read/written by the reader thread only)
    mercury::SourceDoubleBuffer<mercury::ImuStageData> *m_stage{nullptr};

    // Per-cycle accumulator (accessed only by the reader thread)
    int m_frameCount{0};
    mercury::ImuStageData m_accumulator{};
    uint64_t m_sequence{0};
};
