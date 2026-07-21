#pragma once

#include "../../tools/mercury_shm_v2.h"
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
 * 0x515      +1      accZ   (g)         gyroX  (rad/s)
 * 0x516      +2      gyroY  (rad/s)     gyroZ  (rad/s)
 * 0x517      +3      magX   (uT)        magY   (uT)
 * 0x518      +4      magZ   (uT)        eulerX (rad)   [roll]
 * 0x519      +5      eulerY (rad)       eulerZ (rad)   [pitch, yaw]
 * 0x51a      +6      quatW              quatX
 * 0x51b      +7      quatY              quatZ
 */

class ImuReader {
public:
    /// Maximum sequential frames (address space per IMU = 8).
    static constexpr int kMaxFrames = 8;
    /// Two float32 values per CAN frame.
    static constexpr int kMaxFloats = kMaxFrames * 2;
    /// IMU update rate in Hz; used for incremental angle integration.
    static constexpr double kRateHz = 500.0;

    // Named indices into the flat float array.
    enum Idx : int {
        ACC_X = 0,
        ACC_Y = 1,
        ACC_Z = 2,
        GYRO_X = 3,
        GYRO_Y = 4,
        GYRO_Z = 5,
        MAG_X = 6,
        MAG_Y = 7,
        MAG_Z = 8,
        EULER_X = 9,
        EULER_Y = 10,
        EULER_Z = 11, // roll, pitch, yaw
        QUAT_W = 12,
        QUAT_X = 13,
        QUAT_Y = 14,
        QUAT_Z = 15,
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
