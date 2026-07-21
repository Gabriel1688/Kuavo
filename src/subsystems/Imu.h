#pragma once

#include "imu/ImuReader.h"
#include "robot/SubsystemBase.h"
#include <atomic>

/**
 * The Imu subsystem.
 *
 * Thin lifecycle wrapper around ImuReader. The ImuReader receives
 * CAN-over-UDP frames from an LPMS-IG1 IMU and publishes lock-free
 * snapshots to a SourceDoubleBuffer<ImuStageData>.
 */
class Imu : public SubsystemBase {

public:
    Imu();

    ~Imu() override;

    Imu(const Imu &) = delete;

    Imu &operator=(const Imu &) = delete;

    /**
     * Resets all sensors and controller.
     */
    void reset();

    /** Inject the lock-free IMU staging buffer before start(). */
    void setStagingBuffer(mercury::SourceDoubleBuffer<mercury::ImuStageData> *ptr) {
        m_reader.setStagingBuffer(ptr);
    }

    /** Start the underlying CAN reader thread. */
    void start() { m_reader.start(); }

private:
    std::atomic<bool> m_isEnabled{false};
    ImuReader m_reader;
};
