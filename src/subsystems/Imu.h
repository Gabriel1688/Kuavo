#pragma once

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "imu/ImuReader.h"
#include "robot/SubsystemBase.h"
#include "string"
#include <atomic>
#include <vector>

/**
 * The Imu subsystem.
 *
 * Wraps ImuReader (LPMS-IG1 sequential CAN, 32-bit float) and exposes
 * a 7-element state vector:
 *   [eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ]
 *
 * eulerX/Y/Z = roll / pitch / yaw in radians
 * quatW/X/Y/Z = orientation quaternion (normalised)
 */
class Imu : public SubsystemBase {

public:
    Imu();

    ~Imu();

    Imu(const Imu &) = delete;

    Imu &operator=(const Imu &) = delete;

    /**
     * Resets all sensors and controller.
     */
    void reset();

    /**
     * Returns the Imu state estimate.
     * [eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ]
     */
    const Eigen::Vector<double, 7> &getStates() const;

    /** Access the underlying CAN reader (for raw float access). */
    const ImuReader &reader() const { return m_reader; }
    void update(const float *payload);

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    float m_headingOffset = 0.0;

    Eigen::Vector<double, 2> m_u = Eigen::Vector<double, 2>::Zero();

    void init();

    void reboot();

    void setEnable(bool _enable);

    void resting();

    bool isEnabled();

private:
    std::atomic<bool> m_isEnabled{false};
    ImuReader m_reader;
    mutable Eigen::Vector<double, 7> m_state = Eigen::Vector<double, 7>::Zero();
    // Data (protected by m_mutex)
    mutable std::mutex m_mutex;
};
