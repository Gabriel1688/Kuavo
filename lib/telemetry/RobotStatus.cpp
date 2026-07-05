#include "telemetry/RobotStatus.h"
#include "ds/DriverStation.h"
#include "spdlog/spdlog.h"
#include <cmath>
#include <cstring>

// ── construction ────────────────────────────────────────────────────────────

RobotStatus::RobotStatus(MqttClient &mqtt) : m_mqtt(mqtt) {
    std::memset(&m_wire, 0, sizeof(m_wire));
    m_wire.magic   = kRobotStatusMagic;
    m_wire.version = kRobotStatusVersion;
}

// ── helpers ─────────────────────────────────────────────────────────────────

static void fillLeg(LegStatusWire &leg,
                    const std::vector<std::shared_ptr<Motor>> &motors) {
    const int n = std::min(static_cast<int>(motors.size()), kMotorsPerLeg);
    for (int i = 0; i < n; ++i) {
        const auto &m = motors[i];

        auto &s   = leg.state[i];
        s.state    = m->getState();
        s.position = m->getPosition();
        s.velocity = m->getVelocity();
        s.torque   = m->getTorque();
        s.t_mos    = m->getStateTmos();
        s.t_rotor  = m->getStateTrotor();

        MITParam cmd = m->getLastMitParam();
        auto &c  = leg.command[i];
        c.kp  = cmd.kp;
        c.kd  = cmd.kd;
        c.q   = cmd.q;
        c.dq  = cmd.dq;
        c.tau = cmd.tau;
    }
}

static void fillDriver(DriverCommandWire &drv) {
    HAL_ControlWord cw;
    HAL_GetControlWord(&cw);
    std::memcpy(&drv.controlWord, &cw, sizeof(uint32_t));

    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(0, &axes);
    drv.axesCount = axes.count;
    std::memcpy(drv.axes, axes.axes, sizeof(drv.axes));

    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(0, &povs);
    drv.povsCount = povs.count;
    std::memcpy(drv.povs, povs.povs, sizeof(drv.povs));

    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(0, &buttons);
    drv.buttons     = buttons.buttons;
    drv.buttonCount = buttons.count;
}

/// Convert a 7-element state vector [x, y, z, qw, qx, qy, qz] to Euler
/// angles (pitch, roll, yaw) in radians.
static void quaternionToEuler(const double *s, ImuWire &imu) {
    // Indices: 3=qw, 4=qx, 5=qy, 6=qz
    double qw = s[3], qx = s[4], qy = s[5], qz = s[6];

    // roll  (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    imu.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        imu.pitch = std::copysign(M_PI / 2.0, sinp);   // clamp to +/-90
    else
        imu.pitch = std::asin(sinp);

    // yaw   (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    imu.yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ── public API ──────────────────────────────────────────────────────────────

void RobotStatus::collect(
    const std::vector<std::shared_ptr<Motor>> &leftMotors,
    const std::vector<std::shared_ptr<Motor>> &rightMotors,
    const double *imuState7) {

    auto now = std::chrono::system_clock::now();
    m_wire.timestamp_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch())
            .count();
    m_wire.frameId = m_frameCounter++;

    fillLeg(m_wire.leg[0], leftMotors);
    fillLeg(m_wire.leg[1], rightMotors);
    fillDriver(m_wire.driver);
    quaternionToEuler(imuState7, m_wire.imu);
}

void RobotStatus::publish() {
    if (!m_mqtt.isConnected()) {
        return;
    }
    // Wrap the raw bytes in a std::string — std::string can hold binary data.
    std::string payload(reinterpret_cast<const char *>(&m_wire), sizeof(m_wire));
    m_mqtt.publish(m_topic, payload);
}
