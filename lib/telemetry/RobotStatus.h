#pragma once

#include "mqtt/MqttClient.h"
#include "motor/Motor.h"
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// ============================================================================
// Wire-format structs — packed to 1-byte alignment for deterministic layout.
// Total packet size ~890 bytes, well within a single MQTT message.
// ============================================================================

#pragma pack(push, 1)

static constexpr uint32_t kRobotStatusMagic   = 0x4B564156; // "KVAU"
static constexpr uint8_t  kRobotStatusVersion = 1;
static constexpr int      kMotorsPerLeg       = 5;

struct MotorStateWire {
    int32_t  state;
    double   position;
    double   velocity;
    double   torque;
    int32_t  t_mos;
    int32_t  t_rotor;
};
static_assert(sizeof(MotorStateWire) == 36, "unexpected MotorStateWire size");

struct MotorCommandWire {
    double kp;
    double kd;
    double q;
    double dq;
    double tau;
};
static_assert(sizeof(MotorCommandWire) == 40, "unexpected MotorCommandWire size");

struct LegStatusWire {
    MotorStateWire   state[kMotorsPerLeg];
    MotorCommandWire command[kMotorsPerLeg];
};
static_assert(sizeof(LegStatusWire) == 380, "unexpected LegStatusWire size");

struct ImuWire {
    double pitch;
    double roll;
    double yaw;
};
static_assert(sizeof(ImuWire) == 24, "unexpected ImuWire size");

struct DriverCommandWire {
    uint32_t controlWord;       // HAL_ControlWord packed as uint32
    int16_t  axesCount;
    float    axes[12];          // HAL_kMaxJoystickAxes
    int16_t  povsCount;
    int16_t  povs[12];          // HAL_kMaxJoystickPOVs
    uint32_t buttons;
    uint8_t  buttonCount;
};
static_assert(sizeof(DriverCommandWire) == 85, "unexpected DriverCommandWire size");

struct RobotStatusWire {
    uint32_t         magic;
    uint8_t          version;
    int64_t          timestamp_us;   // microseconds since epoch
    int64_t          frameId;
    LegStatusWire    leg[2];         // [0]=left, [1]=right
    DriverCommandWire driver;
    ImuWire          imu;
};
static_assert(sizeof(RobotStatusWire) == 4+1+8+8+760+85+24,
              "unexpected RobotStatusWire size");

#pragma pack(pop)

// ============================================================================
// RobotStatus — collects one snapshot per control period and publishes binary.
// ============================================================================

class RobotStatus {
public:
    explicit RobotStatus(MqttClient &mqtt);

    /// Collect the full robot snapshot from subsystems. Call once per period.
    void collect(const std::vector<std::shared_ptr<Motor>> &leftMotors,
                 const std::vector<std::shared_ptr<Motor>> &rightMotors,
                 const double *imuState7);

    /// Publish the last collected snapshot over MQTT as a binary blob.
    void publish();

private:
    MqttClient       &m_mqtt;
    RobotStatusWire   m_wire{};
    int64_t           m_frameCounter{0};
    std::string       m_topic{"/telemetry/robotstatus"};
};
