#pragma once

#include "ds/DSControlWord.h"
#include "ds/DriverStationTypes.h"
#include "motor/DmFrame.h"
#include "motor/Motor.h"
#include "mqtt/MqttClient.h"
#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * DataLog captures sensor data every control period and publishes it
 * over MQTT in SenML-compliant JSON.
 *
 * MQTT topic layout:
 *   /telemetry/driverstation                   – DSControlWord + joystick data
 *   /telemetry/subsystem/<name>/controller      – MITParam per joint
 *   /telemetry/subsystem/<name>/motor           – StateResult per motor
 *   /telemetry/subsystem/imu                    – IMU state vector
 */
class DataLog {
public:
    /**
     * Construct a DataLog that publishes via the given MqttClient.
     * @param mqtt  Reference to the application-wide MqttClient.
     */
    explicit DataLog(MqttClient &mqtt);

    // ── per-period entry point ──────────────────────────────────────────

    /**
     * Log driver-station control word, joystick axes, POVs and buttons.
     * Publishes to /telemetry/driverstation.
     */
    void logDriverStation();

    /**
     * Log the MIT controller parameters for every joint of one subsystem.
     * Publishes to /telemetry/subsystem/<name>/controller.
     *
     * @param name    Subsystem name ("left" or "right").
     * @param params  One MITParam per joint.
     */
    void logController(const std::string &name,
                       const std::vector<MITParam> &params);

    /**
     * Log motor state feedback for every motor of one subsystem.
     * Publishes to /telemetry/subsystem/<name>/motor.
     *
     * @param name    Subsystem name ("left" or "right").
     * @param motors  Shared pointers to the Motor objects.
     */
    void logMotors(const std::string &name,
                   const std::vector<std::shared_ptr<Motor>> &motors);

    /**
     * Log IMU state vector.
     * Publishes to /telemetry/subsystem/imu.
     *
     * @param state  7-dimensional state estimate.
     */
    void logImu(const Eigen::Vector<double, 7> &state);

private:
    MqttClient &m_mqtt;

    // ── helpers ─────────────────────────────────────────────────────────

    /** Current epoch timestamp in seconds. */
    static int64_t now();

    /** Publish a JSON string on the given topic. */
    void publish(const std::string &topic, const std::string &json);
};
/*  -Refactor to publish the binary data instead of json format to save the bandwidth.
 *
 * Frame definition of robot status
robotStatus {
  timestamp timestamp;
  uint frameId
  subSystem[2];
  driverCommand ;
  imu;
}

LegState
{
   motorState [5];
   controlCommand[5];
}

DriverCommand {
   DSControlWord cw;
   HAL_JoystickAxes axes;
   HAL_JoystickPOVs  pov;
   HAL_JoystickButtons buttons;
};
imu;


  1) motor state info per motor
     struct StateResult {
   int state;
   double position;
   double velocity;
   double torque;
   int t_mos;
   int t_rotor;
  };

  2) controller info
     struct MITParam {
   double kp;
   double kd;
   double q;
   double dq;
   double tau;
  };
  3) driver station info
     DSControlWord
     HAL_JoystickAxes
     HAL_JoystickPOVs
     HAL_JoystickButtons
 * */