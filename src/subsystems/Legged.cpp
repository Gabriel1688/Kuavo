#include "Legged.h"
#include "Eigen/Core"
#include "common/Config.h"
#include "ds/DriverStation.h"
#include "motor/Motor.h"
#include "motor/UdpServer.h"
#include "robot/ControlledSubsystemBase.h"
#include "robot/RobotBase.h"
#include "spdlog/sinks/rotating_file_sink.h"// For size-based rotation
#include "spdlog/spdlog.h"
#include <unistd.h>
//TODO:: how to get the contact state from the gamepad?
//TODO:: refactor the base class of subsystem.

Legged::Legged(int baseId) : baseId(baseId) {
    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    reset(Pose2d{0.0, 0.0, 0.0});
    int motorsPerLeg = Config::instance().motor().motorsPerLeg;
    for (int idx = baseId; idx < baseId + motorsPerLeg; idx++) {
        motors.emplace_back(std::make_shared<Motor>(MotorType::DM8009, idx));
    }
    // Enable all motors
    SPDLOG_INFO("Enabling Subsystem {}", baseId == 6 ? "Left" : "Right");
    for (auto motor : motors) {
        motor->enableMotor();
    }
    usleep(200);
}

void Legged::reset(const Pose2d &initialPose) {
}

void Legged::controllerPeriodic() {
    m_controller.getInputs();
    //TODO:: enrich the message set for each command received from the gamepad.
    SPDLOG_TRACE("[{}] Leg controllerPeriodic is called.", baseId == 6 ? "Left" : "Right");
    char data[4] = {1, 2, 3, 4};
    MESSAGE msg = {0};
    msg.sid = COM_DS;
    msg.did = COM_AGENT;
    msg.length = 4;
    msg.type = SMM_OutGoingRequest;
    memcpy(msg.Union.smm_OutGoingRequest.PhoneNumber, data, 4);

    std::shared_ptr<MESSAGE> msgPtr = std::make_shared<MESSAGE>(msg);
    //message(msgPtr, nullptr);
}

void Legged::onMessage(std::shared_ptr<MESSAGE> message, TCallback callback) {
    SPDLOG_TRACE("[{}] leg received message type = [{}]", baseId == 6 ? "Left" : "Right", message->type);
    std::string resposne;
    resposne.append(baseId == 6 ? "Left" : "Right").append(" leg reply to message type =").append(std::to_string(message->sid));
    callback(resposne);
}

void Legged::updateState(TCallback &callback) {
    char data[4] = {1, 2, 3, 4};
    MESSAGE msg = {0};
    msg.sid = COM_DS;
    msg.did = COM_AGENT;
    msg.length = 4;
    msg.type = SMM_OutGoingRequest;
    memcpy(msg.Union.smm_OutGoingRequest.PhoneNumber, data, 4);

    std::shared_ptr<MESSAGE> msgPtr = std::make_shared<MESSAGE>(msg);
    message(msgPtr, callback);
}
static int counter = 0;
void Legged::robotPeriodic() {
    //https://github.com/frc3512/Robot-2023/blob/8f8287bd0887d7570b1931c3e101e5cd7c99061f/src/main/java/frc3512/robot/subsystems/Arm.java#L141
    controllerPeriodic();
    SPDLOG_TRACE("[{}] Leg robotPeriodic is called.", baseId == 6 ? "Left" : "Right");

    counter++;
#if 0
    // Query motor status
    for (auto motor : motors) {
        motor->getMotorStatus();
        SPDLOG_TRACE("Motor: {}, position : {}", motor->getSendId(), motor->getPosition());
    }

    // Query motor id
    for (auto motor : motors) {
        motor->getRegParam(static_cast<int>(RID::MST_ID));
        // Access motors through components
        SPDLOG_TRACE("Motor: {}, MST_ID value : {}", motor->getSendId(),
                     motor->getParam(static_cast<int>(RID::MST_ID)));
    }
#endif
    // Control  motors with position control
    for (auto motor : motors) {
        motor->setMitControl(MITParam{2, 1, 0, 0, 0});
        motor->getMotorStatus();
        if (counter % 10 == 0) {
            SPDLOG_INFO("Motor: {}, position : {}", motor->getSendId(), motor->getPosition());
            if (counter == 10000) {
                counter = 0;
            }
        }
    }
#if 0
        // Control arm motors with torque control
        for (auto motor : motors) {
            motor->setMitControl(MITParam{0, 0, 0, 0, 0.1});
            //usleep(10);
            motor->getMotorStatus();
            SPDLOG_INFO("Motor: {}, position : {}", motor->getSendId(), motor->getPosition());
        }
#endif
}
//TODO:: get reference documents for control command of subsystem via the gamepad?
void Legged::disabledInit() {
    //TODO::consider what to be done here. <documents to be written>
    disable();
}

void Legged::autonomousInit() {
    //SetTurningTolerance(0.25);
    enable();
}

void Legged::teleopInit() {
    setEnable(true);

    //    setJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);

    //    setJointSpeed(DEFAULT_JOINT_SPEED);
    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    // m_controller.AbortTrajectories();

    // If the robot was disabled while still turning in place in
    // autonomous, it will continue to do so in teleop. This aborts any
    // turning action so teleop driving can occur.
    // AbortTurnInPlace();

    //enable();
}

void Legged::teleopPeriodic() {
    /*
    using Input = Controller::Input;

    static frc::Joystick driveStick1{HWConfig::kDriveStick1Port};
    static frc::Joystick driveStick2{HWConfig::kDriveStick2Port};

    double y = frc::ApplyDeadband(-driveStick1.GetY(), Constants::kJoystickDeadband);
    double x = frc::ApplyDeadband(driveStick2.GetX(), Constants::kJoystickDeadband);

    if (driveStick2.getRawButton(2)) {
        x *= 0.4;
    }

    auto [left, right] = frc::DifferentialDrive::CurvatureDriveIK(
            y, x, driveStick2.getRawButton(2));

    // Implicit model following
    // TODO: Velocities need filtering
    Eigen::Vector<double, 2> u =
            m_imf.calculate(Eigen::Vector<double, 2>{GetLeftVelocity().value(),
                                                     GetRightVelocity().value()},
                            Eigen::Vector<double, 2>{left * 12.0, right * 12.0});

    if (!IsVisionAiming()) {
        m_leftGrbx.SetVoltage(units::volt_t{u(Input::kLeftVoltage)});
        m_rightGrbx.SetVoltage(units::volt_t{u(Input::kRightVoltage)});
    }

    m_headingGoalEntry.SetBoolean(AtHeading());
    m_hasHeadingGoalEntry.SetBoolean(HasHeadingGoal());

    m_yawControllerEntry.SetDouble(GetVisionYaw().value());
    m_rangeControllerEntry.SetDouble(m_controller.GetVisionRange().value());
*/
}

Legged::~Legged() {
    for (size_t j = 0; j < motors.size(); j++)
        motors[j].reset();
}

void Legged::reboot() {
    SPDLOG_TRACE("[{}] Leg is rebooted.", baseId == 6 ? "Left" : "Right");
    //usleep(50);
}

void Legged::setEnable(bool _enable) {
    SPDLOG_TRACE("[{}] Leg  is set to [{}].", baseId == 6 ? "Left" : "Right", _enable ? "Enabled" : "Disabled");
    _enable ? enable() : disable();
}

bool Legged::isEnabled() {
    return m_isEnabled;
}
