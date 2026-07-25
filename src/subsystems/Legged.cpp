#include "Legged.h"
#include "../../include/mercury_shm.h"
#include "Eigen/Core"
#include "common/Config.h"
#include "ds/DriverStation.h"
#include "motor/Motor.h"
#include "motor/UdpServer.h"
#include "robot/ControlledSubsystemBase.h"
#include "robot/RobotBase.h"
#include "spdlog/sinks/rotating_file_sink.h"// For size-based rotation
#include "spdlog/spdlog.h"
#include <cstdio>
#include <unistd.h>
//TODO:: how to get the contact state from the gamepad?
//TODO:: refactor the base class of subsystem.

static constexpr uint64_t COMMAND_STALE_THRESHOLD_NS = mercury::HEARTBEAT_STALE_NS;
static constexpr uint64_t HEARTBEAT_TIMEOUT_NS = mercury::HEARTBEAT_STALE_NS;

Legged::Legged(int baseId,
               mercury::SharedMemoryLayout* shm,
               mercury::SourceDoubleBuffer<mercury::MotorGroupStageData>* staging,
               mercury::MotorParamCache* paramCache)
    : baseId(baseId), m_shm(shm), m_staging(staging) {
    // Compute joint index offset: left leg = 0, right leg = MOTORS_PER_GROUP
    m_groupOffset = (baseId == 1) ? 0 : mercury::MOTORS_PER_GROUP;

    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    reset(Pose2d{0.0, 0.0, 0.0});
    int motorsPerLeg = Config::instance().motor().motorsPerLeg;
    for (int idx = baseId; idx < baseId + motorsPerLeg; idx++) {
        motors.emplace_back(std::make_shared<Motor>(MotorType::DM8009, idx, paramCache));
    }
    m_motorResponsive.resize(motors.size(), false);
    // Enable all motors
    SPDLOG_INFO("Enabling Subsystem {}", baseId == 1 ? "Left" : "Right");
    for (auto motor : motors) {
        motor->enableMotor();
    }
    usleep(200);
    // Signal that the derived class is fully constructed and
    // controllerPeriodic() is safe to call from the RT thread.
    markReady();
    
    // Set thread name now that derived class is fully constructed
    setThreadName();
}

void Legged::reset(const Pose2d &initialPose) {
}

void Legged::controllerPeriodic() {
    uint64_t cp_start = mercury::get_monotonic_ns();
    uint64_t now_ns = cp_start;

    // Snapshot the SHM pointer into a local so it cannot change mid-function.
    // The atomic load ensures we see the nullptr written by setShmPointers()
    // even if the munmap has already reclaimed the underlying pages.
    auto* shm = m_shm.load(std::memory_order_acquire);

    // SHM lifecycle validation (every 2.5ms / 400Hz)
    if (!shm) {
        SPDLOG_TRACE("[{}] controllerPeriodic: SHM not attached, skipping.", getName());
        return;
    }

    uint32_t magic = shm->magic.load(std::memory_order_acquire);
    if (magic != mercury::SHM_MAGIC) {
        SPDLOG_TRACE("[{}] controllerPeriodic: invalid SHM magic, skipping.", getName());
        disableAllMotorsOnce(SHM_INVALID_MAGIC);
        return;
    }

    if (shm->version != mercury::SHM_VERSION) {
        SPDLOG_WARN("[{}] controllerPeriodic: SHM version mismatch (expected {}, got {}), disabling motors.",
                    getName(), mercury::SHM_VERSION, shm->version);
        disableAllMotorsOnce(SHM_VERSION_MISMATCH);
        return;
    }

    auto lifecycle = static_cast<mercury::ShmLifecycle>(
        shm->lifecycle_state.load(std::memory_order_acquire));
    if (lifecycle != mercury::ShmLifecycle::RUNNING) {
        SPDLOG_TRACE("[{}] controllerPeriodic: SHM lifecycle not RUNNING, skipping.", getName());
        disableAllMotorsOnce(SHM_LIFECYCLE_NOT_RUNNING);
        return;
    }

    uint64_t heartbeat = shm->controller_heartbeat_ns.load(std::memory_order_acquire);
    uint64_t hb_age_ns = (heartbeat != 0 && heartbeat <= now_ns)
                        ? (now_ns - heartbeat) : 0;
    if (heartbeat == 0 || hb_age_ns > HEARTBEAT_TIMEOUT_NS) {
        SPDLOG_WARN("[{}] controllerPeriodic: producer heartbeat stale ({}ms), disabling motors.",
                    getName(), hb_age_ns / 1'000'000ULL);
        disableAllMotorsOnce(HEARTBEAT_STALE);
        return;
    }

    // TODO: Re-enable emergency_stop motor disable once the producer/operator
    // emergency-stop semantics are finalized.
#if 0
    if (shm->emergency_stop.load(std::memory_order_acquire)) {
        SPDLOG_TRACE("[{}] controllerPeriodic: emergency_stop active, disabling motors.", getName());
        disableAllMotorsOnce(EMERGENCY_STOP_ACTIVE);
        return;
    }
#endif

    // Gate actuation on isEnabled().  Observation (motor feedback staging
    // below) always runs so the Composer can track liveness and clear
    // staleness even while the leg is disabled.
    if (isEnabled()) {
        // Read Mercury_Command from SHM double buffer (lock-free)
        uint32_t cmd_idx = shm->cmd_write_idx.load(std::memory_order_acquire);
        if (cmd_idx > 1) {
            SPDLOG_WARN("[{}] controllerPeriodic: invalid cmd_write_idx ({}), disabling motors.",
                        getName(), cmd_idx);
            disableAllMotorsOnce(CMD_WRITE_IDX_INVALID);
        } else {
            const mercury::Command& cmd = shm->cmd_buffers[cmd_idx];

            // Check command freshness (guard unsigned underflow when
            // cmd.timestamp_ns is slightly ahead of the captured now_ns)
            uint64_t cmd_age_ns = (cmd.timestamp_ns > 0 && cmd.timestamp_ns <= now_ns)
                                ? (now_ns - cmd.timestamp_ns) : 0;
            if (cmd.timestamp_ns > 0 && cmd_age_ns > COMMAND_STALE_THRESHOLD_NS) {
                SPDLOG_WARN("[{}] controllerPeriodic: command stale ({}ms), skipping MIT dispatch.",
                            getName(), cmd_age_ns / 1'000'000ULL);
            } else {
                // Dispatch MIT commands to motors
                size_t dispatched = 0;
                for (size_t i = 0; i < motors.size(); i++) {
                    int j = m_groupOffset + static_cast<int>(i);

                    // Bounds check: j must be within NUM_ACT_JOINT
                    if (j < 0 || j >= mercury::NUM_ACT_JOINT) {
                        SPDLOG_ERROR("[{}] controllerPeriodic: joint index {} out of range [0,{}), skipping.",
                                     getName(), j, mercury::NUM_ACT_JOINT);
                        continue;
                    }

                    // 4.6: Check enabled field — skip MIT dispatch if motor not enabled
                    // Controllers write 1 (boolean true) for enabled, 0 for disabled.
                    if (!cmd.enabled[j]) {
                        continue;
                    }

                    // 4.5: Extract per-joint command
                    // 4.7: Construct MITParam and call setMitControl
                    MITParam mit;
                    mit.kp  = cmd.kp[j];
                    mit.kd  = cmd.kd[j];
                    mit.q   = cmd.jpos_cmd[j];
                    mit.dq  = cmd.jvel_cmd[j];
                    mit.tau = cmd.jtorque_cmd[j];

                    motors[i]->setMitControl(mit);
                    dispatched++;
                }
            }
        }
    }

    // 4.8: Aggregate motor feedback into MotorGroupStageData
    // 6.3: Check motor responsiveness (100ms timeout)
    auto* staging = m_staging.load(std::memory_order_acquire);
    if (staging) {
        mercury::MotorGroupStageData stageData{};
        static constexpr uint64_t MOTOR_RESPONSIVE_TIMEOUT_NS = 100'000'000ULL;

        for (size_t i = 0; i < motors.size(); i++) {
            stageData.joint_jpos[i]  = motors[i]->getPosition();
            stageData.joint_jvel[i]  = motors[i]->getVelocity();
            stageData.motor_jpos[i]  = motors[i]->getPosition();
            stageData.motor_jvel[i]  = motors[i]->getVelocity();
            stageData.jtorque[i]     = motors[i]->getTorque();
            stageData.mos_temperature[i]   = motors[i]->getStateTmos();
            stageData.rotor_temperature[i] = motors[i]->getStateTrotor();
            stageData.motor_status[i] = static_cast<uint8_t>(motors[i]->getState());

            uint64_t lastUpdate = motors[i]->getLastUpdateTime();
            // Guard against unsigned underflow: if lastUpdate > now_ns the
            // motor callback fired after we sampled cp_start — treat as
            // responsive (age = 0).
            uint64_t age_ns = (lastUpdate != 0 && lastUpdate <= now_ns)
                            ? (now_ns - lastUpdate) : 0;
            bool responsive = (lastUpdate != 0) && (age_ns < MOTOR_RESPONSIVE_TIMEOUT_NS);
            m_motorResponsive[i] = responsive;
        }

        // Embed controller timing into staging data (lock-free, no MQTT)
        uint64_t cp_end = mercury::get_monotonic_ns();
        stageData.controller_duration_us = static_cast<uint32_t>((cp_end - cp_start) / 1000ULL);
        stageData.controller_interval_us = (m_lastControllerStartNs != 0)
            ? static_cast<uint32_t>((cp_start - m_lastControllerStartNs) / 1000ULL) : 0;
        m_lastControllerStartNs = cp_start;

        // 4.9: Publish to staging buffer with current timestamp
        stageData.timestamp_ns = now_ns;
        stageData.sequence = shm->composed_sequence.load(std::memory_order_relaxed) + 1;
        staging->publish(stageData);
    }
    SPDLOG_DEBUG("[timing] {} controllerPeriodic duration_us={} interval_us={}",
                 getName(),
                 m_lastControllerStartNs != 0
                     ? static_cast<uint32_t>((mercury::get_monotonic_ns() - cp_start) / 1000ULL) : 0,
                 0);
}

void Legged::onMessage(std::shared_ptr<MESSAGE> message, TCallback callback) {
    SPDLOG_TRACE("[{}] leg received message type = [{}]", baseId == 1 ? "Left" : "Right", message->type);
    std::string response;
    switch (message->type) {
    case MSG_ENABLE_SUBSYSTEM:
        setEnable(true);
        response = getName() + " leg enabled by subsystem message";
        break;
    case MSG_DISABLE_SUBSYSTEM:
        setEnable(false);
        response = getName() + " leg disabled by subsystem message";
        break;
    case MSG_EMERGENCY_STOP:
        SPDLOG_INFO("[{}] Received MSG_EMERGENCY_STOP", getName());
        setEnable(false);
        response = getName() + " leg emergency stopped";
        break;
    default:
        response = getName() + " leg reply to message type =" + std::to_string(message->sid);
        break;
    }

    callback(response);
}

void Legged::updateState(TCallback &callback) {
    char data[4] = {1, 2, 3, 4};
    MESSAGE msg = {};
    msg.sid = COM_DS;
    msg.did = COM_AGENT;
    msg.length = 4;
    msg.type = SMM_OutGoingRequest;
    memcpy(msg.Union.smm_OutGoingRequest.PhoneNumber, data, 4);

    std::shared_ptr<MESSAGE> msgPtr = std::make_shared<MESSAGE>(msg);
    message(msgPtr, callback);
}

void Legged::robotPeriodic() {
    // 7.1: Lightweight supervisory-only: no motor I/O, no MIT commands.
    // Motor control (setMitControl, getMotorStatus) moved to controllerPeriodic() at 400Hz.
    SPDLOG_TRACE("[{}] Leg robotPeriodic is called.", baseId == 1 ? "Left" : "Right");

    // 7.2-7.3: Parameter query round-robin at 10Hz (every 10th call)
    if (m_paramQueryCycle % 10 == 0) {
        size_t motor_idx = (m_paramQueryCycle / 10) % motors.size();
        if (motor_idx < motors.size()) {
            // Query bus voltage (RID for voltage varies by motor firmware; using a common one)
            motors[motor_idx]->getRegParam(21);  // RID 21 = PMAX (example; adjust per Damiao spec)
        }
    }
    m_paramQueryCycle++;
}
//TODO:: get reference documents for control command of subsystem via the gamepad?
void Legged::disabledInit() {
    //TODO::consider what to be done here. <documents to be written>
    disable();
}

void Legged::autonomousInit() {
    setEnable(true);
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

uint32_t Legged::getMotorStatusBits() const {
    uint32_t bits = 0;
    for (size_t i = 0; i < m_motorResponsive.size(); i++) {
        if (m_motorResponsive[i])
            bits |= (1u << i);
    }
    return bits;
}

Legged::~Legged() {
    // Stop the RT thread while Legged's vtable is still installed,
    // preventing a "pure virtual method called" crash.
    stopThread();

    // 9.2: Reset all motors on shutdown (setZeroCommand + disableMotor)
    for (auto& motor : motors) {
        motor->setZeroCommand();
        motor->disableMotor();
    }
}

void Legged::reboot() {
    SPDLOG_INFO("[{}] Leg is rebooted.", baseId == 1 ? "Left" : "Right");
    //usleep(50);
}

void Legged::setEnable(bool _enable) {
    if (_enable && !isEnabled()) {
        m_motorsFaultDisabled.store(false, std::memory_order_release);
        // Reset responsiveness tracking so the first controllerPeriodic()
        // after enable does not produce spurious "unresponsive" warnings
        // before enable-feedback has arrived.
        std::fill(m_motorResponsive.begin(), m_motorResponsive.end(), false);
        std::for_each(motors.begin(),motors.end(), [](const auto &motor) {
            motor->enableMotor();
        } );
        enable();
        SPDLOG_INFO("[{}] Leg  is Enabled.", baseId == 1 ? "Left" : "Right");
    }
    else if (!_enable && isEnabled()) {
        std::for_each(motors.begin(),motors.end(), [](const auto &motor) {
            motor->disableMotor();
        });
        disable();
        SPDLOG_INFO("[{}] Leg  is Disabled.", baseId == 1 ? "Left" : "Right");
    }
}

static const char* disableReasonString(int reason) {
    switch (reason) {
        case Legged::SHM_INVALID_MAGIC:
            return "invalid SHM magic";
        case Legged::SHM_VERSION_MISMATCH:
            return "SHM version mismatch";
        case Legged::SHM_LIFECYCLE_NOT_RUNNING:
            return "SHM lifecycle not RUNNING";
        case Legged::HEARTBEAT_STALE:
            return "producer heartbeat stale";
        case Legged::EMERGENCY_STOP_ACTIVE:
            return "emergency_stop active";
        case Legged::CMD_WRITE_IDX_INVALID:
            return "invalid cmd_write_idx";
        default:
            return "unknown";
    }
}

void Legged::disableAllMotorsOnce(int reason) {
    bool expected = false;
    if (m_motorsFaultDisabled.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        for (auto& motor : motors) {
            motor->disableMotor();
        }
        SPDLOG_WARN("[{}] All motors disabled (fault shutdown): reason={} ({}).",
                    getName(), reason, disableReasonString(reason));
    }
}

void Legged::setShmPointers(mercury::SharedMemoryLayout* shm,
                            mercury::SourceDoubleBuffer<mercury::MotorGroupStageData>* staging) {
    m_shm.store(shm, std::memory_order_release);
    m_staging.store(staging, std::memory_order_release);
}
