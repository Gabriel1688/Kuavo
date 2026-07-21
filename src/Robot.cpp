#include "Robot.h"
#include "message.h"
#include "motor/Motor.h"
#include "motor/UdpServer.h"
#include "Test1.hpp"
#include "spdlog/sinks/rotating_file_sink.h"// For size-based rotation
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <cerrno>
#include <cstring>
#include <memory>
#include <unistd.h>

using namespace spdlog;
//TODO::
// 2. Added support for the button handling, command handling for the imu/legged.
// 3. add the interface with the kuavo controller.
// 5. Implement the controller interface for each leg;
// 6. Integrate the dynacore to the robot controller.
// 7. measure the performance of the robot controller.
// Reference: https://github.com/frc3512/Robot-2020/blob/b416c202794fb7deea0081beff2f986de7001ed9/docs/system-architecture.md?plain=1#L120
//https://github.com/bridgedp/hunter_bipedal_control/blob/37310dde100e2e8373fc7c2c02e825c358e6fd2e/legged_hw/include/legged_hw/LeggedHW.h#L32
//https://github.com/collin80/GEVCU6/blob/DEV/DeviceManager.h

namespace {
std::shared_ptr<MESSAGE> makeSubsystemMessage(uint8_t msgType) {
    MESSAGE m = {};
    m.sid = COM_DS;
    m.did = COM_AGENT;
    m.type = msgType;
    m.length = 0;
    return std::make_shared<MESSAGE>(m);
}
}  // namespace

void Robot::robotInit() {
    // Load MQTT config and start the libwebsockets client before anything else.
    m_mqttClient.loadConfig("");
    m_mqttClient.start();

    // Wire the shared MotorParamCache into both UdpServer instances
    // before any parameter queries are dispatched by robotPeriodic().
    UdpServer::getInstance(0).setParamCache(&m_paramCache);
    UdpServer::getInstance(1).setParamCache(&m_paramCache);

    // Attach to POSIX shared memory for health monitoring
    m_shm_fd = shm_open(mercury::SHM_NAME, O_RDWR, 0666);
    if (m_shm_fd < 0) {
        // If SHM doesn't exist yet, create it (first process to start)
        m_shm_fd = shm_open(mercury::SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (m_shm_fd < 0) {
            SPDLOG_ERROR("Failed to open/create SHM {}: {}", mercury::SHM_NAME, strerror(errno));
        } else {
            if (ftruncate(m_shm_fd, sizeof(mercury::SharedMemoryLayout)) != 0) {
                SPDLOG_ERROR("Failed to size SHM: {}", strerror(errno));
            }
        }
    }
    if (m_shm_fd >= 0) {
        void* ptr = mmap(nullptr, sizeof(mercury::SharedMemoryLayout),
                         PROT_READ | PROT_WRITE, MAP_SHARED, m_shm_fd, 0);
        if (ptr == MAP_FAILED) {
            SPDLOG_ERROR("Failed to mmap SHM: {}", strerror(errno));
            close(m_shm_fd);
            m_shm_fd = -1;
        } else {
            m_shm = static_cast<mercury::SharedMemoryLayout*>(ptr);
            SPDLOG_INFO("Attached to SHM {} ({}B)", mercury::SHM_NAME, sizeof(mercury::SharedMemoryLayout));
        }
    }

    // Pass SHM and staging buffer pointers to leg subsystems
    if (m_shm) {
        leftLeg.setShmPointers(m_shm, &m_shm->motor_group_a_stage);
        rightLeg.setShmPointers(m_shm, &m_shm->motor_group_b_stage);
    }

    // Wire the IMU staging buffer before starting the reader thread
    if (m_shm) {
        imu_subsystem.setStagingBuffer(&m_shm->imu_stage);
    }
    imu_subsystem.start();

    // Helper no-op callback for async subsystem messages
    TCallback noop = [](std::string &) {};

    const auto &dsButtons = Config::instance().driverStation().buttons;
    SPDLOG_INFO("DriverStation button mapping loaded:");
    for (const auto &kv : dsButtons) {
        SPDLOG_INFO("  button {} -> {}", kv.first, kv.second);
    }

    // Resolve button numbers from config (default to design mapping if missing)
    auto buttonNumber = [&dsButtons](const std::string &action) -> int {
        for (const auto &kv : dsButtons) {
            if (kv.second == action) {
                try {
                    return std::stoi(kv.first);
                } catch (...) {
                    return -1;
                }
            }
        }
        return -1;
    };

    int btnEnableLeft = buttonNumber("enable_left_leg");
    int btnDisableLeft = buttonNumber("disable_left_leg");
    int btnEnableRight = buttonNumber("enable_right_leg");
    int btnDisableRight = buttonNumber("disable_right_leg");
    if (btnEnableLeft <= 0) btnEnableLeft = 1;
    if (btnDisableLeft <= 0) btnDisableLeft = 2;
    if (btnEnableRight <= 0) btnEnableRight = 3;
    if (btnDisableRight <= 0) btnDisableRight = 4;

    // Per-button BooleanEvent objects for the four main face buttons
    auto rawButton = [&](int number) -> std::function<bool()> {
        return [&joystick = m_joystick, number] { return joystick.getRawButton(number); };
    };

    BooleanEvent enableLeftButton{&m_loop, rawButton(btnEnableLeft)};
    BooleanEvent disableLeftButton{&m_loop, rawButton(btnDisableLeft)};
    BooleanEvent enableRightButton{&m_loop, rawButton(btnEnableRight)};
    BooleanEvent disableRightButton{&m_loop, rawButton(btnDisableRight)};

    enableLeftButton.rising().ifHigh([this, noop] { leftLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), noop); });
    disableLeftButton.rising().ifHigh([this, noop] { leftLeg.message(makeSubsystemMessage(MSG_DISABLE_SUBSYSTEM), noop); });
    enableRightButton.rising().ifHigh([this, noop] { rightLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), noop); });
    disableRightButton.rising().ifHigh([this, noop] { rightLeg.message(makeSubsystemMessage(MSG_DISABLE_SUBSYSTEM), noop); });

    // LB (5) + RB (6) -> enable both legs
    BooleanEvent lb{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(5); }};
    BooleanEvent rb{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(6); }};
    auto enableAll = lb && [&rb] { return rb.getAsBoolean(); };
    enableAll.rising().ifHigh([this, noop] {
        leftLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), noop);
        rightLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), noop);
    });

    // Back (7) + Start (8) -> emergency stop: set SHM flag and disable both legs
    BooleanEvent back{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(7); }};
    BooleanEvent start{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(8); }};
    auto emergency = back && [&start] { return start.getAsBoolean(); };
    emergency.rising().ifHigh([this, noop] {
        if (m_shm) {
            m_shm->emergency_stop.store(true, std::memory_order_release);
        }
        leftLeg.message(makeSubsystemMessage(MSG_EMERGENCY_STOP), noop);
        rightLeg.message(makeSubsystemMessage(MSG_EMERGENCY_STOP), noop);
        SPDLOG_ERROR("EMERGENCY STOP activated by operator");
    });

    // Start Composer thread (reads staging buffers, writes composed SHM + SPSC ring)
    if (m_shm) {
        m_composer = std::make_unique<mercury::Composer>(
            m_shm->imu_stage,
            m_shm->motor_group_a_stage,
            m_shm->motor_group_b_stage,
            m_paramCache,
            *m_shm,
            m_logRing);
        m_composer->start();
        SPDLOG_INFO("Composer thread started");

        // Start Logger drain thread after Composer
        m_logger = std::make_unique<mercury::Logger>(m_logRing, m_mqttClient,
                                                   static_cast<uint32_t>(Config::instance().mqtt().robotId));
        m_logger->start();
        SPDLOG_INFO("Logger thread started");
    } else {
        SPDLOG_WARN("SHM not attached — Composer and Logger threads not started");
    }
}
Robot::~Robot() {
    // Shutdown Logger thread before Composer to stop publishing
    if (m_logger) {
        m_logger->shutdown();
        m_logger.reset();
    }

    // Shutdown Composer thread before unmapping SHM
    if (m_composer) {
        m_composer->shutdown();
        m_composer.reset();
    }

    // Stop the MQTT client
    m_mqttClient.shutdown();

    // Clean up shared memory mapping
    if (m_shm) {
        munmap(m_shm, sizeof(mercury::SharedMemoryLayout));
        m_shm = nullptr;
    }
    if (m_shm_fd >= 0) {
        close(m_shm_fd);
        m_shm_fd = -1;
    }
}

void Robot::autonomousInit() {
    // D4: Enable both legs on mode entry
    leftLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), [](std::string &) {});
    rightLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), [](std::string &) {});
    SubsystemBase::runAllAutonomousInit();
}

/**
 * Initialization code for teleop mode should go here.
 */
void Robot::teleopInit() {
    // D4: Enable both legs on mode entry
    leftLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), [](std::string &) {});
    rightLeg.message(makeSubsystemMessage(MSG_ENABLE_SUBSYSTEM), [](std::string &) {});
    SubsystemBase::runAllTeleopInit();
}

/**
 * Periodic code for all modes should go here.
 */
void Robot::robotPeriodic() {
    // D4 Task 2: Button event polling
    m_loop.poll();

    // D4 Tasks 4-5: Health monitoring + safety validation via composed SHM buffer
    if (m_composer && m_shm) {
        // D5: Check per-source staleness via Composer bitmask
        uint8_t stale = m_composer->check_staleness();

        // D5: IMU two-tier staleness
        if (stale & mercury::Composer::STALE_IMU) {
            if (m_imu_stale_counter == 0) {
                SPDLOG_WARN("IMU data stale (>{}ms)", mercury::Composer::IMU_STALE_TIMEOUT_MS);
            }
            m_imu_stale_counter++;
            if (m_imu_stale_counter > 20) {  // 200ms at 100Hz
                SPDLOG_ERROR("IMU critically stale (>200ms) — emergency stop");
                m_shm->emergency_stop.store(true, std::memory_order_release);
            }
        } else {
            m_imu_stale_counter = 0;
        }

        // D5: Motor group staleness -> disable affected leg
        if (stale & mercury::Composer::STALE_MOTOR_GROUP_A) {
            SPDLOG_WARN("Motor Group A stale — disabling left leg");
            leftLeg.setEnable(false);
        }
        if (stale & mercury::Composer::STALE_MOTOR_GROUP_B) {
            SPDLOG_WARN("Motor Group B stale — disabling right leg");
            rightLeg.setEnable(false);
        }

        // D5: Mercury Controller heartbeat check
        uint64_t hb = m_shm->controller_heartbeat_ns.load(std::memory_order_acquire);
        if (hb > 0) {
            uint64_t now_ns = mercury::get_monotonic_ns();
            if (now_ns - hb > 100'000'000ULL) {  // > 100ms stale
                SPDLOG_ERROR("Mercury Controller heartbeat stale (>100ms) — emergency stop");
                m_shm->emergency_stop.store(true, std::memory_order_release);
            }
        }
    }

    // D6: Parameter query round-robin at 10Hz (every 10th cycle)
    if (m_cycle % 10 == 0) {
        size_t motor_idx = (m_cycle / 10) % mercury::NUM_ACT_JOINT;
        // Route to appropriate leg based on motor index
        if (motor_idx < mercury::MOTORS_PER_GROUP) {
            auto& leg_motors = leftLeg.getMotors();
            if (motor_idx < leg_motors.size()) {
                leg_motors[motor_idx]->getRegParam(21);  // Query PMAX register
            }
        } else {
            size_t right_idx = motor_idx - mercury::MOTORS_PER_GROUP;
            auto& leg_motors = rightLeg.getMotors();
            if (right_idx < leg_motors.size()) {
                leg_motors[right_idx]->getRegParam(21);
            }
        }
    }

    // D4 Task 7: Subsystem periodic dispatch (lightweight)
    SubsystemBase::runAllRobotPeriodic();

    m_cycle++;

    //https://github.com/frc3512/Robot-2020/blob/b416c202794fb7deea0081beff2f986de7001ed9/src/main/cpp/Robot.cpp#L126
}

void Robot::autonomousPeriodic() {
    SubsystemBase::runAllAutonomousPeriodic();
    driveWithJoystick(false);
}

void Robot::teleopPeriodic() {
    driveWithJoystick(true);
    // TODO:: Test behavior of mode switch<test->autonomous->teleop->autonomous->test>.
}

void Robot::driveWithJoystick(__attribute__((unused)) bool fieldRelative) {
    // Joystick axis values (for future drive commands)
    // const auto xSpeed = m_controller.getLeftY();
    // const auto ySpeed = m_controller.getLeftX();
    // const auto rot = m_controller.getRightX();
    //TODO:: consider what need to be done here.
}

void Robot::updateStateCallback(std::string result) {
    SPDLOG_TRACE("Async command response :[{}].", result);
    //TODO:: consider what need to be done here.
}
void setupLogger();
int main() {
    setupLogger();
    //https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpilibcExamples/src/main/cpp/examples/HAL/c/Robot.c#L52
    return StartRobot<Robot>();
}

void setupLogger() {
    const auto &cfg = Config::instance().logger();
    std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v";
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(cfg.path, cfg.maxSize, cfg.rotation);
    auto logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list{console_sink, file_sink});
    if (cfg.level == "debug") {
        console_sink->set_level(spdlog::level::debug);
        file_sink->set_level(spdlog::level::debug);
        logger->set_level(spdlog::level::debug);
        logger->flush_on(spdlog::level::debug);
    }
    console_sink->set_pattern(pattern);
    file_sink->set_pattern(pattern);
    spdlog::set_default_logger(logger);
}