#include "Robot.h"
#include "motor/Motor.h"
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
void Robot::robotInit() {
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
        // rightLeg.setShmPointers(m_shm, &m_shm->motor_group_b_stage);
    }

    BooleanEvent startButton{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(1); }};
    BooleanEvent stopButton{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(2); }};
    BooleanEvent rebootButton{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(3); }};
    BooleanEvent updateStateButton{&m_loop, [&joystick = m_joystick] { return joystick.getRawButton(4); }};

    //startButton.ifHigh([this] {  leftLeg.setEnable(true);    rightLeg.setEnable(true); });
    //stopButton.ifHigh([this] {  leftLeg.setEnable(false);    rightLeg.setEnable(false); });
    startButton.rising().ifHigh([this] {  leftLeg.setEnable(true); });
    stopButton.rising().ifHigh([this] {  leftLeg.setEnable(false); });
    //rebootButton.ifHigh([this] {  leftLeg.reboot();    rightLeg.reboot(); });

    updateStateButton.ifHigh([this] {
        TCallback callback = [this](std::string &result) { updateStateCallback(result); };
        leftLeg.updateState(callback);   /* rightLeg.updateState(callback);*/ });
    //https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpilibcExamples/src/main/cpp/examples/EventLoop/cpp/Robot.cpp#L11

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
    } else {
        SPDLOG_WARN("SHM not attached — Composer thread not started");
    }
}
Robot::~Robot() {
    // Shutdown Composer thread before unmapping SHM
    if (m_composer) {
        m_composer->shutdown();
        m_composer.reset();
    }
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
    //TODO:: consider what need to be done here.
    SubsystemBase::runAllAutonomousInit();
}

/**
 * Initialization code for teleop mode should go here.
 */
void Robot::teleopInit() {
    //TODO:: consider what need to be done here.
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
        // Right leg disabled until wired up
        // if (stale & mercury::Composer::STALE_MOTOR_GROUP_B) {
        //     SPDLOG_WARN("Motor Group B stale — disabling right leg");
        //     rightLeg.setEnable(false);
        // }

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
        }
        // Right leg queries (when enabled):
        // else {
        //     size_t right_idx = motor_idx - mercury::MOTORS_PER_GROUP;
        //     auto& leg_motors = rightLeg.getMotors();
        //     if (right_idx < leg_motors.size()) {
        //         leg_motors[right_idx]->getRegParam(21);
        //     }
        // }
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