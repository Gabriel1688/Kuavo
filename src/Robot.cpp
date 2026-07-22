#include "Robot.h"
#include "message.h"
#include "motor/Motor.h"
#include "motor/UdpServer.h"
#include "Test1.hpp"
#include "spdlog/sinks/rotating_file_sink.h"// For size-based rotation
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <cerrno>
#include <cstdio>
#include <csignal>
#include <cstdlib>
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

    // Wire the shared MotorParamCache into both UdpServer instances
    // before any parameter queries are dispatched by robotPeriodic().
    UdpServer::getInstance(0).setParamCache(&m_paramCache);
    UdpServer::getInstance(1).setParamCache(&m_paramCache);

    // Attach to the producer's POSIX shared memory. The producer owns the
    // SHM lifecycle; the consumer refuses to start without a valid region.
    m_shm = tryAttachSharedMemory();
    if (!m_shm) {
        SPDLOG_ERROR("Failed to attach to producer SHM — exiting");
        std::exit(EXIT_FAILURE);
    }
    attachSharedMemory();

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

    // Clean up shared memory mapping
    if (m_shm) {
        munmap(m_shm, sizeof(mercury::SharedMemoryLayout));
        m_shm = nullptr;
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
    uint64_t rp_start = mercury::get_monotonic_ns();

    // D4 Task 2: Button event polling
    m_loop.poll();

    // IPC SHM lifecycle: check producer liveness and reattach if needed
    if (m_shm) {
        uint64_t now_ns = mercury::get_monotonic_ns();
        uint32_t magic = m_shm->magic.load(std::memory_order_acquire);
        auto lifecycle = static_cast<mercury::ShmLifecycle>(
            m_shm->lifecycle_state.load(std::memory_order_acquire));
        uint64_t heartbeat = m_shm->controller_heartbeat_ns.load(std::memory_order_acquire);

        if (magic != mercury::SHM_MAGIC ||
            m_shm->version != mercury::SHM_VERSION ||
            lifecycle != mercury::ShmLifecycle::RUNNING ||
            heartbeat == 0 || heartbeat > now_ns ||
            (now_ns - heartbeat) > mercury::HEARTBEAT_STALE_NS) {
            SPDLOG_ERROR("Producer SHM lost — detaching and disabling subsystems");
            leftLeg.setEnable(false);
            rightLeg.setEnable(false);
            detachSharedMemory();
        }
    }

    if (!m_shm && (m_cycle % 10 == 0)) {
        SPDLOG_INFO("Attempting SHM reattach (cycle={})", m_cycle);
        m_shm = tryAttachSharedMemory();
        if (m_shm) {
            SPDLOG_INFO("Reconnected to producer SHM");
            attachSharedMemory();
        }
    }

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
            if (hb > now_ns || (now_ns - hb) > mercury::HEARTBEAT_STALE_NS) {  // > 100ms stale
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

    uint64_t rp_end = mercury::get_monotonic_ns();
    uint32_t robot_us = static_cast<uint32_t>((rp_end - rp_start) / 1000ULL);
    uint32_t robot_jitter_us = 0;
    if (m_lastRobotStartNs != 0) {
        robot_jitter_us = static_cast<uint32_t>((rp_start - m_lastRobotStartNs) / 1000ULL);
    }
    m_lastRobotStartNs = rp_start;

    if (m_mqtt) {
        char buf[256];
        int n = std::snprintf(buf, sizeof(buf),
            R"([{"bn":"kuavo:robot:","n":"robotPeriodic","t":0,"v":%u,"u":"us"},{"n":"robotJitter","v":%u,"u":"us"},{"n":"cycle","v":%u}])",
            robot_us, robot_jitter_us, static_cast<uint32_t>(m_cycle));
        m_mqtt->publish_binary("robot/timing",
                               reinterpret_cast<const uint8_t*>(buf),
                               static_cast<size_t>(n), 0, false);
    }
    SPDLOG_DEBUG("[timing] robotPeriodic cycle={} duration_us={} jitter_us={}", m_cycle, robot_us, robot_jitter_us);

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

mercury::SharedMemoryLayout* Robot::tryAttachSharedMemory() {
    int fd = shm_open(mercury::SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        SPDLOG_INFO("SHM {} not found: {}", mercury::SHM_NAME, strerror(errno));
        return nullptr;
    }

    struct stat st;
    if (fstat(fd, &st) < 0) {
        SPDLOG_WARN("SHM {} fstat failed: {}", mercury::SHM_NAME, strerror(errno));
        close(fd);
        return nullptr;
    }

    if (st.st_size < static_cast<off_t>(sizeof(mercury::SharedMemoryLayout))) {
        SPDLOG_WARN("SHM {} too small: {} < {}", mercury::SHM_NAME, st.st_size, sizeof(mercury::SharedMemoryLayout));
        close(fd);
        return nullptr;
    }

    void* ptr = mmap(nullptr, sizeof(mercury::SharedMemoryLayout),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) {
        SPDLOG_WARN("SHM {} mmap failed: {}", mercury::SHM_NAME, strerror(errno));
        return nullptr;
    }

    auto* shm = static_cast<mercury::SharedMemoryLayout*>(ptr);

    uint32_t magic = shm->magic.load(std::memory_order_acquire);
    if (magic != mercury::SHM_MAGIC) {
        SPDLOG_INFO("SHM {} invalid magic: 0x{:08X}", mercury::SHM_NAME, magic);
        munmap(shm, sizeof(mercury::SharedMemoryLayout));
        return nullptr;
    }

    if (shm->version != mercury::SHM_VERSION) {
        SPDLOG_WARN("SHM {} version mismatch: expected {}, got {}",
                     mercury::SHM_NAME, mercury::SHM_VERSION, shm->version);
        munmap(shm, sizeof(mercury::SharedMemoryLayout));
        return nullptr;
    }

    auto lifecycle = static_cast<mercury::ShmLifecycle>(
        shm->lifecycle_state.load(std::memory_order_acquire));
    if (lifecycle != mercury::ShmLifecycle::RUNNING) {
        SPDLOG_INFO("SHM {} not RUNNING: {}", mercury::SHM_NAME,
                     static_cast<uint32_t>(lifecycle));
        munmap(shm, sizeof(mercury::SharedMemoryLayout));
        return nullptr;
    }

    uint64_t now = mercury::get_monotonic_ns();
    uint64_t heartbeat = shm->controller_heartbeat_ns.load(std::memory_order_acquire);
    if (heartbeat == 0 || heartbeat > now || (now - heartbeat) > mercury::HEARTBEAT_STALE_NS) {
        SPDLOG_INFO("SHM {} heartbeat stale ({} ms)", mercury::SHM_NAME,
                     heartbeat > now ? 0 : (now - heartbeat) / 1'000'000ULL);
        munmap(shm, sizeof(mercury::SharedMemoryLayout));
        return nullptr;
    }

    SPDLOG_INFO("Attached to SHM {} ({}B) v{}", mercury::SHM_NAME,
                sizeof(mercury::SharedMemoryLayout), shm->version);
    return shm;
}

void Robot::detachSharedMemory() {
    if (!m_shm) return;

    SPDLOG_INFO("Detaching from SHM {} (m_shm={})", mercury::SHM_NAME, reinterpret_cast<uintptr_t>(m_shm));

    // Save pointer for deferred munmap; clear m_shm FIRST so leg threads
    // and robotPeriodic() stop dereferencing it before we unmap.
    auto* shm_to_unmap = m_shm;
    m_shm = nullptr;

    leftLeg.setEnable(false);
    rightLeg.setEnable(false);
    leftLeg.pause();
    rightLeg.pause();

    // Clear subsystem SHM pointers while threads are paused
    leftLeg.setShmPointers(nullptr, nullptr);
    rightLeg.setShmPointers(nullptr, nullptr);
    imu_subsystem.setStagingBuffer(nullptr);

    // Resume leg threads (they will see m_shm==null and return immediately)
    leftLeg.resume();
    rightLeg.resume();

    imu_subsystem.stop();

    // Shutdown Logger before Composer (Logger drains from the ring that
    // Composer writes to)
    if (m_logger) {
        m_logger->shutdown();
        m_logger.reset();
    }
    if (m_composer) {
        m_composer->shutdown();
        m_composer.reset();
    }

    // Now safe to unmap — no threads hold references to shm_to_unmap
    munmap(shm_to_unmap, sizeof(mercury::SharedMemoryLayout));
    SPDLOG_INFO("SHM detached and unmapped");
    m_imu_stale_counter = 0;
}

void Robot::attachSharedMemory() {
    if (!m_shm) return;

    SPDLOG_INFO("Attaching subsystems to SHM {}", mercury::SHM_NAME);

    imu_subsystem.setStagingBuffer(&m_shm->imu_stage);
    imu_subsystem.start();

    // Pause leg threads while we inject new SHM pointers
    leftLeg.pause();
    rightLeg.pause();

    leftLeg.setShmPointers(m_shm, &m_shm->motor_group_a_stage);
    rightLeg.setShmPointers(m_shm, &m_shm->motor_group_b_stage);

    leftLeg.resume();
    rightLeg.resume();

    m_composer = std::make_unique<mercury::Composer>(
        m_shm->imu_stage,
        m_shm->motor_group_a_stage,
        m_shm->motor_group_b_stage,
        m_paramCache,
        *m_shm,
        m_logRing);
    m_composer->start();
    SPDLOG_INFO("Composer thread started");

    MqttClient* globalMqtt = g_mqttClient_ptr.load();
    m_mqtt = globalMqtt;
    if (globalMqtt) {
        m_logger = std::make_unique<mercury::Logger>(m_logRing, *globalMqtt,
                                                     static_cast<uint32_t>(Config::instance().mqtt().robotId));
        m_logger->start();
        SPDLOG_INFO("Logger thread started");
    } else {
        SPDLOG_ERROR("Global MqttClient not available — Logger not started");
    }
}

void setupLogger();

static void handle_sigterm(int) {
    std::exit(0);
}

int main() {
    std::signal(SIGTERM, handle_sigterm);
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