#include "Robot.h"
#include "Test1.hpp"
#include "spdlog/sinks/rotating_file_sink.h"// For size-based rotation
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <memory>

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
}
Robot::~Robot() {
    // Clean up at program exit
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
    m_loop.poll();
    //m_dataLog.logDriverStation();
    //m_dataLog.logMotors(leftLeg.getName(), leftLeg.getMotors());
    //m_dataLog.logMotors(rightLeg.getName(), rightLeg.getMotors());
    //m_dataLog.logImu(imu_subsystem.getStates());

    m_robotStatus.collect(leftLeg.getMotors(),leftLeg.getMotors(), /*rightLeg.getMotors(),*/
                          imu_subsystem.getStates().data());
    m_robotStatus.publish();
    //https://github.com/frc3512/Robot-2020/blob/b416c202794fb7deea0081beff2f986de7001ed9/src/main/cpp/Robot.cpp#L126
}

void Robot::autonomousPeriodic() {
    SubsystemBase::runAllAutonomousPeriodic();
    driveWithJoystick(false);
}

void Robot::teleopPeriodic() {
    SubsystemBase::runAllRobotPeriodic();
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