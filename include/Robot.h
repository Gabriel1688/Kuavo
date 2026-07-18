#pragma once

#include "../src/subsystems/Imu.h"
#include "../src/subsystems/Legged.h"
#include "Constants.h"
#include "common/Config.h"
#include "ds/BooleanEvent.h"
#include "ds/EventLoop.h"
#include "ds/GenericHID.h"
#include "ds/XboxController.h"
#include "robot/TimedRobot.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"
#include "telemetry/DataLog.h"
#include "telemetry/RobotStatus.h"
#include <assert.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <string.h>

using namespace spdlog;

class Robot : public TimedRobot {
public:
    void robotInit() override;

    void driveWithJoystick(bool fieldRelative);

    /**
     * Periodic code for all modes should go here.
     */
    void robotPeriodic() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
    void autonomousInit() override;

    /**
     * Initialization code for teleop mode should go here.
     */
    void teleopInit() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void autonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
    void teleopPeriodic() override;

    Robot() = default;

    ~Robot();

    //Async callback function
    void updateStateCallback(std::string result);

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};

    Legged leftLeg{Config::instance().findLeg("left")->baseId};
    //Legged rightLeg{Config::instance().findLeg("right")->baseId};
    Imu imu_subsystem;

    //DataLog m_dataLog{*g_mqttClient_ptr.load()};
    RobotStatus m_robotStatus{*g_mqttClient_ptr.load()};
};
