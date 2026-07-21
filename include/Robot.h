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
#include <assert.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../tools/mercury_shm_v2.h"
#include "composer/Composer.h"
#include "composer/MotorParamCache.h"

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

    Robot() : TimedRobot(10) {}  // 100 Hz main loop

    ~Robot();

    //Async callback function
    void updateStateCallback(std::string result);

private:
    EventLoop m_loop{};
    XboxController m_controller{0};
    GenericHID m_joystick{0};

    mercury::MotorParamCache m_paramCache;
    Legged leftLeg{Config::instance().findLeg("left")->baseId, nullptr, nullptr, &m_paramCache};
    //Legged rightLeg{Config::instance().findLeg("right")->baseId, nullptr, nullptr, &m_paramCache};
    Imu imu_subsystem;

    // Shared memory for health monitoring (lock-free composed buffer)
    mercury::SharedMemoryLayout* m_shm = nullptr;
    int m_shm_fd = -1;

    // Composer thread and supporting infrastructure
    mercury::SPSCRingBuffer<mercury::BatchLogRecord, mercury::BATCH_RING_CAPACITY> m_logRing;
    std::unique_ptr<mercury::Composer> m_composer;

    // Supervisory loop state
    uint64_t m_cycle = 0;
    uint32_t m_imu_stale_counter = 0;

};
