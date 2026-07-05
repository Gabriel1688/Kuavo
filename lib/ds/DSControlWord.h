// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "DriverStationTypes.h"

/**
 * A wrapper around Driver Station control word.
 */
class DSControlWord {
public:
    /**
         * DSControlWord constructor.
         *
         * Upon construction, the current Driver Station control word is read and
         * stored internally.
         */
    DSControlWord();

    /**
         * Check if the DS has enabled the robot.
         *
         * @return True if the robot is enabled and the DS is connected
         */
    bool isEnabled() const;

    /**
         * Check if the robot is disabled.
         *
         * @return True if the robot is explicitly disabled or the DS is not connected
         */
    bool isDisabled() const;

    /**
         * Check if the robot is e-stopped.
         *
         * @return True if the robot is e-stopped
         */
    bool isEStopped() const;

    /**
         * Check if the DS is commanding autonomous mode.
         *
         * @return True if the robot is being commanded to be in autonomous mode
         */
    bool isAutonomous() const;

    /**
         * Check if the DS is commanding autonomous mode and if it has enabled the
         * robot.
         *
         * @return True if the robot is being commanded to be in autonomous mode and
         * enabled.
         */
    bool isAutonomousEnabled() const;

    /**
         * Check if the DS is commanding teleop mode.
         *
         * @return True if the robot is being commanded to be in teleop mode
         */
    bool isTeleop() const;

    /**
         * Check if the DS is commanding teleop mode and if it has enabled the robot.
         *
         * @return True if the robot is being commanded to be in teleop mode and
         * enabled.
         */
    bool isTeleopEnabled() const;

    /**
         * Check if the DS is commanding test mode.
         *
         * @return True if the robot is being commanded to be in test mode
         */
    bool isTest() const;

    /**
         * Check if the DS is attached.
         *
         * @return True if the DS is connected to the robot
         */
    bool isDSAttached() const;

private:
    HAL_ControlWord m_controlWord;
};
