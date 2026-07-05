#include "IterativeRobotBase.h"
#include "ds/DSControlWord.h"
#include "ds/DriverStation.h"
#include <iostream>

IterativeRobotBase::IterativeRobotBase(int period)
    : m_period(period) {}

void IterativeRobotBase::robotInit() {}

void IterativeRobotBase::startCompetition() {}

void IterativeRobotBase::driverStationConnected() {}

void IterativeRobotBase::autonomousInit() {}

void IterativeRobotBase::teleopInit() {}

void IterativeRobotBase::robotPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}

void IterativeRobotBase::autonomousPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}

void IterativeRobotBase::teleopPeriodic() {
    static bool firstRun = true;
    if (firstRun) {
        std::cout << "Default {}() method... Override me!" << std::endl;
        firstRun = false;
    }
}

void IterativeRobotBase::autonomousExit() {}

void IterativeRobotBase::teleopExit() {}

int IterativeRobotBase::getPeriod() const {
    return m_period;
}

void IterativeRobotBase::loopFunc() {
    DriverStation::refreshData();
    // Get current mode by link to the control word object from the DS
    DSControlWord word;
    Mode mode = Mode::kNone;
    if (word.isAutonomous()) {
        mode = Mode::kAutonomous;
    } else if (word.isTeleop()) {
        mode = Mode::kTeleop;
    }

    if (!m_calledDsConnected && word.isDSAttached()) {
        m_calledDsConnected = true;
        driverStationConnected();
    }

    // If mode changed, call mode exit and entry functions
    if (m_lastMode != mode) {
        // Call last mode's exit function
        if (m_lastMode == Mode::kAutonomous) {
            autonomousExit();
        } else if (m_lastMode == Mode::kTeleop) {
            teleopExit();
        }

        // Call current mode's entry function
        if (mode == Mode::kAutonomous) {
            autonomousInit();
            //m_watchdog.AddEpoch("autonomousInit()");
        } else if (mode == Mode::kTeleop) {
            teleopInit();
            //m_watchdog.AddEpoch("teleopInit()");
        }

        m_lastMode = mode;
    }

    // Call the appropriate function depending upon the current robot mode
    if (mode == Mode::kAutonomous) {
        //GW HAL_ObserveUserProgramAutonomous();
        autonomousPeriodic();
        //m_watchdog.AddEpoch("autonomousPeriodic()");
    } else if (mode == Mode::kTeleop) {
        //GW HAL_ObserveUserProgramTeleop();
        teleopPeriodic();
        //m_watchdog.AddEpoch("teleopPeriodic()");
    }

    robotPeriodic();
}
