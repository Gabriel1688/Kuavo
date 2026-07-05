#include "SubsystemBase.h"
#include <algorithm>

std::vector<SubsystemBase *> SubsystemBase::m_subsystems;

SubsystemBase::SubsystemBase() { m_subsystems.emplace_back(this); }

SubsystemBase::~SubsystemBase() {
    m_subsystems.erase(std::remove(m_subsystems.begin(), m_subsystems.end(), this));
}

void SubsystemBase::runAllDisabledInit() {
    for (auto &subsystem: m_subsystems) {
        subsystem->disabledInit();
    }
}

void SubsystemBase::runAllAutonomousInit() {
    for (auto &subsystem: m_subsystems) {
        subsystem->autonomousInit();
    }
}

void SubsystemBase::runAllTeleopInit() {
    consumeButtonEdgeEvents();

    for (auto &subsystem: m_subsystems) {
        subsystem->teleopInit();
    }
}

void SubsystemBase::runAllRobotPeriodic() {
    for (auto &subsystem: m_subsystems) {
        subsystem->robotPeriodic();
    }
}

void SubsystemBase::runAllDisabledPeriodic() {
    for (auto &subsystem: m_subsystems) {
        subsystem->disabledPeriodic();
    }
}

void SubsystemBase::runAllAutonomousPeriodic() {
    for (auto &subsystem: m_subsystems) {
        subsystem->autonomousPeriodic();
    }
}

void SubsystemBase::runAllTeleopPeriodic() {
    for (auto &subsystem: m_subsystems) {
        subsystem->teleopPeriodic();
    }
}

void SubsystemBase::consumeButtonEdgeEvents() {
    // Consumes button edge events produced in disabled mode
    //    for (int stick = 0; stick < frc::DriverStation::kJoystickPorts; ++stick) {
    //        for (int button = 1; button < 32; ++button) {
    //            DriverStation::getStickButtonPressed(stick, button);
    //            DriverStation::getStickButtonReleased(stick, button);
    //        }
    //    }
}
