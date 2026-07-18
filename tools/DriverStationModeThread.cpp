#include "DriverStationModeThread.h"
#include <common/Synchronization.h>
#include <ds/DriverStation.h>
#include <iostream>

DriverStationModeThread::DriverStationModeThread() {
    m_keepAlive = true;
    m_thread = std::thread{[this] { Run(); }};
}

DriverStationModeThread::~DriverStationModeThread() {
    m_keepAlive = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void DriverStationModeThread::InDisabled(bool entering) {
    m_userInDisabled = entering;
}

void DriverStationModeThread::InAutonomous(bool entering) {
    m_userInAutonomous = entering;
}

void DriverStationModeThread::InTeleop(bool entering) {
    m_userInTeleop = entering;
}

void DriverStationModeThread::InTest(bool entering) {
    m_userInTest = entering;
}

void DriverStationModeThread::Run() {
    wpi::Event event{false, false};
    HAL_ProvideNewDataEventHandle(event.getHandle());

    while (m_keepAlive.load()) {
        wpi::waitForObject(event.getHandle());

        DriverStation::refreshData();

        if (m_userInDisabled) {
            std::cout << "DriverStationModeThread::Run user disabled." << std::endl;
        }
        if (m_userInAutonomous) {
            std::cout << "DriverStationModeThread::Run InAutonomous." << std::endl;
        }
        if (m_userInTeleop) {
            std::cout << "DriverStationModeThread::Run InTeleop." << std::endl;
        }
    }
    HAL_RemoveNewDataEventHandle(event.getHandle());
}
