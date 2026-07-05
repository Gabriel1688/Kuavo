#pragma once

#include <atomic>
#include <thread>
/**
 * For internal use only.
 */
class DriverStationModeThread {
public:
    /**
         * For internal use only.
         */
    DriverStationModeThread();

    ~DriverStationModeThread();

    DriverStationModeThread(const DriverStationModeThread &other) = delete;
    DriverStationModeThread(DriverStationModeThread &&other) = delete;
    DriverStationModeThread &operator=(const DriverStationModeThread &other) = delete;
    DriverStationModeThread &operator=(DriverStationModeThread &&other) = delete;

    /**
         * Only to be used to tell the Driver Station what code you claim to be
         * executing for diagnostic purposes only.
         *
         * @param entering If true, starting disabled code; if false, leaving disabled
         * code
         */
    void InDisabled(bool entering);

    /**
         * Only to be used to tell the Driver Station what code you claim to be
         * executing for diagnostic purposes only.
         *
         * @param entering If true, starting autonomous code; if false, leaving
         * autonomous code
         */
    void InAutonomous(bool entering);

    /**
         * Only to be used to tell the Driver Station what code you claim to be
         * executing for diagnostic purposes only.
         *
         * @param entering If true, starting teleop code; if false, leaving teleop
         * code
         */
    void InTeleop(bool entering);

    /**
         * Only to be used to tell the Driver Station what code you claim to be
         * executing for diagnostic purposes only.
         *
         * @param entering If true, starting test code; if false, leaving test code
         */
    void InTest(bool entering);

private:
    std::atomic_bool m_keepAlive{false};
    std::thread m_thread;
    void Run();
    std::atomic_bool m_userInDisabled{false};
    std::atomic_bool m_userInAutonomous{false};
    std::atomic_bool m_userInTeleop{false};
    std::atomic_bool m_userInTest{false};
};
