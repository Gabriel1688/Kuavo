#pragma once
/**
 * IterativeRobotBase implements a specific type of robot program framework,
 * extending the RobotBase class.
 *
 * The IterativeRobotBase class does not implement startCompetition(), so it
 * should not be used by teams directly.
 *
 * This class provides the following functions which are called by the main
 * loop, startCompetition(), at the appropriate times:
 *
 * robotInit() -- provide for initialization at robot power-on
 *
 * driverStationConnected() -- provide for initialization the first time the DS
 * is connected
 *
 * init() functions -- each of the following functions is called once when the
 * appropriate mode is entered:
 *
 * \li disabledInit() -- called each and every time disabled is entered from
 *   another mode
 * \li autonomousInit() -- called each and every time autonomous is entered from
 *   another mode
 * \li teleopInit() -- called each and every time teleop is entered from another
 *   mode
 * \li testInit() -- called each and every time test is entered from another
 *   mode
 *
 * Periodic() functions -- each of these functions is called on an interval:
 *
 * \li robotPeriodic()
 * \li disabledPeriodic()
 * \li autonomousPeriodic()
 * \li teleopPeriodic()
 * \li TestPeriodic()
 *
 * Exit() functions -- each of the following functions is called once when the
 * appropriate mode is exited:
 *
 * \li DisabledExit() -- called each and every time disabled is exited
 * \li autonomousExit() -- called each and every time autonomous is exited
 * \li teleopExit() -- called each and every time teleop is exited
 * \li TestExit() -- called each and every time test is exited
 */
#include "RobotBase.h"

class IterativeRobotBase : public RobotBase {
public:
    /**
     * Robot-wide initialization code should go here.
     *
     * Users should override this method for default Robot-wide initialization
     * which will be called when the robot is first powered on. It will be called
     * exactly one time.
     *
     * Note: This method is functionally identical to the class constructor so
     * that should be used instead.
     */
    virtual void robotInit();

    /**
     * Code that needs to know the DS state should go here.
     *
     * Users should override this method for initialization that needs to occur
     * after the DS is connected, such as needing the alliance information.
     */
    virtual void driverStationConnected();

    /**
     * Initialization code for autonomous mode should go here.
     *
     * Users should override this method for initialization code which will be
     * called each time the robot enters autonomous mode.
     */
    virtual void autonomousInit();

    /**
     * Initialization code for teleop mode should go here.
     *
     * Users should override this method for initialization code which will be
     * called each time the robot enters teleop mode.
     */
    virtual void teleopInit();

    /**
     * Periodic code for all modes should go here.
     *
     * This function is called each time a new packet is received from the driver
     * station.
     */
    virtual void robotPeriodic();

    /**
     * Periodic code for autonomous mode should go here.
     *
     * Users should override this method for code which will be called each time a
     * new packet is received from the driver station and the robot is in
     * autonomous mode.
     */
    virtual void autonomousPeriodic();

    /**
     * Periodic code for teleop mode should go here.
     *
     * Users should override this method for code which will be called each time a
     * new packet is received from the driver station and the robot is in teleop
     * mode.
     */
    virtual void teleopPeriodic();

    /**
     * Exit code for autonomous mode should go here.
     *
     * Users should override this method for code which will be called each time
     * the robot exits autonomous mode.
     */
    virtual void autonomousExit();

    /**
     * Exit code for teleop mode should go here.
     *
     * Users should override this method for code which will be called each time
     * the robot exits teleop mode.
     */
    virtual void teleopExit();

    /**
     * Gets time period between calls to Periodic() functions.
     */
    int getPeriod() const;

    /**
     * Prints list of epochs added so far and their times.
     */
    void printWatchdogEpochs();

    /**
     * Constructor for IterativeRobotBase.
     *
     * @param period Period.
     */
    explicit IterativeRobotBase(int period);

    ~IterativeRobotBase() {};

    virtual void startCompetition();

protected:
    IterativeRobotBase(IterativeRobotBase &&) = default;

    IterativeRobotBase &operator=(IterativeRobotBase &&) = default;

    /**
     * Loop function.
     */
    void loopFunc();

private:
    enum class Mode {
        kNone, kAutonomous, kTeleop
    };

    Mode m_lastMode = Mode::kNone;
    int m_period;
    bool m_ntFlushEnabled = true;
    bool m_lwEnabledInTest = false;
    bool m_calledDsConnected = false;
};

