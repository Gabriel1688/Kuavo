#pragma once

#include <vector>

/**
 * A standardized base for subsystems.
 */
class SubsystemBase {
public:
    /**
     * Constructs a SubsystemBase.
     */
    SubsystemBase();

    /**
     * Copy constructor.
     */
    SubsystemBase(const SubsystemBase &) = default;

    /**
     * Copy assignment operator.
     */
    SubsystemBase &operator=(const SubsystemBase &) = default;

    /**
     * Move constructor.
     */
    SubsystemBase(SubsystemBase &&) = default;

    /**
     * Move assignment operator.
     */
    SubsystemBase &operator=(SubsystemBase &&) = default;

    virtual ~SubsystemBase();

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * Users should override this method for default Robot-wide simulation
     * related initialization which will be called when the robot is first
     * started. It will be called exactly one time after robotInit is called
     * only when the robot is in simulation.
     */
    virtual void simulationInit() {}

    /**
     * Initialization code for disabled mode should go here.
     */
    virtual void disabledInit() {}

    /**
     * Initialization code for autonomous mode should go here.
     */
    virtual void autonomousInit() {}

    /**
     * Initialization code for teleop mode should go here.
     */
    virtual void teleopInit() {}

    /**
     * Initialization coe for test mode should go here.
     */
    virtual void testInit() {}

    /**
     * Periodic code for all modes should go here.
     */
    virtual void robotPeriodic() {}

    /**
     * Periodic simulation code should go here.
     *
     * This function is called in a simulated robot after user code executes.
     */
    virtual void simulationPeriodic() {}

    /**
     * Periodic code for disabled mode should go here.
     */
    virtual void disabledPeriodic() {}

    /**
     * Periodic code for autonomous mode should go here.
     */
    virtual void autonomousPeriodic() {}

    /**
     * Periodic code for teleop mode should go here.
     */
    virtual void teleopPeriodic() {}

    /**
     * Call all subsystems's disabledInit().
     */
    static void runAllDisabledInit();

    /**
     * Call all subsystems's autonomousInit().
     */
    static void runAllAutonomousInit();

    /**
     * Call all subsystems's teleopInit().
     */
    static void runAllTeleopInit();

    /**
     * Call all subsystems's robotPeriodic().
     */
    static void runAllRobotPeriodic();

    /**
     * Call all subsystems's disabledPeriodic().
     */
    static void runAllDisabledPeriodic();

    /**
     * Call all subsystems's autonomousPeriodic().
     */
    static void runAllAutonomousPeriodic();

    /**
     * Call all subsystems's teleopPeriodic().
     */
    static void runAllTeleopPeriodic();

private:
    static std::vector<SubsystemBase *> m_subsystems;

    /**
     * Consumes button edge events produced in disabled mode.
     */
    static void consumeButtonEdgeEvents();
};