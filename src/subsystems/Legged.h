#pragma once

#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "common/TrajectoryConfig.h"

#include "robot/ControlledSubsystemBase.h"
#include "string"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <vector>

// Forward declarations
class UdpServer;
class Motor;

namespace mercury {
class MotorParamCache;
struct SharedMemoryLayout;
struct MotorGroupStageData;
template<typename T> struct SourceDoubleBuffer;
}  // namespace mercury

/**
 * The Legged subsystem.
 *
 * The Legged uses an unscented Kalman filter for state estimation.
 */
class Legged : public ControlledSubsystemBase<7, 2, 6> {
public:
    /// The Legged length.  unit <meter>
    static constexpr float kLength = 0.9398;

    /**
     * Distance from middle of robot to intake. unit <meter>
     */
    static constexpr float kMiddleOfRobotToIntake = 0.656;

    /**
     * Producer-consumer queue for global pose measurements from Vision
     * subsystem.
     */
    //wpi::static_circular_buffer<Vision::GlobalMeasurement, 8> visionQueue;

    Legged(int baseId,
           mercury::SharedMemoryLayout* shm = nullptr,
           mercury::SourceDoubleBuffer<mercury::MotorGroupStageData>* staging = nullptr,
           mercury::MotorParamCache* paramCache = nullptr);

    ~Legged();

    Legged(const Legged &) = delete;

    Legged &operator=(const Legged &) = delete;

    /**
     * Fault reasons passed to disableAllMotorsOnce().
     */
    enum DisableReason {
        SHM_INVALID_MAGIC = 1,
        SHM_VERSION_MISMATCH,
        SHM_LIFECYCLE_NOT_RUNNING,
        HEARTBEAT_STALE,
        EMERGENCY_STOP_ACTIVE,
        CMD_WRITE_IDX_INVALID
    };

    /**
     * Returns encoder displacement. unit<meter>
     */
    float getPosition() const;

    /**
     * Returns encoder velocity. unit<meters_per_second>
     */
    float getVelocity() const;

    /**
     * Resets all sensors and controller.
     */
    void reset(const Pose2d &initialPose = Pose2d());

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void correctWithGlobalOutputs(float x, float y, long timestamp);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    //    void addTrajectory(const frc::Pose2d& start,
    //                       const std::vector<frc::Translation2d>& interior,
    //                       const frc::Pose2d& end);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by makeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    //    void addTrajectory(const frc::Pose2d& start,
    //                       const std::vector<frc::Translation2d>& interior,
    //                       const frc::Pose2d& end,
    //                       const frc::TrajectoryConfig& config);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     */
    void addTrajectory(const std::vector<Pose2d> &waypoints);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by makeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void addTrajectory(const std::vector<Pose2d> &waypoints,
                       const TrajectoryConfig &config);

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static TrajectoryConfig makeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory.
     * @param endVelocity   The end velocity of the trajectory.
     */
    static TrajectoryConfig makeTrajectoryConfig(
        float startVelocity,
        float endVelocity);

    /**
     * Returns whether the Legged controller is at the goal waypoint.
     */
    bool atGoal() const;

    //TODO:: need to optimize the state/input/output of legged.
    /**
     * Returns the Legged state estimate.
     */
    const Eigen::Vector<double, 7> &getStates() const;

    /**
     * Returns the Legged inputs.
     */
    const Eigen::Vector<double, 2> &getInputs() const;

    /**
     * Returns how many times the vision measurement was too far from the
     * Legged pose estimate.
     */
    int getPoseMeasurementFaultCounter();

    void disabledInit() override;

    void autonomousInit() override;

    void teleopInit() override;

    void robotPeriodic() override;

    void teleopPeriodic() override;

    void controllerPeriodic() override;

    void onMessage(std::shared_ptr<MESSAGE> message, TCallback callback) override;

    void setEnable(bool _enable);
    void reboot();

    void updateState(TCallback &callback);

    /** Read-only access to motors for telemetry. */
    const std::vector<std::shared_ptr<Motor>> &getMotors() const { return motors; }

    /** Base CAN id (1 = left, 7 = right). */
    int getBaseId() const { return baseId; }

    /** Human-readable subsystem name. */
    std::string getName() const { return baseId == 1 ? "left" : "right"; }

    /** Returns bitmask of responsive motors (bit i = motor i is responsive). */
    uint32_t getMotorStatusBits() const;

    /** Set SHM and staging buffer pointers (called from Robot::robotInit after SHM attach). */
    void setShmPointers(mercury::SharedMemoryLayout* shm,
                        mercury::SourceDoubleBuffer<mercury::MotorGroupStageData>* staging);

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    float m_headingOffset = 0.0;

    int m_poseMeasurementFaultCounter = 0;

    void init();

    void setJointSpeed(float _speed);

    void setJointAcceleration(float _acc);

    void updateJointAngles();

    void updateJointAnglesCallback();

    void updateJointPose6D();

    void homing();

    void resting();

    void disableAllMotorsOnce(int reason);

    void setCommandMode(uint32_t _mode);

    std::vector<std::shared_ptr<Motor>> motors;
    std::atomic<bool> contact{false};
    int baseId = 0;
    int m_groupOffset = 0;  // Joint index offset: 0 for left, 6 for right
    std::chrono::steady_clock::time_point m_lastCheckTime{};
    std::vector<bool> m_motorResponsive;
    uint64_t m_paramQueryCycle = 0;  // Round-robin counter for parameter queries

    // Cross-process shared memory pointers (owned by Robot, not by Legged).
    // Atomic so the RT thread's snapshot in controllerPeriodic() sees the
    // nullptr written by setShmPointers() without requiring pause()/resume()
    // to act as a full memory-visibility barrier for non-atomic stores.
    std::atomic<mercury::SharedMemoryLayout*> m_shm{nullptr};
    std::atomic<mercury::SourceDoubleBuffer<mercury::MotorGroupStageData>*> m_staging{nullptr};

    // Motor safety state
    std::atomic<bool> m_motorsFaultDisabled{false};

    // Timing state
    uint64_t m_lastControllerStartNs = 0;
};
