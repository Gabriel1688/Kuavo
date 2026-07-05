#pragma once

#include "common/Pose2d.h"
#include "common/TrajectoryConfig.h"
#include "memory.h"
#include "robot/ControllerBase.h"
#include <functional>
#include <math.h>
#include <tuple>
#include <vector>

/**
 * The Arm controller.
 *
 * The Arm uses a linear time-varying LQR for feedback control. Since the
 * model is control-affine (the dynamics are nonlinear, but the control inputs
 * provide a linear contribution), a plant inversion feedforward was used.
 * Trajectories generated from splines provide the motion profile to follow.
 *
 * The linear time-varying controller has a similar form to the LQR, but the
 * model used to compute the controller gain is the nonlinear model linearized
 * around the drivetrain's current state. We precomputed gains for important
 * places in our state-space, then interpolated between them with a LUT to save
 * computational resources.
 *
 * We decided to control for longitudinal error and cross-track error in the
 * chassis frame instead of x and y error in the global frame, so the state
 * Jacobian simplified such that we only had to sweep velocities (-4m/s to
 * 4m/s).
 *
 * See section 9.6 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 9.6.3.
 */
class Controller : public ControllerBase<7, 2, 4> {
public:
    /**
     * Constructs a Arm controller.
     */
    Controller();

    /**
     * Move constructor.
     */
    Controller(Controller &&) = default;

    /**
     * Move assignment operator.
     */
    Controller &operator=(Controller &&) = default;

    /**
     * Sets the current estimated global pose of the drivetrain.
     */
    void setDrivetrainStates(const Eigen::Vector<double, 7> &x);

    /**
     * Returns whether the Arm controller is at the goal waypoint.
     */
    bool atGoal() const;

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void reset(const Pose2d &initialPose);

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Vector<double, 2> calculate(const Eigen::Vector<double, 7> &x) override;

    /**
     * The Arm system dynamics.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 7> dynamics(const Eigen::Vector<double, 7> &x,
                                             const Eigen::Vector<double, 2> &u);

    /**
     * Returns the global measurements that correspond to the given state and
     * input vectors.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 2> globalMeasurementModel(
        const Eigen::Vector<double, 7> &x, const Eigen::Vector<double, 2> &u);

private:
    static constexpr auto kPositionTolerance = 0.25;
    static constexpr auto kVelocityTolerance = 2;
    static constexpr auto kAngleTolerance = 0.52;

    Pose2d m_goal;

    float m_visionYaw = 0;  //rad
    float m_visionPitch = 0;//rad
    float m_visionRange = 0;//meter
    long m_timestamp = 0;   //sec

    Pose2d m_armNextPoseInGlobal;
    float m_armLeftVelocity = 0.0;
    float m_armRightVelocity = 0.0;
};