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
class ArmController : public ControllerBase<7, 2, 4> {
public:
    /**
     * Constructs a Arm controller.
     */
    ArmController();

    /**
     * Move constructor.
     */
    ArmController(ArmController &&) = default;

    /**
     * Move assignment operator.
     */
    ArmController &operator=(ArmController &&) = default;

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

class DOF6Kinematic {
private:
    const float RAD_TO_DEG = 57.295777754771045f;

    // DH parameters
    struct ArmConfig_t {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
    };
    ArmConfig_t armConfig;

    float DH_matrix[6][4] = {0};// home,d,a,alpha
    float L1_base[3] = {0};
    float L2_arm[3] = {0};
    float L3_elbow[3] = {0};
    float L6_wrist[3] = {0};

    float l_se_2;
    float l_se;
    float l_ew_2;
    float l_ew;
    float atan_e;

public:
    struct Joint6D_t {
        Joint6D_t() = default;

        Joint6D_t(float a1, float a2, float a3, float a4, float a5, float a6)
            : a{a1, a2, a3, a4, a5, a6} {}

        float a[6];

        friend Joint6D_t operator-(const Joint6D_t &_joints1, const Joint6D_t &_joints2);
    };

    struct Pose6D_t {
        Pose6D_t() = default;

        Pose6D_t(float x, float y, float z, float a, float b, float c)
            : X(x), Y(y), Z(z), A(a), B(b), C(c), hasR(false) {}

        float X{}, Y{}, Z{};
        float A{}, B{}, C{};
        float R[9]{};

        // if Pose was calculated by FK then it's true automatically (so that no need to do extra calc),
        // otherwise if manually set params then it should be set to false.
        bool hasR{};
    };

    struct IKSolves_t {
        Joint6D_t config[8];
        char solFlag[8][3];
    };

    DOF6Kinematic(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);
    bool solveFK(const Joint6D_t &_inputJoint6D, Pose6D_t &_outputPose6D);
    bool solveIK(const Pose6D_t &_inputPose6D, const Joint6D_t &_lastJoint6D, IKSolves_t &_outputSolves);
};
