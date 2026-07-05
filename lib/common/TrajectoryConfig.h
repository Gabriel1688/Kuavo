#pragma once

#include "TrajectoryConstraint.h"
#include <memory>
#include <vector>

class TrajectoryConfig {
public:
    /**
         * Constructs a config object.
         * @param maxVelocity The max velocity of the trajectory.
         * @param maxAcceleration The max acceleration of the trajectory.
         */
    TrajectoryConfig(float maxVelocity,
                     float maxAcceleration)
            : m_maxVelocity(maxVelocity), m_maxAcceleration(maxAcceleration) {}

    TrajectoryConfig(const TrajectoryConfig &) = delete;

    TrajectoryConfig &operator=(const TrajectoryConfig &) = delete;

    TrajectoryConfig(TrajectoryConfig &&) = default;

    TrajectoryConfig &operator=(TrajectoryConfig &&) = default;

    /**
         * Sets the start velocity of the trajectory.
         * @param startVelocity The start velocity of the trajectory.
         */
    void setStartVelocity(float startVelocity) {
        m_startVelocity = startVelocity;
    }

    /**
         * Sets the end velocity of the trajectory.
         * @param endVelocity The end velocity of the trajectory.
         */
    void setEndVelocity(float endVelocity) {
        m_endVelocity = endVelocity;
    }

    /**
         * Sets the reversed flag of the trajectory.
         * @param reversed Whether the trajectory should be reversed or not.
         */
    void setReversed(bool reversed) { m_reversed = reversed; }

    /**
         * Adds a user-defined constraint to the trajectory.
         * @param constraint The user-defined constraint.
         */
    template<std::derived_from<TrajectoryConstraint> Constraint>
    void addConstraint(Constraint constraint) {
        m_constraints.emplace_back(std::make_unique<Constraint>(constraint));
    }

    /**
         * Adds a differential drive kinematics constraint to ensure that
         * no wheel velocity of a differential drive goes above the max velocity.
         *
         * @param kinematics The differential drive kinematics.
         */
    //        void SetKinematics(const DifferentialDriveKinematics& kinematics) {
    //            addConstraint( DifferentialDriveKinematicsConstraint(kinematics, m_maxVelocity));
    //        }

    /**
         * Returns the starting velocity of the trajectory.
         * @return The starting velocity of the trajectory.
         */
    float startVelocity() const { return m_startVelocity; }

    /**
         * Returns the ending velocity of the trajectory.
         * @return The ending velocity of the trajectory.
         */
    float endVelocity() const { return m_endVelocity; }

    /**
         * Returns the maximum velocity of the trajectory.
         * @return The maximum velocity of the trajectory.
         */
    float maxVelocity() const { return m_maxVelocity; }

    /**
         * Returns the maximum acceleration of the trajectory.
         * @return The maximum acceleration of the trajectory.
         */
    float maxAcceleration() const {
        return m_maxAcceleration;
    }

    /**
         * Returns the user-defined constraints of the trajectory.
         * @return The user-defined constraints of the trajectory.
         */
    const std::vector<std::unique_ptr<TrajectoryConstraint>> &constraints()
    const {
        return m_constraints;
    }

    /**
         * Returns whether the trajectory is reversed or not.
         * @return whether the trajectory is reversed or not.
         */
    bool isReversed() const { return m_reversed; }

private:
    float m_startVelocity = 0.0;
    float m_endVelocity = 0.0;
    float m_maxVelocity;
    float m_maxAcceleration;
    bool m_reversed = false;
    std::vector<std::unique_ptr<TrajectoryConstraint>> m_constraints;
};
