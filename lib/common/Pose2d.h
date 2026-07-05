#pragma once

#include <cmath>// For M_PI
#include <iostream>

// Simplified Rotation2d class for demonstration
// https://github.com/wpilibsuite/allwpilib/blob/f1b9be551be8909a94f2d39a1a0c79b467a1965f/wpimath/src/main/native/include/units/curvature.h#L12
class Rotation2d {
public:
    Rotation2d(double radians = 0.0) : m_radians(radians) {}

    double radians() const { return m_radians; }

    double degrees() const { return m_radians * 180.0 / M_PI; }

private:
    double m_radians;
};

// Simplified Pose2d class for demonstration
class Pose2d {
public:
    Pose2d(double x = 0.0, double y = 0.0, const Rotation2d &rotation = Rotation2d())
            : m_x(x), m_y(y), m_rotation(rotation) {}

    double X() const { return m_x; }

    double Y() const { return m_y; }

    const Rotation2d &rotation() const { return m_rotation; }

    // Example of a common operation: transforming a pose
    Pose2d operator+(const Pose2d &other) const {
        // This is a simplified addition for demonstration.
        // Real Pose2d addition for transformations is more complex.
        return Pose2d(m_x + other.m_x, m_y + other.m_y, Rotation2d(m_rotation.radians() + other.m_rotation.radians()));
    }

private:
    double m_x;
    double m_y;
    Rotation2d m_rotation;
};
