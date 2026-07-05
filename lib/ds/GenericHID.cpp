#include "GenericHID.h"
#include "BooleanEvent.h"
#include "DriverStation.h"
#include <string>

GenericHID::GenericHID(int port) {
    m_port = port;
}

bool GenericHID::getRawButton(int button) const {
    return DriverStation::getStickButton(m_port, button);
}

bool GenericHID::getRawButtonPressed(int button) {
    return DriverStation::getStickButtonPressed(m_port, button);
}

bool GenericHID::getRawButtonReleased(int button) {
    return DriverStation::getStickButtonReleased(m_port, button);
}

BooleanEvent GenericHID::button(int button, EventLoop *loop) const {
    return BooleanEvent(loop, [this, button]() { return this->getRawButton(button); });
}

double GenericHID::getRawAxis(int axis) const {
    return DriverStation::getStickAxis(m_port, axis);
}

int GenericHID::getPOV(int pov) const {
    return DriverStation::getStickPOV(m_port, pov);
}

BooleanEvent GenericHID::pov(int angle, EventLoop *loop) const {
    return pov(0, angle, loop);
}

BooleanEvent GenericHID::pov(int pov, int angle, EventLoop *loop) const {
    return BooleanEvent(loop, [this, pov, angle] { return this->getPOV(pov) == angle; });
}

BooleanEvent GenericHID::povUp(EventLoop *loop) const {
    return pov(0, loop);
}

BooleanEvent GenericHID::povUpRight(EventLoop *loop) const {
    return pov(45, loop);
}

BooleanEvent GenericHID::povRight(EventLoop *loop) const {
    return pov(90, loop);
}

BooleanEvent GenericHID::povDownRight(EventLoop *loop) const {
    return pov(135, loop);
}

BooleanEvent GenericHID::povDown(EventLoop *loop) const {
    return pov(180, loop);
}

BooleanEvent GenericHID::povDownLeft(EventLoop *loop) const {
    return pov(225, loop);
}

BooleanEvent GenericHID::povLeft(EventLoop *loop) const {
    return pov(270, loop);
}

BooleanEvent GenericHID::povUpLeft(EventLoop *loop) const {
    return pov(315, loop);
}

BooleanEvent GenericHID::povCenter(EventLoop *loop) const {
    return pov(360, loop);
}

BooleanEvent GenericHID::axisLessThan(int axis, double threshold, EventLoop *loop) const {
    return BooleanEvent(loop, [this, axis, threshold]() {
        return this->getRawAxis(axis) < threshold;
    });
}

BooleanEvent GenericHID::axisGreaterThan(int axis, double threshold, EventLoop *loop) const {
    return BooleanEvent(loop, [this, axis, threshold]() {
        return this->getRawAxis(axis) > threshold;
    });
}

int GenericHID::getAxisCount() const {
    return DriverStation::getStickAxisCount(m_port);
}

int GenericHID::getPOVCount() const {
    return DriverStation::getStickPOVCount(m_port);
}

int GenericHID::getButtonCount() const {
    return DriverStation::getStickButtonCount(m_port);
}

bool GenericHID::isConnected() const {
    return DriverStation::isJoystickConnected(m_port);
}

int GenericHID::getPort() const {
    return m_port;
}