#pragma once

#include <stdint.h>
#include <string>

class BooleanEvent;

class EventLoop;

/*
 * Handle input from standard HID devices connected to the Driver Station.
 *
 * <p>This class handles standard input that comes from the Driver Station. Each
 * time a value is requested the most recent value is returned. There is a
 * single class instance for each device and the mapping of ports to hardware
 * buttons depends on the code in the Driver Station.
 */
class GenericHID {
public:
    /**
     * Represents a rumble output on the Joystick.
     */
    enum RumbleType {
        /// Left rumble motor.
        kLeftRumble,
        /// Right rumble motor.
        kRightRumble,
        /// Both left and right rumble motors.
        kBothRumble
    };

    /**
     * USB HID interface type.
     */
    enum HIDType {
        /// Unknown.
        kUnknown = -1,
        /// XInputUnknown.
        kXInputUnknown = 0,
        /// XInputGamepad.
        kXInputGamepad = 1,
        /// XInputWheel.
        kXInputWheel = 2,
        /// XInputArcadeStick.
        kXInputArcadeStick = 3,
        /// XInputFlightStick.
        kXInputFlightStick = 4,
        /// XInputDancePad.
        kXInputDancePad = 5,
        /// XInputGuitar.
        kXInputGuitar = 6,
        /// XInputGuitar2.
        kXInputGuitar2 = 7,
        /// XInputDrumKit.
        kXInputDrumKit = 8,
        /// XInputGuitar3.
        kXInputGuitar3 = 11,
        /// XInputArcadePad.
        kXInputArcadePad = 19,
        /// HIDJoystick.
        kHIDJoystick = 20,
        /// HIDGamepad.
        kHIDGamepad = 21,
        /// HIDDriving.
        kHIDDriving = 22,
        /// HIDFlight.
        kHIDFlight = 23,
        /// HID1stPerson.
        kHID1stPerson = 24
    };

    explicit GenericHID(int port);

    virtual ~GenericHID() = default;

    GenericHID(GenericHID &&) = default;

    GenericHID &operator=(GenericHID &&) = default;

    /**
     * Get the button value (starting at button 1).
     *
     * The buttons are returned in a single 16 bit value with one bit representing
     * the state of each button. The appropriate button is returned as a boolean
     * value.
     *
     * This method returns true if the button is being held down at the time
     * that this method is being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    bool getRawButton(int button) const;

    /**
     * Whether the button was pressed since the last check. %Button indexes begin
     * at 1.
     *
     * This method returns true if the button went from not pressed to held down
     * since the last time this method was called. This is useful if you only
     * want to call a function once when you press the button.
     *
     * @param button The button index, beginning at 1.
     * @return Whether the button was pressed since the last check.
     */
    bool getRawButtonPressed(int button);

    /**
     * Whether the button was released since the last check. %Button indexes begin
     * at 1.
     *
     * This method returns true if the button went from held down to not pressed
     * since the last time this method was called. This is useful if you only
     * want to call a function once when you release the button.
     *
     * @param button The button index, beginning at 1.
     * @return Whether the button was released since the last check.
     */
    bool getRawButtonReleased(int button);

    /**
     * Constructs an event instance around this button's digital signal.
     *
     * @param button the button index
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the button's digital signal attached
     * to the given loop.
     */
    BooleanEvent button(int button, EventLoop *loop) const;

    /**
     * Get the value of the axis.
     *
     * @param axis The axis to read, starting at 0.
     * @return The value of the axis.
     */
    double getRawAxis(int axis) const;

    /**
     * Get the angle in degrees of a pov on the HID.
     *
     * The pov angles start at 0 in the up direction, and increase clockwise
     * (e.g. right is 90, upper-left is 315).
     *
     * @param pov The index of the pov to read (starting at 0)
     * @return the angle of the pov in degrees, or -1 if the pov is not pressed.
     */
    int getPOV(int pov = 0) const;

    /**
     * Constructs a BooleanEvent instance based around this angle of a pov on the
     * HID.
     *
     * <p>The pov angles start at 0 in the up direction, and increase clockwise
     * (eg right is 90, upper-left is 315).
     *
     * @param loop the event loop instance to attach the event to.
     * @param angle pov angle in degrees, or -1 for the center / not pressed.
     * @return a BooleanEvent instance based around this angle of a pov on the
     * HID.
     */
    BooleanEvent pov(int angle, EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around this angle of a pov on the
     * HID.
     *
     * <p>The pov angles start at 0 in the up direction, and increase clockwise
     * (eg right is 90, upper-left is 315).
     *
     * @param loop the event loop instance to attach the event to.
     * @param pov   index of the pov to read (starting at 0). Defaults to 0.
     * @param angle pov angle in degrees, or -1 for the center / not pressed.
     * @return a BooleanEvent instance based around this angle of a pov on the
     * HID.
     */
    BooleanEvent pov(int pov, int angle, EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 0 degree angle (up) of
     * the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 0 degree angle of a pov on
     * the HID.
     */
    BooleanEvent povUp(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 45 degree angle (right
     * up) of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 45 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povUpRight(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 90 degree angle (right)
     * of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 90 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povRight(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 135 degree angle (right
     * down) of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 135 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povDownRight(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 180 degree angle (down)
     * of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 180 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povDown(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 225 degree angle (down
     * left) of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 225 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povDownLeft(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 270 degree angle (left)
     * of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 270 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povLeft(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the 315 degree angle (left
     * up) of the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the 315 degree angle of a pov
     * on the HID.
     */
    BooleanEvent povUpLeft(EventLoop *loop) const;

    /**
     * Constructs a BooleanEvent instance based around the center (not pressed) of
     * the default (index 0) pov on the HID.
     *
     * @return a BooleanEvent instance based around the center of a pov on the
     * HID.
     */
    BooleanEvent povCenter(EventLoop *loop) const;

    /**
     * Constructs an event instance that is true when the axis value is less than
     * threshold
     *
     * @param axis The axis to read, starting at 0.
     * @param threshold The value below which this trigger should return true.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the axis value is less than the
     * provided threshold.
     */
    BooleanEvent axisLessThan(int axis, double threshold, EventLoop *loop) const;

    /**
     * Constructs an event instance that is true when the axis value is greater
     * than threshold
     *
     * @param axis The axis to read, starting at 0.
     * @param threshold The value above which this trigger should return true.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the axis value is greater than
     * the provided threshold.
     */
    BooleanEvent axisGreaterThan(int axis, double threshold, EventLoop *loop) const;

    /**
     * Get the number of axes for the HID.
     *
     * @return the number of axis for the current HID
     */
    int getAxisCount() const;

    /**
     * Get the number of POVs for the HID.
     *
     * @return the number of POVs for the current HID
     */
    int getPOVCount() const;

    /**
     * Get the number of buttons for the HID.
     *
     * @return the number of buttons on the current HID
     */
    int getButtonCount() const;

    /**
     * Get if the HID is connected.
     *
     * @return true if the HID is connected
     */
    bool isConnected() const;

    /**
     * Get the port number of the HID.
     *
     * @return The port number of the HID.
     */
    int getPort() const;

    /**
     * Set a single HID output value for the HID.
     *
     * @param outputNumber The index of the output to set (1-32)
     * @param value        The value to set the output to
     */
    void setOutput(int outputNumber, bool value);

    /**
     * Set all output values for the HID.
     *
     * @param value The 32 bit output value (1 bit for each output)
     */
    void setOutputs(int value);

    /**
     * Set the rumble output for the HID.
     *
     * The DS currently supports 2 rumble values, left rumble and right rumble.
     *
     * @param type  Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    void setRumble(RumbleType type, double value);

private:
    int m_port;
    int m_outputs = 0;
    uint16_t m_leftRumble = 0;
    uint16_t m_rightRumble = 0;
};