#pragma once

#include "GenericHID.h"

/**
 * Handle input from Xbox controllers connected to the Driver Station.
 *
 * This class handles Xbox input that comes from the Driver Station. Each
 * time a value is requested the most recent value is returned. There is a
 * single class instance for each controller and the mapping of ports to
 * hardware buttons depends on the code in the Driver Station.
 *
 * Only first party controllers from Microsoft are guaranteed to have the
 * correct mapping, and only through the official NI DS. Sim is not guaranteed
 * to have the same mapping, as well as any 3rd party controllers.
 */
class XboxController : public GenericHID
//                       public wpi::Sendable,
//                       public wpi::SendableHelper<XboxController>
{

public:
    /**
     * Construct an instance of a controller.
     *
     * The controller index is the USB port on the Driver Station.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     */
    explicit XboxController(int port);

    ~XboxController() override = default;

    XboxController(XboxController &&) = default;

    XboxController &operator=(XboxController &&) = default;

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return the axis value.
     */
    double getLeftX() const;

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return the axis value.
     */
    double getRightX() const;

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return the axis value.
     */
    double getLeftY() const;

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return the axis value.
     */
    double getRightY() const;

    /**
     * Get the left trigger axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    double getLeftTriggerAxis() const;

    /**
     * Constructs an event instance around the axis value of the left trigger.
     * The returned trigger will be true when the axis value is greater than
     * {@code threshold}.
     * @param threshold the minimum axis value for the returned event to be true.
     * This value should be in the range [0, 1] where 0 is the unpressed state of
     * the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis
     * exceeds the provided threshold, attached to the given event loop
     */
    BooleanEvent leftTrigger(double threshold, EventLoop *loop) const;

    /**
     * Constructs an event instance around the axis value of the left trigger.
     * The returned trigger will be true when the axis value is greater than 0.5.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis
     * exceeds 0.5, attached to the given event loop
     */
    BooleanEvent leftTrigger(EventLoop *loop) const;

    /**
     * Get the right trigger axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    double getRightTriggerAxis() const;

    /**
     * Constructs an event instance around the axis value of the right trigger.
     * The returned trigger will be true when the axis value is greater than
     * {@code threshold}.
     * @param threshold the minimum axis value for the returned event to be true.
     * This value should be in the range [0, 1] where 0 is the unpressed state of
     * the axis.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis
     * exceeds the provided threshold, attached to the given event loop
     */
    BooleanEvent rightTrigger(double threshold, EventLoop *loop) const;

    /**
     * Constructs an event instance around the axis value of the right trigger.
     * The returned trigger will be true when the axis value is greater than 0.5.
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis
     * exceeds 0.5, attached to the given event loop
     */
    BooleanEvent rightTrigger(EventLoop *loop) const;

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    bool getAButton() const;

    /**
     * Whether the A button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getAButtonPressed();

    /**
     * Whether the A button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getAButtonReleased();

    /**
     * Constructs an event instance around the A button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's
     * digital signal attached to the given loop.
     */
    BooleanEvent A(EventLoop *loop) const;

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    bool getBButton() const;

    /**
     * Whether the B button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getBButtonPressed();

    /**
     * Whether the B button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getBButtonReleased();

    /**
     * Constructs an event instance around the B button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's
     * digital signal attached to the given loop.
     */
    BooleanEvent B(EventLoop *loop) const;

    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    bool getXButton() const;

    /**
     * Whether the X button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getXButtonPressed();

    /**
     * Whether the X button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getXButtonReleased();

    /**
     * Constructs an event instance around the X button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's
     * digital signal attached to the given loop.
     */
    BooleanEvent X(EventLoop *loop) const;

    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    bool getYButton() const;

    /**
     * Whether the Y button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getYButtonPressed();

    /**
     * Whether the Y button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getYButtonReleased();

    /**
     * Constructs an event instance around the Y button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's
     * digital signal attached to the given loop.
     */
    BooleanEvent Y(EventLoop *loop) const;

    /**
     * Read the value of the left bumper button on the controller.
     *
     * @return The state of the button.
     */
    bool getLeftBumperButton() const;

    /**
     * Whether the left bumper button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getLeftBumperButtonPressed();

    /**
     * Whether the left bumper button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getLeftBumperButtonReleased();

    /**
     * Constructs an event instance around the left bumper button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper button's
     * digital signal attached to the given loop.
     */
    BooleanEvent leftBumper(EventLoop *loop) const;

    /**
     * Read the value of the right bumper button on the controller.
     *
     * @return The state of the button.
     */
    bool getRightBumperButton() const;

    /**
     * Whether the right bumper button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getRightBumperButtonPressed();

    /**
     * Whether the right bumper button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getRightBumperButtonReleased();

    /**
     * Constructs an event instance around the right bumper button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper button's
     * digital signal attached to the given loop.
     */
    BooleanEvent rightBumper(EventLoop *loop) const;

    /**
     * Read the value of the back button on the controller.
     *
     * @return The state of the button.
     */
    bool getBackButton() const;

    /**
     * Whether the back button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getBackButtonPressed();

    /**
     * Whether the back button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getBackButtonReleased();

    /**
     * Constructs an event instance around the back button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's
     * digital signal attached to the given loop.
     */
    BooleanEvent back(EventLoop *loop) const;

    /**
     * Read the value of the start button on the controller.
     *
     * @return The state of the button.
     */
    bool getStartButton() const;

    /**
     * Whether the start button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getStartButtonPressed();

    /**
     * Whether the start button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getStartButtonReleased();

    /**
     * Constructs an event instance around the start button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's
     * digital signal attached to the given loop.
     */
    BooleanEvent start(EventLoop *loop) const;

    /**
     * Read the value of the left stick button on the controller.
     *
     * @return The state of the button.
     */
    bool getLeftStickButton() const;

    /**
     * Whether the left stick button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getLeftStickButtonPressed();

    /**
     * Whether the left stick button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getLeftStickButtonReleased();

    /**
     * Constructs an event instance around the left stick button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's
     * digital signal attached to the given loop.
     */
    BooleanEvent leftStick(EventLoop *loop) const;

    /**
     * Read the value of the right stick button on the controller.
     *
     * @return The state of the button.
     */
    bool getRightStickButton() const;

    /**
     * Whether the right stick button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    bool getRightStickButtonPressed();

    /**
     * Whether the right stick button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    bool getRightStickButtonReleased();

    /**
     * Constructs an event instance around the right stick button's
     * digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's
     * digital signal attached to the given loop.
     */
    BooleanEvent rightStick(EventLoop *loop) const;

    /** Represents a digital button on an XboxController. */
    struct Button {
        /// A button.
        static constexpr int kA = 1;
        /// B button.
        static constexpr int kB = 2;
        /// X button.
        static constexpr int kX = 3;
        /// Y button.
        static constexpr int kY = 4;
        /// Left bumper button.
        static constexpr int kLeftBumper = 5;
        /// Right bumper button.
        static constexpr int kRightBumper = 6;
        /// Back button.
        static constexpr int kBack = 7;
        /// Start button.
        static constexpr int kStart = 8;
        /// Left stick button.
        static constexpr int kLeftStick = 9;
        /// Right stick button.
        static constexpr int kRightStick = 10;
    };

    /** Represents an axis on an XboxController. */
    struct Axis {
        /// Left X axis.
        static constexpr int kLeftX = 0;
        /// Right X axis.
        static constexpr int kRightX = 4;
        /// Left Y axis.
        static constexpr int kLeftY = 1;
        /// Right Y axis.
        static constexpr int kRightY = 5;
        /// Left trigger.
        static constexpr int kLeftTrigger = 2;
        /// Right trigger.
        static constexpr int kRightTrigger = 3;
    };
};
