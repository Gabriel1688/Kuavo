#pragma once

#include <functional>
#include <memory>
#include "EventLoop.h"

/**
 * This class provides an easy way to link actions to active high logic signals.
 * Each object represents a digital signal to which callback actions can be
 * bound using {@link #ifHigh(std::function<void()>)}.
 *
 * <p>BooleanEvents can easily be composed for advanced functionality using
 * {@link #operator&&}, {@link #operator||}, and {@link #operator!}.
 *
 * <p>To get a new BooleanEvent that triggers when this one changes see {@link
 * #falling()} and {@link #rising()}.
 */
class BooleanEvent {
public:
    /**
     * Creates a new event that is active when the condition is true.
     *
     * @param loop the loop that polls this event
     * @param signal the digital signal represented by this object.
     */
    BooleanEvent(EventLoop *loop, std::function<bool()> signal);

    /**
     * Returns the state of this signal (high or low) as of the last loop poll.
     *
     * @return true for the high state, false for the low state. If the event was
     * never polled, it returns the state at event construction.
     */
    bool getAsBoolean() const;

    operator std::function<bool()>();  // NOLINT

    /**
     * Bind an action to this event.
     *
     * @param action the action to run if this event is active.
     */
    void ifHigh(std::function<void()> action);

    /**
     * A method to "downcast" a BooleanEvent instance to a subclass (for example,
     * to a command-based version of this class).
     *
     * @param ctor a method reference to the constructor of the subclass that
     * accepts the loop as the first parameter and the condition/signal as the
     * second.
     * @return an instance of the subclass.
     */
    template<class T>
    T castTo(std::function<T(EventLoop *, std::function<bool()>)> ctor =
    [](EventLoop *loop, std::function<bool()> condition) {
        return T(loop, condition);
    }) {
        return ctor(m_loop, [state = m_state] { return *state; });
    }

    /**
     * Creates a new event that is active when this event is inactive.
     *
     * @return the negated event
     */
    BooleanEvent operator!();

    /**
     * Composes this event with another event, returning a new event that is
     * active when both events are active.
     *
     * <p>The events must use the same event loop. If the events use different
     * event loops, the composed signal won't update until both loops are polled.
     *
     * @param rhs the event to compose with
     * @return the event that is active when both events are active
     */
    BooleanEvent operator&&(std::function<bool()> rhs);

    /**
     * Composes this event with another event, returning a new event that is
     * active when either event is active.
     *
     * <p>The events must use the same event loop. If the events use different
     * event loops, the composed signal won't update until both loops are polled.
     *
     * @param rhs the event to compose with
     * @return the event that is active when either event is active
     */
    BooleanEvent operator||(std::function<bool()> rhs);

    /**
     * Creates a new event that triggers when this one changes from false to true.
     *
     * @return the new event.
     */
    BooleanEvent rising();

    /**
     * Creates a new event that triggers when this one changes from true to false.
     *
     * @return the event.
     */
    BooleanEvent falling();

private:
    EventLoop *m_loop;
    std::function<bool()> m_signal;
    std::shared_ptr<bool> m_state;  // A programmer's worst nightmare.
};