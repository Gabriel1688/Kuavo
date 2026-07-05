#pragma once

#include <functional>
#include <vector>
//#include <wpi/FunctionExtras.h>
//wpiutil/src/main/native/thirdparty/llvm/include/wpi/FunctionExtras.h

/** A declarative way to bind a set of actions to a loop and execute them when
 * the loop is polled. */
class EventLoop {
public:
    EventLoop();

    EventLoop(const EventLoop &) = delete;

    EventLoop &operator=(const EventLoop &) = delete;

    /**
     * Bind a new action to run when the loop is polled.
     *
     * @param action the action to run.
     */
    void bind(std::function<void()> action);

    /**
     * Poll all bindings.
     */
    void poll();

    /**
     * Clear all bindings.
     */
    void clear();

private:
    std::vector<std::function<void()>> m_bindings;
    bool m_running{false};
};
