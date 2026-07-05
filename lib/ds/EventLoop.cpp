#include "EventLoop.h"
#include <utility>

namespace {
    struct RunningSetter {
        bool &m_running;

        explicit RunningSetter(bool &running) noexcept: m_running{running} {
            m_running = true;
        }

        ~RunningSetter() noexcept { m_running = false; }
    };
}// namespace

EventLoop::EventLoop() {}

void EventLoop::bind(std::function<void()> action) {
    m_bindings.emplace_back(std::move(action));
}

void EventLoop::poll() {
    RunningSetter runSetter{m_running};
    for (std::function<void()> &action: m_bindings) {
        action();
    }
}

void EventLoop::clear() {
    m_bindings.clear();
}
