#include "BooleanEvent.h"
#include <memory>
#include <utility>

BooleanEvent::BooleanEvent(EventLoop *loop, std::function<bool()> condition)
        : m_loop(loop), m_signal(std::move(condition)) {
    m_state = std::make_shared<bool>(m_signal());
    m_loop->bind(
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            [condition = m_signal, state = m_state] { *state = condition(); });
}

bool BooleanEvent::getAsBoolean() const {
    return *m_state;
}

BooleanEvent::operator std::function<bool()>() {
    return [state = m_state] { return *state; };
}

void BooleanEvent::ifHigh(std::function<void()> action) {
    m_loop->bind([state = m_state, action = std::move(action)] {
        if (*state) {
            action();
        }
    });
}

BooleanEvent BooleanEvent::operator!() {
    return BooleanEvent(this->m_loop, [state = m_state] { return !*state; });
}

BooleanEvent BooleanEvent::operator&&(std::function<bool()> rhs) {
    return BooleanEvent(this->m_loop, [state = m_state, rhs] { return *state && rhs(); });
}

BooleanEvent BooleanEvent::operator||(std::function<bool()> rhs) {
    return BooleanEvent(this->m_loop, [state = m_state, rhs] { return *state || rhs(); });
}

BooleanEvent BooleanEvent::rising() {
    return BooleanEvent(this->m_loop,
                        [state = m_state, m_previous = *m_state]() mutable {
                            bool present = *state;
                            bool past = m_previous;
                            m_previous = present;
                            return !past && present;
                        });
}

BooleanEvent BooleanEvent::falling() {
    return BooleanEvent(this->m_loop,
                        [state = m_state, m_previous = *m_state]() mutable {
                            bool present = *state;
                            bool past = m_previous;
                            m_previous = present;
                            return past && !present;
                        });
}