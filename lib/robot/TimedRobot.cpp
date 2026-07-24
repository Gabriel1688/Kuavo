#include "TimedRobot.h"
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <utility>

void TimedRobot::startCompetition() {
    robotInit();

    // Set main robot loop to SCHED_FIFO priority 75 (100 Hz, 10 ms)
    struct sched_param param{};
    param.sched_priority = 75;
    int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        SPDLOG_WARN("Main Loop: failed to set SCHED_FIFO/75: {}", strerror(ret));
    } else {
        SPDLOG_INFO("Main Loop: SCHED_FIFO priority 75");
    }

    // Loop forever, calling the appropriate mode-dependent function
    while (true) {
        // We don't have to check there's an element in the queue first because
        // there's always at least one (the constructor adds one). It's reenqueued
        // at the end of the loop.
        auto callback = m_callbacks.pop();

        // Sleep until the next callback is due
        auto now = std::chrono::steady_clock::now();
        std::chrono::microseconds currentTime =
            std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

        if (callback.m_expirationTime > currentTime) {
            std::this_thread::sleep_for(callback.m_expirationTime - currentTime);
            now = std::chrono::steady_clock::now();
            currentTime = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
        }

        callback.m_func();

        // Increment the expiration time by the number of full periods it's behind
        // plus one to avoid rapid repeat fires from a large loop overrun. We assume
        // currentTime ≥ expirationTime rather than checking for it since the
        // callback wouldn't be running otherwise.
        callback.m_expirationTime += callback.m_period + (currentTime - callback.m_expirationTime) / callback.m_period * callback.m_period;
        m_callbacks.push(std::move(callback));

        // Process all other callbacks that are ready to run
        while (m_callbacks.top().m_expirationTime <= currentTime) {
            callback = m_callbacks.pop();

            callback.m_func();

            callback.m_expirationTime +=
                callback.m_period + (currentTime - callback.m_expirationTime) / callback.m_period * callback.m_period;
            m_callbacks.push(std::move(callback));
        }
    }
}

TimedRobot::TimedRobot(int period) : IterativeRobotBase(period) {
    //    m_startTime = std::chrono::microseconds{RobotController::GetFPGATime()};
    m_startTime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch());
    addPeriodic([=, this] { loopFunc(); }, period);
    int32_t status = 0;
}

void TimedRobot::endCompetition() {
    int32_t status = 0;
}

TimedRobot::~TimedRobot() {
}
//NOTE: change period/offset to milliseconds.
void TimedRobot::addPeriodic(std::function<void()> callback, int period, int offset) {
    m_callbacks.emplace(callback,
                        m_startTime,
                        std::chrono::microseconds{static_cast<int64_t>(period * 1e3)},
                        std::chrono::microseconds{static_cast<int64_t>(offset * 1e3)});
}
