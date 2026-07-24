#pragma once

#include "IterativeRobotBase.h"
#include "common/priority_queue.h"
#include <chrono>
#include <ctime>
#include <functional>
#include <utility>
#include <vector>

/**
 * TimedRobot implements the IterativeRobotBase robot program framework.
 *
 * The TimedRobot class is intended to be subclassed by a user creating a
 * robot program.
 *
 * Periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
class TimedRobot : public IterativeRobotBase {
public:
    /// Default loop period.
    static constexpr auto kDefaultPeriod = 20;//2ms need to disable the file sink of spdlog .

    /**
     * Constructor for TimedRobot.
     *
     * @param period Period.
     */
    explicit TimedRobot(int period = kDefaultPeriod);

    TimedRobot(TimedRobot &&) = default;

    TimedRobot &operator=(TimedRobot &&) = default;

    ~TimedRobot();

    /**
     * Add a callback to run at a specific period with a starting time offset.
     *
     * This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
     * run synchronously. Interactions between them are thread-safe.
     *
     * @param callback The callback to run.
     * @param period   The period at which to run the callback.
     * @param offset   The offset from the common starting time. This is useful
     *                 for scheduling a callback in a different timeslot relative
     *                 to TimedRobot.
     */
    void addPeriodic(std::function<void()> callback, int period, int offset = 0);

    void startCompetition();

    void endCompetition();

private:
    /// Read CLOCK_MONOTONIC as microseconds (vDSO, ~20 ns).
    static uint64_t monotonic_us() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL
             + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    }

    class Callback {
    public:
        std::function<void()> m_func;
        uint64_t m_period;          // microseconds
        uint64_t m_expirationTime;  // microseconds (CLOCK_MONOTONIC)

        /**
         * Construct a callback container.
         *
         * @param func      The callback to run.
         * @param startTime The common starting point for all callback scheduling (us).
         * @param period    The period at which to run the callback (us).
         * @param offset    The offset from the common starting time (us).
         */
        Callback(std::function<void()> func, uint64_t startTime,
                 uint64_t period, uint64_t offset)
            : m_func{std::move(func)},
              m_period{period},
              m_expirationTime(startTime + offset + period) {}

        bool operator>(const Callback &rhs) const {
            return m_expirationTime > rhs.m_expirationTime;
        }
    };

    uint64_t m_startTime;  // microseconds (CLOCK_MONOTONIC)
    uint64_t m_loopStartTimeUs = 0;

    priority_queue<Callback, std::vector<Callback>, std::greater<Callback>> m_callbacks;
};
