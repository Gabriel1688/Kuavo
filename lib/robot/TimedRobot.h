#pragma once

#include "IterativeRobotBase.h"
#include "common/priority_queue.h"
#include <chrono>
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
    class Callback {
    public:
        std::function<void()> m_func;
        std::chrono::microseconds m_period;
        std::chrono::microseconds m_expirationTime;

        /**
         * Construct a callback container.
         *
         * @param func      The callback to run.
         * @param startTime The common starting point for all callback scheduling.
         * @param period    The period at which to run the callback.
         * @param offset    The offset from the common starting time.
         */
        Callback(std::function<void()> func, std::chrono::microseconds startTime,
                 std::chrono::microseconds period, std::chrono::microseconds offset)
            : m_func{std::move(func)},
              m_period{period},
              m_expirationTime(startTime + offset + period) {}

        bool operator>(const Callback &rhs) const {
            return m_expirationTime > rhs.m_expirationTime;
        }
    };

    std::chrono::microseconds m_startTime;
    uint64_t m_loopStartTimeUs = 0;

    priority_queue<Callback, std::vector<Callback>, std::greater<Callback>> m_callbacks;
};