#pragma once

#include "GenericTimerQueue.h"
#include "TimerEvent.h"
#include <memory>

/*----------------------------------------------------------------------------*/

/*!
 * \class TimerManager
 * Manages a queue of events to be processed after their associated timeout
 * period has elapsed.
 */
class TimerManager {
public:
    /*!
     * \typedef timer_id_t - type used to uniquely identify each timed event.
     */
    typedef unsigned int timer_id_t;

    /*!
     * This value will never be used to identify a timer.
     */
    static const timer_id_t INVALID_TIMER_ID = 0;

    /*!
     * ctor
     */
    TimerManager();

    /*!
     * dtor
     */
    ~TimerManager();

    /*!
     * Queue an event.
     * \param event - event to queue.
     * \param milliseconds - timeout period
     */
    timer_id_t startTimer(TimerEvent *event, long milliseconds);

    timer_id_t stopTimer(TimerEvent::TimerEventType event_type, timer_id_t &id) {
        (void) event_type;
        return id;
    };

    long getTopExpires();

    /*!
     * Get the event information for an expired timer.
     * \param event - receives the event object.
     * \param id - receives the timer's identifier.
     * \return  true, if an expired timer was found; otherwise, false.
     */
    bool getExpiredTimer(TimerEvent *&event, timer_id_t &id);

private:
    /*!
     * \struct Timer
     * Details for a single timer. Binds the timer's identifier to its event
     * object, enabling them to be queued together as a single entity.
     */
    struct Timer {
        Timer(timer_id_t id, TimerEvent *evt)
            : timer_id(id),
              event(evt) {
            ;
        }

        timer_id_t timer_id;
        TimerEvent *event;
    };

    /*!
     * Generate a timer identifier.
     */
    inline timer_id_t getNextTimerId();

private:
    TimerManager(const TimerManager &);

    TimerManager &operator=(const TimerManager &);

private:
    std::unique_ptr<TimerQueue<Timer>> timer_queue_;
    timer_id_t next_timer_id_;
    std::unique_ptr<TimerQueue<Timer>> curl_timer_queue_;
};

inline TimerManager::timer_id_t TimerManager::getNextTimerId() {
    if (next_timer_id_ == INVALID_TIMER_ID) {
        next_timer_id_ = 1;
    }
    return next_timer_id_++;
}