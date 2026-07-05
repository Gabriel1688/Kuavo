#pragma  once

#include <queue>
#include <cstddef>

/*!
 * \class GenericTimerQueue
 * Container for storing objects sorted by time based keys. The implementation
 * uses relative time values and is independent of their units.
 *
 * \note For type safety this class cannot be instantiated directly, and instead,
 * must be accessed using objects derived from the template class TimerQueue.
 */
class GenericTimerQueue {
protected:

    /*!
     * ctor.
     */
    GenericTimerQueue() {}

    /*!
     * dtor.
     */
    ~GenericTimerQueue() {}

    /*!
     * Create a Timer for the object and push it onto the queue.
     * \param now - current time.
     * \param interval - offset from 'now' when the timer should expire.
     * \param obj - object to queue.
     */
    void push(long now, long interval, void *obj) {
        if (interval < 0) {
            interval = 0;
        }
        queue_.push(Timer(now + interval, obj));
    }

    /*!
     * If the timer at the head of the queue has expired remove it and return
     * its associated object.
     * \param now - current time.
     * \return  The object from the head of the queue or null if empty.
     */
    void *pop(long now) {
        if (!queue_.empty()) {
            Timer timer = queue_.top();
            if ((now - timer.expires) >= 0) {
                queue_.pop();
                return timer.object;
            }
        }
        return 0;
    }

    /*!
     * Unconditionaly remove the timer at the head of the queue and return
     * its associated object.
     * \return  The object from the head of the queue or null if empty.
     */
    void *pop() {
        if (!queue_.empty()) {
            Timer timer = queue_.top();
            queue_.pop();
            return timer.object;
        }
        return 0;
    }

    /*!
     * The expires of the object from the head of the queue.
     * \return The expires of the top object of the queue or -1 if empty.
     */
    long getTopExpires() {
        if (!queue_.empty()) {
            Timer timer = queue_.top();
            return timer.expires;
        }
        return -1;
    }

    /*!
     * Checks if the queue is empty
     * \return  True if the queue is empty.
     */
    bool empty() {
        return queue_.empty();
    }

    /*!
     * Gets the queue size
     * \return  Queue size.
     */
    size_t size() {
        return queue_.size();
    }

private:

    GenericTimerQueue(const GenericTimerQueue &);

    GenericTimerQueue &operator=(const GenericTimerQueue &);

private:

    /*!
     * \struct Timer
     * A timer queue element.
     */
    struct Timer {
        /*!
         * ctor
         * \param when - when timer expires.
         * \param obj  - user object.
         */
        Timer(long when, void *obj)
                : expires(when),
                  object(obj) { ; }

        long expires;
        void *object;
    };

    /*!
     * \struct TimerCompare
     * Compare function required for std::priority_queue.
     * This test results in the queue being sorted in ascending order.
     * The less than zero test handles sign changes due to overflow.
     */
    struct TimerCompare {
        bool operator()(const Timer &lhs, const Timer &rhs) const {
            return (rhs.expires - lhs.expires < 0);
        }
    };

private:

    /*!
     * Timers are stored in a priority_queue in the order that they will
     * expire. This increases the insertion cost but reduces the search cost
     * as the search can stop as soon as an unexpired entry is found.
     */
    std::priority_queue<Timer, std::vector<Timer>, TimerCompare> queue_;
};


/*!
 * \class TimerQueue
 * Type-safe interface to the GenericTimerQueue class.
 */
template<class T>
class TimerQueue : private GenericTimerQueue {
public:

    /*!
     * Add an object to the queue.
     * \param now - current time.
     * \param interval - offset from 'now' when the timer should expire.
     * \param obj - object to queue.
     */
    void push(long now, long interval, T *obj) {
        return GenericTimerQueue::push(now, interval, obj);
    }

    /*!
     * Try to retrieve the object at the head of the queue.
     * \param now - current time.
     * \return  The object or null if no timer has expired.
     */
    T *pop(long now) {
        return static_cast<T *>(GenericTimerQueue::pop(now));
    }

    /*!
     * Try to retrieve the object at the head of the queue.
     * \return  The object or null if the queue is empty.
     */
    T *pop() {
        return static_cast<T *>(GenericTimerQueue::pop());
    }

    /*!
     * The expires of the object from the head of the queue.
     * \return The expires of the top object of the queue or -1 if empty.
     */
    long getTopExpires() {
        return GenericTimerQueue::getTopExpires();
    }

    /*!
     * Checks if the queue is empty
     * \return  True if the queue is empty.
     */
    bool empty() {
        return GenericTimerQueue::empty();
    }

    /*!
     * Gets the queue size
     * \return  Queue size.
     */
    size_t size() {
        return GenericTimerQueue::size();
    }

};
