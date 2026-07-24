
#pragma once

#include "SubsystemBase.h"
#include "common/FdEvent.h"
#include "message.h"
#include "spdlog/spdlog.h"
#include <array>
#include <atomic>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string_view>
#include <chrono>
#include <ctime>
#include <thread>

/**
 * A base class for subsystems with controllers.
 *
 * State, Inputs, and Outputs indices should be specified what they represent in
 * the derived class.
 *
 * @tparam States the number of state estimates in the state vector
 * @tparam Inputs the number of control inputs in the input vector
 * @tparam Outputs the number of local outputs in the output vector
 */
template<int States, int Inputs, int Outputs>
class ControlledSubsystemBase : public SubsystemBase {
public:
    /**
     * Constructs a ControlledSubsystemBase.
     *
     * @param controllerName Name of the controller log file.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    ControlledSubsystemBase() {
        m_entryThreadRunning = true;

        // Configure real-time thread: SCHED_FIFO priority 90, 256 KiB stack
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 256 * 1024);
        if (pthread_create(&thread_id, &attr, EntryOfThread, this) != 0) {
            m_entryThreadRunning = false;
            SPDLOG_ERROR("failed start thread.");
        }
        pthread_attr_destroy(&attr);

        if (m_entryThreadRunning) {
            struct sched_param param{};
            param.sched_priority = 90;
            int ret = pthread_setschedparam(thread_id, SCHED_FIFO, &param);
            if (ret != 0) {
                SPDLOG_WARN("Leg thread: failed to set SCHED_FIFO/90: {}",
                            strerror(ret));
            } else {
                SPDLOG_INFO("Leg thread: SCHED_FIFO priority 90");
            }
        }
    }

    /**
     * Move constructor.
     */
    ControlledSubsystemBase(ControlledSubsystemBase &&) = default;

    /**
     * Move assignment operator.
     */
    ControlledSubsystemBase &operator=(ControlledSubsystemBase &&) = default;

    ~ControlledSubsystemBase() override {
        stopThread();
    }

    /**
     * Stops the RT thread and joins it.  Safe to call multiple times.
     * Derived destructors MUST call this before destroying any state that
     * controllerPeriodic() depends on, so the thread exits while the
     * derived vtable is still installed.
     */
    void stopThread() {
        bool wasRunning = m_entryThreadRunning.exchange(false);
        if (wasRunning) {
            void *res;
            pthread_join(thread_id, &res);
        }
    }

    /**
     * Restarts the RT thread after a stopThread() call.
     * No-op if the thread is already running.
     */
    void startThread() {
        if (m_entryThreadRunning.load(std::memory_order_acquire))
            return;  // already running
        m_entryThreadRunning.store(true, std::memory_order_release);
        m_pauseRequested.store(false, std::memory_order_release);

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 256 * 1024);
        if (pthread_create(&thread_id, &attr, EntryOfThread, this) != 0) {
            m_entryThreadRunning.store(false, std::memory_order_release);
            SPDLOG_ERROR("startThread: failed to recreate RT thread.");
        }
        pthread_attr_destroy(&attr);

        if (m_entryThreadRunning.load(std::memory_order_acquire)) {
            struct sched_param param{};
            param.sched_priority = 90;
            int ret = pthread_setschedparam(thread_id, SCHED_FIFO, &param);
            if (ret != 0) {
                SPDLOG_WARN("startThread: failed to set SCHED_FIFO/90: {}",
                            strerror(ret));
            }
        }
    }

    /**
     * Enables the control loop.
     */
    void enable() {
        // m_lastTime is reset so that a large time delta isn't generated from
        // Update() not being called in a while.
        // m_lastTime = frc::Timer::GetFPGATimestamp() - Constants::kControllerPeriod;
        m_isEnabled = true;
    }

    /**
     * Disables the control loop.
     */
    void disable() { m_isEnabled = false; }

    /**
     * Returns true if the control loop is enabled.
     */
    bool isEnabled() const { return m_isEnabled; }

    /**
     * Pauses the real-time control loop so callers can safely modify shared
     * state (e.g., unmap the shared memory backing the subsystem). Blocks until
     * the current controllerPeriodic() call finishes or a short timeout expires.
     */
    void pause() {
        m_pauseRequested.store(true, std::memory_order_release);
        uint64_t start = monotonic_ns();
        while (m_inControllerPeriodic.load(std::memory_order_acquire)) {
            std::this_thread::yield();
            if (monotonic_ns() - start > 5'000'000ULL) { // 5 ms
                SPDLOG_ERROR("ControlledSubsystemBase::pause timeout");
                break;
            }
        }
    }

    /**
     * Resumes the real-time control loop after pause().
     */
    void resume() {
        m_pauseRequested.store(false, std::memory_order_release);
    }

protected:
    /**
     * Marks the subsystem as ready for periodic execution.  Must be called
     * at the end of the derived class constructor so the RT thread does not
     * invoke the pure-virtual controllerPeriodic() before the vtable is
     * fully installed.
     */
    void markReady() { m_ready.store(true, std::memory_order_release); }

public:

    /**
     * Returns the most recent timestep.
     */
    // units::second_t GetDt() const { return m_dt; }

    /**
     * Runs periodic observer and controller update.
     */
    virtual void controllerPeriodic() = 0;

    virtual void onMessage(std::shared_ptr<MESSAGE> message, TCallback callback) = 0;

    void message(std::shared_ptr<MESSAGE> message, TCallback callback) {
        using ThisType = typename std::remove_pointer<decltype(this)>::type;

        auto functor = std::make_unique<std::function<void()>>(
            std::bind(&ThisType::onMessage, this, message, callback));
        send_queue_.push(std::move(functor));
    }
    /**
     * Computes current timestep's dt.
     */
    void updateDt() {
        //        m_nowBegin = frc::Timer::GetFPGATimestamp();
        //        m_dt = m_nowBegin - m_lastTime;
        //
        //        if (m_dt == 0_s) {
        //            m_dt = Constants::kControllerPeriod;
        //            fmt::print(stderr, "ERROR @ t = {}: dt = 0\n", m_nowBegin);
        //        }
        //
        //        // Clamp spikes in scheduling latency
        //        if (m_dt > 10_ms) {
        //            m_dt = Constants::kControllerPeriod;
        //        }
    }

private:
    static void *EntryOfThread(void *argv) {
        ControlledSubsystemBase *base = static_cast<ControlledSubsystemBase *>(argv);
        base->Run();
        return nullptr;
    }

    // Helper: read CLOCK_MONOTONIC as nanoseconds (vDSO, ~20 ns).
    static uint64_t monotonic_ns() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
    }

    void Run() {
        struct pollfd item;
        item.fd = send_queue_.getFd();
        item.events = POLLIN;
        item.revents = 0;

        std::vector<pollfd> poll_items;
        poll_items.push_back(item);

         //TODO:: change to 200Hz for real motor communication.
        // Use a high-resolution timer for 2.5ms (400Hz) periodic execution
        static constexpr uint64_t PERIOD_NS = 2'500'000ULL;  // 2.5ms = 400Hz
        uint64_t next_wake_ns = monotonic_ns();

        while (m_entryThreadRunning) {
            // Calculate time until next periodic tick
            uint64_t now_ns = monotonic_ns();
            int timeout_ms = 0;
            if (next_wake_ns > now_ns) {
                // Convert remaining nanoseconds to milliseconds (round up to
                // avoid waking too early)
                timeout_ms = static_cast<int>((next_wake_ns - now_ns + 999'999ULL) / 1'000'000ULL);
            }

            int rc = poll(&poll_items[0], poll_items.size(), timeout_ms);
            if (rc < 0 && errno != EINTR) {
                SPDLOG_ERROR("ControlledSubsystemBase::Run poll error: {}", strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Drain any pending messages from the queue
            if (rc > 0) {
                std::vector<pollfd>::const_iterator i;
                for (i = poll_items.begin(); i != poll_items.end(); ++i) {
                    if ((*i).revents != 0) {
                        auto f = send_queue_.pop();
                        while (f) {
                            (*f)();
                            f = send_queue_.pop();
                        }
                    }
                }
            }

            // Execute controllerPeriodic() on schedule.
            // Always run regardless of m_isEnabled so that observation
            // (motor feedback staging) continues even while the subsystem
            // is disabled.  The subclass gates actuation (MIT dispatch)
            // internally using isEnabled().
            // Guard on m_ready to prevent calling the pure virtual before
            // the derived class constructor has finished.
            now_ns = monotonic_ns();
            if (now_ns >= next_wake_ns && m_ready.load(std::memory_order_acquire)) {
                // Set m_inControllerPeriodic BEFORE checking m_pauseRequested
                // so that pause() spinning on m_inControllerPeriodic will see
                // "true" and wait for us, closing the race window where pause()
                // could return while we are about to enter controllerPeriodic().
                m_inControllerPeriodic.store(true, std::memory_order_release);
                if (m_pauseRequested.load(std::memory_order_acquire)) {
                    // Caller requested pause — do NOT run controllerPeriodic().
                    m_inControllerPeriodic.store(false, std::memory_order_release);
                } else {
                    controllerPeriodic();
                    m_inControllerPeriodic.store(false, std::memory_order_release);
                    next_wake_ns += PERIOD_NS;
                    // Catch up if we fell behind (avoid spiral)
                    if (next_wake_ns < now_ns) {
                        next_wake_ns = now_ns + PERIOD_NS;
                    }
                }
            }

            pthread_testcancel();
        }
    }
    //units::second_t m_dt = Constants::kControllerPeriod;  //5ms
    int m_dt = 5;
    std::atomic<bool> m_isEnabled{false};
    std::atomic<bool> m_ready{false};
    std::atomic<bool> m_entryThreadRunning{false};
    std::atomic<bool> m_pauseRequested{false};
    std::atomic<bool> m_inControllerPeriodic{false};
    pthread_t thread_id;
    Fifo<std::function<void()>> send_queue_;
};
