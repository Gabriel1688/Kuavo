
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
            if (pthread_setschedparam(thread_id, SCHED_FIFO, &param) != 0) {
                SPDLOG_WARN("Leg thread: failed to set SCHED_FIFO/90: {}",
                            strerror(errno));
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
        bool wasRunning = m_entryThreadRunning.exchange(false);
        if (wasRunning) {
            void *res;
            pthread_join(thread_id, &res);
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
        auto start = std::chrono::steady_clock::now();
        while (m_inControllerPeriodic.load(std::memory_order_acquire)) {
            std::this_thread::yield();
            auto now = std::chrono::steady_clock::now();
            if (now - start > std::chrono::milliseconds(5)) {
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

    void Run() {
        struct pollfd item;
        item.fd = send_queue_.getFd();
        item.events = POLLIN;
        item.revents = 0;

        std::vector<pollfd> poll_items;
        poll_items.push_back(item);

         //TODO:: change to 200Hz for real motor communication.
        // Use a high-resolution timer for 2.5ms (400Hz) periodic execution
        auto next_wake = std::chrono::steady_clock::now();
        static constexpr auto PERIOD = std::chrono::microseconds(2500);  // 2.5ms = 400Hz

        while (m_entryThreadRunning) {
            // Calculate time until next periodic tick
            auto now = std::chrono::steady_clock::now();
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(next_wake - now);
            int timeout_ms = remaining.count() > 0 ? static_cast<int>(remaining.count()) : 0;

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

            // Execute controllerPeriodic() on schedule
            now = std::chrono::steady_clock::now();
            if (now >= next_wake && m_isEnabled && !m_pauseRequested.load(std::memory_order_acquire)) {
                m_inControllerPeriodic.store(true, std::memory_order_release);
                controllerPeriodic();
                m_inControllerPeriodic.store(false, std::memory_order_release);
                next_wake += PERIOD;
                // Catch up if we fell behind (avoid spiral)
                if (next_wake < now) {
                    next_wake = now + PERIOD;
                }
            }

            pthread_testcancel();
        }
    }
    //units::second_t m_dt = Constants::kControllerPeriod;  //5ms
    int m_dt = 5;
    std::atomic<bool> m_isEnabled{false};
    std::atomic<bool> m_entryThreadRunning{false};
    std::atomic<bool> m_pauseRequested{false};
    std::atomic<bool> m_inControllerPeriodic{false};
    pthread_t thread_id;
    Fifo<std::function<void()>> send_queue_;
};
