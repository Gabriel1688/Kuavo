#include "Synchronization.h"
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <unordered_map>

static std::atomic_bool gShutdown{false};

namespace {

    struct State {
        int signaled{0};
        bool autoReset{false};
        std::vector<std::condition_variable *> waiters;
    };

    struct HandleManager {
        ~HandleManager() { gShutdown = true; }

        std::mutex mutex;
        std::vector<int> eventIds;
        std::vector<int> semaphoreIds;
        std::unordered_map<WPI_Handle, State> states;
    };

}// namespace

static HandleManager &GetManager() {
    static HandleManager manager;
    return manager;
}

WPI_EventHandle wpi::createEvent(bool manualReset, bool initialState) {
    auto &manager = GetManager();
    if (gShutdown) {
        return {};
    }
    std::scoped_lock lock{manager.mutex};

    auto index = manager.eventIds.emplace_back(0);
    WPI_EventHandle handle = (kHandleTypeEvent << 24) | (index & 0xffffff);

    // configure state data
    auto &state = manager.states[handle];
    state.signaled = initialState ? 1 : 0;
    state.autoReset = !manualReset;

    return handle;
}

void wpi::destroyEvent(WPI_EventHandle handle) {
    if ((handle >> 24) != kHandleTypeEvent) {
        return;
    }

    destroySignalObject(handle);

    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};

    std::vector<int>::iterator position = std::find(manager.eventIds.begin(), manager.eventIds.end(), handle);
    if (position != manager.eventIds.end()) {
        manager.eventIds.erase(position);
    }
}

void wpi::setEvent(WPI_EventHandle handle) {
    if ((handle >> 24) != kHandleTypeEvent) {
        return;
    }

    setSignalObject(handle);
}

void wpi::resetEvent(WPI_EventHandle handle) {
    if ((handle >> 24) != kHandleTypeEvent) {
        return;
    }

    resetSignalObject(handle);
}

WPI_SemaphoreHandle wpi::createSemaphore(int initialCount, int maximumCount) {
    auto &manager = GetManager();
    if (gShutdown) {
        return {};
    }
    std::scoped_lock lock{manager.mutex};

    auto index = manager.semaphoreIds.emplace_back(maximumCount);
    WPI_EventHandle handle = (kHandleTypeSemaphore << 24) | (index & 0xffffff);

    // configure state data
    auto &state = manager.states[handle];
    state.signaled = initialCount;
    state.autoReset = true;

    return handle;
}

void wpi::destroySemaphore(WPI_SemaphoreHandle handle) {
    if ((handle >> 24) != kHandleTypeSemaphore) {
        return;
    }

    destroySignalObject(handle);

    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};
    std::vector<int>::iterator position = std::find(manager.eventIds.begin(), manager.eventIds.end(), handle);
    if (position != manager.eventIds.end()) {
        manager.eventIds.erase(position);
    }
}

bool wpi::releaseSemaphore(WPI_SemaphoreHandle handle, int releaseCount,
                           int *prevCount) {
    if ((handle >> 24) != kHandleTypeSemaphore) {
        return false;
    }
    if (releaseCount <= 0) {
        return false;
    }
    int index = handle & 0xffffff;

    auto &manager = GetManager();
    if (gShutdown) {
        return true;
    }
    std::scoped_lock lock{manager.mutex};
    auto it = manager.states.find(handle);
    if (it == manager.states.end()) {
        return false;
    }
    auto &state = it->second;
    int maxCount = manager.eventIds[index];
    if (prevCount) {
        *prevCount = state.signaled;
    }
    if ((maxCount - state.signaled) < releaseCount) {
        return false;
    }
    state.signaled += releaseCount;
    for (auto &waiter: state.waiters) {
        waiter->notify_all();
    }
    return true;
}

bool wpi::waitForObject(WPI_Handle handle) {
    return waitForObject(handle, -1, nullptr);
}

bool wpi::waitForObject(WPI_Handle handle, double timeout, bool *timedOut) {
    WPI_Handle signaledValue;
    auto signaled = waitForObjects(
            std::span(&handle, 1), std::span(&signaledValue, 1), timeout, timedOut);
    if (signaled.empty()) {
        return false;
    }
    return (signaled[0] & 0x80000000ul) == 0;
}

std::span<WPI_Handle> wpi::waitForObjects(std::span<const WPI_Handle> handles,
                                          std::span<WPI_Handle> signaled) {
    return waitForObjects(handles, signaled, -1, nullptr);
}

std::span<WPI_Handle> wpi::waitForObjects(std::span<const WPI_Handle> handles,
                                          std::span<WPI_Handle> signaled,
                                          double timeout, bool *timedOut) {
    auto &manager = GetManager();
    if (gShutdown) {
        if (timedOut) *timedOut = false;
        return {};
    }
    std::unique_lock lock{manager.mutex};
    std::condition_variable cv;
    bool addedWaiters = false;
    bool timedOutVal = false;
    size_t count = 0;

    for (;;) {
        for (auto handle: handles) {
            auto it = manager.states.find(handle);
            if (it == manager.states.end()) {
                if (count < signaled.size()) {
                    // treat a non-existent handle as signaled, but set the error bit
                    signaled[count++] = handle | 0x80000000ul;
                }
            } else {
                auto &state = it->second;
                if (state.signaled > 0) {
                    if (count < signaled.size()) {
                        signaled[count++] = handle;
                    }
                    if (state.autoReset) {
                        --state.signaled;
                        if (state.signaled < 0) {
                            state.signaled = 0;
                        }
                    }
                }
            }
        }

        if (timedOutVal || count != 0) {
            break;
        }

        if (timeout == 0) {
            timedOutVal = true;
            break;
        }

        if (!addedWaiters) {
            addedWaiters = true;
            for (auto handle: handles) {
                auto &state = manager.states[handle];
                state.waiters.emplace_back(&cv);
            }
        }

        if (timeout < 0) {
            cv.wait(lock);
        } else {
            auto timeoutTime = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
            if (cv.wait_until(lock, timeoutTime) == std::cv_status::timeout) {
                timedOutVal = true;
            }
        }
    }

    if (addedWaiters) {
        for (auto handle: handles) {
            auto &state = manager.states[handle];
            auto it = std::find(state.waiters.begin(), state.waiters.end(), &cv);
            if (it != state.waiters.end()) {
                state.waiters.erase(it);
            }
        }
    }

    if (timedOut) {
        *timedOut = timedOutVal;
    }

    return signaled.subspan(0, count);
}

void wpi::createSignalObject(WPI_Handle handle, bool manualReset,
                             bool initialState) {
    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};
    auto &state = manager.states[handle];
    state.signaled = initialState ? 1 : 0;
    state.autoReset = !manualReset;
}

void wpi::setSignalObject(WPI_Handle handle) {
    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};
    auto it = manager.states.find(handle);
    if (it == manager.states.end()) {
        return;
    }
    auto &state = it->second;
    state.signaled = 1;
    for (auto &waiter: state.waiters) {
        waiter->notify_all();
        if (state.autoReset) {
            // expect the first waiter to reset it
            break;
        }
    }
}

void wpi::resetSignalObject(WPI_Handle handle) {
    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};
    auto it = manager.states.find(handle);
    if (it != manager.states.end()) {
        it->second.signaled = 0;
    }
}

void wpi::destroySignalObject(WPI_Handle handle) {
    auto &manager = GetManager();
    if (gShutdown) {
        return;
    }
    std::scoped_lock lock{manager.mutex};

    auto it = manager.states.find(handle);
    if (it != manager.states.end()) {
        // wake up any waiters
        for (auto &waiter: it->second.waiters) {
            waiter->notify_all();
        }
        manager.states.erase(it);
    }
}

extern "C" {

WPI_EventHandle WPI_CreateEvent(int manual_reset, int initial_state) {
    return wpi::createEvent(manual_reset != 0, initial_state != 0);
}

void WPI_DestroyEvent(WPI_EventHandle handle) {
    wpi::destroyEvent(handle);
}

void WPI_SetEvent(WPI_EventHandle handle) {
    wpi::setEvent(handle);
}

void WPI_ResetEvent(WPI_EventHandle handle) {
    wpi::resetEvent(handle);
}

WPI_SemaphoreHandle WPI_CreateSemaphore(int initial_count, int maximum_count) {
    return wpi::createSemaphore(initial_count, maximum_count);
}

void WPI_DestroySemaphore(WPI_SemaphoreHandle handle) {
    wpi::destroySemaphore(handle);
}

int WPI_ReleaseSemaphore(WPI_SemaphoreHandle handle, int release_count,
                         int *prev_count) {
    return wpi::releaseSemaphore(handle, release_count, prev_count);
}

int WPI_WaitForObject(WPI_Handle handle) {
    return wpi::waitForObject(handle);
}

int WPI_WaitForObjectTimeout(WPI_Handle handle, double timeout,
                             int *timed_out) {
    bool timedOutBool;
    int rv = wpi::waitForObject(handle, timeout, &timedOutBool);
    *timed_out = timedOutBool ? 1 : 0;
    return rv;
}

int WPI_WaitForObjects(const WPI_Handle *handles, int handles_count,
                       WPI_Handle *signaled) {
    return wpi::waitForObjects(std::span(handles, handles_count),
                               std::span(signaled, handles_count))
            .size();
}

int WPI_WaitForObjectsTimeout(const WPI_Handle *handles, int handles_count,
                              WPI_Handle *signaled, double timeout,
                              int *timed_out) {
    bool timedOutBool;
    auto signaledResult = wpi::waitForObjects(std::span(handles, handles_count),
                                              std::span(signaled, handles_count),
                                              timeout, &timedOutBool);
    *timed_out = timedOutBool ? 1 : 0;
    return signaledResult.size();
}

void WPI_CreateSignalObject(WPI_Handle handle, int manual_reset,
                            int initial_state) {
    wpi::createSignalObject(handle, manual_reset, initial_state);
}

void WPI_SetSignalObject(WPI_Handle handle) {
    wpi::setSignalObject(handle);
}

void WPI_ResetSignalObject(WPI_Handle handle) {
    wpi::resetSignalObject(handle);
}

void WPI_DestroySignalObject(WPI_Handle handle) {
    wpi::destroySignalObject(handle);
}

}// extern "C"
