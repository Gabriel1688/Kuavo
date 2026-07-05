#pragma once

#include "common/Synchronization.h"

typedef unsigned int WPI_Handle;
/** An event handle. */
typedef WPI_Handle WPI_EventHandle;

#include <algorithm>
#include <mutex>
#include <vector>

struct EventVector {
    std::mutex mutex;
    std::vector<WPI_EventHandle> events;

    /**
     * Adds an event to the event vector.
     *
     * @param handle the event to add
     */
    void add(WPI_EventHandle handle) {
        std::scoped_lock lock{mutex};
        events.emplace_back(handle);
    }

    /**
     * Removes an event from the event vector.
     *
     * @param handle The event to remove
     */
    void remove(WPI_EventHandle handle) {
        std::scoped_lock lock{mutex};
        auto it = std::find_if(
                events.begin(), events.end(),
                [=](const WPI_EventHandle fromHandle) { return fromHandle == handle; });
        if (it != events.end()) {
            events.erase(it);
        }
    }

    /**
     * Wakes up all events in the event vector.
     */
    void wakeup() {
        std::scoped_lock lock{mutex};
        for (auto &&handle: events) {
            wpi::setEvent(handle);
        }
    }
};