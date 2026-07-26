#pragma once
/**
 * @file Logger.h
 * @brief Asynchronous MQTT binary logger drain thread.
 *
 * Reads `BatchLogRecord` entries from the Composer's SPSC ring buffer,
 * serializes each batch into a binary payload, and publishes it to
 * `robot/sensor/bin` via `MqttClient::publish_binary()`.
 *
 * Thread: SCHED_OTHER, 128 KiB stack, 1ms sleep floor.
 */

#include "../../include/mercury_shm.h"
#include "mqtt/MqttClient.h"

#include <atomic>
#include <cstdint>
#include <pthread.h>
#include <vector>

namespace mercury {

class Logger {
public:
    /**
     * @param ring       Process-local SPSC ring buffer written by Composer
     * @param mqtt       Application MqttClient instance
     * @param robot_id   Robot identifier for payload header
     * @param downsample_every  Publish 1 in every N drained batches (default 5)
     */
    Logger(SPSCRingBuffer<BatchLogRecord, LOG_RING_CAPACITY>& ring,
           MqttClient& mqtt,
           uint32_t robot_id = 1,
           size_t downsample_every = 5);

    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /** Start the logger drain thread. */
    void start();

    /** Signal stop and join the thread. */
    void shutdown();

private:
    static void* threadEntry(void* arg);
    void run();
    void drainOnce();

    static std::vector<uint8_t> serializeBatch(const BatchLogRecord& batch);

    SPSCRingBuffer<BatchLogRecord, LOG_RING_CAPACITY>& ring_;
    MqttClient& mqtt_;
    uint32_t robot_id_;

    // Thread state
    pthread_t thread_id_{};
    std::atomic<bool> running_{false};
    bool thread_created_{false};

    // Connect-delay guard
    bool connect_seen_{false};
    uint64_t connect_seen_ts_ns_{0};
    bool push_enabled_{false};

    // Downsampling control
    size_t downsample_every_ = 5;
    size_t drain_skip_counter_ = 0;

    // Backpressure warning (once)
    bool backpressure_warned_{false};
};

} // namespace mercury
