#include "logger/Logger.h"
#include "spdlog/spdlog.h"
#include <cstring>
#include <ctime>
#include <unistd.h>

namespace mercury {

static constexpr size_t DRAIN_BATCH_LIMIT = 10;
static constexpr uint64_t CONNECT_DELAY_NS = 20'000'000'000ULL; // 20 s
static constexpr uint64_t SLEEP_NS = 1'000'000ULL;              // 1 ms

Logger::Logger(SPSCRingBuffer<BatchLogRecord, LOG_RING_CAPACITY>& ring,
             MqttClient& mqtt,
             uint32_t robot_id,
             size_t downsample_every)
    : ring_(ring), mqtt_(mqtt), robot_id_(robot_id),
      downsample_every_(downsample_every == 0 ? 1 : downsample_every) {}

Logger::~Logger() {
    if (running_.load(std::memory_order_acquire) || thread_created_) {
        shutdown();
    }
}

void Logger::start() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }

    pthread_attr_t attr{};
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 128 * 1024);

    thread_created_ = (pthread_create(&thread_id_, &attr, &Logger::threadEntry, this) == 0);
    if (!thread_created_) {
        SPDLOG_ERROR("Failed to create MQTT Logger thread");
        running_.store(false, std::memory_order_release);
    } else {
        // Set thread name for debugging
        pthread_setname_np(thread_id_, "mqtt-logger");
        
        SPDLOG_INFO("MQTT Logger thread started");
    }
    pthread_attr_destroy(&attr);
}

void Logger::shutdown() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (thread_created_) {
        pthread_join(thread_id_, nullptr);
        thread_created_ = false;
        SPDLOG_INFO("MQTT Logger thread stopped");
    }
}

void* Logger::threadEntry(void* arg) {
    static_cast<Logger*>(arg)->run();
    return nullptr;
}

void Logger::run() {
    while (running_.load(std::memory_order_acquire)) {
        // Connect-delay guard
        if (!push_enabled_) {
            if (mqtt_.isConnected()) {
                if (!connect_seen_) {
                    connect_seen_ts_ns_ = get_monotonic_ns();
                    connect_seen_ = true;
                    SPDLOG_INFO("MQTT connected — waiting 20s before draining ring");
                }
                uint64_t elapsed = get_monotonic_ns() - connect_seen_ts_ns_;
                if (elapsed >= CONNECT_DELAY_NS) {
                    push_enabled_ = true;
                    SPDLOG_INFO("MQTT Logger: 20s elapsed, ring drain enabled");
                }
            }
        }

        if (push_enabled_) {
            drainOnce();
        }

        struct timespec ts;
        ts.tv_sec = SLEEP_NS / 1'000'000'000ULL;
        ts.tv_nsec = SLEEP_NS % 1'000'000'000ULL;
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
    }
}

void Logger::drainOnce() {
    for (size_t i = 0; i < DRAIN_BATCH_LIMIT; ++i) {
        BatchLogRecord batch;
        if (!ring_.pop(batch)) {
            break; // ring empty
        }

        const bool should_publish = (drain_skip_counter_ % downsample_every_ == 0);
        ++drain_skip_counter_;
        if (!should_publish) {
            continue;
        }

        // Update header fields before serialization
        batch.header.magic = PAYLOAD_MAGIC;
        batch.header.version = PAYLOAD_VERSION;
        batch.header.record_type = static_cast<uint8_t>(RecordType::SENSOR_BATCH);
        batch.header.robot_id = robot_id_;
        batch.header.payload_size = static_cast<uint32_t>(
            sizeof(uint32_t) * 2 + batch.sample_count * sizeof(SensorCommandPair));
        batch.header._reserved = 0;

        auto payload = serializeBatch(batch);
        if (!mqtt_.publish_binary(MQTT_TOPIC_SENSOR,
                                  payload.data(),
                                  payload.size(),
                                  0, false)) {
            if (!backpressure_warned_) {
                SPDLOG_WARN("MQTT Logger: MqttClient send queue full — dropping batch");
                backpressure_warned_ = true;
            }
        } else {
            backpressure_warned_ = false;
        }
    }
}

std::vector<uint8_t> Logger::serializeBatch(const BatchLogRecord& batch) {
    size_t pairs_size = batch.sample_count * sizeof(SensorCommandPair);
    size_t total = sizeof(BinaryPayloadHeader) + sizeof(uint32_t) * 2 + pairs_size;
    std::vector<uint8_t> buf(total);

    uint8_t* dst = buf.data();
    std::memcpy(dst, &batch.header, sizeof(BinaryPayloadHeader));
    dst += sizeof(BinaryPayloadHeader);
    std::memcpy(dst, &batch.sample_count, sizeof(uint32_t));
    dst += sizeof(uint32_t);
    uint32_t pad = 0;
    std::memcpy(dst, &pad, sizeof(uint32_t));
    dst += sizeof(uint32_t);
    std::memcpy(dst, batch.samples, pairs_size);

    return buf;
}

} // namespace mercury
