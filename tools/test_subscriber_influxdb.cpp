/**
 * @file test_subscriber_influxdb.cpp
 * @brief MQTT Subscriber → InfluxDB writer for x86 remote host
 *
 * Subscribes to robot/sensor/bin and robot/command/bin.
 * Deserializes binary payload using the same mercury_shm.hpp header.
 * Batch-writes data points to InfluxDB via HTTP line protocol.
 *
 * Build (x86):
 *   g++ -O2 -std=c++20 -pthread -lwebsockets -lcurl \
 *       -o test_subscriber_influxdb test_subscriber_influxdb.cpp
 *
 * Usage:
 *   ./test_subscriber_influxdb \
 *       -mqtt_broker localhost -mqtt_port 1883 \
 *       -influx_url http://localhost:8086 \
 *       -influx_bucket mercury_robot \
 *       -influx_org myorg \
 *       -influx_token mytoken
 */

#include "mercury_shm_v2.h"

#include <libwebsockets.h>
#include <curl/curl.h>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace mercury;

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

// ============================================================
// InfluxDB Configuration
// ============================================================
struct InfluxConfig {
    std::string url    = "http://localhost:8086";
    std::string bucket = "mercury_robot";
    std::string org    = "myorg";
    std::string token  = "";
    int batch_size     = 500;      // Points per batch write
    int flush_ms       = 500;      // Max ms between flushes
};

// ============================================================
// InfluxDB Line Protocol Writer
// ============================================================
class InfluxWriter {
public:
    InfluxWriter(const InfluxConfig& cfg) : cfg_(cfg) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
        write_url_ = cfg.url + "/api/v2/write?org=" + cfg.org +
                     "&bucket=" + cfg.bucket + "&precision=ns";
    }

    ~InfluxWriter() { curl_global_cleanup(); }

    // Add a sensor data point as InfluxDB line protocol
    void add_sensor_point(const BinaryPayloadHeader& hdr, const SensorData& s) {
        std::lock_guard<std::mutex> lock(mtx_);

        for (int j = 0; j < NUM_ACT_JOINT; j++) {
            // Measurement: robot_sensor, tag: joint_id
            std::ostringstream line;
            line << "robot_sensor"
                 << ",robot_id=" << hdr.robot_id
                 << ",joint=" << j
                 << " jpos=" << s.joint_jpos[j]
                 << ",jvel=" << s.joint_jvel[j]
                 << ",jtorque=" << s.jtorque[j]
                 << ",motor_jpos=" << s.motor_jpos[j]
                 << ",motor_jvel=" << s.motor_jvel[j]
                 << ",bus_current=" << s.bus_current[j]
                 << ",bus_voltage=" << s.bus_voltage[j]
                 << " " << hdr.timestamp_ns << "\n";
            buffer_ += line.str();
            point_count_++;
        }

        // IMU (single point per sample, not per joint)
        std::ostringstream imu_line;
        imu_line << "robot_imu"
                 << ",robot_id=" << hdr.robot_id
                 << " gx=" << s.imu_ang_vel[0]
                 << ",gy=" << s.imu_ang_vel[1]
                 << ",gz=" << s.imu_ang_vel[2]
                 << ",ax=" << s.imu_acc[0]
                 << ",ay=" << s.imu_acc[1]
                 << ",az=" << s.imu_acc[2]
                 << " " << s.imu_timestamp_ns << "\n";
        buffer_ += imu_line.str();
        point_count_++;

        // Contact sensors
        std::ostringstream contact_line;
        contact_line << "robot_contact"
                     << ",robot_id=" << hdr.robot_id
                     << " rfoot=" << (int)s.rfoot_contact
                     << ",lfoot=" << (int)s.lfoot_contact
                     << " " << hdr.timestamp_ns << "\n";
        buffer_ += contact_line.str();
        point_count_++;

        if (point_count_ >= static_cast<size_t>(cfg_.batch_size)) {
            flush_locked();
        }
    }

    // Add a command data point
    void add_command_point(const BinaryPayloadHeader& hdr, const Command& c) {
        std::lock_guard<std::mutex> lock(mtx_);

        for (int j = 0; j < NUM_ACT_JOINT; j++) {
            std::ostringstream line;
            line << "robot_command"
                 << ",robot_id=" << hdr.robot_id
                 << ",joint=" << j
                 << " jpos_cmd=" << c.jpos_cmd[j]
                 << ",jvel_cmd=" << c.jvel_cmd[j]
                 << ",jtorque_cmd=" << c.jtorque_cmd[j]
                 << ",kp=" << c.kp[j]
                 << ",kd=" << c.kd[j]
                 << " " << hdr.timestamp_ns << "\n";
            buffer_ += line.str();
            point_count_++;
        }

        if (point_count_ >= static_cast<size_t>(cfg_.batch_size)) {
            flush_locked();
        }
    }

    // Force flush remaining points
    void flush() {
        std::lock_guard<std::mutex> lock(mtx_);
        flush_locked();
    }

    uint64_t total_points() const { return total_written_; }
    uint64_t total_flushes() const { return flush_count_; }
    uint64_t total_errors() const { return error_count_; }

private:
    void flush_locked() {
        if (buffer_.empty()) return;

        CURL* curl = curl_easy_init();
        if (!curl) {
            error_count_++;
            buffer_.clear();
            point_count_ = 0;
            return;
        }

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: text/plain");
        if (!cfg_.token.empty()) {
            std::string auth = "Authorization: Token " + cfg_.token;
            headers = curl_slist_append(headers, auth.c_str());
        }

        curl_easy_setopt(curl, CURLOPT_URL, write_url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, buffer_.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, buffer_.size());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

        // Suppress response body output
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
            +[](void*, size_t s, size_t n, void*) -> size_t { return s * n; });

        CURLcode res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            if (error_count_ % 100 == 0) {
                fprintf(stderr, "  InfluxDB write failed: %s (errors=%lu)\n",
                        curl_easy_strerror(res), error_count_);
            }
            error_count_++;
        } else {
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
            if (http_code != 204) {
                fprintf(stderr, "  InfluxDB HTTP %ld\n", http_code);
                error_count_++;
            } else {
                total_written_ += point_count_;
            }
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        buffer_.clear();
        point_count_ = 0;
        flush_count_++;
    }

    InfluxConfig cfg_;
    std::string write_url_;
    std::mutex mtx_;
    std::string buffer_;
    size_t point_count_ = 0;
    uint64_t total_written_ = 0;
    uint64_t flush_count_ = 0;
    uint64_t error_count_ = 0;
};

// ============================================================
// LWS MQTT Subscriber Callback
// ============================================================

static InfluxWriter* g_influx = nullptr;
static uint64_t g_msg_count = 0;
static uint64_t g_sensor_count = 0;
static uint64_t g_cmd_count = 0;
static uint64_t g_invalid_count = 0;
static TimingStats g_deserialize_stats;

static void process_binary_payload(const uint8_t* payload, size_t len) {
    uint64_t start = get_monotonic_ns();

    // Validate minimum size
    if (len < sizeof(BinaryPayloadHeader)) {
        g_invalid_count++;
        return;
    }

    // Deserialize header
    BinaryPayloadHeader hdr;
    std::memcpy(&hdr, payload, sizeof(BinaryPayloadHeader));

    // Validate magic
    if (hdr.magic != PAYLOAD_MAGIC) {
        g_invalid_count++;
        return;
    }

    // Validate version
    if (hdr.version != PAYLOAD_VERSION) {
        g_invalid_count++;
        return;
    }

    const uint8_t* data_ptr = payload + sizeof(BinaryPayloadHeader);
    size_t data_len = len - sizeof(BinaryPayloadHeader);

    if (hdr.record_type == static_cast<uint8_t>(RecordType::SENSOR)) {
        // Validate payload size matches our sizeof(SensorData)
        if (hdr.payload_size != sizeof(SensorData) || data_len < sizeof(SensorData)) {
            fprintf(stderr, "  SensorData size mismatch: hdr=%u local=%zu\n",
                    hdr.payload_size, sizeof(SensorData));
            g_invalid_count++;
            return;
        }

        SensorData sensor;
        std::memcpy(&sensor, data_ptr, sizeof(SensorData));
        g_influx->add_sensor_point(hdr, sensor);
        g_sensor_count++;

    } else if (hdr.record_type == static_cast<uint8_t>(RecordType::COMMAND)) {
        if (hdr.payload_size != sizeof(Command) || data_len < sizeof(Command)) {
            fprintf(stderr, "  Command size mismatch: hdr=%u local=%zu\n",
                    hdr.payload_size, sizeof(Command));
            g_invalid_count++;
            return;
        }

        Command cmd;
        std::memcpy(&cmd, data_ptr, sizeof(Command));
        g_influx->add_command_point(hdr, cmd);
        g_cmd_count++;
    }

    g_msg_count++;
    g_deserialize_stats.record(get_monotonic_ns() - start);
}

static int lws_mqtt_sub_callback(struct lws* wsi, enum lws_callback_reasons reason,
                                  void* user, void* in, size_t len) {
    (void)user;

    switch (reason) {
    case LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED: {
        lwsl_user("Subscriber connected to MQTT broker\n");

        // Subscribe to both binary topics
        lws_mqtt_subscribe_param_t sub;
        memset(&sub, 0, sizeof(sub));
        sub.num_topics = 2;

        lws_mqtt_topic_elem_t topics[2];
        memset(topics, 0, sizeof(topics));
        topics[0].name = MQTT_TOPIC_SENSOR;
        topics[0].qos = static_cast<lws_mqtt_qos_levels_t>(0);
        topics[1].name = MQTT_TOPIC_CMD;
        topics[1].qos = static_cast<lws_mqtt_qos_levels_t>(0);
        sub.topic = topics;

        lws_mqtt_client_send_subcribe(wsi, &sub);
        lwsl_user("Subscribed to %s and %s\n", MQTT_TOPIC_SENSOR, MQTT_TOPIC_CMD);
        break;
    }

    case LWS_CALLBACK_MQTT_CLIENT_RX:
        if (in && len > 0) {
            process_binary_payload(static_cast<const uint8_t*>(in), len);
        }
        break;

    case LWS_CALLBACK_MQTT_CLIENT_CLOSED:
        lwsl_user("Subscriber disconnected\n");
        break;

    default:
        break;
    }

    return 0;
}

static const struct lws_protocols sub_protocols[] = {
    {"mqtt", lws_mqtt_sub_callback, 0, 65536},
    LWS_PROTOCOL_LIST_TERM
};

// ============================================================
// Periodic Flush Thread — flushes InfluxDB buffer on timer
// ============================================================
static void flush_thread_fn(int flush_ms) {
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(flush_ms));
        g_influx->flush();
    }
    g_influx->flush(); // Final flush
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    const char* mqtt_broker = "localhost";
    int mqtt_port = 1883;
    InfluxConfig influx_cfg;
    double duration = 0; // 0 = forever

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-mqtt_broker") == 0 && i + 1 < argc)
            mqtt_broker = argv[++i];
        else if (strcmp(argv[i], "-mqtt_port") == 0 && i + 1 < argc)
            mqtt_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "-influx_url") == 0 && i + 1 < argc)
            influx_cfg.url = argv[++i];
        else if (strcmp(argv[i], "-influx_bucket") == 0 && i + 1 < argc)
            influx_cfg.bucket = argv[++i];
        else if (strcmp(argv[i], "-influx_org") == 0 && i + 1 < argc)
            influx_cfg.org = argv[++i];
        else if (strcmp(argv[i], "-influx_token") == 0 && i + 1 < argc)
            influx_cfg.token = argv[++i];
        else if (strcmp(argv[i], "-batch") == 0 && i + 1 < argc)
            influx_cfg.batch_size = atoi(argv[++i]);
        else if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n"
                   "  -mqtt_broker HOST    (default: localhost)\n"
                   "  -mqtt_port PORT      (default: 1883)\n"
                   "  -influx_url URL      (default: http://localhost:8086)\n"
                   "  -influx_bucket NAME  (default: mercury_robot)\n"
                   "  -influx_org ORG      (default: myorg)\n"
                   "  -influx_token TOKEN\n"
                   "  -batch N             (default: 500)\n"
                   "  -dur SECONDS         (default: 0=forever)\n",
                   argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("Mercury MQTT Subscriber → InfluxDB\n");
    printf("  MQTT:    %s:%d\n", mqtt_broker, mqtt_port);
    printf("  InfluxDB: %s  bucket=%s  org=%s\n",
           influx_cfg.url.c_str(), influx_cfg.bucket.c_str(),
           influx_cfg.org.c_str());
    printf("  Batch:   %d points\n", influx_cfg.batch_size);
    printf("  Payload: SensorData=%zu bytes  Command=%zu bytes  Header=%zu bytes\n",
           sizeof(SensorData), sizeof(Command), sizeof(BinaryPayloadHeader));
    printf("\n");

    // Initialize InfluxDB writer
    InfluxWriter influx(influx_cfg);
    g_influx = &influx;

    // Start periodic flush thread
    std::thread flush_t(flush_thread_fn, influx_cfg.flush_ms);

    // Create lws context — no TLS
    struct lws_context_creation_info ctx_info;
    memset(&ctx_info, 0, sizeof(ctx_info));
    ctx_info.port = CONTEXT_PORT_NO_LISTEN;
    ctx_info.protocols = sub_protocols;

    struct lws_context* ctx = lws_create_context(&ctx_info);
    if (!ctx) {
        fprintf(stderr, "Failed to create lws context\n");
        return 1;
    }

    // Connect to MQTT broker
    struct lws_client_connect_info conn_info;
    memset(&conn_info, 0, sizeof(conn_info));
    conn_info.context = ctx;
    conn_info.address = mqtt_broker;
    conn_info.port = mqtt_port;
    conn_info.path = "/mqtt";
    conn_info.host = mqtt_broker;
    conn_info.origin = mqtt_broker;
    conn_info.protocol = "mqtt";
    conn_info.method = "MQTT";
    conn_info.ssl_connection = 0;

    lws_mqtt_client_connect_param_t mqtt_conn;
    memset(&mqtt_conn, 0, sizeof(mqtt_conn));
    mqtt_conn.client_id = "mercury_influx_subscriber";
    mqtt_conn.keep_alive = 30;
    mqtt_conn.clean_start = 1;
    conn_info.mqtt_cp = &mqtt_conn;

    struct lws* wsi = lws_client_connect_via_info(&conn_info);
    if (!wsi) {
        fprintf(stderr, "MQTT connection failed\n");
        lws_context_destroy(ctx);
        return 1;
    }

    printf("Connected. Waiting for data...\n\n");

    // Main event loop
    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        lws_service(ctx, 50); // 50ms timeout

        if (duration > 0) {
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration<double>(elapsed).count() >= duration)
                g_running = false;
        }

        // Periodic status
        if (g_msg_count > 0 && g_msg_count % 5000 == 0) {
            printf("  [msgs=%lu  sensor=%lu  cmd=%lu  invalid=%lu  "
                   "influx_pts=%lu  flushes=%lu  errors=%lu]\n",
                   g_msg_count, g_sensor_count, g_cmd_count,
                   g_invalid_count, influx.total_points(),
                   influx.total_flushes(), influx.total_errors());
        }
    }

    g_running = false;
    flush_t.join();
    influx.flush(); // Final flush
    lws_context_destroy(ctx);

    printf("\n============================================================\n");
    printf("  SUBSCRIBER REPORT\n");
    printf("============================================================\n");
    printf("  MQTT messages:       %lu\n", g_msg_count);
    printf("    Sensor records:    %lu\n", g_sensor_count);
    printf("    Command records:   %lu\n", g_cmd_count);
    printf("    Invalid/rejected:  %lu\n", g_invalid_count);
    printf("  InfluxDB points:     %lu\n", influx.total_points());
    printf("  InfluxDB flushes:    %lu\n", influx.total_flushes());
    printf("  InfluxDB errors:     %lu\n", influx.total_errors());
    printf("\n");
    g_deserialize_stats.print("Binary deserialize (per msg)");
    printf("============================================================\n");

    return 0;
}
/*
Cross-Platform Validation
The #pragma pack(push, 1) directive in mercury_shm.hpp ensures identical struct layout on ARM64 and x86-64. The sizeof assertions catch mismatches at compile time. The BinaryPayloadHeader includes payload_size so the subscriber can validate at runtime that the publisher's sizeof(SensorData) matches its own — if they differ (e.g., due to a version mismatch), the message is rejected and counted in g_invalid_count rather than silently deserializing garbage .

The binary payload format follows the same raw-bytes-over-the-wire philosophy used by the Damiao motor CAN-over-UDP protocol, where 13-byte frames are sent as-is without any serialization framework . The feedback frame decoding on the subscriber side uses std::memcpy for type-punning, matching the uint8s_to_float pattern used by the openarm library .

## Build & Run

```bash
# ============================================================
# ARM Edge Device (aarch64)
# ============================================================
# Install dependencies
sudo apt install libwebsockets-dev

# Build actuator + logger
aarch64-linux-gnu-g++ -O2 -std=c++20 -pthread -lrt -lwebsockets \
    -o test_actuator_logger test_actuator_logger.cpp

# Build controller (same as before, no MQTT dependency)
aarch64-linux-gnu-g++ -O2 -std=c++20 -pthread -lrt \
    -o test_controller test_controller.cpp

# Run
./test_controller -freq 1000 -dur 30 -log &
./test_actuator_logger -broker 192.168.1.100 -port 1883 -dur 30

# ============================================================
# x86 Remote Host
# ============================================================
# Install dependencies
sudo apt install libwebsockets-dev libcurl4-openssl-dev

# Install InfluxDB 2.x
# https://docs.influxdata.com/influxdb/v2/install/

# Build subscriber
g++ -O2 -std=c++20 -pthread -lwebsockets -lcurl \
    -o test_subscriber_influxdb test_subscriber_influxdb.cpp

# Create InfluxDB bucket
influx bucket create -n mercury_robot -o myorg -r 7d

# Run subscriber
./test_subscriber_influxdb \
    -mqtt_broker localhost -mqtt_port 1883 \
    -influx_url http://localhost:8086 \
    -influx_bucket mercury_robot \
    -influx_org myorg \
    -influx_token <your-influxdb-token>
*/