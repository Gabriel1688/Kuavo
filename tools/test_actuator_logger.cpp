/**
 * @file test_actuator_logger.cpp
 * @brief Combined Actuator + MQTT Logger for ARM edge device
 *
 * Thread 1: IMU writer (500Hz) → imu_stage [1]
 * Thread 2: Motor Group A (1kHz, joints 0-5) → motor_group_a_stage
 * Thread 3: Motor Group B (1kHz, joints 6-11) → motor_group_b_stage
 * Thread 4: Composer (1kHz) → composed_buffers + push to SPSC ring
 * Thread 5: MQTT Logger → drain SPSC ring → lws_service() → broker
 *
 * MQTT: libwebsockets, no TLS, port 1883, binary payload
 * Topics: robot/command/bin, robot/sensor/bin, robot/status
 *
 * Build (ARM):
 *   aarch64-linux-gnu-g++ -O2 -std=c++20 -pthread -lrt -lwebsockets \
 *       -o test_actuator_logger test_actuator_logger.cpp
 *
 * Usage:
 *   ./test_actuator_logger -broker 192.168.1.100 -port 1883 -dur 10
 */

#include "mercury_shm_v2.h"

#include <libwebsockets.h>
#include <arpa/inet.h>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <queue>

using namespace mercury;

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

// ============================================================
// Simulated Motor (same as previous test_actuator)
// ============================================================
struct SimMotor {
    double position = 0, velocity = 0, torque = 0;
    int mos_temp = 35, rotor_temp = 40;
    uint8_t status = 0x00;

    void enable() { status = 0x01; }   // [2]
    void disable() { status = 0x00; position = velocity = torque = 0; }

    // PD control simulation matching Damiao MIT mode [2]
    void simulate(double tp, double tv, double tt, double kp, double kd) {
        if (status != 0x01) return;
        double ct = kp * (tp - position) + kd * (tv - velocity) + tt;
        velocity += ((ct - 0.05 * velocity) / 0.1) * 0.001;
        position += velocity * 0.001;
        torque = ct;
        position = std::max(-P_MAX, std::min(position, P_MAX));
        velocity = std::max(-V_MAX, std::min(velocity, V_MAX));
        torque = std::max(-T_MAX, std::min(torque, T_MAX));
        mos_temp = 35 + static_cast<int>(std::abs(torque) * 2.0);
        rotor_temp = 40 + static_cast<int>(std::abs(torque) * 1.5);
    }
};

// ============================================================
// MQTT Send Queue — thread-safe queue between composer and lws thread
// ============================================================
struct MqttMessage {
    const char* topic;  // Static string pointer (no allocation)
    std::vector<uint8_t> payload;
};

class MqttSendQueue {
public:
    void push(MqttMessage&& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(std::move(msg));
    }

    bool pop(MqttMessage& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        msg = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

private:
    std::mutex mtx_;
    std::queue<MqttMessage> queue_;
};

// ============================================================
// Global State
// ============================================================
static SharedMemoryLayout* g_layout = nullptr;
static SPSCRingBuffer<LogRecord, LOG_RING_CAPACITY> g_log_ring;
static MqttSendQueue g_mqtt_queue;
static uint32_t g_robot_id = 1;
static uint64_t g_records_published = 0;
static uint64_t g_records_dropped = 0;

// ============================================================
// LWS MQTT Callback
// ============================================================

struct MqttClientState {
    struct lws* wsi = nullptr;
    bool connected = false;
    const char* broker_host = "localhost";
    int broker_port = 1883;
};

static MqttClientState g_mqtt_state;

static int lws_mqtt_callback(struct lws* wsi, enum lws_callback_reasons reason,
                              void* user, void* in, size_t len) {
    (void)user;

    switch (reason) {
    case LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED:
        lwsl_user("MQTT connected to broker\n");
        g_mqtt_state.connected = true;
        g_mqtt_state.wsi = wsi;
        lws_callback_on_writable(wsi);
        break;

    case LWS_CALLBACK_MQTT_CLIENT_WRITEABLE: {
        // Drain one message from the send queue per writeable callback
        MqttMessage msg;
        if (g_mqtt_queue.pop(msg)) {
            // Build MQTT PUBLISH
            lws_mqtt_publish_param_t pub;
            memset(&pub, 0, sizeof(pub));
            pub.topic = msg.topic;
            pub.topic_len = strlen(msg.topic);
            pub.payload_len = msg.payload.size();
            pub.qos = static_cast<lws_mqtt_qos_levels_t>(0);  // Fire-and-forget for lowest latency

            if (lws_mqtt_client_send_message(wsi, &pub,
                    msg.payload.data(), msg.payload.size()) == 0) {
                g_records_published++;
            }

            // Request another writeable callback if queue not empty
            if (g_mqtt_queue.size() > 0) {
                lws_callback_on_writable(wsi);
            }
        }
        break;
    }

    case LWS_CALLBACK_MQTT_CLIENT_RX:
        // We don't expect incoming messages in this design
        break;

    case LWS_CALLBACK_MQTT_CLIENT_CLOSED:
        lwsl_user("MQTT connection closed\n");
        g_mqtt_state.connected = false;
        g_mqtt_state.wsi = nullptr;
        break;

    default:
        break;
    }

    return 0;
}

static const struct lws_protocols mqtt_protocols[] = {
    {"mqtt", lws_mqtt_callback, 0, 4096},
    LWS_PROTOCOL_LIST_TERM
};

// ============================================================
// Helper: Serialize LogRecord to binary payload
// ============================================================

static std::vector<uint8_t> serialize_record(const LogRecord& record) {
    size_t payload_data_size;
    const void* payload_data_ptr;

    if (record.header.record_type == static_cast<uint8_t>(RecordType::COMMAND)) {
        payload_data_size = sizeof(Command);
        payload_data_ptr = &record.data.cmd;
    } else {
        payload_data_size = sizeof(SensorData);
        payload_data_ptr = &record.data.sensor;
    }

    size_t total = sizeof(BinaryPayloadHeader) + payload_data_size;
    std::vector<uint8_t> buf(total);

    // Copy header
    std::memcpy(buf.data(), &record.header, sizeof(BinaryPayloadHeader));
    // Copy payload
    std::memcpy(buf.data() + sizeof(BinaryPayloadHeader),
                payload_data_ptr, payload_data_size);

    return buf;
}

// ============================================================
// Thread Functions
// ============================================================

// Thread 1: IMU Writer (500Hz) [1]
static void imu_thread_fn() {
    const uint64_t period_ns = 2'000'000; // 2ms = 500Hz [1]
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t seq = 0;

    printf("  IMU thread started (500Hz)\n");

    while (g_running && !g_layout->emergency_stop.load(std::memory_order_acquire)) {
        ImuStageData imu{};
        double t = static_cast<double>(seq) / 500.0;
        imu.imu_ang_vel[0] = 0.1 * std::sin(2.0 * M_PI * t);
        imu.imu_ang_vel[1] = 0.05 * std::cos(2.0 * M_PI * 0.5 * t);
        imu.imu_ang_vel[2] = 0.02 * std::sin(2.0 * M_PI * 2.0 * t);
        imu.imu_acc[2] = 9.81;
        imu.sequence = seq;
        imu.timestamp_ns = get_monotonic_ns();
        g_layout->imu_stage.publish(imu);

        seq++;
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            ts.tv_sec = (next_wakeup - now) / 1'000'000'000ULL;
            ts.tv_nsec = (next_wakeup - now) % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }
    printf("  IMU thread stopped (%lu iterations)\n", seq);
}

// Thread 2/3: Motor Group Writer (1kHz)
static void motor_thread_fn(SourceDoubleBuffer<MotorGroupStageData>* stage,
                             int group_offset, const char* name) {
    const uint64_t period_ns = 1'000'000; // 1ms = 1kHz
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t seq = 0;
    SimMotor motors[MOTORS_PER_GROUP];

    printf("  %s thread started (1kHz, joints %d-%d)\n",
           name, group_offset, group_offset + MOTORS_PER_GROUP - 1);

    while (g_running && !g_layout->emergency_stop.load(std::memory_order_acquire)) {
        // Read command from shared memory
        uint32_t rb = g_layout->cmd_write_idx.load(std::memory_order_acquire);
        Command cmd;
        std::memcpy(&cmd, &g_layout->cmd_buffers[rb], sizeof(Command));

        // Simulate motors
        MotorGroupStageData grp{};
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            int gj = group_offset + j;
            if (cmd.enabled[gj] && motors[j].status != 0x01) motors[j].enable();
            else if (!cmd.enabled[gj] && motors[j].status == 0x01) motors[j].disable();

            motors[j].simulate(cmd.jpos_cmd[gj], cmd.jvel_cmd[gj],
                               cmd.jtorque_cmd[gj], cmd.kp[gj], cmd.kd[gj]);

            grp.joint_jpos[j] = motors[j].position;
            grp.joint_jvel[j] = motors[j].velocity;
            grp.motor_jpos[j] = motors[j].position;
            grp.motor_jvel[j] = motors[j].velocity;
            grp.jtorque[j] = motors[j].torque;
            grp.motor_status[j] = motors[j].status;
            grp.mos_temperature[j] = motors[j].mos_temp;
            grp.rotor_temperature[j] = motors[j].rotor_temp;
            grp.bus_voltage[j] = 48.0;
            grp.bus_current[j] = std::abs(motors[j].torque) * 0.1;
        }
        grp.sequence = seq;
        grp.timestamp_ns = get_monotonic_ns();
        stage->publish(grp);

        seq++;
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            ts.tv_sec = (next_wakeup - now) / 1'000'000'000ULL;
            ts.tv_nsec = (next_wakeup - now) % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }
    printf("  %s thread stopped (%lu iterations)\n", name, seq);
}

// Thread 4: Composer (1kHz) — merges 3 sources + pushes to SPSC ring
static void composer_thread_fn() {
    const uint64_t period_ns = 1'000'000;
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t seq = 0;

    printf("  Composer thread started (1kHz) — also pushes to log ring\n");

    while (g_running && !g_layout->emergency_stop.load(std::memory_order_acquire)) {
        SensorData snapshot{};

        // Read IMU
        ImuStageData imu = g_layout->imu_stage.read();
        std::memcpy(snapshot.imu_inc, imu.imu_inc, sizeof(imu.imu_inc));
        std::memcpy(snapshot.imu_ang_vel, imu.imu_ang_vel, sizeof(imu.imu_ang_vel));
        std::memcpy(snapshot.imu_acc, imu.imu_acc, sizeof(imu.imu_acc));
        snapshot.imu_timestamp_ns = imu.timestamp_ns;
        snapshot.imu_sequence = imu.sequence;

        // Read Motor Group A (joints 0-5)
        MotorGroupStageData grpA = g_layout->motor_group_a_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            snapshot.joint_jpos[j] = grpA.joint_jpos[j];
            snapshot.joint_jvel[j] = grpA.joint_jvel[j];
            snapshot.motor_jpos[j] = grpA.motor_jpos[j];
            snapshot.motor_jvel[j] = grpA.motor_jvel[j];
            snapshot.bus_current[j] = grpA.bus_current[j];
            snapshot.bus_voltage[j] = grpA.bus_voltage[j];
            snapshot.jtorque[j] = grpA.jtorque[j];
        }
        snapshot.motor_group_a_timestamp_ns = grpA.timestamp_ns;
        snapshot.motor_group_a_sequence = grpA.sequence;

        // Read Motor Group B (joints 6-11)
        MotorGroupStageData grpB = g_layout->motor_group_b_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            int idx = MOTORS_PER_GROUP + j;
            snapshot.joint_jpos[idx] = grpB.joint_jpos[j];
            snapshot.joint_jvel[idx] = grpB.joint_jvel[j];
            snapshot.motor_jpos[idx] = grpB.motor_jpos[j];
            snapshot.motor_jvel[idx] = grpB.motor_jvel[j];
            snapshot.bus_current[idx] = grpB.bus_current[j];
            snapshot.bus_voltage[idx] = grpB.bus_voltage[j];
            snapshot.jtorque[idx] = grpB.jtorque[j];
        }
        snapshot.motor_group_b_timestamp_ns = grpB.timestamp_ns;
        snapshot.motor_group_b_sequence = grpB.sequence;

        snapshot.compose_timestamp_ns = get_monotonic_ns();

        // Publish to shared memory (double buffer)
        uint32_t wb = 1 - g_layout->composed_write_idx.load(std::memory_order_acquire);
        std::memcpy(&g_layout->composed_buffers[wb], &snapshot, sizeof(SensorData));
        g_layout->composed_write_idx.store(wb, std::memory_order_release);
        g_layout->composed_sequence.fetch_add(1, std::memory_order_release);

        // Also read command for logging
        uint32_t crb = g_layout->cmd_write_idx.load(std::memory_order_acquire);
        Command cmd;
        std::memcpy(&cmd, &g_layout->cmd_buffers[crb], sizeof(Command));

        // Push sensor LogRecord to SPSC ring (process-local)
        LogRecord sensor_rec;
        sensor_rec.header.magic = PAYLOAD_MAGIC;
        sensor_rec.header.version = PAYLOAD_VERSION;
        sensor_rec.header.record_type = static_cast<uint8_t>(RecordType::SENSOR);
        sensor_rec.header.robot_id = g_robot_id;
        sensor_rec.header.payload_size = sizeof(SensorData);
        sensor_rec.header.sequence = seq;
        sensor_rec.header.timestamp_ns = snapshot.compose_timestamp_ns;
        sensor_rec.data.sensor = snapshot;
        if (!g_log_ring.push(sensor_rec)) g_records_dropped++;

        // Push command LogRecord
        LogRecord cmd_rec;
        cmd_rec.header.magic = PAYLOAD_MAGIC;
        cmd_rec.header.version = PAYLOAD_VERSION;
        cmd_rec.header.record_type = static_cast<uint8_t>(RecordType::COMMAND);
        cmd_rec.header.robot_id = g_robot_id;
        cmd_rec.header.payload_size = sizeof(Command);
        cmd_rec.header.sequence = seq;
        cmd_rec.header.timestamp_ns = cmd.timestamp_ns;
        cmd_rec.data.cmd = cmd;
        if (!g_log_ring.push(cmd_rec)) g_records_dropped++;

        seq++;
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            ts.tv_sec = (next_wakeup - now) / 1'000'000'000ULL;
            ts.tv_nsec = (next_wakeup - now) % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }
    printf("  Composer thread stopped (%lu iterations)\n", seq);
}

// Thread 5: MQTT Logger — drains SPSC ring via lws event loop
static void mqtt_logger_thread_fn(const char* broker_host, int broker_port) {
    printf("  MQTT Logger thread started (broker=%s:%d)\n", broker_host, broker_port);

    // Create lws context — no TLS
    struct lws_context_creation_info ctx_info;
    memset(&ctx_info, 0, sizeof(ctx_info));
    ctx_info.port = CONTEXT_PORT_NO_LISTEN;
    ctx_info.protocols = mqtt_protocols;
    ctx_info.options = 0;  // No SSL

    struct lws_context* ctx = lws_create_context(&ctx_info);
    if (!ctx) {
        fprintf(stderr, "Failed to create lws context\n");
        return;
    }

    // Connect to MQTT broker
    struct lws_client_connect_info conn_info;
    memset(&conn_info, 0, sizeof(conn_info));
    conn_info.context = ctx;
    conn_info.address = broker_host;
    conn_info.port = broker_port;
    conn_info.path = "/mqtt";
    conn_info.host = broker_host;
    conn_info.origin = broker_host;
    conn_info.protocol = "mqtt";
    conn_info.method = "MQTT";
    conn_info.ssl_connection = 0;  // No TLS

    // MQTT connect params
    lws_mqtt_client_connect_param_t mqtt_conn;
    memset(&mqtt_conn, 0, sizeof(mqtt_conn));
    mqtt_conn.client_id = "mercury_edge_logger";
    mqtt_conn.keep_alive = 30;
    mqtt_conn.clean_start = 1;

    // LWT (Last Will) on robot/status
    lws_mqtt_publish_param_t will;
    memset(&will, 0, sizeof(will));
    will.topic = MQTT_TOPIC_STATUS;
    will.topic_len = strlen(MQTT_TOPIC_STATUS);
    will.qos = static_cast<lws_mqtt_qos_levels_t>(1);
    will.retain = 1;
    static const char* will_payload = "{\"status\":\"offline\"}";
    will.payload_len = strlen(will_payload);
    mqtt_conn.will_param = will;
    mqtt_conn.will_msg = will_payload;
    mqtt_conn.will_msg_len = will.payload_len;

    conn_info.mqtt_cp = &mqtt_conn;

    struct lws* wsi = lws_client_connect_via_info(&conn_info);
    if (!wsi) {
        fprintf(stderr, "MQTT connection initiation failed\n");
        lws_context_destroy(ctx);
        return;
    }

    g_mqtt_state.broker_host = broker_host;
    g_mqtt_state.broker_port = broker_port;

    TimingStats drain_stats, publish_stats;
    uint64_t drain_cycles = 0;

    while (g_running) {
        // Drain SPSC ring buffer into MQTT send queue
        uint64_t drain_start = get_monotonic_ns();
        int drained = 0;
        LogRecord record;

        while (drained < 20 && g_log_ring.pop(record)) {
            const char* topic = (record.header.record_type ==
                                 static_cast<uint8_t>(RecordType::COMMAND))
                                ? MQTT_TOPIC_CMD
                                : MQTT_TOPIC_SENSOR;

            MqttMessage mqtt_msg;
            mqtt_msg.topic = topic;
            mqtt_msg.payload = serialize_record(record);
            g_mqtt_queue.push(std::move(mqtt_msg));
            drained++;
        }

        if (drained > 0) {
            drain_stats.record(get_monotonic_ns() - drain_start);
            drain_cycles++;
            // Request writeable callback to send queued messages
            if (g_mqtt_state.wsi) {
                lws_callback_on_writable(g_mqtt_state.wsi);
            }
        }

        // Service the lws event loop (non-blocking, 1ms timeout)
        uint64_t svc_start = get_monotonic_ns();
        lws_service(ctx, 1);
        publish_stats.record(get_monotonic_ns() - svc_start);

        // Periodic status
        if (drain_cycles > 0 && drain_cycles % 5000 == 0) {
            printf("  [MQTT] published=%lu  dropped=%lu  queue=%zu  ring=%zu\n",
                   g_records_published, g_records_dropped,
                   g_mqtt_queue.size(), g_log_ring.size());
        }
    }

    // Disconnect
    lws_context_destroy(ctx);

    printf("\n  MQTT Logger Report:\n");
    printf("    Records published: %lu\n", g_records_published);
    printf("    Records dropped:   %lu\n", g_records_dropped);
    drain_stats.print("Ring drain (per batch)");
    publish_stats.print("lws_service (per call)");
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    double duration = 10.0;
    const char* broker_host = "localhost";
    int broker_port = 1883;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-broker") == 0 && i + 1 < argc)
            broker_host = argv[++i];
        else if (strcmp(argv[i], "-port") == 0 && i + 1 < argc)
            broker_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-rid") == 0 && i + 1 < argc)
            g_robot_id = atoi(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-broker host] [-port 1883] [-dur sec] [-rid N]\n",
                   argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Attach to shared memory (created by controller)
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) { perror("shm_open (controller running?)"); return 1; }
    void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { perror("mmap"); return 1; }
    g_layout = static_cast<SharedMemoryLayout*>(ptr);
    if (g_layout->magic != SHM_MAGIC) {
        fprintf(stderr, "Invalid SHM magic\n"); return 1;
    }
    g_layout->robot_id = g_robot_id;

    printf("Actuator+Logger: %u joints, robot_id=%u, broker=%s:%d\n",
           g_layout->num_joints, g_robot_id, broker_host, broker_port);
    printf("Payload sizes: SensorData=%zu  Command=%zu  Header=%zu\n",
           sizeof(SensorData), sizeof(Command), sizeof(BinaryPayloadHeader));
    printf("Launching 5 threads...\n\n");

    // Launch threads
    std::thread t_imu(imu_thread_fn);
    std::thread t_grpA(motor_thread_fn,
                       &g_layout->motor_group_a_stage, 0, "Motor Grp A");
    std::thread t_grpB(motor_thread_fn,
                       &g_layout->motor_group_b_stage, 6, "Motor Grp B");
    std::thread t_composer(composer_thread_fn);
    std::thread t_mqtt(mqtt_logger_thread_fn, broker_host, broker_port);

    // Wait for duration
    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count() >= duration)
            g_running = false;
    }

    t_imu.join();
    t_grpA.join();
    t_grpB.join();
    t_composer.join();
    t_mqtt.join();

    munmap(g_layout, sizeof(SharedMemoryLayout));
    return 0;
}
