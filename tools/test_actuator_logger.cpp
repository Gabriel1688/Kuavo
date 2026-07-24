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

#include "../include/mercury_shm.h"

#include <arpa/inet.h>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <libwebsockets.h>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <sys/mman.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

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
static SPSCRingBuffer<BatchLogRecord, 256> g_log_ring;  // 256 batches * 20 samples = 5120 samples buffered
static MqttSendQueue g_mqtt_queue;
static uint32_t g_robot_id = 1;
static uint64_t g_records_published = 0;
static uint64_t g_records_dropped = 0;
static uint64_t g_samples_published = 0;

// ============================================================
// LWS MQTT Callback
// ============================================================

struct MqttClientState {
    struct lws* wsi = nullptr;
    std::atomic<bool> connected{false};  // Atomic — written by lws thread, read by composer
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
        g_mqtt_state.connected.store(true, std::memory_order_release);
        g_mqtt_state.wsi = wsi;
        lws_callback_on_writable(wsi);
        break;

    case LWS_CALLBACK_MQTT_CLIENT_WRITEABLE: {
        // Drain one message from the send queue per writeable callback
        // (lws enforces one lws_write per WRITEABLE callback)
        MqttMessage msg;
        if (g_mqtt_queue.pop(msg)) {
            lws_mqtt_publish_param_t pub;
            memset(&pub, 0, sizeof(pub));
            pub.topic = const_cast<char*>(msg.topic);
            pub.topic_len = static_cast<uint16_t>(strlen(msg.topic));
            pub.payload = msg.payload.data();
            pub.payload_len = static_cast<uint32_t>(msg.payload.size());
            pub.payload_pos = 0;
            pub.qos = static_cast<lws_mqtt_qos_levels_t>(0);  // Fire-and-forget for lowest latency

            if (lws_mqtt_client_send_publish(wsi, &pub,
                    msg.payload.data(), pub.payload_len, LWS_MQTT_FINAL_PART) >= 0) {
                g_records_published++;
            }

            // Always request another writeable callback — lws will suppress if
            // the socket is not ready yet
            lws_callback_on_writable(wsi);
        }
        break;
    }

    case LWS_CALLBACK_MQTT_CLIENT_RX:
        // We don't expect incoming messages in this design
        break;

    case LWS_CALLBACK_MQTT_CLIENT_CLOSED:
        lwsl_user("MQTT connection closed\n");
        g_mqtt_state.connected.store(false, std::memory_order_release);
        g_mqtt_state.wsi = nullptr;
        break;

    default:
        break;
    }

    return 0;
}

static const struct lws_protocols mqtt_protocols[] = {
    {"mqtt", lws_mqtt_callback, 0, 32768},
    LWS_PROTOCOL_LIST_TERM
};

// ============================================================
// Helper: Serialize BatchLogRecord to binary payload
// Layout: BinaryPayloadHeader | sample_count(u32) | pad(u32) | SensorCommandPair[N]
// ============================================================

static std::vector<uint8_t> serialize_batch(const BatchLogRecord& batch) {
    // Only serialize the valid samples, not the full BATCH_SIZE array
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

// Thread 2/3: Motor Group Writer — frequency from SHM control_freq_hz
static void motor_thread_fn(SourceDoubleBuffer<MotorGroupStageData>* stage,
                             int group_offset, const char* name) {
    uint32_t freq = g_layout->control_freq_hz;
    if (freq == 0) freq = 1000;  // fallback
    const uint64_t period_ns = 1'000'000'000ULL / freq;
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t seq = 0;
    SimMotor motors[MOTORS_PER_GROUP];

    printf("  %s thread started (%u Hz, joints %d-%d)\n",
           name, freq, group_offset, group_offset + MOTORS_PER_GROUP - 1);

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

// Thread 4: Composer — frequency from SHM, batches N samples, pushes to SPSC ring
static void composer_thread_fn() {
    uint32_t freq = g_layout->control_freq_hz;
    if (freq == 0) freq = 1000;  // fallback
    const uint64_t period_ns = 1'000'000'000ULL / freq;
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t seq = 0;
    uint64_t batch_seq = 0;

    // Batch accumulator — local to composer thread
    BatchLogRecord batch{};
    uint32_t batch_idx = 0;

    // Connect-delay state: wait 20s after MQTT connects before pushing
    uint64_t connect_seen_ts = 0;
    bool push_enabled = false;

    printf("  Composer thread started (%u Hz) — batching %zu samples per MQTT message\n",
           freq, BATCH_SIZE);

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

        // Publish to shared memory (double buffer) — always, regardless of MQTT state
        uint32_t wb = 1 - g_layout->composed_write_idx.load(std::memory_order_acquire);
        std::memcpy(&g_layout->composed_buffers[wb], &snapshot, sizeof(SensorData));
        g_layout->composed_write_idx.store(wb, std::memory_order_release);
        g_layout->composed_sequence.fetch_add(1, std::memory_order_release);

        // Also read command for logging
        uint32_t crb = g_layout->cmd_write_idx.load(std::memory_order_acquire);
        Command cmd;
        std::memcpy(&cmd, &g_layout->cmd_buffers[crb], sizeof(Command));

        // Debug: report new controller command sequence
        static uint64_t s_last_cmd_seq = UINT64_MAX;
        if (cmd.sequence != s_last_cmd_seq) {
            s_last_cmd_seq = cmd.sequence;
            // printf("  [Composer] new command from controller: seq=%lu  ts=%lu  jpos_cmd[0]=%.4f  enabled[0]=%u\n",
            //        static_cast<unsigned long>(cmd.sequence),
            //        static_cast<unsigned long>(cmd.timestamp_ns),
            //        cmd.jpos_cmd[0],
            //        static_cast<unsigned>(cmd.enabled[0]));
        }

        // ---- Connect guard: only accumulate for MQTT when connected ----
        // Wait 20 seconds after connected becomes true before pushing data,
        // to let lws_service finish its initial handshake stall
        if (!push_enabled) {
            if (g_mqtt_state.connected.load(std::memory_order_acquire)) {
                if (connect_seen_ts == 0) {
                    connect_seen_ts = get_monotonic_ns();
                    printf("  [Composer] MQTT connected, waiting 20s before pushing data...\n");
                }
                uint64_t elapsed_ns = get_monotonic_ns() - connect_seen_ts;
                if (elapsed_ns >= 20'000'000'000ULL) {
                    push_enabled = true;
                    printf("  [Composer] 20s elapsed, starting to push data to ring\n");
                }
            }
        }

        if (push_enabled) {
            // Accumulate sensor+command pair into batch
            batch.samples[batch_idx].sensor = snapshot;
            batch.samples[batch_idx].cmd = cmd;
            batch_idx++;

            // Flush batch when full
            if (batch_idx >= BATCH_SIZE) {
                batch.header.magic = PAYLOAD_MAGIC;
                batch.header.version = PAYLOAD_VERSION;
                batch.header.record_type = static_cast<uint8_t>(RecordType::SENSOR_BATCH);
                batch.header.robot_id = g_robot_id;
                batch.header.payload_size = static_cast<uint32_t>(
                    sizeof(uint32_t) * 2 + batch_idx * sizeof(SensorCommandPair));
                batch.header.sequence = batch_seq++;
                batch.header.timestamp_ns = snapshot.compose_timestamp_ns;
                batch.sample_count = batch_idx;

                if (!g_log_ring.push(batch))
                    g_records_dropped++;

                batch_idx = 0;
            }
        }

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

    // Flush any remaining samples in the partial batch
    if (batch_idx > 0 && push_enabled) {
        batch.header.magic = PAYLOAD_MAGIC;
        batch.header.version = PAYLOAD_VERSION;
        batch.header.record_type = static_cast<uint8_t>(RecordType::SENSOR_BATCH);
        batch.header.robot_id = g_robot_id;
        batch.header.payload_size = static_cast<uint32_t>(
            sizeof(uint32_t) * 2 + batch_idx * sizeof(SensorCommandPair));
        batch.header.sequence = batch_seq++;
        batch.header.timestamp_ns = get_monotonic_ns();
        batch.sample_count = batch_idx;
        g_log_ring.push(batch);
    }

    printf("  Composer thread stopped (%lu iterations, %lu batches)\n", seq, batch_seq);
}

// Thread 5: MQTT Logger — drains SPSC ring via lws event loop
static void mqtt_logger_thread_fn(const char* broker_host, int broker_port) {
    printf("  MQTT Logger thread started (broker=%s:%d)\n", broker_host, broker_port);

    // Create lws context — no TLS
    // Increase pt_serv_buf_size for batched payloads (~30 KB per batch)
    struct lws_context_creation_info ctx_info;
    memset(&ctx_info, 0, sizeof(ctx_info));
    ctx_info.port = CONTEXT_PORT_NO_LISTEN;
    ctx_info.protocols = mqtt_protocols;
    ctx_info.options = 0;  // No SSL
    ctx_info.pt_serv_buf_size = 32768;  // 32 KiB (default 4096 is too small for batched payloads)

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
    mqtt_conn.client_id_nofree = 1;
    mqtt_conn.keep_alive = 30;
    mqtt_conn.clean_start = 1;

    // LWT (Last Will) on robot/status
    mqtt_conn.will_param.topic = MQTT_TOPIC_STATUS;
    mqtt_conn.will_param.message = "{\"status\":\"offline\"}";
    mqtt_conn.will_param.qos = QOS1;
    mqtt_conn.will_param.retain = 1;

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
        BatchLogRecord batch;

        while (drained < 10 && g_log_ring.pop(batch)) {
            MqttMessage mqtt_msg;
            mqtt_msg.topic = MQTT_TOPIC_SENSOR;
            mqtt_msg.payload = serialize_batch(batch);
            g_mqtt_queue.push(std::move(mqtt_msg));
            g_samples_published += batch.sample_count;
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

        // Service the lws event loop
        // Use timeout=0 (non-blocking poll) to avoid the 30s stall during
        // MQTT handshake that blocked ring draining
        uint64_t svc_start = get_monotonic_ns();
        lws_service(ctx, 0);
        publish_stats.record(get_monotonic_ns() - svc_start);

        // Small sleep to avoid busy-looping when idle
        // (1ms matches the batch production interval at 1kHz)
        struct timespec ts_sleep = {0, 1'000'000};
        clock_nanosleep(CLOCK_MONOTONIC, 0, &ts_sleep, nullptr);

        // Periodic status
        if (drain_cycles > 0 && drain_cycles % 500 == 0) {
            printf("  [MQTT] batches_published=%lu  samples=%lu  batches_dropped=%lu  queue=%zu  ring=%zu\n",
                   g_records_published, g_samples_published, g_records_dropped,
                   g_mqtt_queue.size(), g_log_ring.size());
        }
    }

    // Disconnect
    lws_context_destroy(ctx);

    printf("\n  MQTT Logger Report:\n");
    printf("    Batches published: %lu  (samples: %lu)\n", g_records_published, g_samples_published);
    printf("    Batches dropped:   %lu\n", g_records_dropped);
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

    // Validate SHM state before using it
    uint32_t magic = g_layout->magic.load(std::memory_order_acquire);
    if (magic != SHM_MAGIC) {
        fprintf(stderr, "Invalid SHM magic\n"); return 1;
    }
    if (g_layout->version != SHM_VERSION) {
        fprintf(stderr, "Invalid SHM version (expected %u, got %u)\n",
                SHM_VERSION, g_layout->version); return 1;
    }
    auto lifecycle = static_cast<ShmLifecycle>(
        g_layout->lifecycle_state.load(std::memory_order_acquire));
    if (lifecycle != ShmLifecycle::RUNNING) {
        fprintf(stderr, "SHM not in RUNNING state\n"); return 1;
    }
    g_layout->robot_id = g_robot_id;

    printf("Actuator+Logger: %u joints, robot_id=%u, broker=%s:%d, state=RUNNING\n",
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
