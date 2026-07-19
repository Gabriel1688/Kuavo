/**
 * @file test_actuator.cpp
 * @brief Actuator test — reads commands from SHM, simulates motors,
 *        writes sensor data back, optionally bridges to Damiao motors
 *        via CAN-over-UDP [1]
 *
 * Timing measurements:
 *   1. SHM read latency (command fetch)
 *   2. SHM write latency (sensor data publish)
 *   3. Motor simulation / UDP round-trip time
 *   4. Full loop cycle time
 *   5. Command-to-publish latency
 *
 * Modes:
 *   --sim     Simulate motors locally (no UDP, fastest)
 *   --udp     Bridge to Damiao simulator via CAN-over-UDP [1]
 *
 * Usage:
 *   ./test_actuator [-freq 1000] [-dur 10] [--sim|--udp]
 *   ./test_actuator --udp -local 8887 -remote 8886
 */

#include "mercury_shm.h"
#include <arpa/inet.h>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace mercury;

static volatile bool g_running = true;

static void signal_handler(int) {
    g_running = false;
}

// ============================================================
// Protocol helpers — Damiao motor encoding/decoding [2][3]
// ============================================================

static constexpr double P_MAX = 12.5;  // Position range [2]
static constexpr double V_MAX = 45.0;  // Velocity range [2]
static constexpr double T_MAX = 18.0;  // Torque range [2]

static uint16_t double_to_uint(double x, double x_min, double x_max, int bits) {
    x = std::max(x_min, std::min(x, x_max));
    return static_cast<uint16_t>(((x - x_min) / (x_max - x_min)) * ((1 << bits) - 1));
}

static double uint_to_double(uint16_t x, double min_v, double max_v, int bits) {
    return (static_cast<double>(x) / ((1 << bits) - 1)) * (max_v - min_v) + min_v;
}

// ============================================================
// Simulated Motor State (one per joint)
// ============================================================
struct SimMotor {
    double position = 0.0;
    double velocity = 0.0;
    double torque   = 0.0;
    int    mos_temp = 35;
    int    rotor_temp = 40;
    uint8_t status  = 0x00;  // Disabled by default [2]

    void enable()  { status = 0x01; }
    void disable() { status = 0x00; position = velocity = torque = 0; }

    /**
     * Simple first-order motor simulation.
     * In a real system, this would be replaced by actual
     * CAN-over-UDP communication with Damiao motors [1].
     */
    void simulate(double target_pos, double target_vel, double target_torque,
                  double kp, double kd) {
        if (status != 0x01) return; // Must be enabled [2]

        // PD control simulation (MIT mode) [2]
        double pos_error = target_pos - position;
        double vel_error = target_vel - velocity;
        double computed_torque = kp * pos_error + kd * vel_error + target_torque;

        // Simple dynamics: torque → acceleration → velocity → position
        double inertia = 0.1;  // kg·m²
        double damping = 0.05; // Nm·s/rad
        double dt = 0.001;     // 1ms step

        double acceleration = (computed_torque - damping * velocity) / inertia;
        velocity += acceleration * dt;
        position += velocity * dt;
        torque = computed_torque;

        // Clamp to Damiao motor limits [2]
        position = std::max(-P_MAX, std::min(position, P_MAX));
        velocity = std::max(-V_MAX, std::min(velocity, V_MAX));
        torque   = std::max(-T_MAX, std::min(torque, T_MAX));

        // Temperature model
        mos_temp   = 35 + static_cast<int>(std::abs(torque) * 2.0);
        rotor_temp = 40 + static_cast<int>(std::abs(torque) * 1.5);
    }
};

// ============================================================
// UDP Bridge to Damiao Simulator [1]
// ============================================================
class UdpBridge {
public:
    bool init(int localPort, int remotePort) {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sockfd_ < 0) return false;

        // Non-blocking socket [1]
        fcntl(sockfd_, F_SETFL, O_NONBLOCK);

        struct sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(localPort);
        local.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (bind(sockfd_, (struct sockaddr*)&local, sizeof(local)) < 0) {
            perror("bind");
            close(sockfd_);
            return false;
        }

        remote_.sin_family = AF_INET;
        remote_.sin_port = htons(remotePort);
        remote_.sin_addr.s_addr = inet_addr("127.0.0.1");

        printf("UDP bridge: local=%d remote=%d\n", localPort, remotePort);
        return true;
    }

    /**
     * Send MIT control command via CAN-over-UDP [1][2][3].
     * Frame format: 13 bytes = DLC(1) + CAN_ID(4) + DATA(8) [1]
     */
    void send_mit_command(uint16_t can_id, double pos, double vel,
                          double kp, double kd, double torque) {
        uint8_t frame[13];
        frame[0] = 0x08; // DLC [1]
        // CAN ID (big-endian) [1]
        frame[1] = 0x00;
        frame[2] = 0x00;
        frame[3] = (can_id >> 8) & 0xFF;
        frame[4] = can_id & 0xFF;

        // MIT encoding [2][3]
        uint16_t p   = double_to_uint(pos, -P_MAX, P_MAX, 16);
        uint16_t v   = double_to_uint(vel, -V_MAX, V_MAX, 12);
        uint16_t kp_u = double_to_uint(kp, 0, 500, 12);
        uint16_t kd_u = double_to_uint(kd, 0, 5, 12);
        uint16_t tau = double_to_uint(torque, -T_MAX, T_MAX, 12);

        frame[5]  = (p >> 8) & 0xFF;
        frame[6]  = p & 0xFF;
        frame[7]  = (v >> 4) & 0xFF;
        frame[8]  = ((v & 0xF) << 4) | ((kp_u >> 8) & 0xF);
        frame[9]  = kp_u & 0xFF;
        frame[10] = (kd_u >> 4) & 0xFF;
        frame[11] = ((kd_u & 0xF) << 4) | ((tau >> 8) & 0xF);
        frame[12] = tau & 0xFF;

        sendto(sockfd_, frame, 13, 0,
               (struct sockaddr*)&remote_, sizeof(remote_));
        udp_tx_count_++;
    }

    /**
     * Send enable command (0xFC) [2]
     */
    void send_enable(uint16_t can_id) {
        uint8_t frame[13] = {0x08, 0, 0, static_cast<uint8_t>((can_id>>8)&0xFF),
                             static_cast<uint8_t>(can_id&0xFF),
                             0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        sendto(sockfd_, frame, 13, 0,
               (struct sockaddr*)&remote_, sizeof(remote_));
    }

    /**
     * Receive and decode Damiao feedback frame [2].
     * Returns true if a valid frame was received.
     */
    bool receive_feedback(uint8_t& motor_id, double& pos, double& vel,
                          double& torque, int& t_mos, int& t_rotor,
                          uint8_t& err) {
        uint8_t frame[13];
        ssize_t n = recvfrom(sockfd_, frame, 13, 0, nullptr, nullptr);
        if (n < 13) return false;

        // Decode feedback [2][3]
        motor_id = frame[5] & 0x0F;
        err = (frame[5] >> 4) & 0x0F;

        uint16_t p_u = (static_cast<uint16_t>(frame[6]) << 8) | frame[7];
        uint16_t v_u = (static_cast<uint16_t>(frame[8]) << 4) | (frame[9] >> 4);
        uint16_t t_u = (static_cast<uint16_t>(frame[9] & 0x0F) << 8) | frame[10];

        pos    = uint_to_double(p_u, -P_MAX, P_MAX, 16);
        vel    = uint_to_double(v_u, -V_MAX, V_MAX, 12);
        torque = uint_to_double(t_u, -T_MAX, T_MAX, 12);
        t_mos  = static_cast<int>(frame[11]);
        t_rotor = static_cast<int>(frame[12]);

        udp_rx_count_++;
        return true;
    }

    uint64_t tx_count() const { return udp_tx_count_; }
    uint64_t rx_count() const { return udp_rx_count_; }

    ~UdpBridge() { if (sockfd_ >= 0) close(sockfd_); }

private:
    int sockfd_ = -1;
    struct sockaddr_in remote_{};
    uint64_t udp_tx_count_ = 0;
    uint64_t udp_rx_count_ = 0;
};

// ============================================================
// Actuator Test Bench
// ============================================================
class ActuatorTestBench {
public:
    bool init(bool use_udp, int localPort, int remotePort) {
        use_udp_ = use_udp;

        // Attach to shared memory
        int fd = shm_open(SHM_NAME, O_RDWR, 0666);
        if (fd < 0) {
            perror("shm_open (is controller running?)");
            return false;
        }
        void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                         PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (ptr == MAP_FAILED) {
            perror("mmap");
            return false;
        }

        layout_ = static_cast<SharedMemoryLayout*>(ptr);
        if (layout_->magic != SHM_MAGIC) {
            fprintf(stderr, "Invalid SHM magic: 0x%08X\n", layout_->magic);
            return false;
        }

        num_joints_ = layout_->num_joints;
        control_freq_ = layout_->control_freq_hz;
        loop_period_ns_ = 1'000'000'000ULL / control_freq_;

        printf("Actuator attached: %u joints @ %u Hz  mode=%s\n",
               num_joints_, control_freq_,
               use_udp_ ? "UDP (CAN-over-UDP)" : "SIM (local)");

        // Initialize simulated motors
        for (uint32_t j = 0; j < num_joints_; j++) {
            motors_[j] = SimMotor{};
        }

        // Initialize UDP bridge if needed [1]
        if (use_udp_) {
            if (!udp_.init(localPort, remotePort)) return false;
        }

        return true;
    }

    void run(double duration_seconds) {
        printf("Running actuator for %.1f seconds...\n\n", duration_seconds);

        uint64_t total_iterations = static_cast<uint64_t>(
            duration_seconds * control_freq_);
        uint64_t iteration = 0;
        uint64_t cmd_stale_count = 0;
        uint64_t last_cmd_seq = 0;

        Command cmd{};
        SensorData sensor{};
        std::memset(&sensor, 0, sizeof(sensor));

        uint64_t next_wakeup = get_monotonic_ns();

        while (g_running && iteration < total_iterations) {
            uint64_t cycle_start = get_monotonic_ns();

            // Check emergency stop
            if (layout_->emergency_stop.load(std::memory_order_acquire)) {
                printf("EMERGENCY STOP received\n");
                for (uint32_t j = 0; j < num_joints_; j++)
                    motors_[j].disable();
                break;
            }

            // ---- Timed SHM Read (command fetch) ----
            uint64_t read_start = get_monotonic_ns();

            uint32_t read_buf = layout_->cmd_write_idx.load(
                std::memory_order_acquire);
            std::memcpy(&cmd, &layout_->cmd_buffers[read_buf], sizeof(Command));

            uint64_t read_end = get_monotonic_ns();
            read_stats_.record(read_end - read_start);

            // Check command freshness
            if (cmd.sequence == last_cmd_seq && iteration > 10) {
                cmd_stale_count++;
            }
            last_cmd_seq = cmd.sequence;

            // ---- Motor Processing (sim or UDP) ----
            uint64_t motor_start = get_monotonic_ns();

            if (use_udp_) {
                // Send commands to Damiao motors via CAN-over-UDP [1]
                for (uint32_t j = 0; j < num_joints_; j++) {
                    uint16_t can_id = layout_->motor_can_ids[j];
                    if (cmd.enabled[j]) {
                        udp_.send_mit_command(can_id,
                            cmd.jpos_cmd[j], cmd.jvel_cmd[j],
                            cmd.kp[j], cmd.kd[j], cmd.jtorque_cmd[j]);
                    }
                }

                // Receive feedback from all motors [2]
                for (int attempts = 0; attempts < 50; attempts++) {
                    uint8_t mid, err;
                    double pos, vel, torque;
                    int t_mos, t_rotor;
                    if (!udp_.receive_feedback(mid, pos, vel, torque,
                                               t_mos, t_rotor, err))
                        break;

                    // Map motor ID to joint index
                    for (uint32_t j = 0; j < num_joints_; j++) {
                        if ((layout_->motor_can_ids[j] & 0xFF) == mid) {
                            sensor.joint_jpos[j] = pos;
                            sensor.joint_jvel[j] = vel;
                            sensor.jtorque[j] = torque;
                            sensor.mos_temperature[j] = t_mos;
                            sensor.rotor_temperature[j] = t_rotor;
                            sensor.motor_status[j] = err;
                            break;
                        }
                    }
                }
            } else {
                // Local motor simulation
                for (uint32_t j = 0; j < num_joints_; j++) {
                    if (cmd.enabled[j] && motors_[j].status != 0x01)
                        motors_[j].enable();
                    else if (!cmd.enabled[j] && motors_[j].status == 0x01)
                        motors_[j].disable();

                    motors_[j].simulate(cmd.jpos_cmd[j], cmd.jvel_cmd[j],
                                        cmd.jtorque_cmd[j],
                                        cmd.kp[j], cmd.kd[j]);

                    sensor.joint_jpos[j]       = motors_[j].position;
                    sensor.joint_jvel[j]       = motors_[j].velocity;
                    sensor.jtorque[j]          = motors_[j].torque;
                    sensor.motor_jpos[j]       = motors_[j].position;
                    sensor.motor_jvel[j]       = motors_[j].velocity;
                    sensor.mos_temperature[j]  = motors_[j].mos_temp;
                    sensor.rotor_temperature[j]= motors_[j].rotor_temp;
                    sensor.motor_status[j]     = motors_[j].status;
                }
            }

            uint64_t motor_end = get_monotonic_ns();
            motor_stats_.record(motor_end - motor_start);

            // Simulated contact sensors
            sensor.rfoot_contact = (std::abs(sensor.joint_jpos[10]) < 0.1);
            sensor.lfoot_contact = (std::abs(sensor.joint_jpos[11]) < 0.1);

            // ---- Timed SHM Write (sensor publish) ----
            uint64_t write_start = get_monotonic_ns();

            sensor.timestamp_ns = write_start;
            sensor.sequence = iteration;

            uint32_t write_buf = 1 - layout_->state_write_idx.load(
                std::memory_order_acquire);
            std::memcpy(&layout_->state_buffers[write_buf], &sensor,
                        sizeof(SensorData));
            layout_->state_write_idx.store(write_buf, std::memory_order_release);
            layout_->state_sequence.fetch_add(1, std::memory_order_release);
            layout_->actuator_heartbeat_ns.store(write_start,
                std::memory_order_release);

            uint64_t write_end = get_monotonic_ns();
            write_stats_.record(write_end - write_start);

            // ---- Command-to-publish latency ----
            if (cmd.timestamp_ns > 0) {
                uint64_t cmd_to_pub = write_end - cmd.timestamp_ns;
                cmd_to_pub_stats_.record(cmd_to_pub);
            }

            // ---- Full cycle timing ----
            uint64_t cycle_end = get_monotonic_ns();
            cycle_stats_.record(cycle_end - cycle_start);

            // ---- Sleep ----
            next_wakeup += loop_period_ns_;
            uint64_t now = get_monotonic_ns();
            if (next_wakeup > now) {
                struct timespec ts;
                uint64_t sleep_ns = next_wakeup - now;
                ts.tv_sec = sleep_ns / 1'000'000'000ULL;
                ts.tv_nsec = sleep_ns % 1'000'000'000ULL;
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
            } else {
                next_wakeup = now + loop_period_ns_;
            }

            iteration++;

            // Periodic progress
            if (iteration % (control_freq_ * 2) == 0) {
                printf("  [iter=%6lu]  cycle=%.1fus  motor=%.1fus  "
                       "cmd_stale=%lu",
                       iteration, cycle_stats_.avg_us(),
                       motor_stats_.avg_us(), cmd_stale_count);
                if (use_udp_) {
                    printf("  udp_tx=%lu  udp_rx=%lu",
                           udp_.tx_count(), udp_.rx_count());
                }
                printf("\n");
            }
        }

        // ---- Final Report ----
        printf("\n");
        printf("============================================================\n");
        printf("  ACTUATOR TIMING REPORT  [mode=%s]\n",
               use_udp_ ? "UDP" : "SIM");
        printf("============================================================\n");
        printf("  Total iterations:     %lu\n", iteration);
        printf("  Command stale reads:  %lu (%.2f%%)\n",
               cmd_stale_count,
               100.0 * cmd_stale_count / std::max(iteration, 1UL));
        if (use_udp_) {
            printf("  UDP TX packets:       %lu\n", udp_.tx_count());
            printf("  UDP RX packets:       %lu\n", udp_.rx_count());
        }
        printf("\n");
        printf("  Latency Breakdown:\n");
        read_stats_.print("SHM read (cmd fetch)");
        motor_stats_.print("Motor processing");
        write_stats_.print("SHM write (sensor pub)");
        cycle_stats_.print("Full cycle");
        cmd_to_pub_stats_.print("Cmd-to-publish");
        printf("============================================================\n\n");

        // Print last motor states
        printf("  Last motor states:\n");
        for (uint32_t j = 0; j < std::min(num_joints_, 6u); j++) {
            printf("    J%u: pos=%+7.3f vel=%+7.3f tau=%+7.3f "
                   "mos=%dC rotor=%dC status=0x%02X\n",
                   j, sensor.joint_jpos[j], sensor.joint_jvel[j],
                   sensor.jtorque[j], sensor.mos_temperature[j],
                   sensor.rotor_temperature[j], sensor.motor_status[j]);
        }
        if (num_joints_ > 6) printf("    ... (%u more joints)\n", num_joints_ - 6);
        printf("\n");
    }

    ~ActuatorTestBench() {
        if (layout_) {
            munmap(layout_, sizeof(SharedMemoryLayout));
        }
    }

private:
    SharedMemoryLayout* layout_ = nullptr;
    uint32_t num_joints_ = num_act_joint;
    uint32_t control_freq_ = 1000;
    uint64_t loop_period_ns_ = 1'000'000;
    bool use_udp_ = false;

    SimMotor motors_[num_act_joint];
    UdpBridge udp_;

    TimingStats read_stats_;
    TimingStats write_stats_;
    TimingStats motor_stats_;
    TimingStats cycle_stats_;
    TimingStats cmd_to_pub_stats_;
};

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    double duration = 10.0;
    bool use_udp = false;
    int localPort = 8887;    // [1]
    int remotePort = 8886;   // [1]

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "--sim") == 0)
            use_udp = false;
        else if (strcmp(argv[i], "--udp") == 0)
            use_udp = true;
        else if (strcmp(argv[i], "-local") == 0 && i + 1 < argc)
            localPort = atoi(argv[++i]);
        else if (strcmp(argv[i], "-remote") == 0 && i + 1 < argc)
            remotePort = atoi(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-dur sec] [--sim|--udp] "
                   "[-local port] [-remote port]\n", argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ActuatorTestBench bench;
    if (!bench.init(use_udp, localPort, remotePort)) return 1;

    bench.run(duration);
    return 0;
}
/*
* # Build both programs
g++ -O2 -std=c++20 -pthread -lrt -o test_controller test_controller.cpp
g++ -O2 -std=c++20 -pthread -lrt -o test_actuator test_actuator.cpp

# ============================================================
# Test 1: Pure shared memory (SIM mode — no Damiao hardware)
# ============================================================
# Terminal 1: Start controller (creates SHM)
./test_controller -freq 1000 -dur 10 -joints 12

# Terminal 2: Start actuator (attaches to SHM)
./test_actuator -dur 10 --sim

# ============================================================
# Test 2: With Damiao simulator via CAN-over-UDP [1]
# ============================================================
# Terminal 1: Start Damiao multi-motor simulator
./damiao_multi_simulator -ids 1,2,3,4,5,6,7,8,9,10,11,12 \
    -local 8886 -remote 8887

# Terminal 2: Start controller
./test_controller -freq 1000 -dur 10 -joints 12

# Terminal 3: Start actuator in UDP bridge mode
./test_actuator -dur 10 --udp -local 8887 -remote 8886

 */
