/**
 * @file test_controller.cpp
 * @brief Whole-body controller test — writes commands, reads sensor data
 *
 * Creates the shared memory region, runs a 1kHz control loop,
 * and prints detailed timing statistics.
 *
 * Timing measurements:
 *   1. SHM write latency (command publish)
 *   2. SHM read latency (sensor data fetch)
 *   3. Full loop cycle time (jitter)
 *   4. Round-trip latency (command → sensor data return)
 *   5. State sequence freshness
 *
 * Usage:
 *   ./test_controller [-freq 1000] [-dur 10] [-joints 12]
 */

#include "mercury_shm.h"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

using namespace mercury;

static volatile bool g_running = true;

static void signal_handler(int) {
    g_running = false;
}

class ControllerTestBench {
public:
    bool init(uint32_t num_joints, uint32_t control_freq) {
        num_joints_ = num_joints;
        control_freq_ = control_freq;
        loop_period_ns_ = 1'000'000'000ULL / control_freq;

        // Create shared memory
        int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            perror("shm_open");
            return false;
        }
        if (ftruncate(fd, sizeof(SharedMemoryLayout)) < 0) {
            perror("ftruncate");
            close(fd);
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

        // Initialize header
        layout_->magic = SHM_MAGIC;
        layout_->version = 1;
        layout_->num_joints = num_joints;
        layout_->control_freq_hz = control_freq;
        layout_->cmd_write_idx.store(0, std::memory_order_relaxed);
        layout_->cmd_sequence.store(0, std::memory_order_relaxed);
        layout_->state_write_idx.store(0, std::memory_order_relaxed);
        layout_->state_sequence.store(0, std::memory_order_relaxed);
        layout_->emergency_stop.store(false, std::memory_order_relaxed);
        layout_->controller_heartbeat_ns.store(0, std::memory_order_relaxed);
        layout_->actuator_heartbeat_ns.store(0, std::memory_order_relaxed);

        std::memset(layout_->cmd_buffers, 0, sizeof(layout_->cmd_buffers));
        std::memset(layout_->state_buffers, 0, sizeof(layout_->state_buffers));

        // Default CAN IDs: 1..num_joints [2]
        for (uint32_t i = 0; i < num_joints; i++) {
            layout_->motor_can_ids[i] = static_cast<uint16_t>(i + 1);
        }

        printf("Controller initialized: %u joints @ %u Hz\n",
               num_joints, control_freq);
        printf("Loop period target: %lu ns (%.2f us)\n",
               loop_period_ns_, loop_period_ns_ / 1000.0);
        printf("SHM size: %zu bytes\n", sizeof(SharedMemoryLayout));
        printf("Command size: %zu bytes\n", sizeof(Command));
        printf("SensorData size: %zu bytes\n\n", sizeof(SensorData));

        return true;
    }

    void run(double duration_seconds) {
        printf("Running controller for %.1f seconds...\n\n", duration_seconds);

        uint64_t total_iterations = static_cast<uint64_t>(duration_seconds * control_freq_);
        uint64_t iteration = 0;
        uint64_t last_state_seq = 0;
        uint64_t stale_count = 0;
        uint64_t missed_deadlines = 0;

        Command cmd{};
        SensorData sensor{};

        // Pre-fill command with sinusoidal trajectory [2]
        for (uint32_t j = 0; j < num_joints_; j++) {
            cmd.enabled[j] = 1;
            cmd.control_mode[j] = 0; // MIT mode [2]
            cmd.kp[j] = 50.0;        // Kp range [0,500] [2]
            cmd.kd[j] = 1.0;         // Kd range [0,5] [2]
        }

        uint64_t loop_start = get_monotonic_ns();
        uint64_t next_wakeup = loop_start;

        while (g_running && iteration < total_iterations) {
            uint64_t cycle_start = get_monotonic_ns();

            // ---- Generate sinusoidal trajectory ----
            double t = static_cast<double>(iteration) / control_freq_;
            for (uint32_t j = 0; j < num_joints_; j++) {
                double phase = 2.0 * M_PI * 0.5 * t + j * M_PI / 6.0;
                cmd.jpos_cmd[j] = 1.0 * std::sin(phase);      // ±1 rad
                cmd.jvel_cmd[j] = 1.0 * M_PI * std::cos(phase); // derivative
                cmd.jtorque_cmd[j] = 0.0;                       // feedforward
            }
            cmd.sequence = iteration;

            // ---- Timed SHM Write ----
            uint64_t write_start = get_monotonic_ns();

            cmd.timestamp_ns = write_start;
            uint32_t write_buf = 1 - layout_->cmd_write_idx.load(
                std::memory_order_acquire);
            std::memcpy(&layout_->cmd_buffers[write_buf], &cmd, sizeof(Command));
            layout_->cmd_write_idx.store(write_buf, std::memory_order_release);
            layout_->cmd_sequence.fetch_add(1, std::memory_order_release);
            layout_->controller_heartbeat_ns.store(write_start,
                std::memory_order_release);

            uint64_t write_end = get_monotonic_ns();
            write_stats_.record(write_end - write_start);

            // ---- Timed SHM Read ----
            uint64_t read_start = get_monotonic_ns();

            uint32_t read_buf = layout_->state_write_idx.load(
                std::memory_order_acquire);
            std::memcpy(&sensor, &layout_->state_buffers[read_buf],
                        sizeof(SensorData));

            uint64_t read_end = get_monotonic_ns();
            read_stats_.record(read_end - read_start);

            // ---- Round-trip latency ----
            if (sensor.timestamp_ns > 0 && sensor.timestamp_ns <= cycle_start) {
                uint64_t rtt = cycle_start - sensor.timestamp_ns;
                rtt_stats_.record(rtt);
            }

            // ---- State freshness check ----
            uint64_t state_seq = layout_->state_sequence.load(
                std::memory_order_acquire);
            if (state_seq == last_state_seq && iteration > 10) {
                stale_count++;
            }
            last_state_seq = state_seq;

            // ---- Full cycle timing ----
            uint64_t cycle_end = get_monotonic_ns();
            uint64_t cycle_duration = cycle_end - cycle_start;
            cycle_stats_.record(cycle_duration);

            // Check deadline
            if (cycle_duration > loop_period_ns_) {
                missed_deadlines++;
            }

            // ---- Sleep until next period ----
            next_wakeup += loop_period_ns_;
            uint64_t now = get_monotonic_ns();
            if (next_wakeup > now) {
                uint64_t sleep_ns = next_wakeup - now;
                struct timespec ts;
                ts.tv_sec = sleep_ns / 1'000'000'000ULL;
                ts.tv_nsec = sleep_ns % 1'000'000'000ULL;
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
            } else {
                // Already past deadline — reset
                next_wakeup = now + loop_period_ns_;
            }

            iteration++;

            // ---- Periodic progress ----
            if (iteration % (control_freq_ * 2) == 0) {
                printf("  [%6.1fs] iter=%lu  state_seq=%lu  "
                       "cycle=%.1fus  rtt=%.1fus  stale=%lu\n",
                       t, iteration, state_seq,
                       cycle_stats_.avg_us(), rtt_stats_.avg_us(), stale_count);
            }
        }

        uint64_t total_elapsed = get_monotonic_ns() - loop_start;

        // ---- Print final report ----
        printf("\n");
        printf("============================================================\n");
        printf("  CONTROLLER TIMING REPORT\n");
        printf("============================================================\n");
        printf("  Total iterations:     %lu\n", iteration);
        printf("  Total elapsed:        %.3f seconds\n",
               total_elapsed / 1e9);
        printf("  Effective frequency:  %.1f Hz\n",
               iteration / (total_elapsed / 1e9));
        printf("  Missed deadlines:     %lu (%.2f%%)\n",
               missed_deadlines,
               100.0 * missed_deadlines / std::max(iteration, 1UL));
        printf("  Stale state reads:    %lu (%.2f%%)\n",
               stale_count,
               100.0 * stale_count / std::max(iteration, 1UL));
        printf("\n");
        printf("  Latency Breakdown:\n");
        write_stats_.print("SHM write (cmd publish)");
        read_stats_.print("SHM read (sensor fetch)");
        cycle_stats_.print("Full cycle (compute+IO)");
        rtt_stats_.print("Round-trip (cmd→sensor)");
        printf("============================================================\n\n");

        // ---- Print last sensor snapshot ----
        printf("  Last sensor data (joint 0):\n");
        printf("    position:     %.4f rad\n", sensor.joint_jpos[0]);
        printf("    velocity:     %.4f rad/s\n", sensor.joint_jvel[0]);
        printf("    torque:       %.4f Nm\n", sensor.jtorque[0]);
        printf("    mos_temp:     %d C\n", sensor.mos_temperature[0]);
        printf("    rotor_temp:   %d C\n", sensor.rotor_temperature[0]);
        printf("    motor_status: 0x%02X\n", sensor.motor_status[0]);
        printf("    rfoot:        %s\n", sensor.rfoot_contact ? "true" : "false");
        printf("    lfoot:        %s\n", sensor.lfoot_contact ? "true" : "false");
        printf("\n");
    }

    ~ControllerTestBench() {
        if (layout_) {
            layout_->emergency_stop.store(true, std::memory_order_release);
            munmap(layout_, sizeof(SharedMemoryLayout));
        }
        shm_unlink(SHM_NAME);
    }

private:
    SharedMemoryLayout* layout_ = nullptr;
    uint32_t num_joints_ = num_act_joint;
    uint32_t control_freq_ = 1000;
    uint64_t loop_period_ns_ = 1'000'000;

    TimingStats write_stats_;
    TimingStats read_stats_;
    TimingStats cycle_stats_;
    TimingStats rtt_stats_;
};

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    uint32_t freq = 1000;
    double duration = 10.0;
    uint32_t joints = num_act_joint;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-freq") == 0 && i + 1 < argc)
            freq = atoi(argv[++i]);
        else if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-joints") == 0 && i + 1 < argc)
            joints = atoi(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-freq Hz] [-dur seconds] [-joints N]\n", argv[0]);
            return 0;
        }
    }

    if (joints > static_cast<uint32_t>(num_act_joint)) {
        fprintf(stderr, "Max %d joints\n", num_act_joint);
        return 1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ControllerTestBench bench;
    if (!bench.init(joints, freq)) return 1;

    printf("Waiting 2 seconds for actuator to attach...\n\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    bench.run(duration);
    return 0;
}
