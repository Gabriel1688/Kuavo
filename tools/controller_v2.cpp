/**
 * @file controller_v2.cpp
 * @brief Whole-body controller test — multi-source aware (v2 SHM layout)
 *
 * Reads the composed SensorData (merged from 3 sources by the composer).
 * Detects per-source staleness independently.
 * Uses mercury_shm_v2.h — compatible with test_actuator_logger.
 *
 * Usage:
 *   ./test_controller_v2 [-freq 1000] [-dur 10] [-joints 12]
 */

#include "mercury_shm_v2.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <thread>

using namespace mercury;

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

class ControllerTestBench {
public:
    bool init(uint32_t num_joints, uint32_t control_freq) {
        num_joints_ = num_joints;
        control_freq_ = control_freq;
        loop_period_ns_ = 1'000'000'000ULL / control_freq;

        int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) { perror("shm_open"); return false; }
        if (ftruncate(fd, sizeof(SharedMemoryLayout)) < 0) {
            perror("ftruncate"); close(fd); return false;
        }
        void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                         PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
        if (ptr == MAP_FAILED) { perror("mmap"); return false; }

        layout_ = static_cast<SharedMemoryLayout*>(ptr);

        // Initialize header
        layout_->magic = SHM_MAGIC;
        layout_->version = 2;  // v2 layout
        layout_->num_joints = num_joints;
        layout_->control_freq_hz = control_freq;
        layout_->robot_id = 1;
        layout_->emergency_stop.store(false, std::memory_order_relaxed);

        std::memset(layout_->cmd_buffers, 0, sizeof(layout_->cmd_buffers));
        std::memset(layout_->composed_buffers, 0, sizeof(layout_->composed_buffers));

        for (uint32_t i = 0; i < num_joints; i++) {
            layout_->motor_can_ids[i] = static_cast<uint16_t>(i + 1);
        }

        printf("Controller v2 initialized: %u joints @ %u Hz (multi-source v2)\n",
               num_joints, control_freq);
        printf("SHM layout size: %zu bytes\n", sizeof(SharedMemoryLayout));
        printf("  ImuStageData:        %zu bytes\n", sizeof(ImuStageData));
        printf("  MotorGroupStageData: %zu bytes\n", sizeof(MotorGroupStageData));
        printf("  Composed SensorData: %zu bytes\n", sizeof(SensorData));
        printf("  Command:             %zu bytes\n\n", sizeof(Command));
        return true;
    }

    void run(double duration_seconds) {
        printf("Running controller for %.1f seconds...\n\n", duration_seconds);

        uint64_t total_iterations = static_cast<uint64_t>(duration_seconds * control_freq_);
        uint64_t iteration = 0;
        uint64_t missed_deadlines = 0;

        // Per-source staleness counters
        uint64_t last_imu_seq = 0, last_grpA_seq = 0, last_grpB_seq = 0;
        uint64_t imu_stale_count = 0, grpA_stale_count = 0, grpB_stale_count = 0;

        Command cmd{};
        SensorData sensor{};

        for (uint32_t j = 0; j < num_joints_; j++) {
            cmd.enabled[j] = 1;
            cmd.control_mode[j] = 0;  // MIT mode [2]
            cmd.kp[j] = 50.0;         // Kp [0, 500] [2]
            cmd.kd[j] = 1.0;          // Kd [0, 5] [2]
        }

        uint64_t next_wakeup = get_monotonic_ns();

        while (g_running && iteration < total_iterations) {
            uint64_t cycle_start = get_monotonic_ns();

            // ---- Generate sinusoidal trajectory ----
            double t = static_cast<double>(iteration) / control_freq_;
            for (uint32_t j = 0; j < num_joints_; j++) {
                double phase = 2.0 * M_PI * 0.5 * t + j * M_PI / 6.0;
                cmd.jpos_cmd[j] = 1.0 * std::sin(phase);
                cmd.jvel_cmd[j] = 1.0 * M_PI * std::cos(phase);
                cmd.jtorque_cmd[j] = 0.0;
            }
            cmd.sequence = iteration;

            // ---- Timed SHM Write (command) ----
            uint64_t write_start = get_monotonic_ns();
            cmd.timestamp_ns = write_start;
            uint32_t wb = 1 - layout_->cmd_write_idx.load(std::memory_order_acquire);
            std::memcpy(&layout_->cmd_buffers[wb], &cmd, sizeof(Command));
            layout_->cmd_write_idx.store(wb, std::memory_order_release);
            layout_->cmd_sequence.fetch_add(1, std::memory_order_release);
            layout_->controller_heartbeat_ns.store(write_start, std::memory_order_release);
            uint64_t write_end = get_monotonic_ns();
            write_stats_.record(write_end - write_start);

            // ---- Timed SHM Read (composed sensor data) ----
            uint64_t read_start = get_monotonic_ns();
            uint32_t rb = layout_->composed_write_idx.load(std::memory_order_acquire);
            std::memcpy(&sensor, &layout_->composed_buffers[rb], sizeof(SensorData));
            uint64_t read_end = get_monotonic_ns();
            read_stats_.record(read_end - read_start);

            // ---- Per-source staleness check ----
            if (sensor.imu_sequence == last_imu_seq && iteration > 10)
                imu_stale_count++;
            if (sensor.motor_group_a_sequence == last_grpA_seq && iteration > 10)
                grpA_stale_count++;
            if (sensor.motor_group_b_sequence == last_grpB_seq && iteration > 10)
                grpB_stale_count++;

            last_imu_seq  = sensor.imu_sequence;
            last_grpA_seq = sensor.motor_group_a_sequence;
            last_grpB_seq = sensor.motor_group_b_sequence;

            // ---- Per-source age measurement ----
            uint64_t now = get_monotonic_ns();
            if (sensor.imu_timestamp_ns > 0) {
                imu_age_stats_.record(now - sensor.imu_timestamp_ns);
            }
            if (sensor.motor_group_a_timestamp_ns > 0) {
                grpA_age_stats_.record(now - sensor.motor_group_a_timestamp_ns);
            }
            if (sensor.motor_group_b_timestamp_ns > 0) {
                grpB_age_stats_.record(now - sensor.motor_group_b_timestamp_ns);
            }

            // ---- Full cycle timing ----
            uint64_t cycle_end = get_monotonic_ns();
            uint64_t cycle_dur = cycle_end - cycle_start;
            cycle_stats_.record(cycle_dur);
            if (cycle_dur > loop_period_ns_) missed_deadlines++;

            // ---- Compose latency (time between compose and read) ----
            if (sensor.compose_timestamp_ns > 0) {
                compose_age_stats_.record(now - sensor.compose_timestamp_ns);
            }

            // ---- Sleep ----
            next_wakeup += loop_period_ns_;
            uint64_t now2 = get_monotonic_ns();
            if (next_wakeup > now2) {
                struct timespec ts;
                uint64_t sleep_ns = next_wakeup - now2;
                ts.tv_sec = sleep_ns / 1'000'000'000ULL;
                ts.tv_nsec = sleep_ns % 1'000'000'000ULL;
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
            } else {
                next_wakeup = now2 + loop_period_ns_;
            }

            iteration++;

            if (iteration % (control_freq_ * 2) == 0) {
                printf("  [%6.1fs] iter=%lu  imu_seq=%lu  grpA_seq=%lu  grpB_seq=%lu  "
                       "cycle=%.1fus\n",
                       t, iteration,
                       sensor.imu_sequence,
                       sensor.motor_group_a_sequence,
                       sensor.motor_group_b_sequence,
                       cycle_stats_.avg_us());
            }
        }

        printf("\n");
        printf("============================================================\n");
        printf("  CONTROLLER TIMING REPORT (v2 layout)\n");
        printf("============================================================\n");
        printf("  Total iterations:     %lu\n", iteration);
        printf("  Effective frequency:  %.1f Hz\n",
               static_cast<double>(iteration) * 1e9 /
               (cycle_stats_.total_ns > 0 ? cycle_stats_.total_ns : 1));
        printf("  Missed deadlines:     %lu (%.2f%%)\n",
               missed_deadlines,
               100.0 * missed_deadlines / std::max(iteration, 1UL));
        printf("\n");
        printf("  Per-Source Staleness:\n");
        printf("    IMU stale reads:        %lu (%.2f%%)\n",
               imu_stale_count, 100.0 * imu_stale_count / std::max(iteration, 1UL));
        printf("    Motor Grp A stale:      %lu (%.2f%%)\n",
               grpA_stale_count, 100.0 * grpA_stale_count / std::max(iteration, 1UL));
        printf("    Motor Grp B stale:      %lu (%.2f%%)\n",
               grpB_stale_count, 100.0 * grpB_stale_count / std::max(iteration, 1UL));
        printf("\n");
        printf("  SHM Latency:\n");
        write_stats_.print("SHM write (cmd publish)");
        read_stats_.print("SHM read (composed sensor)");
        cycle_stats_.print("Full cycle (compute+IO)");
        printf("\n");
        printf("  Per-Source Data Age (at read time):\n");
        imu_age_stats_.print("IMU data age");
        grpA_age_stats_.print("Motor Grp A data age");
        grpB_age_stats_.print("Motor Grp B data age");
        compose_age_stats_.print("Compose snapshot age");
        printf("============================================================\n\n");

        printf("  Last sensor snapshot:\n");
        printf("    IMU ang_vel:     [%.3f, %.3f, %.3f]\n",
               sensor.imu_ang_vel[0], sensor.imu_ang_vel[1], sensor.imu_ang_vel[2]);
        printf("    Joint 0 pos:     %.4f rad\n", sensor.joint_jpos[0]);
        printf("    Joint 6 pos:     %.4f rad  (from Grp B)\n", sensor.joint_jpos[6]);
        printf("    rfoot_contact:   %u\n", sensor.rfoot_contact);
        printf("    IMU ts age:      %.1f us\n",
               (get_monotonic_ns() - sensor.imu_timestamp_ns) / 1000.0);
        printf("    Grp A ts age:    %.1f us\n",
               (get_monotonic_ns() - sensor.motor_group_a_timestamp_ns) / 1000.0);
        printf("    Grp B ts age:    %.1f us\n",
               (get_monotonic_ns() - sensor.motor_group_b_timestamp_ns) / 1000.0);
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
    uint32_t num_joints_ = NUM_ACT_JOINT;
    uint32_t control_freq_ = 1000;
    uint64_t loop_period_ns_ = 1'000'000;

    TimingStats write_stats_, read_stats_, cycle_stats_;
    TimingStats imu_age_stats_, grpA_age_stats_, grpB_age_stats_;
    TimingStats compose_age_stats_;
};

int main(int argc, char* argv[]) {
    uint32_t freq = 1000;
    double duration = 10.0;
    uint32_t joints = NUM_ACT_JOINT;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-freq") == 0 && i + 1 < argc)
            freq = atoi(argv[++i]);
        else if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-joints") == 0 && i + 1 < argc)
            joints = atoi(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-freq Hz] [-dur sec] [-joints N]\n", argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ControllerTestBench bench;
    if (!bench.init(joints, freq)) return 1;

    printf("Waiting 2 seconds for actuator threads to attach...\n\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    bench.run(duration);
    return 0;
}
