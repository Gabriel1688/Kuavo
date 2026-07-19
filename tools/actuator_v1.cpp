/**
 * @file test_actuator.cpp
 * @brief Multi-source actuator test — 3 writer threads + composer
 *
 * Thread layout:
 *   Thread 1 (IMU):     Simulates IMU at 500Hz → imu_stage [1]
 *   Thread 2 (Motors A): Simulates motors 0-5 at 1kHz → motor_group_a_stage
 *   Thread 3 (Motors B): Simulates motors 6-11 at 1kHz → motor_group_b_stage
 *   Thread 4 (Composer): Merges all 3 → composed_buffers at 1kHz
 *
 * Usage:
 *   ./test_actuator [-dur 10]
 */

#include "mercury_shm_v1.h"
#include <cmath>
#include <csignal>
#include <cstdio>
#include <thread>
#include <vector>

using namespace mercury;

static volatile bool g_running = true;
static void signal_handler(int) { g_running = false; }

static constexpr double P_MAX = 12.5;  // [2]
static constexpr double V_MAX = 45.0;  // [2]
static constexpr double T_MAX = 18.0;  // [2]

// ============================================================
// Simulated Motor
// ============================================================
struct SimMotor {
    double position = 0.0;
    double velocity = 0.0;
    double torque   = 0.0;
    int mos_temp    = 35;
    int rotor_temp  = 40;
    uint8_t status  = 0x00;

    void enable()  { status = 0x01; }  // [2]
    void disable() { status = 0x00; position = velocity = torque = 0; }

    void simulate(double tp, double tv, double tt, double kp, double kd) {
        if (status != 0x01) return;
        double pe = tp - position;
        double ve = tv - velocity;
        double ct = kp * pe + kd * ve + tt;
        double acc = (ct - 0.05 * velocity) / 0.1;
        velocity += acc * 0.001;
        position += velocity * 0.001;
        torque = ct;
        position = std::max(-P_MAX, std::min(position, P_MAX));
        velocity = std::max(-V_MAX, std::min(velocity, V_MAX));
        torque   = std::max(-T_MAX, std::min(torque, T_MAX));
        mos_temp  = 35 + static_cast<int>(std::abs(torque) * 2.0);
        rotor_temp = 40 + static_cast<int>(std::abs(torque) * 1.5);
    }
};

// ============================================================
// Per-Thread Timing Stats (thread-local storage)
// ============================================================
struct ThreadStats {
    TimingStats read_cmd;
    TimingStats motor_sim;
    TimingStats stage_write;
    TimingStats cycle;
    uint64_t iterations = 0;
    uint64_t cmd_stale = 0;
};

// ============================================================
// IMU Writer Thread — 500Hz [1]
// ============================================================
void imu_thread_fn(SharedMemoryLayout* layout, ThreadStats* stats) {
    const uint64_t period_ns = 2'000'000;  // 2ms = 500Hz [1]
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t iteration = 0;

    printf("  IMU thread started (500Hz)\n");

    while (g_running && !layout->emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        // Simulate IMU data (sinusoidal angular velocity)
        double t = static_cast<double>(iteration) / 500.0;
        ImuStageData imu{};
        imu.imu_ang_vel[0] = 0.1 * std::sin(2.0 * M_PI * 1.0 * t);
        imu.imu_ang_vel[1] = 0.05 * std::cos(2.0 * M_PI * 0.5 * t);
        imu.imu_ang_vel[2] = 0.02 * std::sin(2.0 * M_PI * 2.0 * t);
        imu.imu_acc[0] = 0.0;
        imu.imu_acc[1] = 0.0;
        imu.imu_acc[2] = 9.81;
        imu.imu_inc[0] = imu.imu_ang_vel[0] * 0.002;
        imu.imu_inc[1] = imu.imu_ang_vel[1] * 0.002;
        imu.imu_inc[2] = imu.imu_ang_vel[2] * 0.002;
        imu.sequence = iteration;

        // Timed stage write
        uint64_t write_start = get_monotonic_ns();
        imu.timestamp_ns = write_start;
        layout->imu_stage.publish(imu);
        uint64_t write_end = get_monotonic_ns();
        stats->stage_write.record(write_end - write_start);

        uint64_t cycle_end = get_monotonic_ns();
        stats->cycle.record(cycle_end - cycle_start);
        stats->iterations = ++iteration;

        // Sleep to maintain 500Hz [1]
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }

    printf("  IMU thread stopped after %lu iterations\n", iteration);
}

// ============================================================
// Motor Group Writer Thread — 1kHz
// ============================================================
void motor_thread_fn(SharedMemoryLayout* layout,
                     SourceDoubleBuffer<MotorGroupStageData>* stage,
                     int group_offset,  // 0 for Grp A, 6 for Grp B
                     const char* name,
                     ThreadStats* stats) {
    const uint64_t period_ns = 1'000'000;  // 1ms = 1kHz
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t iteration = 0;
    uint64_t last_cmd_seq = 0;

    SimMotor motors[MOTORS_PER_GROUP];

    printf("  %s thread started (1kHz, joints %d-%d)\n",
           name, group_offset, group_offset + MOTORS_PER_GROUP - 1);

    while (g_running && !layout->emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        // ---- Read command from SHM ----
        uint64_t read_start = get_monotonic_ns();
        uint32_t rb = layout->cmd_write_idx.load(std::memory_order_acquire);
        Command cmd;
        std::memcpy(&cmd, &layout->cmd_buffers[rb], sizeof(Command));
        uint64_t read_end = get_monotonic_ns();
        stats->read_cmd.record(read_end - read_start);

        if (cmd.sequence == last_cmd_seq && iteration > 10)
            stats->cmd_stale++;
        last_cmd_seq = cmd.sequence;

        // ---- Simulate motors ----
        uint64_t sim_start = get_monotonic_ns();
        MotorGroupStageData grp{};

        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            int global_j = group_offset + j;

            if (cmd.enabled[global_j] && motors[j].status != 0x01)
                motors[j].enable();
            else if (!cmd.enabled[global_j] && motors[j].status == 0x01)
                motors[j].disable();

            motors[j].simulate(
                cmd.jpos_cmd[global_j], cmd.jvel_cmd[global_j],
                cmd.jtorque_cmd[global_j],
                cmd.kp[global_j], cmd.kd[global_j]);

            grp.joint_jpos[j]            = motors[j].position;
            grp.joint_jvel[j]            = motors[j].velocity;
            grp.motor_jpos[j]            = motors[j].position;
            grp.motor_jvel[j]            = motors[j].velocity;
            grp.jtorque[j]               = motors[j].torque;
            grp.motor_status[j]          = motors[j].status;
            grp.mos_temperature[j]       = motors[j].mos_temp;
            grp.rotor_temperature[j]     = motors[j].rotor_temp;
            grp.bus_voltage[j]           = 48.0;
            grp.bus_current[j]           = std::abs(motors[j].torque) * 0.1;
        }
        grp.sequence = iteration;
        uint64_t sim_end = get_monotonic_ns();
        stats->motor_sim.record(sim_end - sim_start);

        // ---- Write to per-source staging buffer ----
        uint64_t write_start = get_monotonic_ns();
        grp.timestamp_ns = write_start;
        stage->publish(grp);
        uint64_t write_end = get_monotonic_ns();
        stats->stage_write.record(write_end - write_start);

        uint64_t cycle_end = get_monotonic_ns();
        stats->cycle.record(cycle_end - cycle_start);
        stats->iterations = ++iteration;

        // Sleep
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }

    printf("  %s thread stopped after %lu iterations\n", name, iteration);
}

// ============================================================
// Composer Thread — 1kHz
// ============================================================
void composer_thread_fn(SharedMemoryLayout* layout, ThreadStats* stats) {
    const uint64_t period_ns = 1'000'000;  // 1ms = 1kHz
    uint64_t next_wakeup = get_monotonic_ns();
    uint64_t iteration = 0;

    printf("  Composer thread started (1kHz)\n");

    while (g_running && !layout->emergency_stop.load(std::memory_order_acquire)) {
        uint64_t cycle_start = get_monotonic_ns();

        SensorData snapshot{};

        // ---- Read IMU stage ----
        ImuStageData imu = layout->imu_stage.read();
        std::memcpy(snapshot.imu_inc,     imu.imu_inc,     sizeof(imu.imu_inc));
        std::memcpy(snapshot.imu_ang_vel, imu.imu_ang_vel, sizeof(imu.imu_ang_vel));
        std::memcpy(snapshot.imu_acc,     imu.imu_acc,     sizeof(imu.imu_acc));
        snapshot.imu_timestamp_ns = imu.timestamp_ns;
        snapshot.imu_sequence     = imu.sequence;

        // ---- Read Motor Group A stage (joints 0-5) ----
        MotorGroupStageData grpA = layout->motor_group_a_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            snapshot.joint_jpos[j]              = grpA.joint_jpos[j];
            snapshot.joint_jvel[j]              = grpA.joint_jvel[j];
            snapshot.motor_jpos[j]              = grpA.motor_jpos[j];
            snapshot.motor_jvel[j]              = grpA.motor_jvel[j];
            snapshot.bus_current[j]             = grpA.bus_current[j];
            snapshot.bus_voltage[j]             = grpA.bus_voltage[j];
            snapshot.jtorque[j]                 = grpA.jtorque[j];
            snapshot.motor_current[j]           = grpA.motor_current[j];
            snapshot.reflected_rotor_inertia[j] = grpA.reflected_rotor_inertia[j];
        }
        snapshot.motor_group_a_timestamp_ns = grpA.timestamp_ns;
        snapshot.motor_group_a_sequence     = grpA.sequence;

        // ---- Read Motor Group B stage (joints 6-11) ----
        MotorGroupStageData grpB = layout->motor_group_b_stage.read();
        for (int j = 0; j < MOTORS_PER_GROUP; j++) {
            int idx = MOTORS_PER_GROUP + j;
            snapshot.joint_jpos[idx]              = grpB.joint_jpos[j];
            snapshot.joint_jvel[idx]              = grpB.joint_jvel[j];
            snapshot.motor_jpos[idx]              = grpB.motor_jpos[j];
            snapshot.motor_jvel[idx]              = grpB.motor_jvel[j];
            snapshot.bus_current[idx]             = grpB.bus_current[j];
            snapshot.bus_voltage[idx]             = grpB.bus_voltage[j];
            snapshot.jtorque[idx]                 = grpB.jtorque[j];
            snapshot.motor_current[idx]           = grpB.motor_current[j];
            snapshot.reflected_rotor_inertia[idx] = grpB.reflected_rotor_inertia[j];
        }
        snapshot.motor_group_b_timestamp_ns = grpB.timestamp_ns;
        snapshot.motor_group_b_sequence     = grpB.sequence;

        // ---- Read Contact stage ----
        ContactStageData contact = layout->contact_stage.read();
        snapshot.rfoot_contact = contact.rfoot_contact;
        snapshot.lfoot_contact = contact.lfoot_contact;

        // ---- Publish composed snapshot ----
        snapshot.compose_timestamp_ns = get_monotonic_ns();

        uint64_t write_start = get_monotonic_ns();
        uint32_t wb = 1 - layout->composed_write_idx.load(std::memory_order_acquire);
        std::memcpy(&layout->composed_buffers[wb], &snapshot, sizeof(SensorData));
        layout->composed_write_idx.store(wb, std::memory_order_release);
        layout->composed_sequence.fetch_add(1, std::memory_order_release);
        uint64_t write_end = get_monotonic_ns();
        stats->stage_write.record(write_end - write_start);

        uint64_t cycle_end = get_monotonic_ns();
        stats->cycle.record(cycle_end - cycle_start);
        stats->iterations = ++iteration;

        // Sleep
        next_wakeup += period_ns;
        uint64_t now = get_monotonic_ns();
        if (next_wakeup > now) {
            struct timespec ts;
            uint64_t sn = next_wakeup - now;
            ts.tv_sec = sn / 1'000'000'000ULL;
            ts.tv_nsec = sn % 1'000'000'000ULL;
            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
        } else {
            next_wakeup = now + period_ns;
        }
    }

    printf("  Composer thread stopped after %lu iterations\n", iteration);
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    double duration = 10.0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dur") == 0 && i + 1 < argc)
            duration = atof(argv[++i]);
        else if (strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [-dur sec]\n", argv[0]);
            return 0;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Attach to shared memory
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) { perror("shm_open (is controller running?)"); return 1; }
    void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { perror("mmap"); return 1; }

    auto* layout = static_cast<SharedMemoryLayout*>(ptr);
    if (layout->magic != SHM_MAGIC) {
        fprintf(stderr, "Invalid SHM magic: 0x%08X\n", layout->magic);
        return 1;
    }

    printf("Actuator attached: %u joints, version %u\n",
           layout->num_joints, layout->version);
    printf("Launching 4 threads (1 IMU + 2 Motor + 1 Composer)...\n\n");

    // Per-thread timing stats
    ThreadStats imu_stats, grpA_stats, grpB_stats, composer_stats;

    // Launch 4 threads
    std::thread t_imu(imu_thread_fn, layout, &imu_stats);
    std::thread t_grpA(motor_thread_fn, layout,
                       &layout->motor_group_a_stage, 0, "Motor Grp A", &grpA_stats);
    std::thread t_grpB(motor_thread_fn, layout,
                       &layout->motor_group_b_stage, 6, "Motor Grp B", &grpB_stats);
    std::thread t_composer(composer_thread_fn, layout, &composer_stats);

    // Wait for duration
    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration<double>(elapsed).count() >= duration) {
            g_running = false;
        }
    }

    t_imu.join();
    t_grpA.join();
    t_grpB.join();
    t_composer.join();

    // ---- Final Report ----
    printf("\n");
    printf("============================================================\n");
    printf("  ACTUATOR TIMING REPORT (Multi-Source v2)\n");
    printf("============================================================\n");

    printf("\n  IMU Thread (500Hz) — %lu iterations:\n", imu_stats.iterations);
    imu_stats.stage_write.print("Stage write (imu_stage)");
    imu_stats.cycle.print("Full cycle");

    printf("\n  Motor Grp A Thread (1kHz, joints 0-5) — %lu iterations:\n",
           grpA_stats.iterations);
    grpA_stats.read_cmd.print("SHM read (cmd fetch)");
    grpA_stats.motor_sim.print("Motor simulation (6 motors)");
    grpA_stats.stage_write.print("Stage write (motor_grp_a)");
    grpA_stats.cycle.print("Full cycle");
    printf("    Cmd stale reads: %lu\n", grpA_stats.cmd_stale);

    printf("\n  Motor Grp B Thread (1kHz, joints 6-11) — %lu iterations:\n",
           grpB_stats.iterations);
    grpB_stats.read_cmd.print("SHM read (cmd fetch)");
    grpB_stats.motor_sim.print("Motor simulation (6 motors)");
    grpB_stats.stage_write.print("Stage write (motor_grp_b)");
    grpB_stats.cycle.print("Full cycle");
    printf("    Cmd stale reads: %lu\n", grpB_stats.cmd_stale);

    printf("\n  Composer Thread (1kHz) — %lu iterations:\n",
           composer_stats.iterations);
    composer_stats.stage_write.print("Composed write");
    composer_stats.cycle.print("Full cycle (read 3 + merge + write)");

    printf("\n============================================================\n\n");

    munmap(layout, sizeof(SharedMemoryLayout));
    return 0;
}