/**
 * @file test_shm_lifecycle.cpp
 * @brief Integration tests: SHM owner (Robot) lifecycle scenarios
 *
 * The test harness creates and initializes SHM (simulating the Robot),
 * then launches mercury_controller as a consumer child process.
 * Test scenarios:
 *   T1  Controller attaches to existing Robot SHM
 *   T2  Robot graceful shutdown (lifecycle → SHUTTING_DOWN → TERMINATED,
 *       shm_unlink) — controller detects staleness and detaches
 *   T3  Robot crash (SHM removed externally) — controller detects and
 *       re-enters retry loop
 *   T4  Robot restart — controller reconnects to new SHM
 *
 * Usage:
 *   ./test_shm_lifecycle [path/to/mercury_controller]
 */

#include "../include/mercury_shm.h"

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace mercury;

// ============================================================
// Helpers
// ============================================================

static const char* PASS = "\033[32mPASS\033[0m";
static const char* FAIL = "\033[31mFAIL\033[0m";

static int g_assertions = 0;
static int g_failures   = 0;

#define ASSERT_TRUE(cond, msg)                                          \
    do {                                                                \
        g_assertions++;                                                 \
        if (!(cond)) {                                                  \
            g_failures++;                                               \
            printf("  [%s] %s  (%s:%d)\n", FAIL, msg, __FILE__, __LINE__); \
        } else {                                                        \
            printf("  [%s] %s\n", PASS, msg);                           \
        }                                                               \
    } while (0)

#define ASSERT_EQ(a, b, msg) ASSERT_TRUE((a) == (b), msg)

/// Create and initialize SHM as the Robot would.
/// Returns the mapped pointer or nullptr on failure.
static SharedMemoryLayout* create_robot_shm() {
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) { perror("shm_open"); return nullptr; }
    if (ftruncate(fd, sizeof(SharedMemoryLayout)) < 0) {
        perror("ftruncate"); close(fd); return nullptr;
    }
    void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (ptr == MAP_FAILED) { perror("mmap"); return nullptr; }

    auto* shm = static_cast<SharedMemoryLayout*>(ptr);

    // Initialize fields as Robot::robotInit() does
    shm->version = SHM_VERSION;
    shm->num_joints = NUM_ACT_JOINT;
    shm->control_freq_hz = 400;
    shm->robot_id = 1;
    shm->emergency_stop.store(false, std::memory_order_relaxed);
    shm->controller_emergency_stop.store(false, std::memory_order_relaxed);
    std::memset(shm->cmd_buffers, 0, sizeof(shm->cmd_buffers));
    std::memset(shm->composed_buffers, 0, sizeof(shm->composed_buffers));
    for (uint32_t i = 0; i < NUM_ACT_JOINT; i++) {
        shm->motor_can_ids[i] = static_cast<uint16_t>(i + 1);
    }

    // Write lifecycle and magic last (release order)
    shm->lifecycle_state.store(
        static_cast<uint32_t>(ShmLifecycle::RUNNING),
        std::memory_order_release);
    shm->magic.store(SHM_MAGIC, std::memory_order_release);

    return shm;
}

/// Simulate Robot's Composer writing composed sensor data with fresh timestamps.
static void update_compose_timestamp(SharedMemoryLayout* shm) {
    uint32_t wb = 1 - shm->composed_write_idx.load(std::memory_order_acquire);
    shm->composed_buffers[wb].compose_timestamp_ns = get_monotonic_ns();
    shm->composed_write_idx.store(wb, std::memory_order_release);
    shm->composed_sequence.fetch_add(1, std::memory_order_release);
}

static void close_shm(SharedMemoryLayout* shm) {
    if (shm) munmap(shm, sizeof(SharedMemoryLayout));
}

/// Launch the controller as a child process.  Returns the child pid, or -1.
static pid_t launch_controller(const char* controller_path) {
    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        return -1;
    }
    if (pid == 0) {
        // Child — redirect stdout/stderr to /dev/null to keep test output clean
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) { dup2(devnull, STDOUT_FILENO); dup2(devnull, STDERR_FILENO); close(devnull); }
        execl(controller_path, "mercury_controller",
              "-freq", "200", "-dur", "1000", "-joints", "12", nullptr);
        perror("execl");
        _exit(127);
    }
    return pid;
}

/// Send a signal and wait for the child to exit.
static bool stop_process(pid_t pid, int sig, int timeout_ms) {
    kill(pid, sig);
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        int status;
        pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) return true;
        usleep(10'000);
        elapsed += 10;
    }
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
    return false;
}

// ============================================================
// Test T1 — Controller attaches to existing Robot SHM
// ============================================================

static bool test_t1(const char* controller_path) {
    printf("\n============================================================\n");
    printf("  TEST T1: Controller attaches to existing Robot SHM\n");
    printf("============================================================\n");

    shm_unlink(SHM_NAME);

    // 1. Robot creates SHM
    SharedMemoryLayout* shm = create_robot_shm();
    ASSERT_TRUE(shm != nullptr, "Robot created SHM");
    if (!shm) return false;

    // 2. Start writing compose timestamps (simulate Composer)
    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm);
        usleep(10'000);
    }

    // 3. Launch controller
    pid_t pid = launch_controller(controller_path);
    ASSERT_TRUE(pid > 0, "Controller launched");
    if (pid <= 0) { close_shm(shm); shm_unlink(SHM_NAME); return false; }

    // 4. Give controller time to attach (up to 2 seconds)
    //    Controller should find SHM immediately since it already exists.
    printf("  Waiting for controller to attach (up to 2s)...\n");
    usleep(500'000);  // 500 ms should be plenty

    // Continue writing compose timestamps
    for (int i = 0; i < 10; i++) {
        update_compose_timestamp(shm);
        usleep(50'000);
    }

    // 5. Verify controller is writing commands
    uint64_t cmd_seq1 = shm->cmd_sequence.load(std::memory_order_acquire);
    for (int i = 0; i < 10; i++) {
        update_compose_timestamp(shm);
        usleep(50'000);
    }
    uint64_t cmd_seq2 = shm->cmd_sequence.load(std::memory_order_acquire);
    ASSERT_TRUE(cmd_seq2 > cmd_seq1, "Controller command sequence advancing");

    // 6. Verify command timestamps are fresh
    uint32_t idx = shm->cmd_write_idx.load(std::memory_order_acquire);
    if (idx <= 1) {
        uint64_t cmd_ts = shm->cmd_buffers[idx].timestamp_ns;
        uint64_t now = get_monotonic_ns();
        ASSERT_TRUE(cmd_ts > 0, "Controller command timestamp non-zero");
        if (cmd_ts > 0 && cmd_ts <= now) {
            uint64_t age_ms = (now - cmd_ts) / 1'000'000ULL;
            ASSERT_TRUE(age_ms < 200, "Controller command timestamp fresh (<200ms)");
        }
    }

    // Clean up
    stop_process(pid, SIGTERM, 3000);
    close_shm(shm);
    shm_unlink(SHM_NAME);

    printf("  ---- Test T1 complete ----\n");
    return true;
}

// ============================================================
// Test T2 — Robot graceful shutdown
// ============================================================

static bool test_t2(const char* controller_path) {
    printf("\n============================================================\n");
    printf("  TEST T2: Robot graceful shutdown (lifecycle transition)\n");
    printf("============================================================\n");

    shm_unlink(SHM_NAME);

    // 1. Robot creates SHM
    SharedMemoryLayout* shm = create_robot_shm();
    ASSERT_TRUE(shm != nullptr, "Robot created SHM");
    if (!shm) return false;

    // 2. Start compose timestamps
    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm);
        usleep(10'000);
    }

    // 3. Launch controller
    pid_t pid = launch_controller(controller_path);
    ASSERT_TRUE(pid > 0, "Controller launched");
    if (pid <= 0) { close_shm(shm); shm_unlink(SHM_NAME); return false; }

    // 4. Let controller run for a bit
    printf("  Letting controller run for 1 second...\n");
    for (int i = 0; i < 20; i++) {
        update_compose_timestamp(shm);
        usleep(50'000);
    }

    // Verify controller is writing
    uint64_t cmd_seq1 = shm->cmd_sequence.load(std::memory_order_acquire);
    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm);
        usleep(50'000);
    }
    uint64_t cmd_seq2 = shm->cmd_sequence.load(std::memory_order_acquire);
    ASSERT_TRUE(cmd_seq2 > cmd_seq1, "Controller writing commands before shutdown");

    // 5. Simulate Robot graceful shutdown (as ~Robot does)
    printf("  Simulating Robot graceful shutdown...\n");
    shm->lifecycle_state.store(
        static_cast<uint32_t>(ShmLifecycle::SHUTTING_DOWN),
        std::memory_order_release);
    // Stop updating compose timestamps (Robot is shutting down)
    shm->lifecycle_state.store(
        static_cast<uint32_t>(ShmLifecycle::TERMINATED),
        std::memory_order_release);

    // 6. Unmap and unlink (Robot calls detachSharedMemory + shm_unlink)
    close_shm(shm);
    shm = nullptr;
    shm_unlink(SHM_NAME);
    printf("  SHM unlinked.\n");

    // 7. Wait for controller to detect the loss.
    //    The controller checks compose_timestamp_ns staleness (>100ms).
    //    After shm_unlink, the fd is still open in the controller but
    //    compose data stops updating.  The controller should detect
    //    staleness within ~200ms and detach.
    printf("  Waiting 500ms for controller to detect staleness...\n");
    usleep(500'000);

    // 8. Verify the controller process is still alive (it re-enters retry loop)
    int status;
    pid_t r = waitpid(pid, &status, WNOHANG);
    ASSERT_TRUE(r == 0, "Controller still running (in retry loop, waiting for Robot)");

    // Clean up
    stop_process(pid, SIGTERM, 3000);

    printf("  ---- Test T2 complete ----\n");
    return true;
}

// ============================================================
// Test T3 — Robot crash (SHM removed externally)
// ============================================================

static bool test_t3(const char* controller_path) {
    printf("\n============================================================\n");
    printf("  TEST T3: Robot crash (SHM removed externally)\n");
    printf("============================================================\n");

    shm_unlink(SHM_NAME);

    // 1. Robot creates SHM
    SharedMemoryLayout* shm = create_robot_shm();
    ASSERT_TRUE(shm != nullptr, "Robot created SHM");
    if (!shm) return false;

    // 2. Start compose timestamps and launch controller
    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm);
        usleep(10'000);
    }

    pid_t pid = launch_controller(controller_path);
    ASSERT_TRUE(pid > 0, "Controller launched");
    if (pid <= 0) { close_shm(shm); shm_unlink(SHM_NAME); return false; }

    // 3. Let controller attach and run
    printf("  Letting controller run for 1 second...\n");
    for (int i = 0; i < 20; i++) {
        update_compose_timestamp(shm);
        usleep(50'000);
    }

    // 4. Simulate Robot crash: stop compose updates and unlink SHM
    //    (ExecStopPost would clean up)
    printf("  Simulating Robot crash (stop updates + unlink)...\n");
    close_shm(shm);
    shm = nullptr;
    shm_unlink(SHM_NAME);

    // 5. Wait for controller to detect staleness (>100ms compose age)
    printf("  Waiting 500ms for controller staleness detection...\n");
    usleep(500'000);

    // 6. Controller should still be alive (in retry loop)
    int status;
    pid_t r = waitpid(pid, &status, WNOHANG);
    ASSERT_TRUE(r == 0, "Controller still running (in retry loop after crash)");

    // Clean up
    stop_process(pid, SIGTERM, 3000);

    printf("  ---- Test T3 complete ----\n");
    return true;
}

// ============================================================
// Test T4 — Controller reconnects after Robot restart
// ============================================================

static bool test_t4(const char* controller_path) {
    printf("\n============================================================\n");
    printf("  TEST T4: Controller reconnects after Robot restart\n");
    printf("============================================================\n");

    shm_unlink(SHM_NAME);

    // --- Phase 1: Robot up, controller attaches ---
    SharedMemoryLayout* shm1 = create_robot_shm();
    ASSERT_TRUE(shm1 != nullptr, "Phase 1: Robot created SHM");
    if (!shm1) return false;

    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm1);
        usleep(10'000);
    }

    pid_t pid = launch_controller(controller_path);
    ASSERT_TRUE(pid > 0, "Controller launched");
    if (pid <= 0) { close_shm(shm1); shm_unlink(SHM_NAME); return false; }

    printf("  Phase 1: Letting controller run for 1 second...\n");
    for (int i = 0; i < 20; i++) {
        update_compose_timestamp(shm1);
        usleep(50'000);
    }

    uint64_t seq_before = shm1->cmd_sequence.load(std::memory_order_acquire);
    for (int i = 0; i < 5; i++) {
        update_compose_timestamp(shm1);
        usleep(50'000);
    }
    uint64_t seq_after = shm1->cmd_sequence.load(std::memory_order_acquire);
    ASSERT_TRUE(seq_after > seq_before, "Phase 1: Controller writing commands");

    // --- Phase 2: Robot crashes ---
    printf("  Phase 2: Simulating Robot crash...\n");
    close_shm(shm1);
    shm1 = nullptr;
    shm_unlink(SHM_NAME);

    // Wait for controller to detect staleness
    printf("  Waiting 500ms for staleness detection...\n");
    usleep(500'000);

    // Controller should still be alive (retry loop)
    int status;
    pid_t r = waitpid(pid, &status, WNOHANG);
    ASSERT_TRUE(r == 0, "Phase 2: Controller still alive (retry loop)");

    // --- Phase 3: Robot restarts ---
    printf("  Phase 3: Simulating Robot restart...\n");
    SharedMemoryLayout* shm2 = create_robot_shm();
    ASSERT_TRUE(shm2 != nullptr, "Phase 3: Robot re-created SHM");
    if (!shm2) { stop_process(pid, SIGKILL, 2000); return false; }

    // Write compose timestamps so the controller can validate
    printf("  Writing compose timestamps for 2 seconds...\n");
    for (int i = 0; i < 40; i++) {
        update_compose_timestamp(shm2);
        usleep(50'000);
    }

    // --- Phase 4: Verify controller reconnected ---
    uint64_t cmd_seq3 = shm2->cmd_sequence.load(std::memory_order_acquire);
    for (int i = 0; i < 10; i++) {
        update_compose_timestamp(shm2);
        usleep(50'000);
    }
    uint64_t cmd_seq4 = shm2->cmd_sequence.load(std::memory_order_acquire);
    ASSERT_TRUE(cmd_seq4 > cmd_seq3,
                "Phase 4: Controller reconnected and writing commands to new SHM");

    // Verify command timestamps are fresh
    uint32_t idx = shm2->cmd_write_idx.load(std::memory_order_acquire);
    if (idx <= 1) {
        uint64_t cmd_ts = shm2->cmd_buffers[idx].timestamp_ns;
        uint64_t now = get_monotonic_ns();
        if (cmd_ts > 0 && cmd_ts <= now) {
            uint64_t age_ms = (now - cmd_ts) / 1'000'000ULL;
            ASSERT_TRUE(age_ms < 200,
                        "Phase 4: Controller command timestamp fresh after reconnect");
        }
    }

    // Clean up
    stop_process(pid, SIGTERM, 3000);
    close_shm(shm2);
    shm_unlink(SHM_NAME);

    printf("  ---- Test T4 complete ----\n");
    return true;
}

// ============================================================
// Main
// ============================================================

int main(int argc, char* argv[]) {
    const char* controller_path = nullptr;
    if (argc > 1) {
        controller_path = argv[1];
    } else {
        const char* candidates[] = {
            "./mercury_controller",
            "../tools/mercury_controller",
            nullptr
        };
        for (int i = 0; candidates[i]; i++) {
            if (access(candidates[i], X_OK) == 0) {
                controller_path = candidates[i];
                break;
            }
        }
        if (!controller_path) {
            fprintf(stderr, "Usage: %s [path/to/mercury_controller]\n", argv[0]);
            fprintf(stderr, "Could not find mercury_controller in common locations.\n");
            return 1;
        }
    }

    printf("============================================================\n");
    printf("  SHM Lifecycle Integration Tests (Robot-owner model)\n");
    printf("  Controller: %s\n", controller_path);
    printf("============================================================\n");

    shm_unlink(SHM_NAME);

    test_t1(controller_path);
    test_t2(controller_path);
    test_t3(controller_path);
    test_t4(controller_path);

    printf("\n============================================================\n");
    printf("  SUMMARY: %d/%d assertions passed",
           g_assertions - g_failures, g_assertions);
    if (g_failures > 0)
        printf("  (%d FAILED)", g_failures);
    printf("\n============================================================\n\n");

    return g_failures > 0 ? 1 : 0;
}
