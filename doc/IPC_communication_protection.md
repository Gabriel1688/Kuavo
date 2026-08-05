
# Comprehensive Shared Memory Protection for Kuavo Robot Controller

## Problem

The `Legged::controllerPeriodic()` crashes at `std::atomic<uint32_t>::load()` (line 59) because `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance [1] **immediately in the constructor**, before the owner process (Kuavo Robot) has finished creating and initializing the POSIX shared memory region. The Mercury Controller is the **consumer** — it attaches to the Robot's shared memory but does not own or create it.

The constructor call chain shows the race clearly:

```
Robot::Robot() [Robot.h:64]
  → Legged::Legged() [Legged.cpp:24]
    → ControlledSubsystemBase() [ControlledSubsystemBase.h:51]
      → pthread_create() → Thread T4 starts IMMEDIATELY
        → controllerPeriodic() [Legged.cpp:59]
          → atomic<uint32_t>::load() → SEGV
```

The thread starts running `controllerPeriodic()` before the `Robot` constructor even finishes [1]. At this point, the shared memory region may not yet be fully initialized via `shm_open()` + `ftruncate()`.

---

## Solution: 5-Layer Defense-in-Depth

Each layer addresses a different failure mode. All layers are needed because no single layer covers every scenario.

```
Layer 1: Deferred Thread Start        → Prevents startup crash
Layer 2: Shared Memory Validation     → Prevents reading uninitialized SHM
Layer 3: Owner Lifecycle State         → Handles graceful owner (Robot) shutdown
Layer 4: Command Staleness Watchdog   → Detects controller disconnect
Layer 5: Safe Reconnection Loop       → Recovers from controller reconnection
```

---

## Layer 1: Deferred Thread Start (Fixes the Crash)

`ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance [1] in its constructor at line 51. The thread immediately starts executing `controllerPeriodic()`, which reads from shared memory that may not exist yet.

**Fix:** Separate object construction from thread launch. The pthread is created in the constructor but starts in a **suspended state** (blocked on a condition variable). An explicit `start()` method is called only after shared memory is verified.

```cpp
// ControlledSubsystemBase.h — modified constructor and start()

template <int States, int Inputs, int Outputs>
class ControlledSubsystemBase : public SubsystemBase {
public:
    ControlledSubsystemBase() {
        // Create the thread but have it wait on a condition variable
        // instead of immediately running controllerPeriodic()
        m_running.store(false, std::memory_order_release);
        pthread_create(&m_thread, nullptr, &EntryOfThread, this);
    }

    // Called AFTER shared memory is verified
    void start() {
        std::lock_guard<std::mutex> lock(m_startMutex);
        m_running.store(true, std::memory_order_release);
        m_startCond.notify_one();
    }

    void stop() {
        m_running.store(false, std::memory_order_release);
        m_startCond.notify_one();
    }

private:
    std::atomic<bool> m_running{false};
    std::mutex m_startMutex;
    std::condition_variable m_startCond;

    static void* EntryOfThread(void* arg) {
        auto* self = static_cast<ControlledSubsystemBase*>(arg);
        // Block until start() is called
        {
            std::unique_lock<std::mutex> lock(self->m_startMutex);
            self->m_startCond.wait(lock, [self] {
                return self->m_running.load(std::memory_order_acquire);
            });
        }
        self->Run();
        return nullptr;
    }

    void Run() {
        while (m_running.load(std::memory_order_acquire)) {
            controllerPeriodic();
            // ... period sleep logic
        }
    }
};
```

The caller in `Robot` defers `start()` until shared memory is ready:

```cpp
// Robot.cpp — robotInit() called AFTER constructor completes

void Robot::robotInit() {
    // Verify shared memory before starting any subsystem thread
    if (m_shm && m_shm->magic == SHM_MAGIC) {
        m_leftLeg.start();
        // m_rightLeg.start();  // when enabled [1]
    } else {
        spdlog::error("Shared memory not ready — subsystem threads not started");
    }
}
```

This directly addresses the race where Thread T4 reads shared memory before the Robot initializes it. The `ControlledSubsystemBase` still spawns a dedicated pthread per subsystem instance [1], but the thread waits for an explicit signal before accessing any shared state.

---

## Layer 2: Shared Memory Validation (Guards Every Read)

Even after `start()` is called, the shared memory could become invalid during operation (owner unmaps it, OS reclaims pages). Every access to shared memory in `controllerPeriodic()` must be guarded by a validity check.

The magic number pattern is used because the binary `RobotStatusWire` packet already uses magic number 0x4B564155 for validation [1] — the same pattern applies to shared memory.

```cpp
// mercury_shm.h — shared memory layout with magic validation

static constexpr uint32_t SHM_MAGIC   = 0x4D455243;  // "MERC"
static constexpr uint32_t SHM_VERSION = 5;

struct SharedMemoryLayout {
    std::atomic<uint32_t> magic;         // Must equal SHM_MAGIC
    std::atomic<uint32_t> version;       // Must equal SHM_VERSION (5)
    uint32_t num_joints;
    uint32_t control_freq_hz;

    // Lifecycle state (Layer 3)
    std::atomic<uint32_t> lifecycle_state;

    // Emergency stop flags
    std::atomic<bool> emergency_stop;             // Set by Robot when controller is stale
    std::atomic<bool> controller_emergency_stop;  // Set by Controller to signal e-stop to Robot

    // Command double buffer
    MercuryCommand cmd_buffers[2];       // Each contains a timestamp_ns field
    std::atomic<uint32_t> cmd_write_idx;
    // ... rest of layout
};
```

```cpp
// Legged.cpp — controllerPeriodic() with validation guard

void Legged::controllerPeriodic() {
    // Layer 2: Validate shared memory before every access
    if (!m_shm) {
        return;  // Pointer is null — not yet attached
    }

    uint32_t magic = m_shm->magic.load(std::memory_order_acquire);
    if (magic != SHM_MAGIC) {
        // SHM exists but owner has not initialized it yet,
        // or the memory has been corrupted/reclaimed
        return;
    }

    // Safe to read command buffer
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
    MercuryCommand cmd;
    std::memcpy(&cmd, &m_shm->cmd_buffers[rb], sizeof(MercuryCommand));

    // ... MIT command dispatch to motors [1]
}
```

This check runs every cycle (every 5 ms at the 200 Hz inner loop rate [2]). The `std::memory_order_acquire` ensures the magic check is not reordered past the subsequent reads.

---

## Layer 3: Owner Lifecycle State (Graceful Shutdown)

When the owner (Robot) shuts down gracefully, it should notify consumers before closing the shared memory. A `lifecycle_state` atomic variable inside the shared memory allows this coordination.

```cpp
// mercury_shm.h — lifecycle states

enum class ShmLifecycle : uint32_t {
    UNINITIALIZED = 0,     // Owner has not started
    INITIALIZING  = 1,     // Owner is setting up fields
    RUNNING       = 2,     // Normal operation — safe to read/write
    SHUTTING_DOWN = 3,     // Owner is about to exit
    TERMINATED    = 4,     // Owner has exited
};
```

**Owner side** (Robot — the SHM owner):

```cpp
// Owner startup — Robot creates and initializes SHM
int fd = shm_open("/mercury_robot_ipc", O_CREAT | O_RDWR, 0666);
ftruncate(fd, sizeof(SharedMemoryLayout));
void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                 PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
close(fd);
auto* shm = static_cast<SharedMemoryLayout*>(ptr);

shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::INITIALIZING),
    std::memory_order_release);

// ... initialize all fields ...

shm->magic.store(SHM_MAGIC, std::memory_order_release);
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::RUNNING),
    std::memory_order_release);

// Owner shutdown (graceful) — Robot tears down SHM
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::SHUTTING_DOWN),
    std::memory_order_release);
// ... cleanup ...
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::TERMINATED),
    std::memory_order_release);
munmap(shm, sizeof(SharedMemoryLayout));
shm_unlink("/mercury_robot_ipc");  // Robot calls shm_unlink on shutdown
```

**Consumer side** (Mercury Controller — attaches to Robot's SHM):

```cpp
// Controller attaches — no O_CREAT, controller never creates SHM
int fd = shm_open("/mercury_robot_ipc", O_RDWR, 0666);
if (fd < 0) {
    // Robot has not created SHM yet — retry later
    return;
}
// ... fstat + mmap as in Layer 5 ...

// Controller checks lifecycle in its loop
auto state = static_cast<ShmLifecycle>(
    shm->lifecycle_state.load(std::memory_order_acquire));

if (state != ShmLifecycle::RUNNING) {
    if (state == ShmLifecycle::SHUTTING_DOWN ||
        state == ShmLifecycle::TERMINATED) {
        // Owner (Robot) is shutting down — stop sending commands
        spdlog::warn("Robot shutting down — controller detaching");
    }
    return;  // Do not write command buffer
}
```

**Robot side** — reads lifecycle and reacts to controller disconnect:

```cpp
void Legged::controllerPeriodic() {
    if (!m_shm) return;

    // Layer 2: Magic check
    if (m_shm->magic.load(std::memory_order_acquire) != SHM_MAGIC) {
        return;
    }

    // Layer 3: Lifecycle check
    auto state = static_cast<ShmLifecycle>(
        m_shm->lifecycle_state.load(std::memory_order_acquire));

    if (state != ShmLifecycle::RUNNING) {
        if (state == ShmLifecycle::SHUTTING_DOWN ||
            state == ShmLifecycle::TERMINATED) {
            // Owner is shutting down — disable motors safely
            // Motor disable requires 0xFD command [1]
            disableAllMotors();
            spdlog::warn("Owner shutting down — motors disabled");
        }
        return;  // Do not read command buffer
    }

    // Safe to proceed with normal operation
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
    MercuryCommand cmd;
    std::memcpy(&cmd, &m_shm->cmd_buffers[rb], sizeof(MercuryCommand));
    // ... dispatch MIT commands [1]
}
```

Motor disable uses the 0xFD command — the same command used by the existing motor safety state machine which enforces disable via explicit 0xFD [1].

---

## Layer 4: Command Staleness Watchdog (Detects Controller Disconnect)

Layer 3 handles graceful shutdown but not crashes. If the controller process (Mercury Controller) is killed by SIGKILL or segfaults, the `lifecycle_state` field will remain at `RUNNING` forever. The Robot detects a dead controller by checking the staleness of the most recent command timestamp in the shared memory.

The motor responsiveness timeout is 500 ms [2] — the staleness timeout should be tighter to detect controller death before the motor timeout fires.

```cpp
// mercury_shm.h — command buffer includes timestamp
// Each MercuryCommand contains a timestamp_ns field written by the controller.
// The controller_emergency_stop field allows the controller to signal e-stop.
// std::atomic<bool> controller_emergency_stop;  // (already in SharedMemoryLayout above)
```

**Controller side** — updates command timestamp every cycle:

```cpp
// In the controller's control loop (e.g., every 5 ms):
auto& buf = shm->cmd_buffers[next_write_idx];
buf.timestamp_ns = get_monotonic_ns();
// ... fill command fields ...
shm->cmd_write_idx.store(next_write_idx, std::memory_order_release);
```

**Robot side** — checks command timestamp staleness:

```cpp
static constexpr uint64_t CMD_STALENESS_TIMEOUT_NS = 100'000'000;  // 100 ms
// Tighter than the 500 ms motor responsiveness timeout [2]

void Legged::controllerPeriodic() {
    if (!m_shm) return;

    // Layer 2: Magic
    if (m_shm->magic.load(std::memory_order_acquire) != SHM_MAGIC) return;

    // Layer 3: Lifecycle
    auto state = static_cast<ShmLifecycle>(
        m_shm->lifecycle_state.load(std::memory_order_acquire));
    if (state != ShmLifecycle::RUNNING) {
        disableAllMotors();
        return;
    }

    // Layer 4a: Controller emergency stop
    if (m_shm->controller_emergency_stop.load(std::memory_order_acquire)) {
        spdlog::error("Controller signalled emergency stop — disabling motors");
        disableAllMotors();
        m_controllerAlive = false;
        return;
    }

    // Layer 4b: Command timestamp staleness
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
    uint64_t cmd_timestamp = m_shm->cmd_buffers[rb].timestamp_ns;
    uint64_t now = get_monotonic_ns();

    if (cmd_timestamp == 0) {
        // Controller has never written a command — skip
        return;
    }

    if ((now - cmd_timestamp) > CMD_STALENESS_TIMEOUT_NS) {
        // Controller has not updated commands for > 100 ms
        // Motor responsiveness timeout is 500 ms [2] — we detect first
        spdlog::error("Controller commands stale ({} ms) — setting emergency stop",
                      (now - cmd_timestamp) / 1'000'000);
        m_shm->emergency_stop.store(true, std::memory_order_release);
        disableAllMotors();
        m_controllerAlive = false;
        return;
    }

    m_controllerAlive = true;

    // Safe to read commands
    MercuryCommand cmd;
    std::memcpy(&cmd, &m_shm->cmd_buffers[rb], sizeof(MercuryCommand));
    // ... dispatch
}
```

The 100 ms timeout is chosen to be tighter than the 500 ms motor responsiveness timeout [2], ensuring the Robot detects a dead controller before the motors are individually flagged as unresponsive.

---

## Layer 5: Safe Reconnection Loop (Controller Reconnection Recovery)

After the controller disconnects and reconnects, it attaches to the Robot's **existing** shared memory region. The Robot owns the SHM for its entire lifetime — the controller is always a consumer. The reconnection check runs in `robotPeriodic()` (the 20 ms / 50 Hz main loop [1]), not in `controllerPeriodic()`.

```cpp
// Robot.cpp — reconnection logic in robotPeriodic()

void Robot::robotPeriodic() {
    // ... button events via m_loop.poll() [1]
    // ... mode management [1]

    // Layer 5: Controller reconnection check (runs at 50 Hz)
    if (!m_leftLeg.isControllerAlive()) {
        spdlog::warn("Controller lost — waiting for reconnection...");

        // Stop the subsystem thread safely
        m_leftLeg.stop();

        // Clear the emergency_stop flag so the controller can reconnect
        if (m_shm) {
            m_shm->emergency_stop.store(false, std::memory_order_release);
            m_shm->controller_emergency_stop.store(false, std::memory_order_release);
        }

        // Check if controller has started writing fresh commands
        if (m_shm && m_shm->magic.load(std::memory_order_acquire) == SHM_MAGIC) {
            auto state = static_cast<ShmLifecycle>(
                m_shm->lifecycle_state.load(std::memory_order_acquire));

            if (state == ShmLifecycle::RUNNING) {
                uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
                uint64_t cmd_ts = m_shm->cmd_buffers[rb].timestamp_ns;
                uint64_t now = get_monotonic_ns();

                if (cmd_ts != 0 && (now - cmd_ts) < CMD_STALENESS_TIMEOUT_NS) {
                    // Controller is back and writing fresh commands
                    m_leftLeg.setSharedMemory(m_shm);
                    m_leftLeg.start();
                    spdlog::info("Controller reconnected — subsystem restarted");
                }
            }
        }
    }
}
```

**Controller side** — attaches to Robot's SHM (consumer, no `O_CREAT`):

```cpp
SharedMemoryLayout* Controller::tryAttachSharedMemory() {
    // Controller opens existing SHM — no O_CREAT, never creates
    int fd = shm_open("/mercury_robot_ipc", O_RDWR, 0666);
    if (fd < 0) {
        return nullptr;  // Robot has not created SHM yet
    }

    // Check the file size before mapping
    struct stat st;
    if (fstat(fd, &st) < 0 || st.st_size < static_cast<off_t>(sizeof(SharedMemoryLayout))) {
        close(fd);
        return nullptr;  // SHM exists but not yet sized by ftruncate()
    }

    void* ptr = mmap(nullptr, sizeof(SharedMemoryLayout),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<SharedMemoryLayout*>(ptr);
}

// Controller NEVER calls shm_unlink — only the Robot does on shutdown.
```

The `fstat()` check before `mmap()` is critical — it prevents the exact crash you experienced. If the Robot has called `shm_open()` but not yet `ftruncate()`, the file size is 0. Mapping it and accessing any offset beyond byte 0 triggers SIGSEGV because there is no backing page. The `fstat()` check catches this case before `mmap()`.

---

## Motor Disable Helper

All layers call `disableAllMotors()` when the controller becomes unavailable. This sends the 0xFD disable command to all motors, using the same motor safety mechanism already in the system — motor disable requires explicit 0xFD command [1]:

```cpp
void Legged::disableAllMotors() {
    // Motor disable requires 0xFD command [1]
    // Motor device IDs 1-5 left leg, 6-10 right leg [1]
    for (auto& motor : m_motors) {
        motor.sendCommand(0xFD);  // Disable
    }
    spdlog::warn("All {} motors disabled (safety shutdown)", m_motors.size());
}
```

Motor safety state machine already handles the 0xFD command — it transitions the motor to the disabled state where overvoltage (0x08), undervoltage (0x09), overcurrent (0x0A), MOS overtemp (0x0B), coil overtemp (0x0C), comm loss (0x0D), and overload (0x0E) are all cleared [1].

---

## Complete controllerPeriodic() with All Layers

```cpp
void Legged::controllerPeriodic() {
    // Layer 1: Thread only runs after start() is called
    // (enforced by ControlledSubsystemBase condition variable)

    // Layer 2: Shared memory pointer and magic validation
    if (!m_shm) {
        return;
    }
    if (m_shm->magic.load(std::memory_order_acquire) != SHM_MAGIC) {
        return;
    }

    // Layer 3: Owner lifecycle state
    auto lifecycle = static_cast<ShmLifecycle>(
        m_shm->lifecycle_state.load(std::memory_order_acquire));
    if (lifecycle != ShmLifecycle::RUNNING) {
        if (lifecycle == ShmLifecycle::SHUTTING_DOWN ||
            lifecycle == ShmLifecycle::TERMINATED) {
            disableAllMotors();
        }
        m_controllerAlive = false;
        return;
    }

    // Layer 4a: Controller emergency stop
    if (m_shm->controller_emergency_stop.load(std::memory_order_acquire)) {
        disableAllMotors();
        m_controllerAlive = false;
        return;
    }

    // Layer 4b: Command staleness watchdog
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
    uint64_t cmd_timestamp = m_shm->cmd_buffers[rb].timestamp_ns;
    uint64_t now = get_monotonic_ns();

    if (cmd_timestamp == 0 || (now - cmd_timestamp) > CMD_STALENESS_TIMEOUT_NS) {
        m_shm->emergency_stop.store(true, std::memory_order_release);
        disableAllMotors();
        m_controllerAlive = false;
        return;
    }

    m_controllerAlive = true;

    // All layers passed — safe to read commands
    MercuryCommand cmd;
    std::memcpy(&cmd, &m_shm->cmd_buffers[rb], sizeof(MercuryCommand));

    // Dispatch MIT commands to motors
    // MIT control parameters bounded by motor-type limits [1]:
    //   DM8009: ±12.5 rad position, ±45 rad/s velocity, ±54 Nm torque
    for (int j = 0; j < m_motorCount; j++) {
        if (cmd.enabled[j]) {
            m_motors[j].setMitControl(
                cmd.jpos_cmd[j], cmd.jvel_cmd[j],
                cmd.kp[j], cmd.kd[j], cmd.jtorque_cmd[j]);
        }
    }

    // Layer 5: Reconnection is handled in robotPeriodic() (main loop)
}
```

---

## Layer Summary

| Layer | Failure Scenario | Detection | Response | Where |
|:-----:|-----------------|-----------|----------|-------|
| **1** | Thread starts before SHM exists | Condition variable blocks thread until `start()` called | Thread waits — no crash | `ControlledSubsystemBase` constructor [1] |
| **2** | SHM exists but not initialized | `magic != SHM_MAGIC` | Skip cycle, return | `controllerPeriodic()` every 5 ms [1] |
| **3** | Owner (Robot) shuts down gracefully | `lifecycle_state == SHUTTING_DOWN` | Disable motors (0xFD) [1], stop reading | `controllerPeriodic()` every 5 ms |
| **4** | Controller crashes or disconnects | `now - cmd_timestamp > 100 ms` or `controller_emergency_stop` | Set `emergency_stop`, disable motors, flag `m_controllerAlive = false` | `controllerPeriodic()` every 5 ms |
| **5** | Controller reconnects to Robot's SHM | `m_controllerAlive == false` + fresh `cmd_timestamp` in main loop | Clear e-stop, restart subsystem thread | `robotPeriodic()` every 20 ms [1] |