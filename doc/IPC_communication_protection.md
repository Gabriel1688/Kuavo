
# Comprehensive Shared Memory Protection for Kuavo Robot Controller

## Problem

The `Legged::controllerPeriodic()` crashes at `std::atomic<uint32_t>::load()` (line 59) because `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance [1] **immediately in the constructor**, before the producer process (Mercury Controller) has created or initialized the POSIX shared memory region. This process is the **consumer only** — it does not own the shared memory lifecycle.

The constructor call chain shows the race clearly:

```
Robot::Robot() [Robot.h:64]
  → Legged::Legged() [Legged.cpp:24]
    → ControlledSubsystemBase() [ControlledSubsystemBase.h:51]
      → pthread_create() → Thread T4 starts IMMEDIATELY
        → controllerPeriodic() [Legged.cpp:59]
          → atomic<uint32_t>::load() → SEGV
```

The thread starts running `controllerPeriodic()` before the `Robot` constructor even finishes [1]. At this point, the shared memory pointer may reference a region that the producer has not yet created via `shm_open()` + `ftruncate()`.

---

## Solution: 5-Layer Defense-in-Depth

Each layer addresses a different failure mode. All layers are needed because no single layer covers every scenario.

```
Layer 1: Deferred Thread Start        → Prevents startup crash
Layer 2: Shared Memory Validation     → Prevents reading uninitialized SHM
Layer 3: Producer Lifecycle State      → Handles graceful producer shutdown
Layer 4: Heartbeat Watchdog           → Detects producer crash
Layer 5: Safe Reconnection Loop       → Recovers from producer restart
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

This directly addresses the race where Thread T4 reads shared memory before the producer initializes it. The `ControlledSubsystemBase` still spawns a dedicated pthread per subsystem instance [1], but the thread waits for an explicit signal before accessing any shared state.

---

## Layer 2: Shared Memory Validation (Guards Every Read)

Even after `start()` is called, the shared memory could become invalid during operation (producer unmaps it, OS reclaims pages). Every access to shared memory in `controllerPeriodic()` must be guarded by a validity check.

The magic number pattern is used because the binary `RobotStatusWire` packet already uses magic number 0x4B564155 for validation [1] — the same pattern applies to shared memory.

```cpp
// mercury_shm.h — shared memory layout with magic validation

static constexpr uint32_t SHM_MAGIC = 0x4D455243;  // "MERC"

struct SharedMemoryLayout {
    std::atomic<uint32_t> magic;         // Must equal SHM_MAGIC
    std::atomic<uint32_t> version;       // Protocol version
    uint32_t num_joints;
    uint32_t control_freq_hz;

    // Lifecycle state (Layer 3)
    std::atomic<uint32_t> lifecycle_state;

    // Heartbeat (Layer 4)
    std::atomic<uint64_t> producer_heartbeat_ns;

    // Command double buffer
    MercuryCommand cmd_buffers[2];
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
        // SHM exists but producer has not initialized it yet,
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

## Layer 3: Producer Lifecycle State (Graceful Shutdown)

When the producer (Mercury Controller) shuts down gracefully, it should notify the consumer before closing the shared memory. A `lifecycle_state` atomic variable inside the shared memory allows this coordination.

```cpp
// mercury_shm.h — lifecycle states

enum class ShmLifecycle : uint32_t {
    UNINITIALIZED = 0,     // Producer has not started
    INITIALIZING  = 1,     // Producer is setting up fields
    RUNNING       = 2,     // Normal operation — safe to read
    SHUTTING_DOWN = 3,     // Producer is about to exit
    TERMINATED    = 4,     // Producer has exited
};
```

**Producer side** (Mercury Controller — the other process):

```cpp
// Producer startup
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::INITIALIZING),
    std::memory_order_release);

// ... initialize all fields ...

shm->magic.store(SHM_MAGIC, std::memory_order_release);
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::RUNNING),
    std::memory_order_release);

// Producer shutdown (graceful)
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::SHUTTING_DOWN),
    std::memory_order_release);
// ... cleanup ...
shm->lifecycle_state.store(
    static_cast<uint32_t>(ShmLifecycle::TERMINATED),
    std::memory_order_release);
```

**Consumer side** (Kuavo — `controllerPeriodic()`):

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
            // Producer is shutting down — disable motors safely
            // Motor disable requires 0xFD command [1]
            disableAllMotors();
            spdlog::warn("Producer shutting down — motors disabled");
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

## Layer 4: Heartbeat Watchdog (Detects Producer Crash)

Layer 3 handles graceful shutdown but not crashes. If the producer process is killed by SIGKILL or segfaults, the `lifecycle_state` field will remain at `RUNNING` forever. A heartbeat timestamp detects this case.

The motor responsiveness timeout is 500 ms [2] — the heartbeat timeout should be tighter to detect producer death before the motor timeout fires.

```cpp
// mercury_shm.h — heartbeat field (already in SharedMemoryLayout above)
// std::atomic<uint64_t> producer_heartbeat_ns;
```

**Producer side** — updates heartbeat every cycle:

```cpp
// In the producer's control loop (e.g., every 5 ms):
shm->producer_heartbeat_ns.store(
    get_monotonic_ns(), std::memory_order_release);
```

**Consumer side** — checks heartbeat staleness:

```cpp
static constexpr uint64_t HEARTBEAT_TIMEOUT_NS = 200'000'000;  // 200 ms
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

    // Layer 4: Heartbeat
    uint64_t last_heartbeat =
        m_shm->producer_heartbeat_ns.load(std::memory_order_acquire);
    uint64_t now = get_monotonic_ns();

    if (last_heartbeat == 0) {
        // Producer has never written a heartbeat
        return;
    }

    if ((now - last_heartbeat) > HEARTBEAT_TIMEOUT_NS) {
        // Producer has not updated heartbeat for > 200 ms
        // Motor responsiveness timeout is 500 ms [2] — we detect first
        spdlog::error("Producer heartbeat stale ({} ms) — disabling motors",
                      (now - last_heartbeat) / 1'000'000);
        disableAllMotors();
        m_producerAlive = false;
        return;
    }

    m_producerAlive = true;

    // Safe to read commands
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
    MercuryCommand cmd;
    std::memcpy(&cmd, &m_shm->cmd_buffers[rb], sizeof(MercuryCommand));
    // ... dispatch
}
```

The 200 ms timeout is chosen to be tighter than the 500 ms motor responsiveness timeout [2], ensuring the consumer detects a dead producer before the motors are individually flagged as unresponsive.

---

## Layer 5: Safe Reconnection Loop (Producer Restart Recovery)

After the producer crashes and restarts, it creates a **new** shared memory region. The consumer must detect this, detach from the stale region, and reattach to the new one. This runs in `robotPeriodic()` (the 20 ms / 50 Hz main loop [1]), not in `controllerPeriodic()`.

```cpp
// Robot.cpp — reconnection logic in robotPeriodic()

void Robot::robotPeriodic() {
    // ... button events via m_loop.poll() [1]
    // ... mode management [1]

    // Layer 5: SHM reconnection check (runs at 50 Hz)
    if (!m_leftLeg.isProducerAlive()) {
        spdlog::warn("Producer lost — attempting reconnection...");

        // Stop the subsystem thread safely
        m_leftLeg.stop();

        // Detach from stale shared memory
        if (m_shm) {
            munmap(m_shm, sizeof(SharedMemoryLayout));
            m_shm = nullptr;
        }

        // Attempt to reattach
        m_shm = tryAttachSharedMemory();

        if (m_shm && m_shm->magic.load(std::memory_order_acquire) == SHM_MAGIC) {
            auto state = static_cast<ShmLifecycle>(
                m_shm->lifecycle_state.load(std::memory_order_acquire));

            if (state == ShmLifecycle::RUNNING) {
                // Producer is back and initialized
                m_leftLeg.setSharedMemory(m_shm);
                m_leftLeg.start();
                spdlog::info("Reconnected to producer — subsystem restarted");
            }
        }
    }
}

SharedMemoryLayout* Robot::tryAttachSharedMemory() {
    int fd = shm_open("/mercury_robot_ipc", O_RDWR, 0666);
    if (fd < 0) {
        return nullptr;  // Producer has not created SHM yet
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
```

The `fstat()` check before `mmap()` is critical — it prevents the exact crash you experienced. If the producer has called `shm_open()` but not yet `ftruncate()`, the file size is 0. Mapping it and accessing any offset beyond byte 0 triggers SIGSEGV because there is no backing page. The `fstat()` check catches this case before `mmap()`.

---

## Motor Disable Helper

All layers call `disableAllMotors()` when the producer becomes unavailable. This sends the 0xFD disable command to all motors, using the same motor safety mechanism already in the system — motor disable requires explicit 0xFD command [1]:

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

    // Layer 3: Producer lifecycle state
    auto lifecycle = static_cast<ShmLifecycle>(
        m_shm->lifecycle_state.load(std::memory_order_acquire));
    if (lifecycle != ShmLifecycle::RUNNING) {
        if (lifecycle == ShmLifecycle::SHUTTING_DOWN ||
            lifecycle == ShmLifecycle::TERMINATED) {
            disableAllMotors();
        }
        m_producerAlive = false;
        return;
    }

    // Layer 4: Heartbeat watchdog
    uint64_t heartbeat =
        m_shm->producer_heartbeat_ns.load(std::memory_order_acquire);
    uint64_t now = get_monotonic_ns();

    if (heartbeat == 0 || (now - heartbeat) > HEARTBEAT_TIMEOUT_NS) {
        disableAllMotors();
        m_producerAlive = false;
        return;
    }

    m_producerAlive = true;

    // All layers passed — safe to read commands
    uint32_t rb = m_shm->cmd_write_idx.load(std::memory_order_acquire);
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
| **3** | Producer shuts down gracefully | `lifecycle_state == SHUTTING_DOWN` | Disable motors (0xFD) [1], stop reading | `controllerPeriodic()` every 5 ms |
| **4** | Producer crashes unexpectedly | `now - heartbeat > 200 ms` | Disable motors, flag `m_producerAlive = false` | `controllerPeriodic()` every 5 ms |
| **5** | Producer restarts with new SHM | `m_producerAlive == false` in main loop | Detach, reattach (`fstat` + `mmap`), restart thread | `robotPeriodic()` every 20 ms [1] |