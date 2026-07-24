# IPC Shared Memory Safety Handling — Kuavo Robot Controller

This document captures the design decisions from the safety review of the POSIX shared memory (SHM) path between the Mercury Controller producer (`tools/mercury_controller.cpp`) and the Kuavo consumer (`src/Robot.cpp` / `src/subsystems/Legged.cpp`).

## 1. Scope and Goal

- Prevent the consumer from reading uninitialized or stale shared memory.
- Make the consumer resilient to producer crash, restart, and graceful shutdown.
- Keep all real-time motor safety paths inside `Legged::controllerPeriodic()` at 400 Hz.
- Keep producer lifecycle management explicit and owned by the producer only.

## 2. Shared Memory Lifecycle

| Rule | Decision |
|------|----------|
| Owner | Producer only. `mercury_controller.cpp` creates and sizes the SHM. Kuavo `Robot` is a strict consumer. |
| Missing at startup | `Robot::robotInit()` logs a distinct error and calls `std::exit(EXIT_FAILURE)` immediately. No `O_CREAT` fallback. |
| Missing fd | `Robot` does not keep `m_shm_fd`; the descriptor is closed after `mmap`. `Robot` only tracks `m_shm`. |
| Mapping lifetime | The mapping is replaced atomically on reconnection. Old mappings are `munmap`ed before a new `shm_open` is accepted. |

```
Producer                           Consumer (Robot)
  |                                     |
  | shm_open + ftruncate                |
  | write fields + magic + RUNNING      |
  |------------------------------------>| robotInit: shm_open, fstat,
  | heartbeat updates                 |            mmap, validate, start
  |                                     |
  | SHUTTING_DOWN + emergency_stop      |
  | TERMINATED                          |
  | munmap + shm_unlink                 | robotPeriodic: detect loss,
  |                                     |              detach, retry attach
  | (restarts)                          |
  | new shm with same name              | robotPeriodic: attach new, restart
```

## 3. Shared Memory Layout Changes

`tools/mercury_shm_v2.h` is updated. Both producer and consumer are recompiled; no backward compatibility is preserved.

```cpp
static constexpr uint32_t SHM_MAGIC   = 0x4D455243; // "MERC"
static constexpr uint32_t SHM_VERSION = 3;

enum class ShmLifecycle : uint32_t {
    UNINITIALIZED = 0,
    INITIALIZING  = 1,
    RUNNING       = 2,
    SHUTTING_DOWN = 3,
    TERMINATED    = 4,
};

struct SharedMemoryLayout {
    std::atomic<uint32_t> magic;
    uint32_t version;
    uint32_t num_joints;
    uint32_t control_freq_hz;

    // existing staging and command buffers ...

    std::atomic<uint32_t> lifecycle_state;
    std::atomic<uint64_t> controller_heartbeat_ns;

    // ... emergency_stop, motor_can_ids, robot_id ...
};
```

- `magic` becomes `std::atomic<uint32_t>`.
- `lifecycle_state` is added as `std::atomic<uint32_t>`.
- `controller_heartbeat_ns` is the existing heartbeat field.
- `version` is bumped to `3`.
- `static_assert`s are added to verify atomic field size/alignment and expected layout offsets.

## 4. Producer Behavior

1. `mercury_controller.cpp` `shm_open`s and `ftruncate`s the SHM.
2. Writes `version`, `num_joints`, `control_freq_hz`, `robot_id`, `motor_can_ids`, and command buffers.
3. Writes `lifecycle_state = RUNNING`.
4. Writes `magic = SHM_MAGIC` as the final initialization sentinel (release ordering).
5. Every control cycle: updates `controller_heartbeat_ns`.
6. On graceful shutdown:
   - `lifecycle_state = SHUTTING_DOWN`
   - `emergency_stop = true`
   - `lifecycle_state = TERMINATED`
   - `munmap`
   - `shm_unlink`

`magic` is written once at startup and never modified per cycle.

## 5. Consumer Read Guards — `Legged::controllerPeriodic()`

The 400 Hz motor control loop checks in this order:

1. `if (!m_shm) return;`
2. `if (magic.load(acquire) != SHM_MAGIC) return;`
3. `if (version != SHM_VERSION) { disableAllMotorsOnce(); return; }`
4. `if (lifecycle_state != RUNNING) { disableAllMotorsOnce(); return; }`
5. `if (heartbeat == 0 || now - heartbeat > 100ms) { disableAllMotorsOnce(); return; }`
6. `if (emergency_stop.load()) { disableAllMotorsOnce(); return; }`
7. Validate `cmd_write_idx` is `0` or `1`.
8. Validate command freshness (existing 100 ms threshold).
9. Dispatch MIT motor commands.

`disableAllMotorsOnce()` is a one-shot fault disable. It is re-armed only in `Legged::setEnable(true)`, which the operator explicitly calls after a successful reconnection.

## 6. Robot Reconnection Flow

`Robot::robotPeriodic()` runs at 100 Hz. The reconnection check is placed at the top, before `Composer`/`Logger` use.

### On producer loss

```
robotPeriodic()
  -> detect invalid magic / lifecycle / heartbeat
  -> leftLeg.setEnable(false)   // disable motors via leg path
  -> pause leg threads          // ControlledSubsystemBase pause()/resume()
  -> imu_subsystem.stop()
  -> m_composer.reset()
  -> m_logger.reset()           // drain old ring, then recreate
  -> munmap(m_shm)
  -> m_shm = nullptr
  -> reset leg/IMU staging pointers
```

### Re-attach retry

- Every 100 ms (10 `robotPeriodic` cycles), call `tryAttachSharedMemory()`.
- `tryAttachSharedMemory()` does `shm_open` + `fstat` + `mmap` + validation.
- Validation requires `magic`, `version`, `lifecycle_state == RUNNING`, and a non-stale `controller_heartbeat_ns`.
- Each failure stage logs a distinct `SPDLOG_ERROR`.
- `fstat` requires `st.st_size >= sizeof(SharedMemoryLayout)`.

### On successful re-attach

```
  -> m_shm = new valid mapping
  -> imu_subsystem.setStagingBuffer(&m_shm->imu_stage)
  -> imu_subsystem.start()
  -> leftLeg.setShmPointers(m_shm, &m_shm->motor_group_a_stage)
  -> rightLeg.setShmPointers(m_shm, &m_shm->motor_group_b_stage)
  -> m_composer = make_unique<Composer>(m_shm references, ...)
  -> m_logger    = make_unique<Logger>(m_logRing, mqtt, robot_id)
  -> resume leg threads
  // Legs remain disabled until the operator explicitly enables them.
```

## 7. Safe Thread Pause for Detach

`ControlledSubsystemBase` gets a lightweight `pause()` / `resume()` pair:

```cpp
std::atomic<bool> m_pauseRequested{false};
std::atomic<bool> m_inControllerPeriodic{false};

// Run() checks the gate before each controllerPeriodic() call:
if (now >= next_wake && m_isEnabled && !m_pauseRequested) {
    m_inControllerPeriodic = true;
    controllerPeriodic();
    m_inControllerPeriodic = false;
}

// pause() spins with yield until m_inControllerPeriodic is false,
// then it is safe to munmap the old SHM.
```

This does not change the initial thread start behavior; the existing `m_isEnabled` gate is kept.

## 8. Composer, Logger, and IMU Lifecycle

- `Composer` accesses `m_shm` every 400 Hz cycle. It must be stopped before `munmap` and recreated after a valid re-attach.
- `Logger` is recreated on re-attach (not just shutdown/start) to reset its internal connect-delay state.
- `Imu`/`ImuReader` writes to `m_shm->imu_stage`. It must be stopped and restarted with the new staging buffer pointer.
- The `m_logRing` is not cleared; `Logger` drains any records that were queued before the disconnect.

## 9. Validation Checklist

| Where | What is checked | Failure response |
|-------|-----------------|------------------|
| `Robot::robotInit()` | `shm_open`, `fstat`, `mmap`, `magic`, `version`, `lifecycle`, heartbeat | `std::exit(EXIT_FAILURE)` |
| `Robot::tryAttachSharedMemory()` | same set as above | return `nullptr`; `robotPeriodic` retries every 100 ms |
| `Robot::robotPeriodic()` | `m_shm` valid + heartbeat fresh | trigger detach sequence |
| `Legged::controllerPeriodic()` | `m_shm`, `magic`, `version`, `lifecycle`, heartbeat, `emergency_stop`, `cmd_write_idx` bounds, command freshness | `disableAllMotorsOnce()` and return |

## 10. Decision Log

| Question | Decision | Rationale |
|----------|----------|-----------|
| Consumer creates SHM? | No | Producer owns lifecycle; consumer refuses to start |
| Missing producer at startup? | Exit immediately | Supervisor can restart; no hidden retry loops |
| `magic` write order? | After all init fields | `magic` acts as the final release-acquire sentinel |
| `magic` atomic? | Yes | Matches other SHM synchronization fields |
| Heartbeat timeout | 100 ms fixed | Matches existing `Robot::robotPeriodic` and motor responsiveness timeout |
| Thread start | Keep `m_isEnabled` | Already gates `controllerPeriodic()`; condition variable left open and not used for initial start |
| One-shot disable re-arm | In `setEnable(true)` only | Tied to operator's explicit re-enable decision |
| Auto re-enable after reconnect | No | Avoid unexpected motion; operator must press enable |
| Reconnect retry rate | Every 100 ms | Fast recovery without `shm_open` spam every cycle |
| `pause()` implementation | Atomic spin-gate | Avoids priority inversion with `SCHED_FIFO`; rare event |
| `m_shm` pointer type | Plain pointer | Updated only while `controllerPeriodic` is paused |
| `cmd_write_idx` bounds | Validate `0` or `1` | Cheap runtime invariant against corruption |
| `Composer`/`Logger`/`Imu` on reconnect | Stop and restart | All access `m_shm` or staging pointers inside it |
| `m_shm_fd` member | Remove | Only the mapping pointer is needed; fd closed after `mmap` |
| `fstat` size check | `>= sizeof(SharedMemoryLayout)` | Forward-compatible with producer-side expansion |
| `version` validation | Every attach/reconnect, not every 400 Hz cycle | Reconnect is the right boundary to reject mismatched binaries |
| Heartbeat at attach | Must be non-zero and fresh | Proves producer is actively updating, not just initialized |
| `emergency_stop` on producer fault | Not set by `Legged` | `emergency_stop` is an operator/system signal; lifecycle/heartbeat faults are separate |
| Distinct error logs? | Yes | Easier startup/reconnection debugging |
| Initial attach when `magic` not yet valid | Exit immediately | Strict consumer policy; no bounded wait |

## 11. Files Affected

- `tools/mercury_shm_v2.h` — layout and atomic field changes
- `tools/mercury_controller.cpp` — producer lifecycle writes, `TERMINATED` ordering
- `tools/test_actuator_logger.cpp` — read `magic` with atomic load, respect lifecycle
- `lib/robot/ControlledSubsystemBase.h` — `pause()` / `resume()` atomic spin-gate
- `src/subsystems/Legged.h` / `Legged.cpp` — read guards, one-shot disable, `m_shm` validation
- `src/Robot.h` / `Robot.cpp` — attach/detach helpers, reconnection loop, strict startup exit
- `src/subsystems/Imu.h` / `lib/imu/ImuReader.*` — stop/restart support for reconnection
- `lib/composer/Composer.cpp` (and maybe `Logger.h`) — verify restart assumptions

## 12. Related References

- `doc/IPC_communication_protection.md` — the original 5-layer defense-in-depth proposal
- `tools/mercury_controller.cpp` — current Mercury Controller producer
- `src/Robot.cpp` — current consumer attach/startup path
- `src/subsystems/Legged.cpp` — current 400 Hz motor control loop
