## 1. SharedMemoryLayout Changes

- [x] 1.1 Remove `controller_heartbeat_ns` from `SharedMemoryLayout` in `include/mercury_shm.h`.
- [x] 1.2 Add `std::atomic<bool> controller_emergency_stop{false}` to `SharedMemoryLayout` in `include/mercury_shm.h`.
- [x] 1.3 Bump `SHM_VERSION` from 4 to 5 in `include/mercury_shm.h`.
- [x] 1.4 Update `static_assert` checks for `SharedMemoryLayout` size and alignment in `include/mercury_shm.h`.

## 2. Robot becomes SHM Owner

- [x] 2.1 Rewrite `Robot::robotInit()` in `src/Robot.cpp`: replace the 30-second `tryAttachSharedMemory()` polling loop with `shm_open(O_CREAT | O_RDWR)`, `ftruncate(sizeof(SharedMemoryLayout))`, `mmap`, field initialization (version, num_joints, control_freq_hz, robot_id, motor_can_ids, zeroed buffers), then `lifecycle_state.store(RUNNING, release)` and `magic.store(SHM_MAGIC, release)` as the final writes. Call `std::exit(EXIT_FAILURE)` on `shm_open`/`ftruncate`/`mmap` failure.
- [x] 2.2 Keep `attachSharedMemory()` for wiring subsystems (IMU staging buffer, leg SHM pointers, Composer, Logger) — call it immediately after SHM creation. Remove wait/sleep before attach.
- [x] 2.3 Remove `tryAttachSharedMemory()` from `src/Robot.cpp` and `include/Robot.h`.
- [x] 2.4 Update `detachSharedMemory()` in `src/Robot.cpp`: add `shm_unlink(mercury::SHM_NAME)` after `munmap`. This method is now only called during shutdown.
- [x] 2.5 Update `Robot::~Robot()`: set `lifecycle_state = SHUTTING_DOWN` before stopping threads, then `TERMINATED` before calling `detachSharedMemory()`.

## 3. Robot Supervisory Loop Simplification

- [x] 3.1 Remove the producer-liveness check block from `robotPeriodic()` (the `if (m_shm)` block that checks magic/lifecycle/heartbeat and calls `detachSharedMemory()`).
- [x] 3.2 Remove the reconnection retry block from `robotPeriodic()` (the `if (!m_shm && (m_cycle % 10 == 0))` block).
- [x] 3.3 Replace the Mercury Controller heartbeat validation in `robotPeriodic()` with command-timestamp staleness: read `cmd_buffers[cmd_write_idx].timestamp_ns`, skip if zero (controller never connected), set `emergency_stop` if age > 100ms.
- [x] 3.4 Add `controller_emergency_stop` propagation in `robotPeriodic()`: read `m_shm->controller_emergency_stop.load(acquire)`, if true set `emergency_stop` to true and log source.

## 4. Legged SHM Validation Simplification

- [x] 4.1 Remove the `lifecycle_state != RUNNING` check from `Legged::controllerPeriodic()` in `src/subsystems/Legged.cpp`.
- [x] 4.2 Remove the `controller_heartbeat_ns` staleness check from `Legged::controllerPeriodic()`.
- [x] 4.3 Add `controller_emergency_stop` check to `Legged::controllerPeriodic()`: if `m_shm->controller_emergency_stop.load(acquire)` is true, call `disableAllMotorsOnce()` and return.
- [x] 4.4 Remove `HEARTBEAT_STALE` from the `DisableReason` enum in `Legged.h` (or repurpose as `CMD_TIMESTAMP_STALE`). Remove the `HEARTBEAT_TIMEOUT_NS` constant.

## 5. Mercury Controller Flip to Consumer

- [x] 5.1 Rewrite `ControllerTestBench::init()` in `tools/mercury_controller.cpp`: replace `shm_open(O_CREAT | O_RDWR)` + `ftruncate` with `shm_open(O_RDWR)` in a retry loop (poll every 100ms). Validate `fstat` size, `magic`, `version`, `lifecycle_state == RUNNING` before proceeding.
- [x] 5.2 Remove all lifecycle state writes from `mercury_controller.cpp` (`lifecycle_state = RUNNING`, `SHUTTING_DOWN`, `TERMINATED`). Remove `magic` write. Remove `version`/`num_joints`/`control_freq_hz`/`robot_id` initialization.
- [x] 5.3 Remove `shm_unlink()` from `ControllerTestBench` destructor. Keep only `munmap`.
- [x] 5.4 Add `compose_timestamp_ns` staleness detection to the controller's main loop: if `composed_buffers[composed_write_idx].compose_timestamp_ns` is stale > 100ms, stop writing commands, `munmap`, and re-enter the `shm_open` retry loop.
- [x] 5.5 Remove `controller_heartbeat_ns` writes from the controller's main loop (field no longer exists).
- [x] 5.6 Remove `emergency_stop` writes from the controller's shutdown path. Optionally write `controller_emergency_stop = true` if the controller detects a safety condition.

## 6. Systemd Service Updates

- [x] 6.1 Remove `ExecStartPre`/`ExecStopPost` SHM cleanup from `services/mercury-controller.service` (if present). Controller no longer owns SHM.
- [x] 6.2 Document the Kuavo Robot systemd unit configuration in `doc/installation.md`: add `ExecStartPre=/bin/sh -c '/usr/bin/test -e /dev/shm/mercury_robot_ipc && /bin/rm -f /dev/shm/mercury_robot_ipc || true'` and `ExecStopPost=/bin/sh -c '/usr/bin/test -e /dev/shm/mercury_robot_ipc && /bin/rm -f /dev/shm/mercury_robot_ipc || true'` with `Restart=on-failure`.

## 7. Test Updates

- [x] 7.1 Rewrite `tools/test_shm_lifecycle.cpp`: test harness creates and initializes SHM (simulating the Robot), then launches `mercury_controller` as consumer. Test scenarios: controller attaches to existing SHM, Robot graceful shutdown (lifecycle → SHUTTING_DOWN), Robot crash (SHM removed externally), controller reconnects after Robot restart.
- [x] 7.2 Verify `tools/test_actuator_logger.cpp` still works — it already uses `shm_open(O_RDWR)` without `O_CREAT`, so it should attach to the Robot's SHM without changes.

## 8. Build and Manual Verification

- [x] 8.1 Build both `mercury_controller` and `Kuavo` with the updated `mercury_shm.h` (version 5). Verify clean compilation.
- [ ] 8.2 Test Robot startup: Robot creates SHM, starts Composer/IMU/Legged immediately, sensor data flows without waiting for controller.
- [ ] 8.3 Test controller startup: controller attaches to Robot's SHM, validates, starts writing commands. Motors respond to commands.
- [ ] 8.4 Test controller `SIGTERM`: Legged sees stale `cmd.timestamp_ns`, disables motors. Robot keeps running. Controller restarts and resumes writing commands. Motors resume after operator re-enable.
- [ ] 8.5 Test controller `kill -9`: same behavior as 8.4 — command staleness triggers motor disable within 100ms.
- [ ] 8.6 Test Robot `systemctl restart`: `ExecStopPost` removes SHM. Controller enters retry loop. Robot restarts, creates fresh SHM. Controller reattaches.

## 9. Documentation

- [x] 9.1 Update `doc/IPC_communication_protection.md`: replace "producer (Mercury Controller)" with "owner (Robot)" throughout. Update the 5-layer defense description to reflect Robot-as-owner.
- [x] 9.2 Update `doc/SafetyCoordination-ActuatorDriverStationMercuryController.md`: update the SHM Connection State Machine diagram and the safety layer summary table.
- [x] 9.3 Update `doc/installation.md`: replace controller-creates-SHM instructions with Robot-creates-SHM. Update systemd unit examples. Update the "Robot–Controller Connection" section.
