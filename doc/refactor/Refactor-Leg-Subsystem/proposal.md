# Proposal: Refactor Legged Subsystem for Cross-Process Motor Driver Architecture

## What

Refactor the `Legged` subsystem from a self-contained control loop (LQR
controller + motor driver in one thread) into a dedicated **command relay
and motor driver** that bridges the Mercury whole-body dynamics controller
(running as a separate process) to the Damiao motors via MIT impedance
control over CAN-over-UDP.

## Why

The current `Legged` subsystem tightly couples the LQR control algorithm,
motor state management, and telemetry collection into a single periodic
callback chain [1]. This creates several problems:

1. **Controller lock-in:** The LQR controller is compiled into the actuator
   process via `ControllerBase<7, 2, 4>` templates [1]. Replacing the
   controller requires recompiling the actuator. The Mercury Controller
   process provides whole-body dynamics (DynaCoRE) via shared memory [2],
   making the in-process LQR redundant.

2. **Mutex contention on the real-time path:** Motor state and IMU state
   are accessed via mutex-protected accessors (`Imu::getStates()`,
   `Motor` internal state) [1][2]. At the proposed 400Hz inner loop rate,
   mutex contention becomes a latency risk.

3. **Tight telemetry coupling:** Telemetry collection is hardcoded in
   `Robot::robotPeriodic()`, directly accessing subsystem internals rather
   than through a decoupled interface [1]. This forces telemetry to run
   synchronously in the main loop.

4. **Motor count mismatch:** The source code manages 5 motors per leg [1],
   but the Mercury `SensorData` struct defines `num_act_joint = 12` (6 per
   leg). This mismatch causes indices 10-11 to be uninitialized.

5. **Slow motor timeout:** The current 500ms motor responsiveness
   timeout [1][2] is too slow for a 400Hz control loop — 200 missed
   feedback frames before detection.

## Scope

### In Scope

- Increase motor count from 5 to **6 per leg** (12 total)
- Change inner control loop rate from 5ms/200Hz to **2.5ms/400Hz**
- Change main loop rate from 20ms/50Hz to **10ms/100Hz**
- Remove in-process LQR controller — `controllerPeriodic()` becomes a
  command relay reading from Mercury Controller via POSIX shared memory
- Replace mutex-protected motor/IMU state with **lock-free per-source
  staging double buffers**
- Move all MIT command dispatch to `controllerPeriodic()` (inner loop)
- Make `robotPeriodic()` supervisory only (mode management, button events,
  health monitoring, parameter queries at 10Hz)
- Reduce motor responsiveness timeout from 500ms to **100ms**
- Add per-motor parameter query round-robin (bus voltage/current at 10Hz
  via CAN ID 0x7FF, D[2]=0x33) [8]
- Apply `SCHED_FIFO` priority 90 with 256KB stack for leg threads
- Update `config.yaml` motor topology (left: base_id=1, IDs 1-6;
  right: base_id=7, IDs 7-12)

### Out of Scope

- Mercury Controller process changes (separate codebase)
- IMU reader thread changes (stays at 500Hz)
- Composer thread implementation (separate change)
- MQTT binary logger implementation (separate change)
- PREEMPT_RT kernel patching (infrastructure task)
- Damiao simulator updates (separate tool change)

## Impact

### Breaking Changes

| Component | Change | Migration |
|-----------|--------|-----------|
| `Legged` template params | `<7, 2, 5>` → remove template or change to `<7, 2, 6>` | Update all instantiation sites |
| Motor array per leg | `vector<Motor>(5)` → `vector<Motor>(6)` | Update `config.yaml` leg definitions |
| `max_can_device` threshold | `device_id < 6` → `device_id < 7` | Update routing logic in `UdpServer` [3] |
| `controllerPeriodic()` | LQR `Controller::calculate()` call removed | Read from shared memory instead |
| `robotPeriodic()` | Motor queries + MIT dispatch removed | Supervisory only |
| Motor timeout | 500ms → 100ms | Update `Motor` class constant |
| Thread scheduling | Default → `SCHED_FIFO` priority 90 | Requires `PREEMPT_RT` kernel or `CAP_SYS_NICE` |
| State access pattern | Mutex-protected → lock-free double buffer | Replace `getStates()` with staging buffer reads |

### Non-Breaking Changes

- Button event handling (enable/disable) unchanged [1]
- Motor enable (0xFC), disable (0xFD), calibration (0xFE), error
  clear (0xFB) command format unchanged [7]
- MIT frame encoding/decoding unchanged [7][8]
- CAN-over-UDP 13-byte frame format unchanged [6]
- UDP port assignments unchanged (left: 8887/8886, right: 8889/8888) [1]

## Risks

| Risk | Severity | Mitigation |
|------|:--------:|------------|
| `SCHED_FIFO` starvation between leg threads at same priority (90) | Medium | Evaluate CPU core affinity (`pthread_setaffinity_np`) — open question |
| 2.5ms deadline too tight for ARM SBC | Medium | Profile existing 5ms cycle; MIT encoding is ~5μs per motor [7][8] |
| Mercury Controller crash → 10ms of stale commands (4 cycles at 400Hz) | Medium | Accepted — heartbeat detection in `robotPeriodic()` at 100Hz |
| `#pragma pack(1)` performance on ARM for cross-platform SHM structs | Medium | Requires profiling — open question |
| `std::atomic<uint32_t>` not lock-free on ARM toolchain | Low | Add `static_assert(is_always_lock_free)` — open question |