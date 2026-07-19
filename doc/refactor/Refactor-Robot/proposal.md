# Proposal: Refactor robotPeriodic() from Control+Telemetry Hub to Supervisory-Only

## What

Refactor `Robot::robotPeriodic()` from a monolithic method that handles
motor control, telemetry collection, subsystem state access, and mode
management into a **lightweight supervisory-only** periodic callback at
**10ms / 100Hz** (changed from 20ms / 50Hz). All motor control (MIT
command dispatch, motor state queries) moves to the leg subsystem's inner
control loop (`controllerPeriodic()` at 2.5ms / 400Hz). All telemetry
collection moves to the new Composer thread. All telemetry publication
moves to the MQTT Logger thread.

## Why

The current `robotPeriodic()` is a bottleneck that tightly couples
unrelated concerns:

1. **Motor control at the wrong rate:** Currently, `robotPeriodic()` calls
   `Legged::robotPeriodic()` which performs "motor queries + MIT dispatch"
   at 20ms / 50Hz [1]. MIT commands should be dispatched at 400Hz from
   the inner control loop for adequate control bandwidth — not at 50Hz
   from the supervisory loop.

2. **Inline telemetry blocks the control path:** The main loop "collects
   state from all subsystems and publishes via `RobotStatus` and `DataLog`
   each cycle" [1]. Specifically, `robotPeriodic()` calls
   `leftLeg.getMotors()` and `imu.getStates()` synchronously [3], then
   publishes a binary `RobotStatusWire` packet (~890 bytes, magic
   0x4B564155) and JSON SenML data logs [1]. This inline telemetry adds
   latency to the control path and creates mutex contention because
   `imu.getStates()` is mutex-protected [1].

3. **Direct subsystem access creates tight coupling:** Telemetry
   collection is "hardcoded in `Robot::robotPeriodic()`, directly
   accessing subsystem internals rather than through a publish/subscribe
   interface" [1]. This means adding a new subsystem or changing
   subsystem internals requires modifying `robotPeriodic()`.

4. **Rate change needed for cross-process bridge:** The Mercury Controller
   runs as a separate process communicating via POSIX shared memory [2].
   The controller writes commands and reads sensor data at its own rate.
   The main loop must run at 100Hz (10ms) to provide timely health
   monitoring, heartbeat checks, and emergency stop detection — the
   current 50Hz is too slow for the 100ms motor responsiveness timeout.

## Scope

### In Scope

- Change `TimedRobot` main loop period from 20ms / 50Hz to **10ms / 100Hz**
- Remove all motor control logic from `robotPeriodic()`:
  - Remove `Legged::robotPeriodic()` motor query + MIT dispatch calls
  - Remove direct `Motor` object access from the main loop
- Remove all inline telemetry from `robotPeriodic()`:
  - Remove `leftLeg.getMotors()` calls [3]
  - Remove `imu.getStates()` calls [3]
  - Remove `m_robotStatus.publish()` calls [3]
  - Remove `DataLog` JSON SenML publishing [1]
  - Remove the `RobotStatus` collector and `MqttClient` telemetry instance
- Add supervisory responsibilities to `robotPeriodic()`:
  - Mode management (disabled/autonomous/teleop lifecycle)
  - Button event polling (`m_loop.poll()`) [1][3]
  - Motor enable/disable dispatch (one-time supervisory: 0xFC/0xFD/0xFE/0xFB)
  - Health monitoring (read composed sensor data from SHM double buffer)
  - Safety validation (emergency stop, heartbeat checks for Mercury Controller)
  - Motor parameter queries at 10Hz (bus voltage/current via CAN 0x7FF)
- Apply `SCHED_FIFO` priority 75 with 512KB stack for the main loop thread
- Update `config.yaml` with new main loop period

### Out of Scope

- Leg subsystem inner control loop changes (`controllerPeriodic()`)
  — covered in a separate proposal
- Composer thread implementation — separate change
- MQTT binary logger implementation — separate change
- IMU reader thread changes — stays at 500Hz
- Mercury Controller process changes — separate codebase
- PREEMPT_RT kernel patching — infrastructure task

## Impact

### Breaking Changes

| Component | Change | Migration |
|-----------|--------|-----------|
| `TimedRobot` period | 20ms → 10ms | Update `AddPeriodic` callback interval [3] |
| `Robot::robotPeriodic()` | Motor control + telemetry removed | Callers of `getMotors()` / `getStates()` via main loop must use composed SHM buffer instead |
| `RobotStatus` class | Removed entirely | Replaced by Composer + MQTT Logger |
| `DataLog` class | Removed entirely | Replaced by MQTT Logger binary payload |
| `MqttClient` (telemetry instance) | Removed | MQTT Logger uses its own libwebsockets client |
| MQTT topics | `/telemetry/subsystem/<name>/motor` and SenML topics removed | Replaced by `robot/sensor/bin` and `robot/command/bin` |
| `Legged::robotPeriodic()` | No longer dispatches MIT commands or queries motors | Becomes lightweight status reporting only |

### Non-Breaking Changes

- Button event handling (`EventLoop` + `BooleanEvent`) unchanged [1]
- Operating mode lifecycle (disabled/autonomous/teleop) unchanged [1]
- `DriverStation` UDP parsing unchanged [1]
- Motor CAN-over-UDP frame format unchanged (13 bytes) [2]
- `config.yaml` structure (additive — new `data_logger` section)

## Risks

| Risk | Severity | Mitigation |
|------|:--------:|------------|
| 10ms deadline too tight with parameter queries | Low | Parameter queries are lightweight (one CAN frame per cycle at 10Hz); MIT encoding is ~5μs per motor. Total budget consumption <1ms. |
| Mercury Controller crash detection at 100Hz (10ms latency) | Medium | Accepted — up to 4 stale commands dispatched by inner loop before main loop detects crash. Inner loop also checks command timestamp independently. |
| Removing inline telemetry breaks monitoring dashboards | Low | Confirmed: no existing systems depend on the localhost MQTT topics. |
| `SCHED_FIFO` priority 75 may starve DriverStation/EventLoop threads | Low | DriverStation and EventLoop run at `SCHED_OTHER` (non-RT) — they are event-driven and tolerate scheduling delays. Main loop at priority 75 is below all sensor/actuator threads (80-90). |