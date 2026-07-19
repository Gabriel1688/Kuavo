# Design: Refactor Robot/robotPeriodic() to Supervisory-Only

## Context

The `Robot::robotPeriodic()` method is the central callback of the
`TimedRobot` lifecycle [2][3]. Currently, the `loopFunc()` chain fires
every 5ms via a `Notifier`: `Notifier fires → IterativeRobotBase::loopFunc()
→ refreshData() → mode switch → robotPeriodic()` [3]. Inside
`robotPeriodic()`, the following happens synchronously every 20ms cycle [3]:

```
robotPeriodic() {
m_loop.poll()                     // button events [3]
leftLeg.getMotors()               // direct motor state access [3]
imu.getStates()                   // mutex-protected IMU access [1][3]
m_robotStatus.publish()           // 890-byte binary MQTT [1][3]
DataLog.logMotors()               // JSON SenML MQTT [1]
DataLog.logImu()                  // JSON SenML MQTT [1]
runAllRobotPeriodic()             // subsystem periodic dispatch [3]
}
```

This design creates three problems: (1) motor control runs at 50Hz
instead of the required 400Hz, (2) telemetry collection blocks the
control path via mutex-protected `getStates()` [1], and (3) telemetry
is tightly coupled to subsystem internals rather than decoupled through
staging buffers [1].

The Mercury whole-body dynamics controller runs as a separate process
communicating via POSIX shared memory (`mercury_shm.h`) [2]. The inner
control loop acts as a command relay, reading commands from shared memory
and dispatching MIT CAN frames to motors at 400Hz. The main loop must
become supervisory — managing modes, buttons, health, and safety — without
touching motor control or telemetry collection.

### Key constraints

- `TimedRobot` default period is 20ms (50Hz) [1][2]; must change to 10ms
- `ControlledSubsystemBase` spawns a dedicated pthread per subsystem [1]
- All IMU state access is currently mutex-protected [1]
- Motor responsiveness timeout must be 100ms (reduced from 500ms)
- Motor device IDs 1-6 route to left leg, 7-12 to right leg
- No LQR controller in the actuator process — Mercury Controller handles
  control computation via shared memory [2][3]

## Goals / Non-Goals

**Goals:**

- Change `TimedRobot` main loop period from 20ms to **10ms (100Hz)**
- Remove all motor control logic from `robotPeriodic()`
- Remove all inline telemetry (RobotStatus, DataLog, MqttClient)
- Make `robotPeriodic()` supervisory only: mode management, button events,
  health monitoring, safety validation, parameter queries
- Add motor parameter query round-robin at 10Hz (every 10th cycle)
- Apply `SCHED_FIFO` priority 75 with 512KB stack
- Read health data from composed shared memory buffer (not from subsystem
  internals directly)

**Non-Goals:**

- Implementing the Composer thread (reads staging buffers — separate change)
- Implementing the MQTT binary logger (drains SPSC ring — separate change)
- Modifying `controllerPeriodic()` (inner loop — separate change)
- Modifying the Mercury Controller process
- Changing the IMU reader thread (stays at 500Hz)

## Decisions

### D1: Change TimedRobot period from 20ms to 10ms

**Decision:** Update the `AddPeriodic` callback interval in the `Robot`
constructor from 20ms to 10ms. The `loopFunc()` chain —
`Notifier fires → IterativeRobotBase::loopFunc() → refreshData()
→ mode switch → robotPeriodic()` [3] — now fires at 100Hz.

**Why:** The motor responsiveness timeout is reduced from 500ms to 100ms.
At 50Hz, the main loop detects an unresponsive motor only every 20ms —
5 cycles within the 100ms window. At 100Hz, the main loop detects within
10ms — 10 cycles within the window, providing more granular monitoring.
Additionally, the Mercury Controller heartbeat detection at 100Hz limits
stale command exposure to 10ms (4 inner loop cycles at 400Hz) instead of
20ms (8 cycles).

**Impact:** All code in `robotPeriodic()` must complete within 10ms.
Since motor control and telemetry are being removed (see D2, D3), the
remaining supervisory work is lightweight — button polling, mode checks,
health reads from double buffer, and occasional parameter queries.

### D2: Remove motor control from robotPeriodic()

**Decision:** Remove all motor query and MIT dispatch calls from
`Robot::robotPeriodic()` and `Legged::robotPeriodic()`.

**What is removed:**

| Current Call | Location | Why Removed |
|-------------|----------|-------------|
| Motor state queries | `robotPeriodic()` → `leftLeg.getMotors()` [3] | Moved to `controllerPeriodic()` inner loop |
| MIT command dispatch | `robotPeriodic()` → `Legged::robotPeriodic()` → `Motor::setMitControl()` [1] | Moved to `controllerPeriodic()` at 400Hz |
| Direct `Motor` object reads | Throughout `robotPeriodic()` [1] | Replaced by composed SHM buffer reads |

**What stays in `Legged::robotPeriodic()`:** Lightweight status reporting
only — no motor I/O, no MIT commands. The subsystem's `robotPeriodic()`
becomes a no-op or reports aggregate health status.

**Why not keep MIT dispatch at 100Hz in the main loop?** The inner control
loop at 400Hz provides 4× better control bandwidth. The Mercury Controller
produces joint commands at its own rate via shared memory [2]; the inner
loop reads and dispatches at 400Hz, providing the tightest possible
command-to-motor latency. Running MIT dispatch at 100Hz from the main loop
would add unnecessary latency and waste 3 out of 4 inner loop cycles.

### D3: Remove inline telemetry entirely

**Decision:** Remove `RobotStatus`, `DataLog`, and the telemetry
`MqttClient` from the main loop.

**What is removed:**

| Component | Current Behavior | Replacement |
|-----------|-----------------|-------------|
| `RobotStatus::collect()` | Calls `leftLeg.getMotors()` + `imu.getStates()` every 20ms [3] | **Composer thread** reads per-source staging buffers at 400Hz |
| `RobotStatus::publish()` | Publishes 890-byte binary `RobotStatusWire` (magic 0x4B564155) via MQTT [1] | **MQTT Logger thread** publishes binary `LogRecord` via libwebsockets |
| `DataLog::logMotors()` | Publishes JSON SenML to `/telemetry/subsystem/<name>/motor` [1] | Replaced by binary payload on `robot/sensor/bin` |
| `DataLog::logImu()` | Publishes JSON SenML IMU data [1] | Included in composed `SensorData` binary payload |
| `MqttClient` (telemetry) | libwebsockets client publishing to localhost:1883 [1][2] | MQTT Logger has its own libwebsockets client |

**Why replace rather than refactor?** The existing telemetry uses a
fundamentally different pattern — synchronous inline collection at 50Hz
with direct subsystem access [1]. The new architecture uses asynchronous
lock-free staging buffers + SPSC ring buffer + separate logger thread.
Refactoring the existing code to fit this pattern would require rewriting
every data path, so a clean replacement is simpler and safer.

### D4: Define the new robotPeriodic() responsibilities

**Decision:** After refactoring, `robotPeriodic()` performs exactly these
7 tasks at 100Hz:

| # | Responsibility | Rate | Data Source |
|:---:|---|:---:|---|
| 1 | **Mode management** | 100Hz | `IterativeRobotBase` mode switch (disabled/autonomous/teleop) [1] |
| 2 | **Button event polling** | 100Hz | `m_loop.poll()` → `EventLoop` + `BooleanEvent` dispatch [1][3] |
| 3 | **Motor enable/disable** | On-event | One-time supervisory commands (0xFC/0xFD/0xFE/0xFB) via async message to leg subsystem [1] |
| 4 | **Health monitoring** | 100Hz | Read composed `SensorData` from SHM double buffer (lock-free, no mutex) |
| 5 | **Safety validation** | 100Hz | Check Mercury Controller heartbeat, motor group heartbeats, IMU heartbeat; set `emergency_stop` flag if any source is stale |
| 6 | **Parameter queries** | 10Hz | Modulo counter (every 10th cycle); round-robin across 12 motors; bus voltage/current via CAN ID 0x7FF, D[2]=0x33 [8] |
| 7 | **Subsystem periodic dispatch** | 100Hz | `runAllRobotPeriodic()` [3] — calls each subsystem's lightweight `robotPeriodic()` |

**What is explicitly NOT in robotPeriodic():**

- ❌ MIT command dispatch (`setMitControl()`) — in `controllerPeriodic()`
- ❌ Motor state queries (direct `Motor` reads) — in `controllerPeriodic()`
- ❌ Telemetry collection (`getMotors()`, `getStates()`) — in Composer
- ❌ Telemetry publication (`publish()`, `DataLog`) — in MQTT Logger
- ❌ Controller `calculate()` invocation — in Mercury Controller process

### D5: Health monitoring via composed SHM buffer

**Decision:** `robotPeriodic()` reads health data from the composed
`SensorData` double buffer in shared memory — not by directly accessing
subsystem internals. The composed buffer is written by the Composer thread
at 400Hz, which merges data from the three per-source staging buffers
(IMU at 500Hz, motor groups A and B at 400Hz).

**What to check:**

| Check | Source Field | Threshold | Action |
|-------|-------------|:---:|--------|
| Mercury Controller alive | `cmd_buffers` heartbeat timestamp | Stale > 100ms | Set `emergency_stop` |
| IMU alive | `imu_timestamp_ns` in composed snapshot | Stale > 50ms | Log warning; if > 200ms, set `emergency_stop` |
| Motor Group A alive | `motor_group_a_timestamp_ns` | Stale > 100ms | Disable left leg motors (0xFD) |
| Motor Group B alive | `motor_group_b_timestamp_ns` | Stale > 100ms | Disable right leg motors (0xFD) |
| Motor status errors | `motor_status[j]` in composed snapshot | Any non-0x01 value | Log error; disable affected motor |

**Why not read directly from subsystems?** Direct access via
`leftLeg.getMotors()` [3] and `imu.getStates()` [1] uses mutex-protected
state, which creates priority inversion risk under `SCHED_FIFO`. The
composed SHM double buffer is lock-free — the main loop reads with
`std::memory_order_acquire` and never blocks.

### D6: Parameter query round-robin at 10Hz

**Decision:** The main loop sends one motor parameter query per cycle at
10Hz effective rate (every 10th cycle of the 100Hz main loop). With 12
motors and round-robin scheduling, each motor gets queried once per
1.2 seconds. The query uses CAN ID `0x7FF` with D[2]=0x33 (query marker)
and D[3]=RID (parameter index) [8].

**Implementation:** A `query_cycle_counter` modulo counter in
`robotPeriodic()`:

```
cycle % 10 == 0 → send query for motor (cycle / 10) % 12
```

The query response arrives via the existing `Motor::callback()` →
`UdpServer` callback chain [1]. The response is identified when
`D[2]==0x33` [8] and the value is stored in an atomic per-motor
parameter cache (`MotorParamCache`). The Composer reads this cache when
building the composed `SensorData` snapshot, filling in `bus_voltage[j]`
and `bus_current[j]`.

**Why from robotPeriodic() and not a separate timer thread?** The
parameter query shares the same CAN bus as MIT commands [1]. A separate
thread sending queries concurrently with the inner control loop creates
a race condition on the `UdpServer` socket [3]. The main loop's
supervisory time slot avoids this contention.

### D7: Thread scheduling

**Decision:**

| Parameter | Value |
|-----------|-------|
| Scheduling policy | `SCHED_FIFO` |
| Priority | 75 |
| Stack size | 512KB |

Priority 75 is below the leg subsystems (90), UdpServers (88), Composer
(85), and IMU reader (80), ensuring that the supervisory loop never
preempts real-time sensor/actuator threads. It is above `SCHED_OTHER`
threads (DriverStation, EventLoop, MQTT Logger) because supervisory
health monitoring and emergency stop detection are more important than
operator input or telemetry logging.

The 512KB stack provides headroom for `robotPeriodic()` which may call
subsystem `robotPeriodic()` methods, poll the event loop, perform
parameter queries, and read composed SHM data. This is the largest stack
allocation because the main loop has the deepest call chain.

### D8: Motor enable/disable remains in robotPeriodic()

**Decision:** Motor enable (0xFC), disable (0xFD), zero-position
calibration (0xFE), and error clear (0xFB) commands remain in
`robotPeriodic()`, dispatched as **one-time supervisory commands** in
response to button events or mode transitions [1].

**Trigger mapping:**

| Event | Button / Mode | Command | Target |
|-------|--------------|:---:|--------|
| Enter Teleop | Mode transition | 0xFC (enable) | All 12 motors via async message to both leg subsystems |
| Enter Disabled | Mode transition | 0xFD (disable) | All 12 motors |
| Button 1 pressed | `BooleanEvent` rising edge [1] | 0xFC (enable) | Left leg motors 1-6 |
| Button 2 pressed | `BooleanEvent` rising edge [1] | 0xFD (disable) | Left leg motors 1-6 |
| Calibrate request | Operator action | 0xFE (save zero) | Specified motor |
| Error detected | Health monitoring | 0xFB (clear error) | Affected motor |

These commands are sent via async message to the leg subsystem's
`onMessage()` handler [1], which then sends the CAN command on the
subsystem's thread — avoiding direct CAN bus access from the main loop.

## Risks / Trade-offs

- **[10ms deadline]** The main loop budget drops from 20ms to 10ms.
  However, removing motor control and telemetry collection eliminates
  the two heaviest operations. The remaining supervisory tasks (button
  poll, SHM double buffer read, mode check, occasional parameter query)
  should complete in <1ms total. → Mitigation: Profile on target ARM SBC
  after refactoring.

- **[Parameter query contention]** The parameter query (CAN 0x7FF) shares
  the UDP socket with MIT commands from the inner loop [1]. At 10Hz query
  rate vs 400Hz MIT rate, the query occupies <2.5% of CAN bus capacity.
  → Mitigation: CAN bus utilization at ~36% (including queries) is well
  within the safe limit of 70%.

- **[Loss of existing MQTT topics]** The JSON SenML topics
  (`/telemetry/subsystem/<name>/motor`) and the binary `RobotStatusWire`
  (magic 0x4B564155, ~890 bytes) [1] will no longer be published. →
  Mitigation: Confirmed no existing consumers depend on these topics.

- **[Emergency stop latency]** If the Mercury Controller crashes, up to
  4 stale MIT commands (10ms at 400Hz) are dispatched by the inner loop
  before the main loop detects the crash via heartbeat timeout. →
  Mitigation: Accepted limitation. The inner loop also independently
  checks command timestamp freshness as a secondary guard.

## Target Architecture

```
robotPeriodic() (100Hz, SCHED_FIFO 75, 512KB stack)

SUPERVISORY ONLY — no motor control, no telemetry

┌─────────────────────────────────────────────────────┐
│  1. Mode management                                  │
│     disabled/autonomous/teleop lifecycle [1]          │
│                                                       │
│  2. m_loop.poll()                                     │
│     button events via EventLoop + BooleanEvent [1][3] │
│                                                       │
│  3. Motor enable/disable (one-time, on-event)         │
│     0xFC / 0xFD / 0xFE / 0xFB via async message [1]  │
│                                                       │
│  4. Health monitoring                                 │
│     read composed_buffers (SHM double buffer)         │
│     check per-source timestamps + motor status        │
│                                                       │
│  5. Safety validation                                 │
│     Mercury Controller heartbeat (stale > 100ms?)     │
│     IMU heartbeat (stale > 50ms?)                     │
│     Motor group heartbeats (stale > 100ms?)           │
│     → set emergency_stop flag if any fails            │
│                                                       │
│  6. Parameter queries (10Hz, modulo counter)          │
│     bus voltage/current via CAN 0x7FF, D[2]=0x33 [8]  │
│     round-robin across 12 motors                      │
│                                                       │
│  7. runAllRobotPeriodic()                             │
│     calls each subsystem's lightweight robotPeriodic() │
└─────────────────────────────────────────────────────┘
