# Design: Refactor Legged Subsystem for Cross-Process Motor Driver

## Context

The `Legged` subsystem currently implements `ControlledSubsystemBase<7, 2, 5>`
with a tightly coupled LQR controller, 5 motors per leg, mutex-protected
state access, and inline telemetry collection [1]. The inner control loop
runs at 5ms/200Hz via `controllerPeriodic()`, and motor queries plus MIT
dispatch happen at 20ms/50Hz via `robotPeriodic()` [1].

The Mercury whole-body dynamics controller runs as a separate process,
communicating via POSIX shared memory (`mercury_shm.h`) [2]. The current
architecture requires the Legged subsystem to act as a **command relay and
motor driver** — reading joint commands from shared memory and dispatching
MIT CAN frames to the motors — rather than running its own LQR controller.

### Current data flow (to be replaced)

```
Robot Main Loop (20ms) → Legged::robotPeriodic() → Motor queries + MIT dispatch
→ Legged::controllerPeriodic() (5ms) → Controller::calculate() → MIT dispatch
```

### Target data flow

```
Mercury Controller (separate process) → POSIX shared memory → Mercury_Command
Legged::controllerPeriodic() (2.5ms) → Read SHM → MIT encode → CAN-over-UDP → Motor
Motor → feedback → CAN-over-UDP → Legged → decode → MotorGroupStageData staging buffer
Legged::robotPeriodic() (10ms) → Supervisory only (enable/disable, health, param queries)
```

### Key constraints

- Motor MIT frame format is fixed: position (16-bit), velocity (12-bit),
  Kp (12-bit), Kd (12-bit), torque (12-bit) packed into 8 bytes [7]
- CAN send ID = device_id; receive ID = device_id + 0x10 [1][2]
- Motor enable/disable commands (0xFC/0xFD) are one-time supervisory
  operations, not part of the inner control loop [7]
- Two CAN buses at 1 Mbps, 6 motors per bus, ~36% utilization at 400Hz [7]
- Motor firmware processes commands faster than 400Hz
- No LQR in the actuator process — Mercury Controller handles all control

## Goals / Non-Goals

**Goals:**

- Transform `controllerPeriodic()` into a pure command relay and motor
  driver at 400Hz (2.5ms)
- Transform `robotPeriodic()` into a supervisory-only method at 100Hz (10ms)
- Replace mutex-protected motor state with lock-free per-source staging
  double buffers (`SourceDoubleBuffer<MotorGroupStageData>`)
- Increase motor count from 5 to 6 per leg (12 total)
- Reduce motor responsiveness timeout from 500ms to 100ms
- Add parameter query round-robin at 10Hz from `robotPeriodic()`
- Apply real-time thread scheduling (SCHED_FIFO priority 90, 256KB stack)

**Non-Goals:**

- Implementing the Composer thread (separate change — reads staging buffers)
- Implementing the MQTT binary logger (separate change)
- Modifying the Mercury Controller process
- Changing the IMU reader thread (stays at 500Hz)
- Implementing IMU prediction/extrapolation (open question)

## Decisions

### D1: Remove in-process LQR controller

**Decision:** Remove `Controller` (`ControllerBase<7, 2, 4>`) from the
`Legged` class. The `controllerPeriodic()` method no longer calls
`Controller::calculate()`. Instead, it reads `Mercury_Command` from
the POSIX shared memory double buffer.

**Why:** The Mercury Controller (DynaCoRE) runs as a separate process and
produces the joint commands that the inner loop dispatches [2]. The
in-process LQR was a placeholder [3]. Keeping both creates ambiguity
about which controller is active.

**Impact:** The `Legged` template changes from `ControlledSubsystemBase<7, 2, 5>`
to either a simplified non-templated class or `ControlledSubsystemBase<7, 2, 6>`
with the controller member removed. The `Controller.h` and
`Controller.cpp` files in `src/controllers/` can be preserved but are
no longer instantiated by `Legged`.

### D2: `controllerPeriodic()` becomes command relay + motor driver

**Decision:** The inner control loop at 2.5ms/400Hz performs this
fixed sequence per cycle:

| Step | Action | Source |
|:---:|--------|--------|
| 1 | Read `Mercury_Command` from SHM double buffer | Cross-process shared memory |
| 2 | Check command freshness (heartbeat timestamp) | If stale → disable motors (0xFD) [7] |
| 3 | Check `emergency_stop` flag | If set → disable all motors |
| 4 | For each of 6 motors: extract per-joint command | Parse `jpos_cmd[j]`, `jvel_cmd[j]`, `jtorque_cmd[j]`, `kp[j]`, `kd[j]` |
| 5 | Encode MIT frame: `float_to_uint` for position (16-bit), velocity (12-bit), Kp (12-bit), Kd (12-bit), torque (12-bit) [7][8] | Damiao protocol |
| 6 | Send via CAN-over-UDP (13 bytes per motor) [6] | `UdpServer` per leg |
| 7 | Receive feedback frame: decode via `uint_to_float` [7][8] | Motor callback chain |
| 8 | Check motor status (D[0] error flags: overvoltage 0x08 through overload 0x0E) [7] | Motor state machine |
| 9 | Write to `MotorGroupStageData` per-source staging double buffer | Lock-free publish |

**Why not keep MIT dispatch in `robotPeriodic()`?** The main loop runs
at 100Hz (10ms) — 4× slower than the inner loop at 400Hz. Sending MIT
commands at 100Hz would reduce control bandwidth by 4×. The inner loop
provides the tightest control, direct access to motor state via callbacks,
and co-located safety enforcement [1].

### D3: `robotPeriodic()` becomes supervisory only

**Decision:** The main loop at 10ms/100Hz performs only non-real-time
supervisory tasks:

| # | Responsibility | Rate | Notes |
|:---:|---|:---:|---|
| 1 | Mode management (disabled/autonomous/teleop) | 100Hz | `IterativeRobotBase` mode switch [1] |
| 2 | Button event polling (`m_loop.poll()`) | 100Hz | `EventLoop` + `BooleanEvent` dispatch [1] |
| 3 | Motor enable/disable (one-time: 0xFC/0xFD/0xFE/0xFB) | On-event | Via async message to leg subsystem [7] |
| 4 | Health monitoring (read composed sensor data) | 100Hz | From `composed_buffers` double buffer |
| 5 | Safety validation (emergency stop, heartbeat checks) | 100Hz | Set `emergency_stop` flag if needed |
| 6 | Parameter queries at 10Hz | 10Hz (modulo counter) | Bus voltage/current via CAN 0x7FF [8] |
| 7 | Subsystem periodic dispatch | 100Hz | `runAllRobotPeriodic()` [1] |

**What is removed from `robotPeriodic()`:**

- MIT command dispatch (`setMitControl()`) → moved to `controllerPeriodic()`
- Motor state queries (direct `Motor` object reads) → moved to inner loop
- Telemetry collection (`leftLeg.getMotors()`, `imu.getStates()`) → moved to Composer thread
- Telemetry publication (`m_robotStatus.publish()`, `DataLog`) → replaced by MQTT Logger

### D4: Motor count increase from 5 to 6 per leg

**Decision:** Update the motor array from `vector<Motor>(5)` to
`vector<Motor>(6)` per leg. Update `MOTORS_PER_GROUP` from 5 to 6.
Update CAN ID routing: motor IDs 1-6 to UDP server 0 (left leg),
IDs 7-12 to UDP server 1 (right leg).

**Why:** The Mercury `SensorData` struct defines `num_act_joint = 12`.
The source code value of 10 (5 per leg) is incorrect and causes
indices 10-11 in the shared memory struct to be uninitialized.

**Config change:**

```yaml
legs:
  left:
    base_id: 1
    motor_count: 6    # was 5
    motors: [1, 2, 3, 4, 5, 6]
  right:
    base_id: 7         # was 6
    motor_count: 6     # was 5
    motors: [7, 8, 9, 10, 11, 12]

udp:
  max_can_device: 7    # was 6; IDs < 7 go to server 0
```

### D5: Replace mutex-protected state with lock-free staging buffers

**Decision:** Each leg subsystem writes motor state to a
`SourceDoubleBuffer<MotorGroupStageData>` instead of updating
mutex-protected internal state. The `MotorGroupStageData` struct
contains all 6 motors' position, velocity, torque, temperature,
and status — decoded from Damiao feedback frames D[1:7] [7].

```cpp
template<typename T>
struct SourceDoubleBuffer {
    T buffers[2];
    std::atomic<uint32_t> write_idx{0};
    std::atomic<uint64_t> sequence{0};
    std::atomic<uint64_t> heartbeat_ns{0};

    void publish(const T& data);  // Lock-free atomic swap
    T read() const;               // Lock-free atomic read
};
```

**Why:** Eliminates priority inversion risk between leg threads
(SCHED_FIFO 90) and lower-priority readers (Composer at 85,
main loop at 75). The current mutex in `Imu::getStates()` [1]
and implicit Motor state locks create unbounded blocking under
the real-time scheduling policy.

### D6: Motor responsiveness timeout reduced to 100ms

**Decision:** Change the motor responsiveness timeout from 500ms [1][2]
to 100ms. At 400Hz, 100ms corresponds to 40 missed feedback frames.

**Why:** At 400Hz, the old 500ms timeout allows 200 missed frames —
too many for a bipedal robot that could fall within 200-300ms of
losing motor control. The faster inner loop deserves a proportionally
tighter timeout. The Damiao motor firmware processes commands faster
than 400Hz, so 100ms is sufficient for detecting genuine communication
loss (ERR=0x0D) [7].

### D7: Parameter query round-robin from `robotPeriodic()`

**Decision:** The main loop sends one parameter query per cycle at
10Hz effective rate (every 10th cycle of 100Hz). With 12 motors and
round-robin scheduling, each motor gets queried once per 1.2 seconds.

The query uses CAN ID `0x7FF` with D[2]=0x33 and D[3]=RID [8]:

```cpp
CANPacket CanPacketEncoder::create_query_param_command(const Motor& motor, int RID) {
    return {0x7FF, pack_query_param_data(motor.get_send_can_id(), RID)};
}
```

The response arrives via the existing `Motor::callback()` chain and is
identified when D[2]==0x33 [8]. The value is stored in a per-motor
atomic parameter cache (`MotorParamCache`), which the Composer reads
when building the composed `SensorData` snapshot.

**Why not a separate timer thread?** The parameter query shares the
same CAN bus as MIT commands [1]. A separate thread sending queries
concurrently with the inner loop creates socket contention. The main
loop's supervisory time slot avoids this.

### D8: Thread scheduling configuration

**Decision:**

| Thread | Policy | Priority | Stack |
|--------|--------|:--------:|:-----:|
| Left Leg Subsystem | `SCHED_FIFO` | 90 | 256KB |
| Right Leg Subsystem | `SCHED_FIFO` | 90 | 256KB |

Both legs at the same priority because they manage independent motor
groups on separate UDP sockets [1] and never contend for the same
resource. The `ControlledSubsystemBase` pthread creation must be
updated to call `pthread_setschedparam()` after `pthread_create()`.

**Open question:** CPU core affinity (`pthread_setaffinity_np`) should
be evaluated to eliminate starvation risk entirely when both legs run
at `SCHED_FIFO` 90.

## Risks / Trade-offs

- **[2.5ms deadline]** The inner loop budget drops from 5ms to 2.5ms.
  MIT frame encoding is ~5μs per motor (6 motors = ~30μs) [7][8],
  and SHM reads are ~0.15μs, so the computation is well within budget.
  However, UDP send/receive latency and kernel scheduling jitter could
  consume budget. → Mitigation: Profile on target ARM SBC; PREEMPT_RT
  kernel reduces jitter.

- **[Mercury Controller crash]** Up to 4 stale commands (10ms) may be
  dispatched before `robotPeriodic()` detects the crash via heartbeat
  timeout. → Mitigation: Accepted — the inner loop also checks command
  timestamp freshness as a secondary guard.

- **[CAN bus bandwidth]** At 400Hz × 6 motors = 2,400 MIT frames/sec
  per bus + 10 param queries/sec = ~2,410 frames/sec. At 1 Mbps CAN with
  ~148 bits/frame, utilization is ~36%. Well within safe limits (<70%).

- **[Config migration]** Changing `base_id` from 6 to 7 for the right leg
  and `max_can_device` from 6 to 7 requires updating `config.yaml` for
  all deployment environments (hardware, simulation). → Mitigation:
  Simulation uses `127.0.0.1` regardless of motor IDs [2]; hardware
  configs are environment-specific.

## Target Architecture

```
Mercury Controller         POSIX Shared Memory          Kuavo Actuator
(separate process)         (/mercury_robot_ipc)         (Legged Subsystem)

+------------------+                                +------------------------+
| Whole-body       |       Mercury_Command           | controllerPeriodic()   |
| dynamics         | ──────────────────────────────► | (400Hz, FIFO 90)       |
| DynaCoRE [2]     |       double buffer              |                        |
|                  |                                  | 1. Read cmd from SHM   |
|                  |       Mercury_SensorData         | 2. Check heartbeat     |
|                  | ◄────────────────────────────── | 3. MIT encode [7][8]   |
+------------------+       double buffer              | 4. CAN-over-UDP [6]   |
                           (composed by               | 5. Receive feedback    |
                            Composer thread)          | 6. Decode [7][8]       |
                                                      | 7. Write staging buf   |
                                                      +----------+-------------+
                                                                 |
                                                        CAN-over-UDP (13B) [6]
                                                                 |
                                                    +────────────+────────────+
                                                    ▼                         ▼
                                              Motors 1-6              Motors 7-12
                                              (left leg)              (right leg)
                                              UDP 8887/8886           UDP 8889/8888
```

```
robotPeriodic() (100Hz, FIFO 75) — SUPERVISORY ONLY

  1. Mode management (disabled/autonomous/teleop) [1]
  2. m_loop.poll() — button events [1]
  3. Motor enable/disable (one-time: 0xFC/0xFD) [7]
  4. Health monitoring (read composed_buffers)
  5. Safety validation (emergency_stop)
  6. Parameter queries at 10Hz (voltage/current via 0x7FF) [8]
  7. runAllRobotPeriodic() [1]
```
```