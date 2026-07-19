## Revised `domains.md`

```markdown
# Domain Map: Kuavo (Revised)

## Overview

Kuavo is a bipedal humanoid robot control system organized around a modular
subsystem architecture. The software runs as a single C++20 process on an
embedded ARM Linux SBC, coordinating 12 Damiao servo motors (6 per leg), an
LPMS-IG1 IMU, and operator input through a 10-thread architecture with
SCHED_FIFO real-time scheduling on a PREEMPT_RT kernel.

The Mercury whole-body dynamics controller runs as a **separate process**
communicating via POSIX shared memory. The inner control loop acts as a
command relay and motor driver — reading joint commands from shared memory
and dispatching MIT CAN frames to the motors at 400Hz [1][2].

---

## 1. Core Domains

### Motor Control
- **Status:** Core
- **Purpose:** Manages the lifecycle, communication, and impedance control
  of 12 Damiao servo motors (6 per leg) over CAN-over-UDP transport [1].
- **Technical Components:**
  - `Motor` (position, velocity, torque, temperature, status, MIT params)
  - `DmFrame` (CAN frame format, motor state machine)
  - `MotorType` (DM8009/DM10010L specs)
  - `MITParam` (kp, kd, q_des, dq_des, tau_ff)
  - `UdpServer` (x2 singletons, one per leg)
- **Data Flow:**
  - Inner control loop reads `Mercury_Command` from shared memory [2]
  - Encodes MIT frame: `float_to_uint` for position (16-bit), velocity
    (12-bit), Kp (12-bit), Kd (12-bit), torque (12-bit) [5][6]
  - Sends via CAN-over-UDP (13 bytes per motor) [4]
  - Receives feedback, decodes via `uint_to_float` [5][6]
  - Writes to `MotorGroupStageData` per-source staging double buffer
- **Observed Business Rules:**
  * Motor device IDs 1-6 route to UDP server 0 (left leg); IDs 7-12 to
    server 1 (right leg), determined by `max_can_device` threshold [1]
  * Motor enable requires 0xFC; disable requires 0xFD. Zero-position
    calibration uses 0xFE. Error clear uses 0xFB [1][5]
  * Motor state machine enforces safety: overvoltage (0x08), undervoltage
    (0x09), overcurrent (0x0A), MOS overtemp (0x0B), coil overtemp (0x0C),
    comm loss (0x0D), overload (0x0E) [1][5]
  * MIT parameters bounded by motor-type limits (DM8009: ±12.5 rad position,
    ±45 rad/s velocity, ±54 Nm torque) [1][5]
  * Motor responsiveness timeout: **100ms** (reduced from 500ms for 400Hz
    control rate)
  * **Parameter query** for bus voltage/current runs at 10Hz from
    `robotPeriodic()` via CAN ID 0x7FF, D[2]=0x33 [6]. Values stored in
    atomic per-motor parameter cache, merged by composer into SensorData.

### Legged Locomotion
- **Status:** Core
- **Purpose:** Manages bipedal leg subsystems (left and right), running the
  inner control loop as a **command relay and motor driver** that bridges the
  Mercury Controller's output to individual joint MIT commands [1][2].
- **Technical Components:**
  - `Legged` (ControlledSubsystemBase, 6 motors per leg)
  - Dedicated pthread per subsystem instance [1]
  - Per-source `SourceDoubleBuffer<MotorGroupStageData>` (lock-free)
- **Entry Points:**
  - `Legged::controllerPeriodic()` — inner control loop at **2.5ms / 400Hz**
    (changed from 5ms / 200Hz) [1]
  - `Legged::robotPeriodic()` — supervisory only at **10ms / 100Hz**
    (changed from 20ms / 50Hz) [1]
  - `Legged::onMessage()` — async command handler for enable/disable
- **Inner Control Loop (`controllerPeriodic()`) Responsibilities:**
  1. Read `Mercury_Command` from shared memory double buffer
  2. Check command freshness (heartbeat timestamp)
  3. Check `emergency_stop` flag
  4. For each of 6 motors: extract per-joint command
  5. Convert to MIT frame via `float_to_uint` [5][6]
  6. Send via CAN-over-UDP (13 bytes per motor) [4]
  7. Receive feedback, decode via `uint_to_float` [5][6]
  8. Check motor status (D[0] error flags) [5]
  9. Write to `MotorGroupStageData` staging buffer
- **Main Loop (`robotPeriodic()`) Responsibilities:**
  1. Mode management (disabled/autonomous/teleop lifecycle)
  2. Button event polling (`m_loop.poll()`)
  3. Motor enable/disable/calibration (one-time supervisory commands)
  4. Motor health monitoring (read composed sensor data)
  5. Safety validation (emergency stop, heartbeat checks)
  6. Parameter queries at 10Hz (bus voltage/current via CAN 0x7FF) [6]
- **Observed Business Rules:**
  * Each leg manages exactly **6 motors** (updated from 5); base_id=1 for
    left, base_id=7 for right [1]
  * **No LQR controller in the actuator process** — the Mercury Controller
    (separate process) handles whole-body dynamics and control computation
  * The inner loop is a **command relay and motor driver**, not a controller

### Inertial Sensing (IMU)
- **Status:** Core
- **Purpose:** Reads orientation and motion data from LPMS-IG1 IMU via
  sequential CAN-over-UDP frames, providing a 7D state vector [1].
- **Technical Components:**
  - `ImuReader` (dedicated pthread, blocking UDP socket on port 8887)
  - CAN frames 0x514-0x51B (8 sequential frames, 16 float32 values) [1]
  - Per-source `SourceDoubleBuffer<ImuStageData>` (lock-free)
- **Rate:** **500Hz** (unchanged from original — decision to keep 500Hz to
  avoid IMU-to-controller rate mismatch)
- **Observed Business Rules:**
  * IMU data is written to `imu_stage` double buffer, **not** to a
    mutex-protected state vector (mutex replaced with lock-free staging)
  * Prediction/extrapolation for IMU data between updates is an open
    question — less critical at 500:400 IMU-to-controller ratio

### Composer
- **Status:** Core (NEW)
- **Purpose:** Merges data from 3 independent per-source staging buffers
  into one consistent `SensorData` snapshot, preventing torn reads when
  the controller accesses multi-source data.
- **Rate:** **400Hz (2.5ms)**
- **Data Sources:**
  1. `imu_stage` (500Hz IMU data) — ~20% stale reads expected
  2. `motor_group_a_stage` (400Hz, motors 1-6)
  3. `motor_group_b_stage` (400Hz, motors 7-12)
  4. `MotorParamCache` (10Hz, bus voltage/current from parameter queries)
- **Outputs:**
  1. `composed_buffers` shared memory double buffer → Mercury Controller
  2. SPSC ring buffer push (2 LogRecords per cycle) → MQTT Logger thread
- **Observed Business Rules:**
  * The composer is the ONLY writer to `composed_buffers`
  * Per-source timestamps are preserved in the SensorData struct for
    staleness detection by the Mercury Controller

---

## 2. Supporting & Generic Domains

### MQTT Binary Logger
- **Status:** Supporting (NEW — replaces Telemetry & Data Logging)
- **Purpose:** Drains SPSC ring buffer and publishes binary `LogRecord`
  entries over MQTT to a remote x86 host for InfluxDB storage [2].
- **Technical Components:**
  - libwebsockets MQTT v5 client (no TLS, port 1883)
  - SPSC ring buffer (process-local, 4096 entries, ~5.1s buffer at 800 rec/s)
  - Binary payload: `BinaryPayloadHeader` (32 bytes) + raw struct
  - `#pragma pack(1)` for ARM/x86 cross-platform struct layout
- **Topics:**
  - `robot/command/bin` — binary Command records (QoS 0)
  - `robot/sensor/bin` — binary SensorData records (QoS 0)
  - `robot/status` — heartbeat/health (QoS 1, retained)
- **Observed Business Rules:**
  * Logger runs as Thread 5 inside the actuator+logger process — no
    separate process needed
  * Ring buffer overflow drops records silently (controller never blocks)
  * `dropped_` counter data race is accepted (diagnostic only)
  * Robot identity is in the binary payload header, not the MQTT topic

### Operator Interface (DriverStation + EventLoop)
- **Status:** Supporting
- **Purpose:** Parses operator input (Xbox controller/joystick) and
  dispatches button events for motor enable/disable [1][3].
- **Technical Components:**
  - `DriverStation` (UDP receiver, parses control words + joystick axes)
  - `EventLoop` + `BooleanEvent` (rising/falling edge detection)
  - Button 1: enable left leg, Button 2: disable, Button 4: async update [1]
- **Threads:** 2 (DriverStation UDP receiver + EventLoop dispatch)

### Communication Infrastructure
- **Status:** Generic
- **Purpose:** Transport layer for all inter-process and hardware
  communication [1][2].
- **Components:**
  - UDP sockets (motor CAN-over-UDP, IMU)
  - CAN protocol abstraction
  - MQTT client (libwebsockets, replaces previous telemetry MQTT) [2]
  - POSIX shared memory (Mercury Controller bridge) [2]
  - Lock-free double buffers (per-source staging)
  - SPSC ring buffer (process-local, for MQTT logging)
- **Port Assignments:**
  - Left leg: UDP 8887/8886 (motors 1-6) [1]
  - Right leg: UDP 8889/8888 (motors 7-12) [1]
  - IMU: UDP 8887 (shared with left leg server) [1]
  - MQTT broker: 1883 (no TLS)

### Configuration Management
- **Status:** Generic
- **Purpose:** Loads, validates, and provides access to all system
  configuration parameters via `Config` singleton backed by YAML [1].
- **New Configuration Section:**
  ```yaml
  data_logger:
    enabled: true
    mqtt_broker: "192.168.1.100"
    mqtt_port: 1883
    topic_command: "robot/command/bin"
    topic_sensor: "robot/sensor/bin"
    topic_status: "robot/status"
    robot_id: 1
    ring_buffer_capacity: 4096
    drain_batch_size: 20
    qos: 0
  ```

### Simulation & Testing Tools
- **Status:** Supporting
- **Purpose:** Software-in-the-loop testing including multi-motor Damiao
  simulator supporting up to 12 motors [1][4].
- **Changes:** DamiaoSimulator updated to support 12 motors via
  `-ids 1,2,3,4,5,6,7,8,9,10,11,12`

---

## 3. Boundary Map

- **Internal Communication:**
    - **Mercury Controller → Legged Subsystem:** Cross-process POSIX shared
      memory. Controller writes `Mercury_Command`; Legged reads at 400Hz [2]
    - **Legged Subsystem → Motor Control:** Direct `Motor` object method
      calls within the subsystem's inner control thread; motor feedback
      arrives via UDP callback chain [1]
    - **All Staging Buffers → Composer:** Lock-free double buffer reads.
      Composer reads `imu_stage`, `motor_group_a_stage`, `motor_group_b_stage`,
      and `MotorParamCache` at 400Hz
    - **Composer → Shared Memory:** Composed `SensorData` published to double
      buffer for Mercury Controller to read
    - **Composer → MQTT Logger:** SPSC ring buffer push (process-local)
    - **MQTT Logger → Remote Host:** Binary MQTT (libwebsockets, no TLS)
    - **robotPeriodic() → Motors (supervisory):** One-time enable/disable
      commands via async message to leg subsystem; parameter queries at 10Hz
      via CAN 0x7FF [6]
    - **Driver Station → Robot:** Mode transitions and button events flow
      through `EventLoop` callbacks [1]

- **Observed Coupling:**
    - **Motor Control ↔ Legged Locomotion:** Tightly coupled — `Legged`
      directly owns `Motor` instances [1]
    - **IMU ↔ ImuReader:** Direct wrapping, no interface abstraction [1]
    - **Telemetry ↔ All Subsystems:** **DECOUPLED** — telemetry collection
      moved from inline `robotPeriodic()` to composer thread reading from
      per-source staging buffers (previous tight coupling eliminated)
    - **Configuration ↔ Everything:** Global singleton implicit coupling [1]
```

---

## Revised `system.md`

```markdown
# System Specification: Kuavo (Revised)

## 1. Business Context

- **Purpose:** Real-time motor driver and telemetry framework for a bipedal
  humanoid robot (Kuavo platform), providing joint-level Damiao motor control,
  inertial state estimation, operator teleoperation, and binary MQTT data
  logging to a remote InfluxDB instance [1][2].
- **Core Problem:** Coordinates 12 Damiao servo motors across two legs and
  an LPMS-IG1 IMU at hard real-time rates (10ms main loop, 2.5ms inner
  control loop), preventing unsafe motor states, communication timeouts,
  and control divergence. The Mercury whole-body dynamics controller runs
  as a separate process [2].

## 2. Functional Core

- **Bipedal Leg Control:** Manages **6 Damiao motors per leg** (updated
  from 5) using MIT impedance control mode (kp, kd, q_des, dq_des, tau_ff)
  with CAN-over-UDP transport [1][5][6].
- **Command Relay:** Inner control loop reads joint commands from Mercury
  Controller via POSIX shared memory and dispatches MIT CAN frames to
  motors at 400Hz. No LQR controller in the actuator process.
- **Inertial State Estimation:** LPMS-IG1 IMU at 500Hz, 7D state vector [1].
- **Binary MQTT Telemetry:** Replaces previous 50Hz JSON SenML + binary
  `RobotStatusWire` with 400Hz binary `LogRecord` MQTT publishing to
  remote InfluxDB subscriber [1].
- **Parameter Monitoring:** Bus voltage/current queried at 10Hz via Damiao
  parameter query protocol (CAN ID 0x7FF, D[2]=0x33) [6].

## 3. Tech Stack

- **Language/Runtime:** C++20 (GCC, CMake 3.12+) [2]
- **Main Framework:** Custom FRC-inspired (`TimedRobot` hierarchy) [2]
- **Key Libraries:**
  - Eigen 3.4.1 — linear algebra [2]
  - spdlog 1.15.0 — structured logging [2]
  - libwebsockets 4.5.8 — MQTT client (no TLS) [2]
  - DynaCoRE (Mercury_Controller) — whole-body dynamics (separate process) [2]
  - POSIX threads + real-time extensions (pthread, rt) [2]
  - libcurl — InfluxDB HTTP writes (subscriber side only)
- **Kernel:** PREEMPT_RT patched Linux kernel required for SCHED_FIFO
  real-time scheduling

## 4. Architecture & Patterns

- **10-thread architecture** in single process:

| # | Thread | Rate | Priority | Stack |
|:---:|--------|:----:|:--------:|:-----:|
| 1 | Main Robot Loop | 100Hz (10ms) | FIFO 75 | 512KB |
| 2 | Left Leg Subsystem | 400Hz (2.5ms) | FIFO 90 | 256KB |
| 3 | Right Leg Subsystem | 400Hz (2.5ms) | FIFO 90 | 256KB |
| 4 | IMU Reader | 500Hz | FIFO 80 | 128KB |
| 5 | Composer | 400Hz (2.5ms) | FIFO 85 | 256KB |
| 6 | MQTT Logger | Network-paced | OTHER nice+10 | 512KB |
| 7 | DriverStation | Event-driven | OTHER 0 | 128KB |
| 8 | EventLoop | Polled in main | OTHER 0 | 128KB |
| 9 | UdpServer 0 (left) | Async callback | FIFO 88 | 128KB |
| 10 | UdpServer 1 (right) | Async callback | FIFO 88 | 128KB |

- **Total thread stack:** ~2.4 MB (reduced from default 80 MB)
- **Lock-free double buffers** replace all mutex-protected state access
- **Per-source staging** pattern: 3 independent writers + 1 composer
- **SPSC ring buffer** (process-local) between composer and MQTT logger
- **Cross-process shared memory** for Mercury Controller bridge [2]

## 5. Integration & APIs

- **Protocols:**
  - CAN-over-UDP (Damiao protocol v1.4) — 13-byte frames [4][5]
  - UDP sockets — left leg 8887/8886, right leg 8889/8888 [1]
  - MQTT binary — libwebsockets, no TLS, port 1883, binary payload
  - POSIX shared memory — Mercury Controller bridge [2]
  - HTTP line protocol — InfluxDB writes (subscriber side)
- **CAN Bus:** 2 buses at 1 Mbps, 6 motors per bus, ~36% utilization at
  400Hz MIT commands + 10Hz parameter queries [5]
- **MQTT Topics:** `robot/command/bin`, `robot/sensor/bin`, `robot/status`
- **Binary Payload:** `#pragma pack(1)` structs with `sizeof` assertions
  for ARM64/x86-64 cross-platform compatibility. Endianness: both
  little-endian (no byte swapping needed) [5][6]

## 6. Guardrails & Constraints

1. **Motor Safety:** State machine with automatic error detection [1].
   Motor responsiveness timeout: **100ms** (reduced from 500ms) [1][2].
2. **Real-time Threading:** Main loop at **100Hz (10ms)**, inner control
   at **400Hz (2.5ms)** [1][2]. PREEMPT_RT kernel required. All state
   access via **lock-free double buffers** (no mutexes in real-time path).
3. **CAN ID Discipline:** Motor IDs 1-6 = left leg (UDP server 0),
   IDs 7-12 = right leg (UDP server 1). CAN send ID = device_id;
   receive ID = device_id + 0x10 [1][2].
4. **Configuration-Driven:** All settings externalized in `config.yaml` [1][2].
5. **Cross-Process Safety:** If Mercury Controller process crashes,
   heartbeat detection in `robotPeriodic()` at 100Hz detects within 10ms
   and sets `emergency_stop`. Up to 4 stale commands may be dispatched
   before detection (accepted limitation) [2].
6. **No LQR in Actuator:** The actuator process is a command relay and
   motor driver only. The Mercury Controller handles all control
   computation in a separate process.

## 7. Open Questions

| # | Question | Status |
|:---:|----------|--------|
| 1 | SCHED_FIFO starvation between leg threads at same priority | Open — evaluate CPU pinning |
| 2 | IMU prediction/extrapolation between updates | Open — less critical at 500:400 ratio |
| 3 | `std::atomic<uint32_t>::is_always_lock_free` on ARM64 | Open — add compile-time assertion |
| 4 | `#pragma pack(1)` performance impact on ARM | Open — requires profiling |
| 5 | MQTT bandwidth test at 7.7 Mbps sustained | Open — test with fake data |
```

---

## Revised `architecture-diagrams.md`

```markdown
# Kuavo Bipedal Robot Controller — Architecture Diagrams (Revised)

## Level 1: System Context

```
                                    +-----------------------+
                                    |     Human Operator    |
                                    |  (Xbox / Joystick)    |
                                    +----------+------------+
                                               |
                                               | UDP packets
                                               | (DS protocol)
                                               v
+------------------+   CAN-over-UDP   +=============================+
|   Damiao Motors  | <---------------> |                             |
|  (DM8009 x12)   |   13-byte frames  |   Kuavo Actuator + Logger  |
|  6 per leg       |   [4]             |   (C++20, 10 threads)      |
|  MIT control     |                   |   ARM edge device          |
+------------------+                   |                             |
+=============================+
+------------------+   CAN-over-UDP           |            |
|   LPMS-IG1 IMU   | <--------------------+  |            |
|  (CAN sequential)|   13-byte frames [1]    |            |
|  500Hz [1]       |                          |            |
+------------------+                          |            |
|            |
POSIX Shared Memory        |   MQTT binary
(Mercury_Command,          |   (no TLS, 1883)
SensorData)               |   robot/sensor/bin
|   robot/command/bin
+------------------+                          |            |
| Mercury          | <-----------------------+            |
| Controller       |                                      |
| (separate proc)  |                                      v
| DynaCoRE [2]     |                          +------------------+
+------------------+                          | MQTT Broker      |
| (Mosquitto)      |
+--------+---------+
|
v
+------------------+
| x86 Remote Host  |
| MQTT Subscriber  |
| → InfluxDB       |
| → Grafana         |
+------------------+
```

## Level 2: Thread Architecture

```
+===========================================================================+
|                    KUAVO ACTUATOR + LOGGER PROCESS                         |
|                    (10 threads, ARM SBC, PREEMPT_RT)                      |
|                                                                           |
|  MAIN ROBOT LOOP (Thread 1, SCHED_FIFO 75, 100Hz/10ms, 512KB stack)      |
|  +---------------------------------------------------------------------+ |
|  | robotPeriodic() — SUPERVISORY ONLY                                  | |
|  |                                                                     | |
|  | 1. Mode management (disabled/autonomous/teleop) [1]                 | |
|  | 2. Button event polling (m_loop.poll()) [1][3]                      | |
|  | 3. Motor enable/disable (one-time: 0xFC/0xFD/0xFE/0xFB) [1][5]     | |
|  | 4. Health monitoring (read composed sensor data)                    | |
|  | 5. Safety validation (emergency_stop, heartbeat checks)             | |
|  | 6. Parameter queries at 10Hz (bus voltage/current, CAN 0x7FF) [6]  | |
|  | 7. Subsystem periodic dispatch (runAllRobotPeriodic)                | |
|  |                                                                     | |
|  | ** NO motor control, NO telemetry collection, NO MIT dispatch **    | |
|  +---------------------------------------------------------------------+ |
|                                                                           |
|  LEG SUBSYSTEMS (Threads 2-3, SCHED_FIFO 90, 400Hz/2.5ms, 256KB each)   |
|  +-----------------------------------+ +-------------------------------+ |
|  | Left Leg (Thread 2)               | | Right Leg (Thread 3)          | |
|  | Motors 1-6, UDP server 0          | | Motors 7-12, UDP server 1     | |
|  | controllerPeriodic() [1]:         | | (same as left leg)            | |
|  |                                   | |                               | |
|  | 1. Read Mercury_Command from SHM  | |                               | |
|  | 2. Check heartbeat freshness      | |                               | |
|  | 3. Check emergency_stop flag      | |                               | |
|  | 4. For each motor (x6):           | |                               | |
|  |    - Extract per-joint command    | |                               | |
|  |    - float_to_uint encode [5][6]  | |                               | |
|  |    - Send MIT CAN-over-UDP [4]    | |                               | |
|  | 5. Receive feedback, decode [5][6]| |                               | |
|  | 6. Check D[0] error flags [5]     | |                               | |
|  | 7. Write MotorGroupStageData      | |                               | |
|  |    (lock-free double buffer)      | |                               | |
|  +-----------------------------------+ +-------------------------------+ |
|                                                                           |
|  IMU READER (Thread 4, SCHED_FIFO 80, 500Hz, 128KB stack)                |
|  +---------------------------------------------------------------------+ |
|  | ImuReader (dedicated pthread, blocking UDP on port 8887) [1]        | |
|  | Parses 8 CAN frames (0x514-0x51B) → 16 float32 → 7D state [1]     | |
|  | Writes to imu_stage (lock-free double buffer)                       | |
|  +---------------------------------------------------------------------+ |
|                                                                           |
|  COMPOSER (Thread 5, SCHED_FIFO 85, 400Hz/2.5ms, 256KB stack)           |
|  +---------------------------------------------------------------------+ |
|  | Reads 3 per-source staging buffers + param cache:                    | |
|  |   imu_stage (500Hz) → imu_inc, imu_ang_vel, imu_acc                | |
|  |   motor_group_a_stage (400Hz) → joints 1-6 pos/vel/torque          | |
|  |   motor_group_b_stage (400Hz) → joints 7-12 pos/vel/torque         | |
|  |   MotorParamCache (10Hz) → bus_voltage, bus_current                | |
|  |                                                                     | |
|  | Merges into single SensorData snapshot                               | |
|  | Writes to composed_buffers (SHM double buffer → Mercury Controller) | |
|  | Pushes 2 LogRecords (cmd + sensor) to SPSC ring buffer              | |
|  +---------------------------------------------------------------------+ |
|                                                                           |
|  MQTT LOGGER (Thread 6, SCHED_OTHER nice+10, network-paced, 512KB)      |
|  +---------------------------------------------------------------------+ |
|  | Drains SPSC ring buffer (process-local, 4096 entries)               | |
|  | Serializes binary payload (BinaryPayloadHeader + raw struct)        | |
|  | Publishes via lws_service() loop (libwebsockets, no TLS)            | |
|  | Topics: robot/command/bin, robot/sensor/bin, robot/status            | |
|  | QoS 0 for data, QoS 1 retained for status (LWT)                    | |
|  |                                                                     | |
|  | ** Replaces RobotStatus (890B) + DataLog (JSON SenML) [1] **        | |
|  +---------------------------------------------------------------------+ |
|                                                                           |
|  OPERATOR INTERFACE (Threads 7-8, SCHED_OTHER, 128KB each)               |
|  +-----------------------------------+ +-------------------------------+ |
|  | DriverStation (Thread 7)          | | EventLoop (Thread 8)          | |
|  | UDP receiver, parses control      | | BooleanEvent edge detection   | |
|  | words + joystick axes [1][3]      | | Button dispatch to subsystems | |
|  +-----------------------------------+ +-------------------------------+ |
|                                                                           |
|  UDP SERVERS (Threads 9-10, SCHED_FIFO 88, 128KB each)                   |
|  +-----------------------------------+ +-------------------------------+ |
|  | UdpServer 0 (Thread 9)           | | UdpServer 1 (Thread 10)       | |
|  | Left leg, ports 8887/8886 [1]    | | Right leg, ports 8889/8888 [1]| |
|  | Motor feedback callbacks [1]      | | Motor feedback callbacks      | |
|  | + parameter query responses [6]   | | + parameter query responses   | |
|  +-----------------------------------+ +-------------------------------+ |
+===========================================================================+
```

## Level 3: Data Flow

```
Mercury Controller         POSIX Shared Memory          Kuavo Actuator
(separate process)         (/mercury_robot_ipc)         (10-thread process)

+------------------+                                +----------------------+
| Whole-body       |       Mercury_Command           | controllerPeriodic() |
| dynamics         | ──────────────────────────────► | (400Hz, Thread 2/3)  |
| DynaCoRE [2]    |       double buffer              |                      |
|                  |                                  | Read cmd → MIT [5]   |
|                  |       Mercury_SensorData         | → CAN-over-UDP [4]   |
|                  | ◄────────────────────────────── | → feedback decode [5]|
+------------------+       double buffer              | → staging buffer     |
(composed)                 +----------+-----------+
|
┌────────────────────────────────┘
|
v
+------------------------------------------------------------------+
|                    PER-SOURCE STAGING BUFFERS                      |
|                    (lock-free double buffers)                      |
|                                                                    |
|  imu_stage          motor_group_a    motor_group_b    MotorParam  |
|  (Thread 4,         (Thread 2,       (Thread 3,       Cache       |
|   500Hz) [1]         400Hz)           400Hz)          (Thread 1,  |
|                                                        10Hz) [6]  |
+--------+------------------+------------------+-----------+--------+
|                  |                  |           |
└──────┬───────────┴──────────┬───────┘           |
|                      |                   |
v                      v                   |
+──────────────────────────────────────┐          |
|         COMPOSER (Thread 5, 400Hz)   |◄─────────┘
|                                      |
| Read all sources → merge → snapshot  |
|                                      |
+──────────┬──────────────┬────────────+
|              |
v              v
┌──────────────┐  ┌──────────────────┐
| composed_buf |  | SPSC Ring Buffer  |
| (SHM double  |  | (process-local,  |
|  buffer)     |  |  4096 entries)    |
| → Mercury    |  └────────┬─────────┘
|   Controller |           |
└──────────────┘           v
┌──────────────────┐
| MQTT Logger      |
| (Thread 6)       |
| lws_service()    |
| binary payload   |
| no TLS           |
└────────┬─────────┘
|
MQTT (binary, QoS 0)
robot/sensor/bin
robot/command/bin
|
v
┌──────────────┐
| x86 Remote   |
| Subscriber   |
| → InfluxDB   |
| → Grafana    |
└──────────────┘
```

## Level 4: Motor MIT Command Sequence (one 2.5ms cycle)

```
controllerPeriodic()        SharedMemory        Motor           UdpServer
(Thread 2 or 3)             (double buf)        (x6)            (Thread 9/10)
|                         |                 |                  |
|--- read cmd_buffers --->|                 |                  |
|<-- Mercury_Command -----|                 |                  |
|                         |                 |                  |
|--- check heartbeat ---->|                 |                  |
|    (stale? → disable)   |                 |                  |
|                         |                 |                  |
| for each motor j (0-5): |                 |                  |
|    extract jpos_cmd[j]  |                 |                  |
|    extract jvel_cmd[j]  |                 |                  |
|    extract jtorque[j]   |                 |                  |
|    extract kp[j], kd[j] |                 |                  |
|                         |                 |                  |
|--- float_to_uint -------|---------------->|                  |
|    pos: 16-bit [5]      |                 |                  |
|    vel: 12-bit [5]      |                 |                  |
|    kp:  12-bit [5]      |                 |                  |
|    kd:  12-bit [5]      |                 |                  |
|    tau: 12-bit [5]      |                 |                  |
|                         |                 |                  |
|--- MIT CAN frame -------|---------------->|--- UDP send ---->|
|    13 bytes [4]         |                 |    CAN-over-UDP  |
|                         |                 |                  |
|                         |                 |<-- UDP recv -----|
|                         |                 |    feedback [5]  |
|<-- uint_to_float -------|-----------------|                  |
|    D[1:2]=POS 16-bit    |                 |                  |
|    D[3:4]=VEL 12-bit    |                 |                  |
|    D[4:5]=T   12-bit    |                 |                  |
|    D[6]=T_MOS  [5]      |                 |                  |
|    D[7]=T_Rotor [5]     |                 |                  |
|                         |                 |                  |
|--- write staging buf -->|                 |                  |
|    (lock-free double    |                 |                  |
|     buffer publish)     |                 |                  |
```

## Level 4: Parameter Query Sequence (10Hz from robotPeriodic)

```
robotPeriodic()          UdpServer            Motor            MotorParamCache
(Thread 1, 100Hz)        (Thread 9/10)        callback         (atomic)
|                      |                   |                  |
| every 10th cycle:    |                   |                  |
|                      |                   |                  |
|--- query CAN 0x7FF ->|                   |                  |
|    D[2]=0x33 [6]     |--- UDP send ----->|                  |
|    D[3]=RID(voltage) |                   |                  |
|                      |                   |                  |
|                      |<-- UDP recv ------|                  |
|                      |    D[2]=0x33 [6]  |                  |
|                      |    D[3]=RID       |                  |
|                      |    D[4:7]=value   |                  |
|                      |                   |                  |
|                      |--- callback ----->|                  |
|                      |                   |-- atomic store ->|
|                      |                   |   bus_voltage    |
|                      |                   |   bus_current    |
|                      |                   |                  |
|                                                             |
|  (Composer reads param cache at 400Hz)                      |
|                                                             |
```

## Key Changes from Original Architecture

| Aspect | Original [1][2] | Revised |
|--------|-----------------|---------|
| Motor count | 10 (5 per leg) | **12 (6 per leg)** |
| Main loop rate | 20ms / 50Hz | **10ms / 100Hz** |
| Inner control rate | 5ms / 200Hz | **2.5ms / 400Hz** |
| IMU rate | ~500Hz | **500Hz (unchanged)** |
| Controller | LQR in same process | **Mercury Controller, separate process** |
| Motor timeout | 500ms | **100ms** |
| State access | Mutex-protected | **Lock-free double buffers** |
| Telemetry | 50Hz inline (890B + JSON) | **400Hz binary MQTT logger** |
| Telemetry coupling | Direct subsystem access [1] | **Decoupled via staging buffers** |
| Thread count | ~9 | **10** |
| Scheduling | Default | **SCHED_FIFO with PREEMPT_RT** |
| Kernel | Standard Linux | **PREEMPT_RT required** |
| Data logging | Local MQTT (localhost) | **Remote MQTT → InfluxDB** |
```