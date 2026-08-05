
# Module Documentation: Robot, Leg Subsystem, DriverStation, IMU, UDP

---

## `domains.md`

### Robot Framework

- **Status:** Generic
- **Purpose:** Provides the base class hierarchy and lifecycle management for the robot application, including periodic scheduling, subsystem registration, and mode management [2].
- **Technical Components:**
    - `TimedRobot` — main loop orchestrator, default period 20 ms (50 Hz) [2]
    - `IterativeRobotBase` — mode-specific init/periodic callbacks [2]
    - `SubsystemBase` — subsystem registration and lifecycle [2]
    - `ControlledSubsystemBase<States, Inputs, Outputs>` — spawns a dedicated pthread per subsystem instance for non-blocking message processing [2]
- **Entry Points:**
    - `Robot::robotPeriodic()` — 20 ms main loop: motor queries, telemetry collection, button event polling [2]
    - `Robot::robotInit()` — one-time initialization after construction
    - `Robot::disabledInit()` / `autonomousInit()` / `teleopInit()` — mode transition callbacks [2]
- **Data Flow:**
    - `loopFunc()` chain: `Notifier fires → IterativeRobotBase::loopFunc() → refreshData() → mode switch → robotPeriodic()` [2]
    - Robot main loop collects state from all subsystems and publishes via `RobotStatus` and `DataLog` each cycle [2]
    - Configuration loaded via `Config::instance()` singleton from `config/config.yaml` [2]
- **Observed Business Rules:**
    - `TimedRobot` default period is 20 ms (50 Hz); inner control loops run at configurable faster rates [2]
    - Subsystem lifecycle follows init/periodic pattern per operating mode (disabled, autonomous, teleop) [2]
    - Operating modes follow the FRC pattern with `init()` and `periodic()` callbacks per mode [2]
    - All network addresses, port mappings, motor types, leg assignments, IMU parameters, and logger settings are externalized in `config/config.yaml` [3]
    - Hardware vs. simulation switching is done by changing UDP target IPs (192.168.4.x for hardware, 127.0.0.1 for simulation) [3]
- **Coupling:**
    - Telemetry collection is hardcoded in `Robot::robotPeriodic()`, directly accessing subsystem internals rather than through a publish/subscribe interface [2]
    - Configuration uses global singleton `Config::instance()` creating implicit coupling; all domains depend on the YAML structure remaining stable [2]

---

### Legged Locomotion (Leg Subsystem)

- **Status:** Core
- **Purpose:** Manages the bipedal leg subsystems (left and right), orchestrating motor groups, running the inner control loop, and bridging the controller output to individual joint commands [2].
- **Technical Components:**
    - `Legged` — template `ControlledSubsystemBase<7, 2, 5>` (7 states, 2 inputs, 5 outputs per leg) [2]
    - `Controller` — template `ControllerBase<7, 2, 4>` (LTV-LQR) [2]
    - `Motor` (×5 per leg) — position, velocity, torque, temperature, status, MIT params [2]
    - `MITParam` — kp, kd, q_des, dq_des, tau_ff [2]
- **Entry Points:**
    - `Legged::robotPeriodic()` — motor queries + MIT dispatch at 20 ms [2]
    - `Legged::controllerPeriodic()` — inner control loop at 5 ms [2]
    - `Legged::onMessage()` — async command handler for enable/disable [2]
- **Data Flow:**
    - Robot main loop calls `Legged::robotPeriodic()` periodically (synchronous, 20 ms cycle) and dispatches async messages (non-blocking) [2]
    - Legged subsystem directly owns `Motor` objects; motor feedback arrives via UDP callback chain [2]
    - Controller `calculate()` is called from `controllerPeriodic()` within the subsystem thread [2]
    - Motor commands use MIT impedance control with CAN-over-UDP transport [2]
- **Observed Business Rules:**
    - Each leg manages exactly 5 motors; motor array initialized from `config.yaml` leg definitions (left: base_id=1, right: base_id=6) [2]
    - Controller operates on a 7D state vector and produces a 2D input vector; gains are precomputed in lookup tables for real-time performance [2]
    - Subsystem threads use async FIFO message queues with `poll()`-based notification to avoid blocking the main robot loop [2]
    - Right leg subsystem is instantiated but currently disabled in code; only left leg runs the active control loop [2]
    - `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance [2] — thread starts immediately in the constructor
    - Motor responsiveness is tracked with a 500 ms timeout; unresponsive motors are flagged [2]
- **Coupling:**
    - Motor Control ↔ Legged Locomotion: Tightly coupled — `Legged` directly owns and manages `Motor` instances; no abstraction layer between subsystem and motor objects [2]
    - Control Algorithms ↔ Legged: Controller is templated but instantiated within `Legged`; control algorithm selection is compile-time, not runtime [2]

---

### Operator Interface (DriverStation)

- **Status:** Supporting
- **Purpose:** Provides the human-machine interface for robot control, handling Xbox controller input, joystick axes, button events, and operating mode transitions (disabled, autonomous, teleop) [2].
- **Technical Components:**
    - `DriverStation` — UDP receiver, parses control words and joystick axes [2]
    - `EventLoop` — event dispatch loop [2]
    - `BooleanEvent` — rising/falling edge detection for buttons [2]
- **Entry Points:**
    - `DriverStation` — UDP packet parser (event-driven) [2]
    - `EventLoop` — polled via `m_loop.poll()` from `robotPeriodic()` [2]
- **Data Flow:**
    - DriverStation receives UDP packets from operator console [2]
    - EventLoop dispatches button events to subsystem enable/disable methods [2]
    - Mode transitions and button events flow through `EventLoop` callbacks [2]
- **Observed Business Rules:**
    - Button 1 enables the left leg; Button 2 disables it; Button 3 (reboot) is currently disabled; Button 4 triggers async state update [2]
    - `BooleanEvent` provides rising/falling edge detection to prevent repeated triggers from held buttons [2]
    - Operating modes (Disabled/Autonomous/Teleop) follow the FRC pattern with `init()` and `periodic()` callbacks per mode [2]

---

### Inertial Sensing (IMU)

- **Status:** Core
- **Purpose:** Reads orientation and motion data from an LPMS-IG1 IMU sensor via sequential CAN-over-UDP frames, providing a 7D state vector (Euler angles + quaternion) for balance and state estimation [2].
- **Technical Components:**
    - `Imu` — subsystem, 7D state vector [2]
    - `ImuReader` — dedicated pthread, blocking UDP socket on port 8887, CAN frame parser [2]
    - IMU payload — 16 float32 values: accelerometer, gyroscope, magnetometer, Euler angles, quaternion [2]
- **Entry Points:**
    - `ImuReader` — dedicated pthread, blocking UDP socket on port 8887 [2]
    - `Imu::getStates()` — thread-safe state accessor (mutex-protected) [2]
    - `Imu::update()` — frame aggregation callback [2]
- **Data Flow:**
    - LPMS-IG1 hardware sends CAN frames 0x514-0x51B (8 sequential frames) [2]
    - Frames arrive via CAN-over-UDP bridge as UDP packets [2]
    - ImuReader thread parses 16 float32 values per measurement cycle [2]
    - 7D state vector [eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ] is exposed to the main loop [2]
    - IMU state is read via thread-safe `getStates()` accessor; no push mechanism [2]
- **Observed Business Rules:**
    - IMU data arrives as 8 sequential CAN frames (IDs 0x514 through 0x51B), each carrying 2 float32 values [2]
    - State access is mutex-protected via `getStates()` [2]
    - IMU shares UDP port 8887 with UdpServer 0 (left leg motors) [3]
    - CAN base ID is 0x514 (configurable) [3]
- **Coupling:**
    - `Imu` subsystem wraps `ImuReader` directly; no interface abstraction, making it difficult to swap IMU implementations [2]
    - IMU state vector is read via mutex-protected accessor from the main loop, creating potential contention [2]

---

### Communication Infrastructure (UDP)

- **Status:** Generic
- **Purpose:** Provides the transport layer abstractions for all inter-process and hardware communication: UDP sockets, CAN protocol, MQTT client, and shared memory IPC [2].
- **Technical Components:**
    - `UdpServer` (×2 singletons) — one per leg, manages CAN-over-UDP transport [2]
    - `CAN` / `CANAPI` — HAL-style CAN abstraction [2]
    - `MqttClient` — libwebsockets 4.5.8, MQTT over WebSocket [2][3]
    - POSIX shared memory — IPC bridge to Mercury dynamics controller (`mercury_shm.h`) [2][3]
- **Entry Points:**
    - `UdpServer` — UDP socket listener/sender per leg [2]
    - `Motor::callback()` — CAN frame reception callback [2]
- **Data Flow:**
    - Motor commands: `Legged → Motor → CAN/CANAPI → UdpServer → UDP socket → Damiao Motor HW` [2]
    - Motor feedback: `Damiao HW → UDP socket → UdpServer → callback → Motor → Legged` [2]
    - IMU data: `LPMS-IG1 → CAN-over-UDP Bridge → UDP → ImuReader` [2]
    - Telemetry: `Robot main loop → MqttClient → MQTT broker (localhost:1883)` [2]
- **Observed Business Rules:**
    - UDP port assignment follows a formula: `localPort = base_local_port + server_id * 2`, `remotePort = base_remote_port + server_id * 2` [2]
    - Left leg: UDP ports 8887/8886; Right leg: UDP ports 8889/8888 [3]
    - CAN device IDs use a 6-bit address space (0-63) with manufacturer code 2 and device type 4 (motion controller) [2]
    - Motor device IDs 1-5 route to UDP server 0 (left leg); IDs 6-10 to server 1 (right leg), determined by `max_can_device` threshold (device_id < 6) [2]
    - CAN send ID equals device_id; receive ID equals device_id + 0x10 [2][3]
    - MQTT uses libwebsockets with username/password authentication; connection is non-blocking with automatic reconnection [2]
    - Motor CAN-over-UDP frame format: 13 bytes — CAN_ID + DLC + data[8] [3]

---

## `system.md`

### 1. Business Context

- **Purpose:** Real-time control framework for a bipedal humanoid robot (Kuavo platform), providing joint-level motor control, inertial state estimation, trajectory tracking, and operator teleoperation through a modular subsystem architecture [3].
- **Core Problem:** Coordinates 10 Damiao servo motors across two legs and an LPMS-IG1 IMU at hard real-time rates (20 ms main loop, 5 ms control inner loop), preventing unsafe motor states, communication timeouts, and control divergence that could damage hardware or cause the robot to fall [3].

### 2. Functional Core

- **Bipedal Leg Control:** Manages 5 Damiao motors per leg (DM8009/DM10010L) using MIT impedance control mode (kp, kd, q_des, dq_des, tau_ff) with CAN-over-UDP transport, including motor enable/disable, zero-position calibration, and error recovery state machine [3].
- **Inertial State Estimation:** Reads LPMS-IG1 IMU data via sequential CAN frames over UDP, providing a 7D state vector for closed-loop balance control [3].
- **LQR Trajectory Tracking:** Implements a Linear Time-Varying LQR controller with plant-inversion feedforward, spline-based trajectory generation, and precomputed gain lookup tables on a 7-state / 2-input system [3].
- **Teleoperation:** Supports Xbox controller input with event-driven button handling and joystick axes, modeled after the FRC Driver Station pattern [3].
- **Telemetry:** Publishes binary robot status packets (~890 bytes) and JSON SenML data logs over MQTT at 50 Hz [3].

### 3. Tech Stack

- **Language/Runtime:** C++20 (GCC, CMake 3.12+) [3]
- **Main Framework:** Custom FRC-inspired robot framework (`TimedRobot` / `IterativeRobotBase` / `SubsystemBase` / `ControllerBase` template hierarchy) [3]
- **Key Libraries:**
    - Eigen 3.4.1 — linear algebra [3]
    - spdlog 1.15.0 — structured logging [3]
    - libwebsockets 4.5.8 — MQTT client [3]
    - nlohmann/json 3.11.3 — JSON serialization [3]
    - DynaCoRE (Mercury_Controller) — whole-body dynamics [3]
    - POSIX threads + real-time extensions (pthread, rt) [3]

### 4. Architecture

- **Folder Structure:**
    - `src/` — Robot main class, subsystems (Legged, Imu), controllers [3]
    - `lib/` — reusable framework: robot base classes, motor control, IMU reader, driver station, MQTT, telemetry [3]
    - `include/` — public headers [3]
    - `tools/` — standalone utilities: DriverStation, DamiaoSimulator, actuator/controller test harnesses [3]
    - `config/` — YAML configuration [3]
    - `srSimulator/` — Mercury dynamics simulator [3]
- **Design Patterns:**
    - Template-based Controller hierarchy (`ControllerBase<States, Inputs, Outputs>`) [3]
    - Singleton configuration (`Config::instance()`) [3]
    - Observer / callback pattern for motor state updates [3]
    - Event-driven input handling (`EventLoop` + `BooleanEvent`) [3]
    - Async message queue with poll-based threading [3]
    - Periodic scheduler with priority queue (`TimedRobot`) [3]

### 5. Integration & APIs

- **Protocols:**
    - CAN-over-UDP (Damiao motor protocol v1.4) — 13-byte frames, bidirectional motor command/feedback [3]
    - UDP sockets — motor communication (left leg 8887/8886, right leg 8889/8888) and IMU data reception [3]
    - MQTT over WebSocket — telemetry publication to broker (localhost:1883) [3]
    - POSIX shared memory — IPC bridge to Mercury dynamics controller (`mercury_shm.h`) [3]
- **External Dependencies:**
    - DynaCoRE / Mercury_Controller at `/usr/local/include/DynaCoRE` [3]
    - Damiao DM8009 / DM10010L motor controllers via CAN-over-UDP [3]
    - LPMS-IG1 IMU, CAN base ID 0x514, 8 sequential frames [3]
    - MQTT broker (Mosquitto or compatible, localhost:1883) [3]

### 6. Guardrails & Constraints

1. **Motor Safety:** Motors implement a state machine with automatic error detection — overvoltage (0x08), undervoltage (0x09), overcurrent (0x0A), MOS overtemp (0x0B), coil overtemp (0x0C), comm loss (0x0D), overload (0x0E). Motor enable/disable commands are 0xFC/0xFD with 500 ms responsiveness timeout [2][3].
2. **Real-time Threading:** Main control loop at 50 Hz (20 ms), inner loops at 200 Hz (5 ms). All motor and IMU state access is mutex-protected. Subsystem threads use `poll()`-based event notification [3].
3. **CAN ID Discipline:** Motor IDs 1-5 = left leg (server 0), IDs 6-10 = right leg (server 1). Send ID = device_id; receive ID = device_id + 0x10 [2][3].
4. **Configuration-Driven:** All settings externalized in `config/config.yaml`. Hardware vs. simulation switching via UDP target IPs [3].
5. **C++20 Required:** Build requires `-std=c++20` and CMake 3.12+ [3].

---

## `design.md`

### Robot Module

#### Context

The `Robot` class inherits from `TimedRobot` and orchestrates the entire system lifecycle [2][3]. The `loopFunc()` chain — `Notifier fires → IterativeRobotBase::loopFunc() → refreshData() → mode switch → robotPeriodic()` — runs at 20 ms / 50 Hz [2]. Inside `robotPeriodic()`, the robot collects state from all subsystems (motors via `leftLeg.getMotors()`, IMU via `imu.getStates()`) and publishes telemetry via `RobotStatus` and `DataLog` each cycle [2].

#### Key Design Decisions

**D1: Inline telemetry collection in robotPeriodic()**

Telemetry is collected by directly accessing subsystem internals from the main loop [2]. This creates tight coupling — adding a new subsystem or changing subsystem internals requires modifying `robotPeriodic()` [2]. The alternative would be a publish/subscribe pattern, but the current implementation prioritizes simplicity.

**D2: Mode management follows FRC pattern**

Operating modes (Disabled/Autonomous/Teleop) use `init()` and `periodic()` callbacks per mode [2]. Button 1 enables the left leg; Button 2 disables it [2]. This pattern comes from the FIRST Robotics Competition framework and provides a familiar lifecycle model [3].

**D3: Single-process architecture**

All subsystems run within a single C++20 process [3]. The Mercury Controller is the only external process, communicating via POSIX shared memory (`mercury_shm.h`) [3]. This eliminates IPC overhead for the real-time control path but means a crash in any subsystem takes down the entire robot.

---

### Leg Subsystem Module

#### Context

The `Legged` class implements `ControlledSubsystemBase<7, 2, 5>` with 5 motors per leg [2]. The `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance [2]. The inner control loop (`controllerPeriodic()`) runs at 200 Hz (5 ms), performing LQR trajectory tracking via precomputed gain lookup tables [2]. The right leg is instantiated but currently disabled [2].

#### Key Design Decisions

**D1: Thread spawned in constructor**

`ControlledSubsystemBase` creates the pthread immediately in its constructor at line 51 [2]. This means the thread starts running `controllerPeriodic()` before `Robot::Robot()` finishes constructing all members. If `controllerPeriodic()` accesses shared state (e.g., POSIX shared memory from the Mercury Controller) that is not yet initialized, a SEGV results.

**D2: Direct Motor ownership**

`Legged` directly owns and manages `Motor` instances with no abstraction layer [2]. Motor commands use MIT impedance control — the MIT frame packs 5 parameters (position 16-bit, velocity 12-bit, Kp 12-bit, Kd 12-bit, torque 12-bit) into 8 CAN bytes [6]:

```cpp
// MIT encoding from dm_motor_control.cpp [6]
uint16_t q_uint = double_to_uint(mit_param.q, -limits.pMax, limits.pMax, 16);
uint16_t dq_uint = double_to_uint(mit_param.dq, -limits.vMax, limits.vMax, 12);
uint16_t tau_uint = double_to_uint(mit_param.tau, -limits.tMax, limits.tMax, 12);
```

Motor enable uses 0xFC, disable uses 0xFD, zero-position calibration uses 0xFE, error clear uses 0xFB [6]:

```cpp
CANPacket create_enable_command(const Motor& motor) {
    return {motor.get_send_can_id(), pack_command_data(0xFC)};
}
// pack_command_data returns: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd} [6]
```

**D3: Motor safety state machine**

Motor state machine enforces safety with error codes: overvoltage (0x08), undervoltage (0x09), overcurrent (0x0A), MOS overtemp (0x0B), coil overtemp (0x0C), comm loss (0x0D), overload (0x0E) [2]. Motor responsiveness is tracked with 500 ms timeout [2][3].

**D4: Controller is compile-time bound**

Controller is templated (`ControllerBase<7, 2, 4>`) and instantiated within `Legged` [2]. Control algorithm selection is compile-time, not runtime [2]. The controller operates on a 7D state vector and produces a 2D input vector using precomputed gain lookup tables [2].

**D5: Motor feedback decoding**

Feedback frame is decoded by `parse_motor_state_data()` [6]:

```cpp
uint16_t q_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];        // Position 16-bit
uint16_t dq_uint = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4); // Velocity 12-bit
uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5]; // Torque 12-bit
int t_mos = static_cast<int>(data[6]);    // MOS temperature
int t_rotor = static_cast<int>(data[7]);  // Rotor temperature [6]
```

---

### DriverStation Module

#### Context

The DriverStation provides the human-machine interface using an Xbox controller / joystick [2]. It parses UDP control packets and dispatches button events via the `EventLoop` with `BooleanEvent` edge detection [2].

#### Key Design Decisions

**D1: Event-driven button dispatch**

Button events flow through `EventLoop` callbacks to subsystem `onMessage()` handlers [2]. `BooleanEvent` provides rising/falling edge detection to prevent repeated triggers from held buttons [2]. This means a held button only fires once on press and once on release.

**D2: FRC-style mode management**

Operating modes follow the FRC pattern: Disabled, Autonomous, Teleop [2]. Each mode has `init()` and `periodic()` callbacks [2]. Mode transitions are driven by DriverStation control words received via UDP.

**D3: Button-to-action mapping**

Current mapping [2]:

| Button | Action |
|:------:|--------|
| 1 | Enable left leg |
| 2 | Disable left leg |
| 3 | Reboot (disabled) |
| 4 | Async state update |

The mapping is hardcoded — there is no configuration-driven button assignment.

---

### IMU Module

#### Context

The LPMS-IG1 IMU sends 8 sequential CAN frames (IDs 0x514 through 0x51B) per measurement cycle, each carrying 2 float32 values, totaling 16 floats [2]. The `ImuReader` runs as a dedicated pthread with a blocking UDP socket on port 8887, using epoll for non-blocking frame reception [2].

#### Key Design Decisions

**D1: Dedicated pthread for IMU reading**

The `ImuReader` is a standalone pthread that blocks on UDP receive, separate from the main loop and motor control threads [2]. This ensures IMU frame parsing is not delayed by motor command dispatch or telemetry collection.

**D2: Mutex-protected state access**

The 7D state vector is accessed via `Imu::getStates()` which uses a mutex [2]. This creates potential priority inversion if a lower-priority thread holds the mutex when a higher-priority thread needs IMU data. The current design accepts this risk because all threads run at default `SCHED_OTHER` priority [3].

**D3: Sequential frame aggregation**

All 8 CAN frames must be from the same measurement cycle before the state vector is updated [2]. The ImuReader tracks frame arrival by CAN ID offset from the base (0x514) and only publishes the complete 16-float measurement after the 8th frame arrives.

**D4: Port sharing with motor UdpServer**

The IMU reader uses the same UDP port 8887 as UdpServer 0 (left leg motors) [3]. CAN ID range filtering distinguishes IMU frames (0x514-0x51B) from motor feedback frames (device_id + 0x10 = 0x11-0x15) [2].

---

### UDP Module

#### Context

Two `UdpServer` singleton instances manage all CAN-over-UDP motor communication [2]. Port assignment follows a formula: `localPort = base_local_port + server_id * 2`, `remotePort = base_remote_port + server_id * 2` [2]. The 13-byte CAN frame format carries motor commands and feedback bidirectionally [3].

#### Key Design Decisions

**D1: Two singletons for two legs**

Motor device IDs 1-5 route to UDP server 0 (left leg, ports 8887/8886); IDs 6-10 route to UDP server 1 (right leg, ports 8889/8888) [2]. The routing threshold is `device_id < 6` (configurable via `max_can_device`) [2].

**D2: CAN ID offset convention**

CAN send ID equals device_id; receive ID equals device_id + 0x10 [2][3]. This allows bidirectional communication on the same UDP socket pair. Motor CAN IDs are in the range 0x01-0x0A (send) and 0x11-0x1A (receive), while IMU uses 0x514-0x51B — no overlap [2].

**D3: Motor control modes use CAN ID offsets**

Different control modes add offsets to the base CAN ID [6]:
- MIT mode: `send_can_id` (no offset) [6]
- PosVel mode: `send_can_id + 0x100` [6]
- Velocity mode: `send_can_id + 0x200` [6]
- PosForce mode: `send_can_id + 0x300` [6]

Parameter queries use fixed CAN ID `0x7FF` [6]:

```cpp
CANPacket create_query_param_command(const Motor& motor, int RID) {
    return {0x7FF, pack_query_param_data(motor.get_send_can_id(), RID)};
} // [6]
```

**D4: Parameter response identification**

Parameter responses are identified by checking `data[2] == 0x33` (read) or `data[2] == 0x55` (write) [6]. Integer-type RIDs include ranges 7-10, 13-16, and 35-36 (decoded as uint32); all others are decoded as float [6]:

```cpp
if (CanPacketDecoder::is_in_ranges(RID)) {
    num = uint8s_to_uint32(data[4], data[5], data[6], data[7]);
} else {
    std::array<uint8_t, 4> float_bytes = {data[4], data[5], data[6], data[7]};
    num = uint8s_to_float(float_bytes);
} // [6]
```

**D5: Callback-based feedback dispatch**

Motor feedback frames arrive asynchronously via the UdpServer socket. The UdpServer dispatches received CAN frames to `Motor::callback()` based on the CAN ID [2]. The motor state machine processes the feedback and updates internal state, which the `Legged` subsystem reads during its periodic loop [2].



# Logic / Function Block Diagram

## Module Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              KUAVO ROBOT CONTROLLER                                 │
│                              (Single C++20 Process)                                 │
│                                                                                     │
│  ╔═══════════════════════════════════════════════════════════════════════════════╗   │
│  ║                     ROBOT FRAMEWORK (TimedRobot)                             ║   │
│  ║                     Main Loop: 20 ms / 50 Hz [2]                            ║   │
│  ║                                                                             ║   │
│  ║  ┌─────────────────────────────────────────────────────────┐                 ║   │
│  ║  │ Robot::robotPeriodic() [2]                              │                 ║   │
│  ║  │                                                         │                 ║   │
│  ║  │  Notifier ──► loopFunc() ──► refreshData()              │                 ║   │
│  ║  │           ──► mode switch ──► robotPeriodic()            │                 ║   │
│  ║  │                                                         │                 ║   │
│  ║  │  Responsibilities:                                      │                 ║   │
│  ║  │  • Mode management (disabled/autonomous/teleop)         │                 ║   │
│  ║  │  • m_loop.poll() (button events)                        │                 ║   │
│  ║  │  • Telemetry collection from subsystems                 │                 ║   │
│  ║  │  • RobotStatus::collect() + publish()                   │                 ║   │
│  ║  │  • DataLog::logMotors() + logImu()                      │                 ║   │
│  ║  └──────────┬──────────┬──────────┬──────────┬─────────────┘                 ║   │
│  ╚═════════════╪══════════╪══════════╪══════════╪═════════════════════════════════╝   │
│                │          │          │          │                                     │
│     ┌──────────┘   ┌──────┘   ┌──────┘   ┌──────┘                                   │
│     ▼ CTRL         ▼ DATA     ▼ DATA     ▼ CTRL                                     │
│                                                                                     │
│  ╔══════════════════════╗  ╔═══════════════╗  ╔══════════════════════════╗           │
│  ║  LEGGED SUBSYSTEM    ║  ║  IMU          ║  ║  DRIVER STATION         ║           │
│  ║  (×2, per leg) [2]   ║  ║  SUBSYSTEM    ║  ║  + EVENT LOOP [2]      ║           │
│  ║                      ║  ║  [2]          ║  ║                        ║           │
│  ║  ControlledSubsystem ║  ║  ┌──────────┐ ║  ║  ┌──────────────────┐  ║           │
│  ║  Base<7,2,5>         ║  ║  │ImuReader │ ║  ║  │DriverStation    │  ║           │
│  ║  Dedicated pthread   ║  ║  │(pthread) │ ║  ║  │(UDP parser)     │  ║           │
│  ║  [2]                 ║  ║  │epoll     │ ║  ║  └────────┬─────────┘  ║           │
│  ║                      ║  ║  │port 8887 │ ║  ║           │            ║           │
│  ║  ┌────────────────┐  ║  ║  │[2]       │ ║  ║  ┌────────▼─────────┐  ║           │
│  ║  │controllerPer-  │  ║  ║  └─────┬────┘ ║  ║  │EventLoop        │  ║           │
│  ║  │iodic() 5ms [2] │  ║  ║        │      ║  ║  │BooleanEvent     │  ║           │
│  ║  │                │  ║  ║  ┌─────▼────┐ ║  ║  │(edge detection) │  ║           │
│  ║  │ LQR calculate  │  ║  ║  │Imu       │ ║  ║  │[2]              │  ║           │
│  ║  │ MIT dispatch   │  ║  ║  │7D state  │ ║  ║  └──────────────────┘  ║           │
│  ║  └───────┬────────┘  ║  ║  │vector    │ ║  ║                        ║           │
│  ║          │           ║  ║  │(mutex)   │ ║  ║  Buttons:              ║           │
│  ║  ┌───────▼────────┐  ║  ║  │[2]       │ ║  ║  1: enable left leg   ║           │
│  ║  │robotPeriodic() │  ║  ║  └──────────┘ ║  ║  2: disable left leg  ║           │
│  ║  │20ms [2]        │  ║  ║               ║  ║  3: reboot (disabled) ║           │
│  ║  │motor queries   │  ║  ║               ║  ║  4: async state [2]   ║           │
│  ║  └───────┬────────┘  ║  ║               ║  ╚══════════╤═════════════╝           │
│  ║          │           ║  ╚═══════╤═══════╝             │                          │
│  ║  ┌───────▼────────┐  ║          │                     │                          │
│  ║  │Motor (×5) [2]  │  ║          │                     │                          │
│  ║  │pos, vel, torque│  ║          │                     │                          │
│  ║  │temp, status    │  ║          │                     │                          │
│  ║  │MITParam        │  ║          │                     │                          │
│  ║  └───────┬────────┘  ║          │                     │                          │
│  ╚══════════╪════════════╝          │                     │                          │
│             │                      │                     │                          │
│             │                      │                     │                          │
│  ╔══════════╪══════════════════════╪═════════════════════╪══════════════════════╗   │
│  ║          ▼                      ▼                     ▼                      ║   │
│  ║  COMMUNICATION INFRASTRUCTURE                                               ║   │
│  ║                                                                             ║   │
│  ║  ┌──────────────────────────┐  ┌──────────────────┐  ┌──────────────────┐   ║   │
│  ║  │UdpServer (×2 singletons)│  │MqttClient        │  │POSIX Shared Mem  │   ║   │
│  ║  │[2]                      │  │lws 4.5.8 [3]     │  │mercury_shm.h [3] │   ║   │
│  ║  │                         │  │                   │  │                  │   ║   │
│  ║  │Server 0: left leg       │  │Broker:            │  │IPC bridge to     │   ║   │
│  ║  │  local: 8887            │  │127.0.0.1:1883 [3] │  │Mercury Controller│   ║   │
│  ║  │  remote: 8886 [3]       │  │                   │  │(DynaCoRE) [3]    │   ║   │
│  ║  │                         │  │Auth: user/pass    │  │                  │   ║   │
│  ║  │Server 1: right leg      │  │Reconnect: auto [2]│  │                  │   ║   │
│  ║  │  local: 8889            │  └──────────┬────────┘  └────────┬─────────┘   ║   │
│  ║  │  remote: 8888 [3]       │             │                    │             ║   │
│  ║  │                         │             │                    │             ║   │
│  ║  │CAN/CANAPI abstraction   │             │                    │             ║   │
│  ║  │13-byte frame format [3] │             │                    │             ║   │
│  ║  └──────────┬──────────────┘             │                    │             ║   │
│  ╚═════════════╪════════════════════════════╪════════════════════╪══════════════╝   │
│                │                            │                    │                   │
└────────────────┼────────────────────────────┼────────────────────┼───────────────────┘
                 │                            │                    │
                 ▼                            ▼                    ▼
        ┌────────────────┐          ┌──────────────┐    ┌──────────────────┐
        │ Damiao Motors  │          │ MQTT Broker   │    │ Mercury          │
        │ (DM8009 ×10)  │          │ localhost:1883│    │ Controller       │
        │ 5 per leg [2]  │          │ [3]           │    │ DynaCoRE [3]    │
        └────────────────┘          └──────────────┘    └──────────────────┘
                 │
        ┌────────┘
        ▼
        ┌────────────────┐
        │ LPMS-IG1 IMU   │
        │ CAN 0x514-0x51B│
        │ 8 frames [2]   │
        └────────────────┘
```

---

## Interface Definitions

### Interface 1: Robot ↔ Legged Subsystem

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Robot::robotPeriodic()                      Legged → Robot
────────────────────                        ──────────────
  │                                           │
  ├══► Legged::robotPeriodic()                ├───► getMotors()
  │    (synchronous, 20 ms cycle) [2]         │     Motor position, velocity,
  │                                           │     torque, temperature,
  ├══► EventLoop callback                     │     status per motor [2]
  │    → Legged::onMessage()                  │
  │    (async FIFO, non-blocking) [2]         ├───► Motor::getState()
  │    MSG: enable / disable                  │     MITParam readback
  │                                           │
  └══► Mode transitions                      └───► Controller state
       disabledInit/teleopInit [2]                  7D state vector [2]
```

**Interface Type:** Synchronous periodic call + asynchronous message queue
**Coupling:** Tight — Robot directly calls Legged methods and reads Motor state [2]

---

### Interface 2: Legged Subsystem ↔ Controller (LQR)

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Legged::controllerPeriodic()                Controller::calculate()
────────────────────────────                ──────────────────────
  │                                           │
  ╠══► Controller::calculate(state_7d)        ├───► INPUT: 7D state vector
  │    (synchronous, within subsystem         │     [eulerX, eulerY, eulerZ,
  │     thread, 5 ms cycle) [2]               │      quatW, quatX, quatY,
  │                                           │      quatZ] [2]
  ╠══► Controller::dynamics()                 │
  │    (static system model) [2]              ├───► OUTPUT: 2D control input
  │                                           │     (after LQR gain lookup
  ╚══► Controller::globalMeasurementModel()   │      + plant-inversion
       (observation model) [2]                │      feedforward) [2]
                                              │
                                              └───► Gains LUT
                                                    Precomputed offline [2]
```

**Interface Type:** Direct method call within the subsystem's dedicated pthread [2]
**Coupling:** Compile-time — Controller is templated `ControllerBase<7, 2, 4>` and instantiated within Legged [2]

---

### Interface 3: Legged Subsystem ↔ Motor

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Legged → Motor                              Motor → Legged
──────────────                              ──────────────
  │                                           │
  ╠══► Motor::setMitControl()                 ├───► position (double)
  │    MIT command dispatch [2]               ├───► velocity (double)
  │    pos, vel, kp, kd, tau                  ├───► torque (double)
  │                                           ├───► temperature (int)
  ╠══► Motor::sendCommand(0xFC)               ├───► status (error code)
  │    Enable motor [6]                       │     0x00=disabled
  │                                           │     0x01=enabled
  ╠══► Motor::sendCommand(0xFD)               │     0x08=overvoltage
  │    Disable motor [6]                      │     0x09=undervoltage
  │                                           │     0x0A=overcurrent
  ╠══► Motor::sendCommand(0xFE)               │     0x0B=MOS overtemp
  │    Zero-position calibration [6]          │     0x0C=coil overtemp
  │                                           │     0x0D=comm loss
  ╚══► Motor::sendCommand(0xFB)               │     0x0E=overload [2]
       Error clear [6]                        │
                                              └───► MITParam readback
                                                    kp, kd, q_des,
                                                    dq_des, tau_ff [2]
```

**Interface Type:** Direct method calls — Legged owns Motor instances [2]
**Coupling:** Tight — no abstraction layer between subsystem and motor objects [2]

---

### Interface 4: Motor ↔ UdpServer (CAN-over-UDP Transport)

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Motor → UdpServer (SEND)                    UdpServer → Motor (RECEIVE)
────────────────────────                    ──────────────────────────
  │                                           │
  │  13-byte CAN frame [3]:                   │  Feedback frame 8 bytes [6]:
  │  ┌────┬─────────┬──────────┐              │  ┌──────┬────────┬────────┐
  │  │DLC │CAN_ID   │DATA[8]   │              │  │D[0]  │D[1:2]  │D[3:4]  │
  │  │0x08│(2 bytes)│(payload) │              │  │ID|ERR│POS     │VEL     │
  │  └────┴─────────┴──────────┘              │  │<<4   │16-bit  │12-bit  │
  │                                           │  ├──────┼────────┼────────┤
  ├───► MIT mode: CAN_ID = send_id            │  │D[4:5]│D[6]    │D[7]    │
  │     8 bytes packed [6]:                   │  │TAU   │T_MOS   │T_ROTOR │
  │     D[0:1]=pos(16-bit)                    │  │12-bit│(°C)    │(°C)    │
  │     D[2]=vel[11:4]                        │  └──────┴────────┴────────┘
  │     D[3]=vel[3:0]|Kp[11:8]               │
  │     D[4]=Kp[7:0]                          │  Motor::callback()
  │     D[5]=Kd[11:4]                         │  dispatched by CAN ID [2]
  │     D[6]=Kd[3:0]|tau[11:8]               │
  │     D[7]=tau[7:0]                         │  CAN ID routing [2]:
  │                                           │  recv_id = device_id + 0x10
  ├───► PosVel: CAN_ID = send_id + 0x100 [6] │  IDs 0x11-0x15 → server 0
  ├───► Velocity: CAN_ID = send_id + 0x200 [6]│  IDs 0x16-0x1A → server 1
  ├───► PosForce: CAN_ID = send_id + 0x300 [6]│
  └───► Param query: CAN_ID = 0x7FF [6]      │
        D[2]=0x33 (read) or 0x55 (write)      │
```

**Interface Type:** UDP socket send/receive with CAN frame abstraction [2]
**Protocol:** 13-byte CAN-over-UDP frame: CAN_ID + DLC + data[8] [3]
**Encoding:** `double_to_uint(x, x_min, x_max, bits)` for float→int conversion [6]
**Decoding:** `uint_to_double(x, min, max, bits)` for int→float conversion [6]

---

### Interface 5: Robot ↔ IMU Subsystem

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Robot → Imu                                 Imu → Robot
───────────                                 ───────────
  │                                           │
  ╚══► Imu::getStates()                       ├───► 7D state vector [2]:
       (mutex-protected read) [2]             │     eulerX (deg/rad)
       Called from robotPeriodic()            │     eulerY (deg/rad)
       every 20 ms [2]                        │     eulerZ (deg/rad)
                                              │     quatW
  ImuReader (internal, not Robot):            │     quatX
  ═══════════════════════════════             │     quatY
                                              │     quatZ
  LPMS-IG1 → ImuReader                       │
  ─────────────────────                       └───► 16 float32 raw values:
    │                                               accX, accY, accZ (g)
    │  8 CAN frames per cycle [2]:                  gyroX, gyroY, gyroZ (dps)
    │  0x514: accX, accY                            magX, magY, magZ (μT)
    │  0x515: accZ, gyroX                           eulerX, eulerY, eulerZ
    │  0x516: gyroY, gyroZ                          quatW, quatX, quatY,
    │  0x517: magX, magY                            quatZ [2]
    │  0x518: magZ, eulerX
    │  0x519: eulerY, eulerZ
    │  0x51A: quatW, quatX
    │  0x51B: quatY, quatZ [2]
    │
    │  Each frame: 2 × float32
    │  in 8-byte CAN data payload
    │  via 13-byte UDP frame [3]
    │
    └──► Port 8887 (shared with
         UdpServer 0 / left leg) [3]
```

**Interface Type:** Mutex-protected accessor (`getStates()`) — pull model, no push [2]
**Coupling:** Direct wrapping — `Imu` subsystem wraps `ImuReader` with no interface abstraction [2]
**Contention Risk:** Mutex on the 7D state vector creates potential priority inversion [2]

---

### Interface 6: Robot ↔ DriverStation / EventLoop

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

DriverStation → EventLoop → Robot           DriverStation → Robot
──────────────────────────────────          ──────────────────────
  │                                           │
  │  UDP packet received                      ├───► Control words
  │  (operator console) [2]                   │     (mode selection)
  │         │                                 │
  │         ▼                                 ├───► Joystick axes
  │  EventLoop.poll() [2]                     │     (analog input)
  │  Called from robotPeriodic()              │
  │         │                                 └───► Button states
  │         ▼                                       (digital input)
  ╠══► BooleanEvent (rising edge) [2]
  │    Button 1 → leftLeg.enable()
  │
  ╠══► BooleanEvent (rising edge) [2]
  │    Button 2 → leftLeg.disable()
  │
  ╠══► BooleanEvent (rising edge) [2]
  │    Button 3 → reboot (DISABLED) [2]
  │
  ╚══► BooleanEvent (rising edge) [2]
       Button 4 → async state update
```

**Interface Type:** Event-driven with edge detection — polled from `robotPeriodic()` [2]
**Coupling:** Loose — EventLoop dispatches callbacks to subsystems via `onMessage()` [2]

---

### Interface 7: Robot ↔ Telemetry (MQTT)

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Robot → Telemetry                           Telemetry → MQTT Broker
─────────────────                           ──────────────────────
  │                                           │
  ╠══► RobotStatus::collect()                 ├───► Binary RobotStatusWire
  │    leftLeg.getMotors() [2]                │     ~890 bytes [2]
  │    imu.getStates() [2]                    │     Magic: 0x4B564155
  │                                           │     Published every 20 ms
  ╠══► RobotStatus::publish()                 │     (50 Hz) [2]
  │    MqttClient.publish() [2]               │
  │                                           ├───► JSON SenML data logs
  ╠══► DataLog::logMotors() [2]               │     /telemetry/subsystem/
  ╠══► DataLog::logImu() [2]                  │     <name>/motor [2]
  ╚══► DataLog::logDriverStation() [2]        │
                                              └───► Broker: 127.0.0.1:1883
                                                    libwebsockets 4.5.8 [3]
                                                    Auth: user/pass [2]
                                                    Reconnect: auto [2]
```

**Interface Type:** Synchronous inline publish from `robotPeriodic()` [2]
**Coupling:** Tight — telemetry collection is hardcoded in `robotPeriodic()`, directly accessing subsystem internals [2]

---

### Interface 8: Robot ↔ Mercury Controller (Cross-Process)

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Mercury Controller → Kuavo                  Kuavo → Mercury Controller
──────────────────────────                  ──────────────────────────
  │                                           │
  │  POSIX Shared Memory [3]                  │  POSIX Shared Memory [3]
  │  /mercury_robot_ipc                       │  /mercury_robot_ipc
  │                                           │
  ├───► MercuryCommand                        ├───► MercurySensorData
  │     jtorque_cmd[12]                       │     joint_jpos[12]
  │     jpos_cmd[12]                          │     joint_jvel[12]
  │     jvel_cmd[12]                          │     jtorque[12]
  │     kp[12], kd[12]                        │     imu_inc[3]
  │                                           │     imu_ang_vel[3]
  │  DynaCoRE whole-body                      │     imu_acc[3]
  │  dynamics [3]                             │     rfoot_contact
  │  /usr/local/include/DynaCoRE              │     lfoot_contact
  │                                           │
  ╠══► Lifecycle control                      │
  │    (start/stop/heartbeat)                 │
  │                                           │
  ╚══► Emergency stop flag                    │
```

**Interface Type:** POSIX shared memory (`mercury_shm.h`) — cross-process IPC [3]
**Note:** DynaCoRE integration is referenced but not yet active in main control path [2]

---

### Interface 9: Pipeline Tools (Codebase Summarization)

```
CONTROL FLOW (═══)                          DATA FLOW (───)
══════════════════                          ═══════════════

Developer / CI Runner                      pipeline_sync.py
──────────────────                         ────────────────
  │                                           │
  ╠══► python pipeline_sync.py [1]            ├───► Scans C++ files
  │    (full sync)                            │     TARGET_EXTENSIONS:
  │                                           │     .cpp .hpp .h .cc .cxx [1]
  ╠══► python pipeline_sync.py --dry-run [1]  │
  │    (simulation, 0 tokens)                 │     EXCLUDE_DIRS:
  │                                           │     build .git bin obj
  ╚══► python query_index.py "concept" [1]    │     vcpkg node_modules [1]
       (semantic search)                      │
                                              ├───► SHA256 hash per file
                                              │     compute_sha256() [1]
         ┌────────────────┐                   │
         │ Redis (Remote) │◄──────────────────┤  Change detection:
         │ 10.0.0.5:6379  │  file → hash      │  cached_hash != current_hash
         │ [1]            │  mapping           │  [1]
         └────────────────┘                   │
                                              │
         ┌────────────────┐                   ├───► AI summaries via
         │ ChromaDB       │◄──────────────────┤     Devin CLI [1]
         │ (Embedded)     │  vectorized        │     devin -p --prompt-file
         │ ./chroma_db    │  summaries         │
         │ [1]            │                   └───► Semantic search
         └────────────────┘                         query_index.py [1]
```

**Interface Type:** CLI tools — `pipeline_sync.py` (write), `query_index.py` (read) [1]
**Change Detection:** Redis stores `file_path → SHA256 hash`; only changed files are re-summarized [1]
**Search:** ChromaDB embedded vector store with default sentence-transformers [1]

---

## Interface Summary Matrix

| # | From | To | Type | Mechanism | Flow |
|:-:|------|-----|------|-----------|:----:|
| 1 | Robot | Legged | CTRL | Periodic call (20 ms) + async FIFO [2] | ══► |
| 2 | Legged | Controller | CTRL | Direct method call within pthread [2] | ══► |
| 3 | Legged | Motor | CTRL+DATA | Direct ownership, method calls [2] | ◄══► |
| 4 | Motor | UdpServer | DATA | 13-byte CAN frame, UDP socket [3][6] | ◄───► |
| 5 | Robot | IMU | DATA | Mutex-protected `getStates()` [2] | ◄─── |
| 6 | DriverStation | Robot | CTRL | EventLoop callback, edge detect [2] | ══► |
| 7 | Robot | Telemetry | DATA | Inline collect + MQTT publish [2] | ───► |
| 8 | Mercury | Robot | DATA | POSIX shared memory [3] | ◄───► |
| 9 | Developer | Pipeline | CTRL | CLI commands [1] | ══► |

**Legend:**

| Symbol | Meaning |
|:------:|---------|
| ══► | Control / command flow (synchronous or async message) |
| ───► | Data flow (sensor readings, state vectors, telemetry) |
| ◄══► | Bidirectional control |
| ◄───► | Bidirectional data |





# IPC Communication, Synchronization, and Staleness Detection

## Current IPC Architecture

The Kuavo controller coordinates multiple subsystems through different IPC mechanisms, each with its own synchronization model and staleness characteristics.

### IPC Mechanisms by Interface

| Interface | IPC Mechanism | Synchronization | Staleness Risk |
|-----------|--------------|-----------------|:-:|
| Robot ↔ Legged | Periodic call (20 ms) + async FIFO message queue [2] | `poll()`-based notification [2] | Low — synchronous call |
| Legged ↔ Motor | Direct method call within pthread [2] | Same thread — no sync needed | None |
| Motor ↔ UdpServer | UDP socket + callback dispatch [2] | Callback-driven, asynchronous | Medium — network delay |
| Robot ↔ IMU | Mutex-protected `getStates()` [2] | Mutex lock/unlock [2] | Medium — mutex contention |
| Robot ↔ Mercury Controller | POSIX shared memory (`mercury_shm.h`) [3] | Cross-process atomic reads | High — producer may crash or lag |
| Robot ↔ Telemetry | Inline publish from `robotPeriodic()` [2] | Synchronous, no queue | Low — same thread |

---

## Staleness Detection by Subsystem

### Motor Feedback Staleness

Motor feedback arrives asynchronously via the UdpServer callback chain — `Damiao HW → UDP socket → UdpServer → callback → Motor → Legged` [2]. Motor responsiveness is tracked with a **500 ms timeout**; unresponsive motors are flagged [2][3].

The motor state machine detects communication loss via error code **0x0D** in the feedback frame D[0] upper nibble [2]. If a motor stops responding for 500 ms, it is flagged as unresponsive [3]. The motor error codes that indicate operational problems include overvoltage (0x08), undervoltage (0x09), overcurrent (0x0A), MOS overtemp (0x0B), coil overtemp (0x0C), comm loss (0x0D), and overload (0x0E) [2].

**Staleness check approach:** Each motor tracks the timestamp of its last received feedback frame. In `controllerPeriodic()` (5 ms cycle) [2], the elapsed time since last feedback is compared against the 500 ms timeout [3]. If exceeded, the motor status transitions to comm loss (0x0D).

### IMU Data Staleness

The IMU data arrives as 8 sequential CAN frames (IDs 0x514 through 0x51B), each carrying 2 float32 values, totaling 16 floats per measurement cycle [2]. The `ImuReader` runs as a dedicated pthread with a blocking UDP socket on port 8887 [2]. The 7D state vector `[eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ]` is exposed through a mutex-protected `getStates()` accessor [2].

**Staleness check approach:** All IMU state access is mutex-protected [2], but there is no explicit staleness timestamp exposed to the consumer. The robot main loop calls `imu.getStates()` every 20 ms [2], but has no mechanism to determine whether the returned state vector is from the current cycle or from a stale measurement. The IMU reader could stop receiving frames (hardware fault, cable disconnect) and the main loop would continue reading the last-known state indefinitely.

**Missing safeguard:** Unlike motor control which has a 500 ms responsiveness timeout [3], the IMU subsystem has no equivalent timeout mechanism documented in the current architecture [2].

### Mercury Controller Shared Memory Staleness

The Mercury Controller communicates via POSIX shared memory (`mercury_shm.h`) [3]. This is a cross-process IPC bridge where the Mercury Controller (DynaCoRE) writes commands and the Kuavo process reads them [3].

**Staleness check approach:** The shared memory layout should include a heartbeat timestamp written by the producer (Mercury Controller) on every control cycle. The consumer (Kuavo `controllerPeriodic()`) checks if the heartbeat is stale:

- If the producer process crashes, the heartbeat timestamp stops advancing
- If the producer falls behind its expected update rate, the timestamp age increases
- The consumer detects both scenarios by comparing `now - last_heartbeat` against a threshold

**Current status:** DynaCoRE integration is referenced but not yet active in the main control path [2]. The shared memory architecture mentioned in architecture docs is not yet implemented — currently using mutex-protected state [2].

### Telemetry Staleness

Telemetry is published every main loop iteration at 50 Hz (20 ms) [2]. The collection is synchronous — `Robot::robotPeriodic()` directly accesses subsystem internals (`leftLeg.getMotors()`, `imu.getStates()`) and publishes via `RobotStatus` and `DataLog` each cycle [2]. Since collection and publication happen in the same thread during the same cycle, telemetry data is always as fresh as the subsystem state at the time of collection. However, the telemetry consumer (MQTT broker subscriber) has no way to verify freshness — the binary `RobotStatusWire` packet (~890 bytes, magic 0x4B564155) includes a microsecond timestamp and frame counter [2] that a subscriber could use for staleness detection.

---

## Data Aggregation Flow

### Inbound Data Aggregation (Sensor → Robot)

```
MOTOR FEEDBACK PATH (per motor, asynchronous):

  Damiao Motor HW
       │
       │ CAN feedback frame (8 bytes) [6]:
       │   D[0] = ID | ERR<<4
       │   D[1:2] = position (16-bit mapped to [-PMAX, PMAX])
       │   D[3:4] = velocity (12-bit mapped to [-VMAX, VMAX])
       │   D[4:5] = torque (12-bit mapped to [-TMAX, TMAX])
       │   D[6] = T_MOS (°C)
       │   D[7] = T_Rotor (°C)
       │
       ▼
  UdpServer (×2 singletons) [2]
       │ Left: ports 8887/8886
       │ Right: ports 8889/8888 [3]
       │ Routing: device_id < 6 → server 0; else → server 1 [2]
       │
       │ CAN ID check:
       │   recv_id = device_id + 0x10 [2][3]
       │   IDs 0x11-0x15 → left leg motors
       │   IDs 0x16-0x1A → right leg motors
       │
       ▼
  Motor::callback() [2]
       │
       │ Decode via uint_to_double() [6]:
       │   position = uint_to_double(q_uint, -pMax, pMax, 16)
       │   velocity = uint_to_double(dq_uint, -vMax, vMax, 12)
       │   torque = uint_to_double(tau_uint, -tMax, tMax, 12)
       │
       │ Safety state machine check [2]:
       │   ERR field in D[0] upper nibble
       │   0x00=disabled, 0x01=enabled
       │   0x08-0x0E = error states
       │
       │ Update responsiveness timer (500 ms timeout) [3]
       │
       ▼
  Motor internal state (mutex-protected) [2]
       │
       │ Available to Legged subsystem
       │ via direct ownership [2]
       │
       ▼
  Legged::controllerPeriodic() (5 ms) [2]
       │ Reads motor state for LQR controller input
       │ 7D state vector → Controller::calculate() [2]
       │
       └──► Aggregated into control computation


IMU DATA PATH (sequential frames, dedicated thread):

  LPMS-IG1 Hardware
       │
       │ 8 CAN frames per cycle [2]:
       │   0x514: accX, accY (g)
       │   0x515: accZ, gyroX (g, dps)
       │   0x516: gyroY, gyroZ (dps)
       │   0x517: magX, magY (μT)
       │   0x518: magZ, eulerX (μT, deg)
       │   0x519: eulerY, eulerZ (deg)
       │   0x51A: quatW, quatX (unitless)
       │   0x51B: quatY, quatZ (unitless)
       │
       │ Each frame: 2 × float32 in 8-byte CAN data
       │ Via 13-byte UDP frame [3]
       │
       ▼
  ImuReader (dedicated pthread, port 8887) [2]
       │
       │ epoll-based non-blocking receive
       │ CAN ID filter: range [0x514, 0x51B]
       │ (same port as UdpServer 0 / left leg) [3]
       │
       │ Frame aggregation:
       │   Wait for all 8 frames of one cycle
       │   Parse 16 float32 values total
       │
       ▼
  Imu::update() [2]
       │
       │ Aggregate into 7D state vector:
       │   [eulerX, eulerY, eulerZ,
       │    quatW, quatX, quatY, quatZ]
       │
       │ Raw accel/gyro/mag parsed but not
       │ individually exposed in state vector [2]
       │
       ▼
  Imu::m_state (mutex-protected Vector<7>) [2]
       │
       │ getStates() provides thread-safe read [2]
       │
       ▼
  Robot::robotPeriodic() (20 ms) [2]
       │ imu.getStates() called each cycle
       │ State used for telemetry + passed to controller
```

### Outbound Data Dispatch (Robot → Actuators)

```
MOTOR COMMAND DISPATCH (per motor, periodic):

  Controller::calculate() [2]
       │
       │ LQR: 7D state → 2D control input
       │ Gains from precomputed lookup tables [2]
       │
       ▼
  Legged::controllerPeriodic() (5 ms) [2]
       │
       │ For each of 5 motors per leg:
       │
       ▼
  Motor::setMitControl(pos, vel, kp, kd, tau) [2]
       │
       │ Encode via double_to_uint() [6]:
       │   pos_uint = double_to_uint(pos, -pMax, pMax, 16)
       │   vel_uint = double_to_uint(vel, -vMax, vMax, 12)
       │   kp_uint = double_to_uint(kp, 0, 500, 12)
       │   kd_uint = double_to_uint(kd, 0, 5, 12)
       │   tau_uint = double_to_uint(tau, -tMax, tMax, 12)
       │
       │ Pack into 8-byte CAN data [6]:
       │   D[0] = pos_uint >> 8
       │   D[1] = pos_uint & 0xFF
       │   D[2] = vel_uint >> 4
       │   D[3] = (vel_uint & 0xF) << 4 | (kp_uint >> 8)
       │   D[4] = kp_uint & 0xFF
       │   D[5] = kd_uint >> 4
       │   D[6] = (kd_uint & 0xF) << 4 | (tau_uint >> 8)
       │   D[7] = tau_uint & 0xFF
       │
       ▼
  CAN / CANAPI [2]
       │
       │ CAN ID selection by mode [6]:
       │   MIT: send_can_id (no offset)
       │   PosVel: send_can_id + 0x100
       │   Velocity: send_can_id + 0x200
       │   PosForce: send_can_id + 0x300
       │
       │ Enable/Disable/Zero/Clear [6]:
       │   0xFC / 0xFD / 0xFE / 0xFB
       │   Data: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd}
       │
       ▼
  UdpServer [2]
       │
       │ 13-byte CAN-over-UDP frame [3]:
       │   [CAN_ID_lo, CAN_ID_hi, DLC, data[8]]
       │
       │ Routing [2]:
       │   Motor IDs 1-5 → server 0 (ports 8887/8886)
       │   Motor IDs 6-10 → server 1 (ports 8889/8888)
       │
       ▼
  UDP sendto() → Damiao Motor HW


PARAMETER QUERY DISPATCH (on-demand):

  Any caller
       │
       ▼
  Motor::queryParam(RID) [6]
       │
       │ CAN ID = 0x7FF (fixed broadcast) [6]
       │ Data [6]:
       │   D[0:1] = send_can_id (little-endian)
       │   D[2] = 0x33 (read) or 0x55 (write)
       │   D[3] = RID (parameter index)
       │   D[4:7] = 0x00 (read) or value (write)
       │
       ▼
  UdpServer → UDP → Motor HW
       │
       │ Response identification [6]:
       │   data[2] == 0x33 or data[2] == 0x55
       │   Integer RIDs (7-10, 13-16, 35-36):
       │     decoded via uint8s_to_uint32() [6]
       │   Float RIDs (all others):
       │     decoded via uint8s_to_float() [6]
```

### Telemetry Aggregation and Publish

```
TELEMETRY COLLECTION (synchronous, 20 ms / 50 Hz):

  Robot::robotPeriodic() [2]
       │
       ├───► leftLeg.getMotors() [2]
       │     Read motor state for each of 5 motors:
       │     position, velocity, torque, temperature, status
       │
       ├───► imu.getStates() [2]
       │     Mutex-protected read of 7D state vector
       │     [eulerX, Y, Z, quatW, X, Y, Z]
       │
       ├───► DriverStation state [2]
       │     Control words, joystick axes, button states
       │
       ▼
  RobotStatus::collect() [2]
       │
       │ Aggregate all subsystem state into
       │ binary RobotStatusWire (~890 bytes) [2]:
       │   Magic: 0x4B564155
       │   Version byte
       │   Microsecond timestamp
       │   Frame counter
       │   LegStatusWire × 2
       │   DriverCommandWire
       │   ImuWire
       │
       ▼
  RobotStatus::publish() [2]
       │
       │ MqttClient.publish() [2]
       │ Broker: 127.0.0.1:1883 [3]
       │ libwebsockets 4.5.8 [3]
       │ Auth: username/password [2]
       │ Reconnect: automatic [2]
       │
       ▼
  MQTT Broker

  DataLog (parallel, same cycle) [2]:
       ├───► logMotors() → JSON SenML
       ├───► logImu() → JSON SenML
       └───► logDriverStation() → JSON SenML
             Topic: /telemetry/subsystem/<name>/motor [2]
```

---

## Staleness Detection Gaps

Based on the architecture, these staleness detection gaps exist:

| Subsystem | Has Timeout? | Gap |
|-----------|:---:|-----|
| Motor feedback | Yes — 500 ms [3] | Adequate for current 200 Hz control rate |
| IMU data | No explicit timeout [2] | `ImuReader` could stop receiving frames without detection; `getStates()` returns last-known state indefinitely |
| Mercury Controller SHM | Not yet implemented [2] | Cross-process producer could crash; consumer reads stale commands |
| Telemetry | Timestamp in packet [2] | Subscriber has timestamp + frame counter but no enforced staleness policy |
| DriverStation | No explicit timeout [2] | If operator console disconnects, last mode and button states persist |

The motor control domain is the most robust with its 500 ms responsiveness timeout and error state machine [2][3]. The IMU and Mercury Controller interfaces lack equivalent staleness detection, creating a risk where the controller continues operating on stale sensor data without awareness [2].


