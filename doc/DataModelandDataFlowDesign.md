
# Detailed Data Model and Data Flow Design

## Module Data Models

### DriverStation Module

The DriverStation provides the human-machine interface for robot control, handling Xbox controller input, joystick axes, button events, and operating mode transitions (disabled, autonomous, teleop) [2].

**Input Data Model:**

| Field | Type | Source | Description |
|-------|------|--------|-------------|
| `control_word` | uint16 | UDP packet | Operating mode selector (disabled/autonomous/teleop) [2] |
| `joystick_axes[]` | float[6] | UDP packet | Analog stick positions (X/Y per stick + triggers) |
| `button_states` | uint16 | UDP packet | Bitmask of all button states |

**Output Data Model:**

| Field | Type | Consumer | Description |
|-------|------|----------|-------------|
| `mode` | enum | Robot main loop | Current operating mode (Disabled/Autonomous/Teleop) [2] |
| `button_event` | struct | EventLoop | Rising/falling edge via BooleanEvent [2] |
| `joystick_cmd` | struct | Legged subsystem | Analog input for teleoperation |

**Event Dispatch Model:**

| Button | Event Type | Target | Command |
|:------:|:----------:|--------|---------|
| 1 | Rising edge | Left leg `onMessage()` | MSG_ENABLE [2] |
| 2 | Rising edge | Left leg `onMessage()` | MSG_DISABLE [2] |
| 3 | Rising edge | Robot | Reboot (currently disabled) [2] |
| 4 | Rising edge | Robot | Async state update [2] |

Events flow through `EventLoop` callbacks to subsystem enable/disable methods using `BooleanEvent` rising/falling edge detection to prevent repeated triggers from held buttons [2].

---

### Robot Module

The Robot class inherits from `TimedRobot` and orchestrates the entire system lifecycle at 20 ms / 50 Hz [2][3]. The `loopFunc()` chain executes: `Notifier fires → IterativeRobotBase::loopFunc() → refreshData() → mode switch → robotPeriodic()` [2].

**Internal State Model:**

| Field | Type | Update Rate | Description |
|-------|------|:-----------:|-------------|
| `m_leftLeg` | Legged | Owned | Left leg subsystem instance [2] |
| `m_rightLeg` | Legged | Owned | Right leg subsystem (disabled) [2] |
| `m_imu` | Imu | Owned | IMU subsystem instance [2] |
| `m_loop` | EventLoop | Polled | Button event dispatch loop [2] |
| `m_robotStatus` | RobotStatus | 50 Hz | Telemetry collector [2] |
| `m_dataLog` | DataLog | 50 Hz | JSON SenML logger [2] |
| `m_config` | Config* | Once | Singleton YAML configuration [2] |
| `m_shm` | SharedMemoryLayout* | Cross-process | Mercury Controller IPC bridge [3] |
| `m_mode` | OperatingMode | Event-driven | Current mode (Disabled/Autonomous/Teleop) [2] |

**Aggregated Data Model (collected each 20 ms cycle):**

| Field | Source | Access Method | Sync |
|-------|--------|:-------------:|:----:|
| Motor states (×10) | `leftLeg.getMotors()` [2] | Direct read | Mutex |
| IMU 7D state | `imu.getStates()` [2] | Mutex-protected | Mutex |
| DS control words | DriverStation UDP | Event-driven | Async |
| Mercury commands | `m_shm->cmd_buffers[]` [3] | Atomic read | Lock-free |

---

### Legged Subsystem Module

Implements `ControlledSubsystemBase<7, 2, 5>` with 7 states, 2 inputs, and 5 outputs (motors) per leg [2]. `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance for non-blocking message processing [2].

**State Vector Model (7D):**

| Index | Field | Unit | Source | Description |
|:-----:|-------|------|--------|-------------|
| 0 | eulerX | rad | IMU | Roll angle [2] |
| 1 | eulerY | rad | IMU | Pitch angle [2] |
| 2 | eulerZ | rad | IMU | Yaw angle [2] |
| 3 | quatW | — | IMU | Quaternion W [2] |
| 4 | quatX | — | IMU | Quaternion X [2] |
| 5 | quatY | — | IMU | Quaternion Y [2] |
| 6 | quatZ | — | IMU | Quaternion Z [2] |

**Control Input Model (2D):**

| Index | Field | Unit | Source |
|:-----:|-------|------|--------|
| 0 | input_0 | Nm | Controller LQR output [2] |
| 1 | input_1 | Nm | Controller LQR output [2] |

**Motor Group Model (×5 per leg):**

| Field | Type | Unit | Update | Description |
|-------|------|------|:------:|-------------|
| `position` | double | rad | Feedback | Decoded from D[1:2] (16-bit) [6] |
| `velocity` | double | rad/s | Feedback | Decoded from D[3:4] (12-bit) [6] |
| `torque` | double | Nm | Feedback | Decoded from D[4:5] (12-bit) [6] |
| `t_mos` | int | °C | Feedback | From D[6] [6] |
| `t_rotor` | int | °C | Feedback | From D[7] [6] |
| `status` | uint8 | enum | Feedback | From D[0] upper nibble [6] |
| `device_id` | uint8 | — | Config | CAN device ID (1-5 left, 6-10 right) [2] |
| `send_can_id` | uint32 | — | Config | Equals device_id [2] |
| `recv_can_id` | uint32 | — | Config | Equals device_id + 0x10 [2][3] |

**Motor Status Codes:**

| Code | Status | Action |
|:----:|--------|--------|
| 0x00 | Disabled | No control accepted [2] |
| 0x01 | Enabled | Normal operation [2] |
| 0x08 | Overvoltage | Control suspended [2] |
| 0x09 | Undervoltage | Control suspended [2] |
| 0x0A | Overcurrent | Control suspended [2] |
| 0x0B | MOS overtemp | Control suspended [2] |
| 0x0C | Coil overtemp | Control suspended [2] |
| 0x0D | Comm loss | Control suspended (500 ms timeout) [2][3] |
| 0x0E | Overload | Control suspended [2] |

**MIT Command Model (per motor per cycle):**

| Field | Type | Bits | Range | Encoding |
|-------|------|:----:|-------|----------|
| `q_des` | double | 16 | [-PMAX, PMAX] | `double_to_uint(q, -pMax, pMax, 16)` [6] |
| `dq_des` | double | 12 | [-VMAX, VMAX] | `double_to_uint(dq, -vMax, vMax, 12)` [6] |
| `kp` | double | 12 | [0, 500] | `double_to_uint(kp, 0, 500, 12)` [6] |
| `kd` | double | 12 | [0, 5] | `double_to_uint(kd, 0, 5, 12)` [6] |
| `tau_ff` | double | 12 | [-TMAX, TMAX] | `double_to_uint(tau, -tMax, tMax, 12)` [6] |

**MIT Frame Packing (8 bytes):**

| Byte | Content | Source |
|:----:|---------|--------|
| D[0] | `pos_uint >> 8` | Position high byte [6] |
| D[1] | `pos_uint & 0xFF` | Position low byte [6] |
| D[2] | `vel_uint >> 4` | Velocity high 8 bits [6] |
| D[3] | `(vel_uint & 0xF) << 4 \| (kp_uint >> 8)` | Velocity low 4 + Kp high 4 [6] |
| D[4] | `kp_uint & 0xFF` | Kp low byte [6] |
| D[5] | `kd_uint >> 4` | Kd high 8 bits [6] |
| D[6] | `(kd_uint & 0xF) << 4 \| (tau_uint >> 8)` | Kd low 4 + Tau high 4 [6] |
| D[7] | `tau_uint & 0xFF` | Tau low byte [6] |

**Async Message Queue Model:**

| Message | Source | Handler | Motor Command |
|---------|--------|---------|:-------------:|
| MSG_ENABLE | EventLoop Button 1 [2] | `onMessage()` | 0xFC [6] |
| MSG_DISABLE | EventLoop Button 2 [2] | `onMessage()` | 0xFD [6] |
| MSG_ZERO | Operator action | `onMessage()` | 0xFE [6] |
| MSG_CLEAR | Error recovery | `onMessage()` | 0xFB [6] |

Command data format for enable/disable/zero/clear [6]:
```
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd}
```

---

### IMU Module

Reads orientation and motion data from an LPMS-IG1 IMU sensor via sequential CAN-over-UDP frames, providing a 7D state vector for balance and state estimation [2].

**CAN Frame Model (8 frames per cycle, 16 float32 values):**

| CAN ID | Offset | Slot 0 | Slot 1 | Unit |
|:------:|:------:|--------|--------|------|
| 0x514 | 0 | accX | accY | g [2] |
| 0x515 | 1 | accZ | gyroX | g, dps [2] |
| 0x516 | 2 | gyroY | gyroZ | dps [2] |
| 0x517 | 3 | magX | magY | μT [2] |
| 0x518 | 4 | magZ | eulerX | μT, deg [2] |
| 0x519 | 5 | eulerY | eulerZ | deg [2] |
| 0x51A | 6 | quatW | quatX | unitless [2] |
| 0x51B | 7 | quatY | quatZ | unitless [2] |

**Raw Data Model (16 floats per cycle):**

| Index | Field | Unit | CAN Frame | Slot |
|:-----:|-------|------|:---------:|:----:|
| 0 | accX | g | 0x514 | 0 |
| 1 | accY | g | 0x514 | 1 |
| 2 | accZ | g | 0x515 | 0 |
| 3 | gyroX | dps | 0x515 | 1 |
| 4 | gyroY | dps | 0x516 | 0 |
| 5 | gyroZ | dps | 0x516 | 1 |
| 6 | magX | μT | 0x517 | 0 |
| 7 | magY | μT | 0x517 | 1 |
| 8 | magZ | μT | 0x518 | 0 |
| 9 | eulerX | deg | 0x518 | 1 |
| 10 | eulerY | deg | 0x519 | 0 |
| 11 | eulerZ | deg | 0x519 | 1 |
| 12 | quatW | — | 0x51A | 0 |
| 13 | quatX | — | 0x51A | 1 |
| 14 | quatY | — | 0x51B | 0 |
| 15 | quatZ | — | 0x51B | 1 |

**Exposed State Model (7D vector):**

| Index | Field | Unit | Source Index |
|:-----:|-------|------|:----:|
| 0 | eulerX | deg/rad | 9 |
| 1 | eulerY | deg/rad | 10 |
| 2 | eulerZ | deg/rad | 11 |
| 3 | quatW | — | 12 |
| 4 | quatX | — | 13 |
| 5 | quatY | — | 14 |
| 6 | quatZ | — | 15 |

Raw accelerometer, gyroscope, and magnetometer values are parsed but not individually exposed in the 7D state vector [2]. All IMU state access is mutex-protected to ensure thread safety between the receiver thread and the main control loop [2].

---

### Telemetry / Logger Module

Publishes real-time robot status and structured data logs over MQTT for monitoring, debugging, and offline analysis [2].

**Binary RobotStatusWire Model (~890 bytes):**

| Field | Type | Size | Description |
|-------|------|:----:|-------------|
| `magic` | uint32 | 4 | 0x4B564155 ("KVAU") [2] |
| `version` | uint8 | 1 | Protocol version [2] |
| `timestamp_us` | uint64 | 8 | Microsecond timestamp [2] |
| `frame_counter` | uint32 | 4 | Monotonic frame count [2] |
| `left_leg` | LegStatusWire | ~400 | Left leg motor states (×5) |
| `right_leg` | LegStatusWire | ~400 | Right leg motor states (×5) |
| `driver_cmd` | DriverCommandWire | ~40 | DS control words + axes |
| `imu` | ImuWire | ~56 | 7D state vector |

**JSON SenML Data Model:**

| Topic | Content | Rate |
|-------|---------|:----:|
| `/telemetry/subsystem/<name>/motor` | Motor position, velocity, torque, temp per joint | 50 Hz [2] |
| `/telemetry/subsystem/<name>/imu` | 7D state vector | 50 Hz [2] |
| `/telemetry/subsystem/<name>/ds` | Control words, joystick axes, button states | 50 Hz [2] |

MQTT connection: broker at 127.0.0.1:1883 [3], libwebsockets 4.5.8 [3], username/password authentication, non-blocking with automatic reconnection [2].

---

### UDP / Motor Communication Module

Provides the transport layer for all CAN-over-UDP motor communication and IMU data reception [2].

**CAN-over-UDP Frame Model (13 bytes):**

| Byte | Field | Size | Description |
|:----:|-------|:----:|-------------|
| 0-1 | CAN_ID | 2 | CAN identifier [3] |
| 2 | DLC | 1 | Data length code (always 8) [3] |
| 3-10 | data | 8 | CAN payload [3] |

**UdpServer Routing Model:**

| Server | Local Port | Remote Port | Motor IDs | CAN Recv IDs | Leg |
|:------:|:----------:|:-----------:|:---------:|:------------:|:---:|
| 0 | 8887 | 8886 | 1-5 | 0x11-0x15 | Left [2][3] |
| 1 | 8889 | 8888 | 6-10 | 0x16-0x1A | Right [2][3] |

Port assignment formula: `localPort = base_local_port + server_id * 2`, `remotePort = base_remote_port + server_id * 2` [2].

Routing threshold: `device_id < 6` → server 0 (left); else → server 1 (right) [2].

**CAN ID Assignment Model:**

| Mode | CAN ID | Offset | Source |
|------|:------:|:------:|--------|
| MIT control | `send_can_id` | +0 | [6] |
| PosVel | `send_can_id + 0x100` | +256 | [6] |
| Velocity | `send_can_id + 0x200` | +512 | [6] |
| PosForce | `send_can_id + 0x300` | +768 | [6] |
| Enable/Disable/Zero/Clear | `send_can_id` | +0 | [6] |
| Parameter query | `0x7FF` | Fixed | [6] |
| Parameter write | `0x7FF` | Fixed | [6] |
| Refresh | `0x7FF` | Fixed | [6] |
| IMU frames | `0x514-0x51B` | Fixed | [2] |

**Motor Feedback Frame Model (8 bytes):**

| Byte | Field | Bits | Range | Decoding |
|:----:|-------|:----:|-------|----------|
| D[0] | ID \| ERR<<4 | 8 | — | `motor_id = data[0] & 0x0F`, `err = data[0] >> 4` [6] |
| D[1:2] | Position | 16 | [-PMAX, PMAX] | `uint_to_double(q, -pMax, pMax, 16)` [6] |
| D[3:4] | Velocity | 12 | [-VMAX, VMAX] | `uint_to_double(dq, -vMax, vMax, 12)` [6] |
| D[4:5] | Torque | 12 | [-TMAX, TMAX] | `uint_to_double(tau, -tMax, tMax, 12)` [6] |
| D[6] | T_MOS | 8 | 0-255 °C | Direct cast [6] |
| D[7] | T_Rotor | 8 | 0-255 °C | Direct cast [6] |

**Parameter Query/Response Model:**

| Direction | CAN ID | D[0:1] | D[2] | D[3] | D[4:7] |
|-----------|:------:|:------:|:----:|:----:|:------:|
| Query (send) | 0x7FF | send_can_id (LE) | 0x33 | RID | 0x00 [6] |
| Write (send) | 0x7FF | send_can_id (LE) | 0x55 | RID | value [6] |
| Response | 0x7FF | motor_id (LE) | 0x33/0x55 | RID | value [6] |

Integer-type RIDs (decoded as uint32): ranges 7-10, 13-16, 35-36 [6].
Float-type RIDs (decoded as float): all others [6].

---

## Data Flow Design

### Inbound Aggregation (Sensor → Robot)

```
MOTOR FEEDBACK PATH (per motor, asynchronous, callback-driven):

  Damiao Motor HW
       │
       │ CAN feedback frame (8 bytes)
       │ D[0]: ID|ERR<<4  D[1:2]: POS(16-bit)
       │ D[3:4]: VEL(12-bit)  D[4:5]: TAU(12-bit)
       │ D[6]: T_MOS  D[7]: T_ROTOR [6]
       │
       ▼
  UdpServer singleton [2]
       │ Routing: recv_can_id = device_id + 0x10 [2][3]
       │ IDs 0x11-0x15 → server 0 (left)
       │ IDs 0x16-0x1A → server 1 (right) [2]
       │
       ▼
  Motor::callback() [2]
       │ Decode: uint_to_double() [6]
       │   position = uint_to_double(q_uint, -pMax, pMax, 16)
       │   velocity = uint_to_double(dq_uint, -vMax, vMax, 12)
       │   torque = uint_to_double(tau_uint, -tMax, tMax, 12)
       │ Safety: check ERR field in D[0] [2]
       │ Timeout: update responsiveness timer (500 ms) [3]
       │
       ▼
  Motor internal state (within Legged subsystem thread) [2]
       │
       ▼
  Legged::controllerPeriodic() (5 ms / 200 Hz) [2]
       │ Reads motor state for controller input
       │ 7D state → Controller::calculate() [2]


IMU DATA PATH (sequential frames, dedicated thread):

  LPMS-IG1 Hardware
       │
       │ 8 CAN frames per cycle (0x514-0x51B) [2]
       │ Each frame: 2 × float32 in 8-byte CAN data
       │ Port 8887 (shared with UdpServer 0) [3]
       │
       ▼
  ImuReader (dedicated pthread, epoll) [2]
       │ Filter: CAN ID in [0x514, 0x51B]
       │ Parse: 16 float32 values total [2]
       │ Aggregate: wait for all 8 frames
       │
       ▼
  Imu::update() → m_state: Vector<7> [2]
       │ 7D state: [eulerX,Y,Z, quatW,X,Y,Z]
       │ Mutex-protected [2]
       │
       ▼
  Robot::robotPeriodic() (20 ms / 50 Hz) [2]
       │ imu.getStates() — mutex lock/unlock
       │ State used for telemetry + controller
```

### Outbound Dispatch (Robot → Actuators)

```
MOTOR COMMAND DISPATCH (per motor, periodic 5 ms):

  Controller::calculate(state_7d) [2]
       │ LQR: 7D state → 2D control input
       │ Gains from precomputed lookup tables [2]
       │
       ▼
  Legged::controllerPeriodic() [2]
       │ For each of 5 motors per leg
       │
       ▼
  Motor::setMitControl(pos, vel, kp, kd, tau) [2]
       │ Encode: double_to_uint() [6]
       │   pos_uint = double_to_uint(pos, -pMax, pMax, 16)
       │   vel_uint = double_to_uint(vel, -vMax, vMax, 12)
       │   kp_uint = double_to_uint(kp, 0, 500, 12)
       │   kd_uint = double_to_uint(kd, 0, 5, 12)
       │   tau_uint = double_to_uint(tau, -tMax, tMax, 12)
       │ Pack into 8 bytes [6]
       │
       ▼
  CAN / CANAPI [2]
       │ CAN_ID = send_can_id (MIT mode, no offset) [6]
       │ Frame: 13 bytes [3]
       │
       ▼
  UdpServer → UDP sendto() → Damiao Motor HW


ENABLE / DISABLE DISPATCH (one-time, event-driven):

  EventLoop callback (Button 1 or 2) [2]
       │ BooleanEvent rising edge [2]
       │
       ▼
  Legged::onMessage(MSG_ENABLE or MSG_DISABLE) [2]
       │ Async FIFO message queue [2]
       │ poll()-based notification [2]
       │
       ▼
  Motor::sendCommand(0xFC or 0xFD) [6]
       │ Data: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd} [6]
       │ CAN_ID = send_can_id [6]
       │
       ▼
  UdpServer → UDP → Motor HW


PARAMETER QUERY DISPATCH (on-demand):

  Caller (any)
       │
       ▼
  Motor::queryParam(RID) [6]
       │ CAN_ID = 0x7FF (fixed broadcast) [6]
       │ D[0:1] = send_can_id (little-endian)
       │ D[2] = 0x33 (read) [6]
       │ D[3] = RID
       │ D[4:7] = 0x00
       │
       ▼
  UdpServer → UDP → Motor HW
       │
       │ Response: D[2] == 0x33 [6]
       │ If is_in_ranges(RID): uint32 decode [6]
       │ Else: float decode [6]
```

### Telemetry Aggregation and Publish

```
TELEMETRY COLLECTION (synchronous, 20 ms / 50 Hz):

  Robot::robotPeriodic() [2]
       │
       ├──► leftLeg.getMotors() [2]
       │    Motor state for each of 5 motors:
       │    position, velocity, torque, temperature, status
       │
       ├──► imu.getStates() [2]
       │    Mutex-protected 7D state vector
       │
       ├──► DriverStation state [2]
       │    Control words, joystick axes, button states
       │
       ▼
  RobotStatus::collect() [2]
       │ Aggregate into binary RobotStatusWire (~890 bytes) [2]
       │ Magic: 0x4B564155, version, microsecond timestamp,
       │ frame counter, LegStatusWire ×2,
       │ DriverCommandWire, ImuWire [2]
       │
       ▼
  RobotStatus::publish() → MqttClient [2]
       │ Broker: 127.0.0.1:1883 [3]
       │ libwebsockets 4.5.8 [3]
       │
  DataLog (parallel, same cycle) [2]:
       ├──► logMotors() → JSON SenML
       ├──► logImu() → JSON SenML
       └──► logDriverStation() → JSON SenML
            Topic: /telemetry/subsystem/<name>/motor [2]
```

### Cross-Process IPC (Mercury Controller)

```
MERCURY CONTROLLER BRIDGE (POSIX shared memory):

  Mercury Controller (separate process, DynaCoRE) [3]
       │
       │ shm_open() + ftruncate() + mmap()
       │ mercury_shm.h [3]
       │
       ├──► Writes: MercuryCommand
       │    jtorque_cmd[12], jpos_cmd[12], jvel_cmd[12]
       │    kp[12], kd[12]
       │
       │    Kuavo reads via atomic cmd_write_idx
       │
       ├──► Reads: MercurySensorData
       │    joint_jpos[12], joint_jvel[12], jtorque[12]
       │    imu_inc[3], imu_ang_vel[3], imu_acc[3]
       │    rfoot_contact, lfoot_contact
       │
       │    Kuavo writes via atomic state_write_idx
       │
       └──► DynaCoRE installed at /usr/local/include/DynaCoRE [3]
            Integration referenced but not yet active [2]
```