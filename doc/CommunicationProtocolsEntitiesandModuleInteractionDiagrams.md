
# Communication Protocols, Protocol Entities, and Module Interaction Diagrams

---

## Protocol Summary by Module

| Module | Protocol | Transport | Direction | Entity Format |
|--------|----------|-----------|:---------:|---------------|
| Motor Control | Damiao MIT CAN | CAN-over-UDP (13 bytes) | Bidirectional | 8-byte packed integers [6] |
| Motor Enable/Disable | Damiao Command | CAN-over-UDP (13 bytes) | Outbound | `{0xFF×7, cmd}` [6] |
| Motor Parameter Query | Damiao Param | CAN-over-UDP (13 bytes) | Bidirectional | RID + float/uint32 [6] |
| IMU | LPMS-IG1 Sequential CAN | CAN-over-UDP (13 bytes) | Inbound | 2 × float32 per frame [2] |
| Telemetry | MQTT over WebSocket | TCP (lws 4.5.8) | Outbound | Binary 890B + JSON SenML [2][3] |
| Mercury Controller | POSIX Shared Memory | IPC (mmap) | Bidirectional | Structured C structs [3] |
| DriverStation | FRC DS Protocol | UDP | Inbound | Control words + axes [2] |
| Pipeline Sync | Redis Protocol | TCP | Bidirectional | `file_path → SHA256` [1] |
| Pipeline Sync | HTTP/Devin CLI | Subprocess | Outbound | Prompt text → summary [1] |
| Pipeline Query | ChromaDB Embedding | Embedded | Local | Vector similarity search [1] |
| ETL Data Load | MongoDB REST API | HTTP POST | Inbound | JSON array (flat) [5] |

---

## Protocol Entity Definitions

### Entity 1: CAN-over-UDP Frame (13 bytes)

Used by Motor Control, IMU, and all CAN-based communication. This is the universal transport frame for the Kuavo hardware layer [3].

```
Byte:  [0]     [1]      [2]    [3]  [4]  [5]  [6]  [7]  [8]  [9]  [10]
Field: CAN_ID  CAN_ID   DLC    --------- CAN DATA (8 bytes) ----------
       lo      hi       (0x08)  D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]
```

Port assignments [2][3]:
- Left leg motors (IDs 1-5): local 8887, remote 8886
- Right leg motors (IDs 6-10): local 8889, remote 8888
- IMU (IDs 0x514-0x51B): shares port 8887 with left leg [2]

Port formula: `localPort = base_local_port + server_id * 2` [2]

---

### Entity 2: MIT Control Command (8 bytes CAN data)

Encodes 5 control parameters into exactly 8 bytes using bit-level packing [6]:

```
D[0] = (pos_uint >> 8) & 0xFF          Position high byte (16-bit)
D[1] = pos_uint & 0xFF                 Position low byte
D[2] = vel_uint >> 4                   Velocity high 8 bits (12-bit)
D[3] = ((vel_uint & 0xF) << 4) |      Velocity low 4 + Kp high 4
       ((kp_uint >> 8) & 0xF)
D[4] = kp_uint & 0xFF                  Kp low byte (12-bit)
D[5] = kd_uint >> 4                    Kd high 8 bits (12-bit)
D[6] = ((kd_uint & 0xF) << 4) |       Kd low 4 + Tau high 4
       ((tau_uint >> 8) & 0xF)
D[7] = tau_uint & 0xFF                 Tau low byte (12-bit)
```

Parameter encoding uses `double_to_uint()` [6]:
```cpp
uint16_t double_to_uint(double x, double x_min, double x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}
```

| Parameter | Bits | Range | CAN ID |
|-----------|:----:|-------|:------:|
| position | 16 | [-PMAX, PMAX] | `send_can_id` (no offset) [6] |
| velocity | 12 | [-VMAX, VMAX] | — |
| Kp | 12 | [0, 500] | — |
| Kd | 12 | [0, 5] | — |
| torque | 12 | [-TMAX, TMAX] | — |

DM8009 limits: ±12.5 rad position, ±45 rad/s velocity, ±54 Nm torque [2].

---

### Entity 3: Motor Feedback Frame (8 bytes CAN data)

Returned by the motor after every control command. Decoded by `parse_motor_state_data()` [6]:

```
D[0] = motor_id | (error_code << 4)    ID and status
D[1] = (q_uint >> 8) & 0xFF            Position high byte (16-bit)
D[2] = q_uint & 0xFF                   Position low byte
D[3] = (dq_uint >> 4) & 0xFF           Velocity high 8 bits (12-bit)
D[4] = ((dq_uint & 0xF) << 4) |        Velocity low 4 + Torque high 4
       ((tau_uint >> 8) & 0xF)
D[5] = tau_uint & 0xFF                 Torque low byte (12-bit)
D[6] = t_mos                           MOS temperature (°C)
D[7] = t_rotor                         Rotor temperature (°C)
```

Decoding [6]:
```cpp
uint16_t q_uint = (data[1] << 8) | data[2];
uint16_t dq_uint = (data[3] << 4) | (data[4] >> 4);
uint16_t tau_uint = ((data[4] & 0xF) << 8) | data[5];
double position = uint_to_double(q_uint, -pMax, pMax, 16);
double velocity = uint_to_double(dq_uint, -vMax, vMax, 12);
double torque = uint_to_double(tau_uint, -tMax, tMax, 12);
```

CAN receive ID = `device_id + 0x10` [2][3].

Error codes in D[0] upper nibble [2]:

| Code | Status |
|:----:|--------|
| 0x00 | Disabled |
| 0x01 | Enabled |
| 0x08 | Overvoltage |
| 0x09 | Undervoltage |
| 0x0A | Overcurrent |
| 0x0B | MOS overtemp |
| 0x0C | Coil overtemp |
| 0x0D | Comm loss (500 ms timeout) [2] |
| 0x0E | Overload |

---

### Entity 4: Motor Enable/Disable/Zero/Clear Command (8 bytes CAN data)

Fixed payload format [6]:

```cpp
std::vector<uint8_t> pack_command_data(uint8_t cmd) {
    return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
}
```

| Command | `cmd` byte | CAN ID | Purpose |
|---------|:----------:|:------:|---------|
| Enable | 0xFC | `send_can_id` | Start motor [6] |
| Disable | 0xFD | `send_can_id` | Stop motor [6] |
| Zero | 0xFE | `send_can_id` | Calibrate zero position [6] |
| Clear error | 0xFB | `send_can_id` | Clear fault state |

---

### Entity 5: Motor Control Mode CAN ID Offsets

Different control modes add offsets to the base CAN ID [6]:

| Mode | CAN ID | Payload | Source |
|------|:------:|---------|--------|
| MIT | `send_can_id` | 5 params packed (see Entity 2) | `create_mit_control_command()` [6] |
| PosVel | `send_can_id + 0x100` | `[pos(4B float)][vel(4B float)]` | `create_posvel_control_command()` [6] |
| Velocity | `send_can_id + 0x200` | `[vel(4B float)]` | `create_vel_control_command()` [6] |
| PosForce | `send_can_id + 0x300` | `[pos(4B)][vel_uint16(2B)][i_uint16(2B)]` | `create_posforce_control_command()` [6] |

PosVel encoding [6]:
```cpp
auto pb = float_to_uint8s(static_cast<float>(pos));
auto vb = float_to_uint8s(static_cast<float>(vel));
return {pb[0], pb[1], pb[2], pb[3], vb[0], vb[1], vb[2], vb[3]};
```

PosForce encoding scales velocity by 100 and current by 10000 into uint16 little-endian [6].

---

### Entity 6: Parameter Query/Response (CAN ID 0x7FF)

Used for reading and writing motor parameters [6]:

**Query (outbound):**
```
CAN ID = 0x7FF
D[0:1] = send_can_id (little-endian)
D[2]   = 0x33 (read) or 0x55 (write)
D[3]   = RID (parameter index)
D[4:7] = 0x00 (read) or value (write)
```

```cpp
std::vector<uint8_t> pack_query_param_data(uint32_t send_can_id, int RID) {
    return {static_cast<uint8_t>(send_can_id & 0xFF),
            static_cast<uint8_t>((send_can_id >> 8) & 0xFF),
            0x33, static_cast<uint8_t>(RID),
            0x00, 0x00, 0x00, 0x00};
}
```
[6]

**Response (inbound):**
```
D[2] == 0x33 (read response) or 0x55 (write confirmation)
D[3] = RID
D[4:7] = value (float or uint32 depending on RID)
```

Type determination [6]:
```cpp
bool is_in_ranges(int number) {
    return (7 <= number && number <= 10) ||
           (13 <= number && number <= 16) ||
           (35 <= number && number <= 36);
}
// Integer RIDs: 7-10, 13-16, 35-36 → uint8s_to_uint32()
// Float RIDs: all others → uint8s_to_float()
```

**Refresh command** (CAN ID 0x7FF) [6]:
```
D[0:1] = send_can_id (little-endian)
D[2]   = 0xCC
D[3:7] = 0x00
```

---

### Entity 7: IMU Measurement Cycle (8 frames × 2 float32)

LPMS-IG1 sequential CAN mode, 16 float32 values per cycle [2]:

| CAN ID | Slot 0 | Slot 1 | Unit |
|:------:|--------|--------|------|
| 0x514 | accX | accY | g |
| 0x515 | accZ | gyroX | g, dps |
| 0x516 | gyroY | gyroZ | dps |
| 0x517 | magX | magY | μT |
| 0x518 | magZ | eulerX | μT, deg |
| 0x519 | eulerY | eulerZ | deg |
| 0x51A | quatW | quatX | unitless |
| 0x51B | quatY | quatZ | unitless |

Each float32 occupies 4 bytes in the CAN data payload (bytes D[0:3] and D[4:7]), little-endian. The 7D state vector exposed to consumers is `[eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ]` [2].

---

### Entity 8: Telemetry Binary Packet (RobotStatusWire ~890 bytes)

Published every 20 ms (50 Hz) via MQTT [2]:

| Field | Size | Description |
|-------|:----:|-------------|
| Magic | 4B | 0x4B564155 ("KVAU") [2] |
| Version | 1B | Protocol version |
| Timestamp | 8B | Microsecond timestamp |
| Frame counter | 4B | Monotonic count |
| LegStatusWire × 2 | ~800B | Motor states for both legs |
| DriverCommandWire | ~40B | DS control words + axes |
| ImuWire | ~56B | 7D state vector |

JSON SenML data logs published to `/telemetry/subsystem/<name>/motor` [2].

MQTT connection: broker `127.0.0.1:1883`, libwebsockets 4.5.8, username/password auth, non-blocking with automatic reconnection [2][3].

---

### Entity 9: Mercury Controller Shared Memory

POSIX shared memory IPC bridge (`mercury_shm.h`) to Mercury dynamics controller (DynaCoRE) [3]:

**Command buffer (producer → consumer):**

| Field | Type | Count | Description |
|-------|------|:-----:|-------------|
| jtorque_cmd | double | 12 | Joint torque commands |
| jpos_cmd | double | 12 | Joint position commands |
| jvel_cmd | double | 12 | Joint velocity commands |
| kp | double | 12 | Proportional gains |
| kd | double | 12 | Derivative gains |
| timestamp_ns | uint64 | 1 | Command timestamp |
| sequence | uint64 | 1 | Monotonic counter |

**Sensor data buffer (consumer → producer):**

| Field | Type | Count | Description |
|-------|------|:-----:|-------------|
| joint_jpos | double | 12 | Joint positions |
| joint_jvel | double | 12 | Joint velocities |
| jtorque | double | 12 | Joint torques |
| imu_inc | double | 3 | IMU incremental angles |
| imu_ang_vel | double | 3 | Gyroscope angular velocity |
| imu_acc | double | 3 | Accelerometer |
| rfoot_contact | bool | 1 | Right foot contact |
| lfoot_contact | bool | 1 | Left foot contact |

DynaCoRE installed at `/usr/local/include/DynaCoRE` [3]. Integration referenced but not yet active in main control path [2].

---

### Entity 10: DriverStation UDP Packet

Follows the FRC Driver Station pattern [2][3]:

| Field | Description |
|-------|-------------|
| Control word | Operating mode (Disabled/Autonomous/Teleop) [2] |
| Button bitmask | Digital button states [2] |
| Joystick axes | Analog stick positions |

Events processed via `EventLoop` + `BooleanEvent` with rising/falling edge detection [2].

---

## Module Interaction Diagrams

### Interaction 1: Motor Control Cycle (5 ms / 200 Hz)

```
Legged          Controller       Motor           CAN/CANAPI      UdpServer       Damiao HW
Subsystem       (LQR) [2]       (×5) [2]                        [2]
  │                │               │               │               │               │
  │ calculate()    │               │               │               │               │
  ├───────────────►│               │               │               │               │
  │                │ 7D state      │               │               │               │
  │                │ gains LUT [2] │               │               │               │
  │◄───────────────┤               │               │               │               │
  │ 2D control     │               │               │               │               │
  │                │               │               │               │               │
  │ setMitControl(pos,vel,kp,kd,tau) [2]           │               │               │
  ├────────────────────────────────►│               │               │               │
  │                │               │ double_to_uint │               │               │
  │                │               │ pos(16-bit)    │               │               │
  │                │               │ vel(12-bit)    │               │               │
  │                │               │ kp(12-bit)     │               │               │
  │                │               │ kd(12-bit)     │               │               │
  │                │               │ tau(12-bit)    │               │               │
  │                │               │ [6]            │               │               │
  │                │               │ sendMsg()      │               │               │
  │                │               ├───────────────►│               │               │
  │                │               │               │ CAN frame     │               │
  │                │               │               │ 13 bytes [3]  │               │
  │                │               │               ├──────────────►│               │
  │                │               │               │               │ UDP sendto()  │
  │                │               │               │               ├──────────────►│
  │                │               │               │               │               │
  │                │               │               │               │  feedback     │
  │                │               │               │               │◄──────────────┤
  │                │               │               │ UDP recvfrom  │               │
  │                │               │               │◄──────────────┤               │
  │                │               │               │ CAN ID check  │               │
  │                │               │               │ device_id+0x10│               │
  │                │               │               │ [2][3]        │               │
  │                │               │◄──────────────┤ callback [2]  │               │
  │                │               │ uint_to_double │               │               │
  │                │               │ pos,vel,tau [6]│               │               │
  │                │               │ D[6]=t_mos     │               │               │
  │                │               │ D[7]=t_rotor   │               │               │
  │                │               │ D[0]=status [6]│               │               │
  │◄───────────────────────────────┤ state update   │               │               │
```

---

### Interaction 2: Motor Enable/Disable via DriverStation

```
Xbox            DriverStation    EventLoop       Legged          Motor         UdpServer
Controller      [2]              [2]             [2]             [2]           [2]
  │                │               │               │               │             │
  │ Button 1       │               │               │               │             │
  │ pressed        │               │               │               │             │
  ├───────────────►│               │               │               │             │
  │                │ UDP parse     │               │               │             │
  │                │ button state  │               │               │             │
  │                ├──────────────►│               │               │             │
  │                │               │ BooleanEvent  │               │             │
  │                │               │ rising edge   │               │             │
  │                │               │ detection [2] │               │             │
  │                │               │               │               │             │
  │                │               │ m_loop.poll() │               │             │
  │                │               │ (from robot   │               │             │
  │                │               │  Periodic) [2]│               │             │
  │                │               │               │               │             │
  │                │               │ onMessage     │               │             │
  │                │               │ (MSG_ENABLE)  │               │             │
  │                │               ├──async FIFO──►│               │             │
  │                │               │ poll() [2]    │               │             │
  │                │               │               │               │             │
  │                │               │               │ For each      │             │
  │                │               │               │ motor (×5):   │             │
  │                │               │               │               │             │
  │                │               │               │ sendCommand   │             │
  │                │               │               │ (0xFC) [6]    │             │
  │                │               │               ├──────────────►│             │
  │                │               │               │ {0xFF×7,0xFC} │             │
  │                │               │               │ [6]           │             │
  │                │               │               │               │ CAN frame   │
  │                │               │               │               │ 13B [3]     │
  │                │               │               │               ├────────────►│
  │                │               │               │               │             │
  │                │               │               │               │ feedback    │
  │                │               │               │               │◄────────────┤
  │                │               │               │ status=0x01   │             │
  │                │               │               │ (ENABLED) [2] │             │
  │                │               │◄──────────────┤               │             │
```

---

### Interaction 3: IMU Data Acquisition

```
LPMS-IG1        CAN Bridge      ImuReader       Imu             Robot
Hardware                        (pthread) [2]   Subsystem [2]   Main Loop [2]
  │               │               │               │               │
  │ CAN frame     │               │               │               │
  │ 0x514         │               │               │               │
  │ (accX,accY)   │               │               │               │
  ├──────────────►│               │               │               │
  │               │ UDP packet    │               │               │
  │               │ 13B [3]       │               │               │
  │               ├──────────────►│               │               │
  │               │               │ epoll_wait()  │               │
  │               │               │ recvfrom()    │               │
  │               │               │ CAN ID filter │               │
  │               │               │ [0x514,0x51B] │               │
  │               │               │ [2]           │               │
  │               │               │               │               │
  │ 0x515-0x51B   │               │               │               │
  │ (remaining    │               │               │               │
  │  7 frames)    │               │               │               │
  ├──────────────►│──────────────►│               │               │
  │               │               │               │               │
  │               │               │ All 8 frames  │               │
  │               │               │ received [2]  │               │
  │               │               │ 16 float32    │               │
  │               │               │ parsed        │               │
  │               │               │               │               │
  │               │               │ update()      │               │
  │               │               ├──────────────►│               │
  │               │               │               │ 7D state [2]: │
  │               │               │               │ eulerX,Y,Z    │
  │               │               │               │ quatW,X,Y,Z   │
  │               │               │               │               │
  │               │               │               │ getStates()   │
  │               │               │               │ (mutex) [2]   │
  │               │               │               ├──────────────►│
  │               │               │               │               │ telemetry
  │               │               │               │               │ collection
```

---

### Interaction 4: Telemetry Collection and Publish

```
Robot           Legged          Imu             RobotStatus     MqttClient     MQTT
Main Loop [2]   [2]             [2]             [2]             [3]            Broker [3]
  │               │               │               │               │             │
  │ robotPeriodic │               │               │               │             │
  │ (20ms) [2]    │               │               │               │             │
  │               │               │               │               │             │
  │ getMotors()   │               │               │               │             │
  ├──────────────►│               │               │               │             │
  │◄──────────────┤               │               │               │             │
  │ pos,vel,tau   │               │               │               │             │
  │ temp,status   │               │               │               │             │
  │ per motor [2] │               │               │               │             │
  │               │               │               │               │             │
  │ getStates()   │               │               │               │             │
  │ (mutex) [2]   │               │               │               │             │
  ├──────────────────────────────►│               │               │             │
  │◄──────────────────────────────┤               │               │             │
  │ 7D state [2]  │               │               │               │             │
  │               │               │               │               │             │
  │ collect()     │               │               │               │             │
  ├──────────────────────────────────────────────►│               │             │
  │               │               │               │ Pack 890B [2] │             │
  │               │               │               │ magic:        │             │
  │               │               │               │ 0x4B564155    │             │
  │               │               │               │               │             │
  │               │               │               │ publish()     │             │
  │               │               │               ├──────────────►│             │
  │               │               │               │               │ MQTT        │
  │               │               │               │               │ PUBLISH     │
  │               │               │               │               │ lws 4.5.8   │
  │               │               │               │               │ [3]         │
  │               │               │               │               ├────────────►│
  │               │               │               │               │             │
  │ logMotors()   │               │               │               │             │
  │ logImu() [2]  │               │               │               │             │
  │ (JSON SenML)  │               │               │               │             │
  ├──────────────────────────────────────────────────────────────►│             │
  │               │               │               │               ├────────────►│
```

---

## CAN ID Routing Summary

```
CAN ID Space (used by Kuavo):

  0x01 - 0x05    Motor send IDs (left leg)         → UdpServer 0 [2]
  0x06 - 0x0A    Motor send IDs (right leg)        → UdpServer 1 [2]
  0x11 - 0x15    Motor recv IDs (left leg)          → Motor::callback() [2]
  0x16 - 0x1A    Motor recv IDs (right leg)         → Motor::callback() [2]

  0x101 - 0x10A  PosVel mode (send_id + 0x100) [6]
  0x201 - 0x20A  Velocity mode (send_id + 0x200) [6]
  0x301 - 0x30A  PosForce mode (send_id + 0x300) [6]

  0x514 - 0x51B  IMU frames (8 sequential) [2]     → ImuReader

  0x7FF          Parameter query/write/refresh [6]  → Motor param handling

  Routing rule: device_id < 6 → UdpServer 0 (left); else → UdpServer 1 (right) [2]
  IMU filtered by CAN ID range [0x514, 0x51B] within ImuReader [2]
  No overlap between motor IDs (0x01-0x1A) and IMU IDs (0x514-0x51B) [2]
```