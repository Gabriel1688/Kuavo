
# Safety Coordination: Actuator, DriverStation, and Mercury Controller

## Three-Party Safety Architecture

The Kuavo robot safety model involves three independent actors that must coordinate to prevent unsafe motor states, communication timeouts, and control divergence that could damage hardware or cause the robot to fall [3].

```
Mercury Controller              Robot Main Loop                 Damiao Motors
(DynaCoRE, separate              (TimedRobot, 20ms/50Hz)         (DM8009 × 10)
 process) [3]                           |                            |
      |                                 |                            |
      |  POSIX Shared Memory            |   CAN-over-UDP             |
      |  (mercury_shm.h) [3]           |   13-byte frames [3]       |
      |                                 |                            |
      +-------- commands ------------>  |                            |
      |                                 +---- MIT control -------->  |
      | <------- sensor data ---------  |                            |
      |                                 | <---- feedback ----------  |
      |                                 |                            |
      |                          DriverStation                       |
      |                          (Xbox controller)                   |
      |                          EventLoop + BooleanEvent [2]        |
      |                                 |                            |
      |                          enable/disable buttons [2]          |
      |                          mode transitions [2]                |
```

---

## Operating Mode State Machine

Operating modes follow the FRC pattern with `init()` and `periodic()` callbacks per mode [2][3]. The `TimedRobot` default period is 20 ms (50 Hz) [2]. Mode transitions are driven by DriverStation control words received via UDP [2].

```
                    ┌──────────────────────┐
                    │                      │
                    │      DISABLED        │
                    │  (power-up default)  │
                    │                      │
                    │  Motors: 0xFD [6]    │
                    │  Controller: OFF     │
                    │  Threads: running    │
                    │  but not dispatching │
                    │                      │
                    └──────┬───────┬───────┘
                           │       │
            ┌──────────────┘       └──────────────┐
            │ DS: autonomous                      │ DS: teleop
            │ button/mode select                  │ button/mode select
            ▼                                     ▼
  ┌──────────────────────┐          ┌──────────────────────┐
  │                      │          │                      │
  │     AUTONOMOUS       │          │    TELEOPERATION     │
  │                      │          │                      │
  │  Motors: 0xFC [6]    │          │  Motors: 0xFC [6]    │
  │  Controller: ON      │          │  Controller: ON      │
  │  Mercury SHM: active │          │  Mercury SHM: active │
  │  Trajectory: replay  │          │  Joystick: active    │
  │                      │          │  Button 1: enable    │
  │                      │          │  Button 2: disable   │
  │                      │          │  [2]                 │
  └──────────┬───────────┘          └──────────┬───────────┘
             │                                 │
             │ DS: disable                     │ DS: disable
             │ or safety trigger               │ or safety trigger
             │                                 │
             └─────────────┬───────────────────┘
                           │
                           ▼
                    ┌──────────────────────┐
                    │                      │
                    │  EMERGENCY STOP      │
                    │                      │
                    │  Motors: 0xFD all [6]│
                    │  Controller: OFF     │
                    │  SHM: flag set       │
                    │  Recovery: manual    │
                    │                      │
                    └──────────────────────┘
```

### Mode Transition Safety Rules

| Transition | Trigger | Motor Action | Controller Action | SHM Action |
|-----------|---------|:---:|:---:|:---:|
| Disabled → Autonomous | DS mode command [2] | Enable all (0xFC) [6] | Start `controllerPeriodic()` | Begin reading commands |
| Disabled → Teleop | DS mode command [2] | Enable all (0xFC) [6] | Start `controllerPeriodic()` | Begin reading commands |
| Autonomous → Disabled | DS mode or timeout | Disable all (0xFD) [6] | Stop dispatching | Stop reading |
| Teleop → Disabled | DS mode or Button 2 [2] | Disable all (0xFD) [6] | Stop dispatching | Stop reading |
| Any → Emergency Stop | Safety trigger | Disable all (0xFD) [6] | Immediate stop | Set emergency flag |
| Emergency → Disabled | Manual operator reset | Already disabled | Already stopped | Clear flag after verification |

---

## Leg Subsystem Enable/Disable Lifecycle

The `Legged` subsystem implements `ControlledSubsystemBase<7, 2, 5>` with a dedicated pthread per instance [2]. The right leg is instantiated but currently disabled in code [2]. Each leg manages exactly 5 motors initialized from `config.yaml` (left: base_id=1, right: base_id=6) [2].

```
LEG SUBSYSTEM STATE MACHINE:

  ┌────────────────────────┐
  │                        │
  │     CONSTRUCTED        │
  │                        │
  │  pthread created [2]   │
  │  Motors allocated (×5) │
  │  SHM ptr assigned      │
  │  Thread: BLOCKED       │
  │  (waiting for start)   │
  │                        │
  └───────────┬────────────┘
              │
              │ start() called after
              │ SHM verified (magic == SHM_MAGIC)
              ▼
  ┌────────────────────────┐
  │                        │
  │     STARTED            │
  │                        │
  │  Thread: RUNNING       │
  │  controllerPeriodic()  │
  │  at 5ms / 200Hz [2]   │
  │  Motors: not yet       │
  │  enabled               │
  │                        │
  └───────────┬────────────┘
              │
              │ EventLoop callback [2]:
              │ Button 1 → enable left leg
              │ onMessage(MSG_ENABLE)
              ▼
  ┌────────────────────────┐
  │                        │
  │     ENABLED            │
  │  (normal operation)    │
  │                        │
  │  Motors: 0xFC sent [6] │
  │  MIT commands: active  │
  │  Controller: running   │
  │  LQR calculate() [2]  │
  │  Feedback: monitored   │
  │  Timeout: 500ms [3]   │
  │                        │
  └───────┬────────┬───────┘
          │        │
          │        │ EventLoop callback [2]:
          │        │ Button 2 → disable left leg
          │        │ onMessage(MSG_DISABLE)
          │        ▼
          │  ┌────────────────────────┐
          │  │                        │
          │  │     DISABLED           │
          │  │                        │
          │  │  Motors: 0xFD sent [6] │
          │  │  MIT commands: stopped │
          │  │  Controller: paused   │
          │  │  Thread: still running│
          │  │  (checking for re-    │
          │  │   enable message)     │
          │  │                        │
          │  └────────────────────────┘
          │
          │ Motor error detected
          │ (0x08-0x0E) [2][3]
          │ or cmd timestamp stale
          │ or Mercury crash
          ▼
  ┌────────────────────────┐
  │                        │
  │     ERROR / ESTOP      │
  │                        │
  │  Motors: 0xFD all [6]  │
  │  Error clear: 0xFB [6]│
  │  Thread: running       │
  │  (monitoring for       │
  │   recovery)            │
  │  Re-enable requires:   │
  │  1. Error clear (0xFB) │
  │  2. Mode → Disabled    │
  │  3. Mode → Teleop/Auto │
  │  4. Button 1 enable    │
  │                        │
  └────────────────────────┘
```

### Enable/Disable Command Dispatch

Button events flow through `EventLoop` callbacks to subsystem `onMessage()` handlers via async FIFO message queues with `poll()`-based notification [2]. The `BooleanEvent` provides rising/falling edge detection to prevent repeated triggers from held buttons [2].

```
ENABLE SEQUENCE:

  Operator presses Button 1 [2]
       │
       ▼
  EventLoop detects rising edge
  via BooleanEvent [2]
       │
       ▼
  EventLoop callback dispatches
  MSG_ENABLE to Legged::onMessage()
  via async FIFO message queue [2]
       │
       ▼
  Legged subsystem thread receives
  message via poll() notification [2]
       │
       ▼
  For each of 5 motors:
       │
       ├──► Motor::sendCommand(0xFC) [6]
       │    pack_command_data(0xFC):
       │    {0xFF, 0xFF, 0xFF, 0xFF,
       │     0xFF, 0xFF, 0xFF, 0xFC} [6]
       │
       ├──► CAN frame via UdpServer [2]
       │    CAN_ID = motor.send_can_id
       │    13-byte UDP frame [3]
       │
       └──► Motor state → ENABLED (0x01)
            in feedback frame D[0] [6]


DISABLE SEQUENCE:

  Operator presses Button 2 [2]
  OR safety trigger detected
  OR mode → Disabled [2]
       │
       ▼
  Same EventLoop → onMessage path [2]
       │
       ▼
  For each of 5 motors:
       │
       ├──► Motor::sendCommand(0xFD) [6]
       │    pack_command_data(0xFD):
       │    {0xFF, 0xFF, 0xFF, 0xFF,
       │     0xFF, 0xFF, 0xFF, 0xFD} [6]
       │
       └──► Motor state → DISABLED (0x00) [2]
```

---

## Shared Memory Connection Manager

The Mercury Controller communicates via POSIX shared memory (`mercury_shm.h`) [3]. The Robot (Kuavo main process) is the SHM owner — it creates, initializes, and destroys the shared memory segment (`shm_open(O_CREAT | O_RDWR)`, `ftruncate()`, `shm_unlink()` on shutdown). The Mercury Controller is a consumer that attaches to the Robot's existing SHM with `shm_open(O_RDWR)` (no `O_CREAT`). DynaCoRE integration is referenced but not yet active in the main control path [2].

```
SHM CONNECTION STATE MACHINE (Robot is SHM owner):

  ┌────────────────────────┐
  │                        │
  │     DISCONNECTED       │
  │  (power-up default)    │
  │                        │
  │  m_shm = nullptr       │
  │  Subsystems: not       │
  │  started               │
  │  Motors: disabled      │
  │                        │
  └───────────┬────────────┘
              │
              │ robotInit():
              │ shm_open(O_CREAT|O_RDWR)
              │ ftruncate(sizeof(ShmLayout))
              │ mmap() + initialize fields
              │ magic, version=5, lifecycle
              ▼
  ┌────────────────────────┐
  │                        │
  │     SHM CREATED        │
  │  (Robot owns segment)  │
  │                        │
  │  m_shm = mmap result   │
  │  magic = SHM_MAGIC     │
  │  version = SHM_VERSION │
  │  Subsystems: start()   │
  │  Waiting for controller│
  │  to attach             │
  │                        │
  └──────┬─────────────────┘
         │
         │ Controller attaches:
         │ shm_open(O_RDWR)
         │ (no O_CREAT)
         │ fstat() size check
         │ magic + version check
         ▼
  ┌────────────────────────┐
  │                        │
  │     CONNECTED          │
  │  (normal operation)    │
  │                        │
  │  m_shm valid           │
  │  Subsystems: running   │
  │  controllerPeriodic()  │
  │  reads cmd_write_idx   │
  │  writes sensor data    │
  │  Robot monitors cmd    │
  │  timestamp staleness   │
  │                        │
  └──────┬─────────┬───────┘
         │         │
         │    Cmd timestamp stale
         │    > 100ms
         │    OR controller_emergency
         │    _stop == true
         │    OR magic corrupted
         ▼         ▼
  ┌────────────────────────┐
  │                        │
  │     CONTROLLER LOST    │
  │  (consumer lost)       │
  │                        │
  │  Robot sets            │
  │  emergency_stop = true │
  │  Motors: 0xFD all [6]  │
  │  SHM segment remains   │
  │  (Robot still owns it) │
  │                        │
  └───────────┬────────────┘
              │
              │ robotPeriodic():
              │ monitor for fresh
              │ cmd timestamp
              │ (controller reconnects
              │  via shm_open(O_RDWR))
              ▼
  ┌────────────────────────┐
  │                        │
  │     RECONNECTING       │
  │                        │
  │  SHM still mapped      │
  │  Robot watches for     │
  │  fresh cmd timestamp   │
  │  (controller reattach) │
  │                        │
  │  On success:           │
  │  → CONNECTED           │
  │  → emergency_stop=false│
  │  → subsystems start()  │
  │  → motors re-enable    │
  │                        │
  └────────────────────────┘

  ROBOT SHUTDOWN:
  │  Robot calls shm_unlink()
  │  Controller never calls shm_unlink()
```

### SHM Validation Checks per Cycle

Every `controllerPeriodic()` cycle (5 ms) [2] performs these checks before reading commands:

```
controllerPeriodic() entry:
       │
       ├──► Check 1: m_shm != nullptr
       │    IF null → return (not attached)
       │
       ├──► Check 2: magic == SHM_MAGIC
       │    IF wrong → SHM corrupted or reclaimed
       │    → disableAllMotors() [6]
       │    → return
       │
       ├──► Check 3: controller_emergency_stop
       │    IF controller_emergency_stop == true
       │    → disableAllMotors() [6]
       │    → set emergency_stop = true
       │    → return
       │
       ├──► Check 4: cmd timestamp not stale
       │    IF (now - cmd_buffers[cmd_write_idx]
       │        .timestamp_ns) > 100ms
       │    → disableAllMotors() [6]
       │    → set emergency_stop = true
       │    → flag controllerAlive = false
       │    → return
       │
       └──► All checks passed
            → Read cmd_write_idx
            → memcpy command buffer
            → Dispatch MIT commands [6]
```

---

## Thread Architecture

The system uses POSIX threads with real-time extensions (pthread, rt) [3]. `ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance for non-blocking message processing [2]. The main control loop runs at 50 Hz (20 ms period) with subsystem inner loops at 200 Hz (5 ms) [3].

```
THREAD MAP:

Thread 1: MAIN ROBOT LOOP (TimedRobot Notifier, 20ms/50Hz) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Scheduling: Default (SCHED_OTHER)                       │
  │  Period: 20 ms (50 Hz) [2]                               │
  │                                                          │
  │  loopFunc() chain [2]:                                   │
  │  Notifier → loopFunc() → refreshData()                   │
  │          → mode switch → robotPeriodic()                  │
  │                                                          │
  │  robotPeriodic() responsibilities [2]:                   │
  │  • m_loop.poll() (button events) [2]                     │
  │  • Mode management (disabled/autonomous/teleop) [2]      │
  │  • leftLeg.getMotors() (telemetry collection) [2]        │
  │  • imu.getStates() (mutex-protected) [2]                 │
  │  • RobotStatus::collect() + publish() [2]                │
  │  • DataLog::logMotors() + logImu() [2]                   │
  │  • SHM reconnection check (Layer 5)                      │
  │  • runAllRobotPeriodic() (subsystem dispatch) [2]        │
  └──────────────────────────────────────────────────────────┘

Thread 2: LEFT LEG SUBSYSTEM (ControlledSubsystemBase pthread) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Scheduling: Default (SCHED_OTHER) [3]                   │
  │  Period: 5 ms (200 Hz) [2]                               │
  │  Synchronization: poll()-based async FIFO [2]            │
  │                                                          │
  │  controllerPeriodic() [2]:                               │
  │  • SHM validation (magic, cmd timestamp, e-stop)         │
  │  • Read Mercury command from SHM                         │
  │  • Controller::calculate(state_7d) [2]                   │
  │  • Motor::setMitControl() for each of 5 motors [2]      │
  │  • Motor feedback monitoring (500ms timeout) [3]         │
  │                                                          │
  │  onMessage() [2]:                                        │
  │  • MSG_ENABLE → send 0xFC to all motors [6]              │
  │  • MSG_DISABLE → send 0xFD to all motors [6]             │
  │  • MSG_ZERO → send 0xFE to calibration motor [6]         │
  │  • MSG_CLEAR → send 0xFB to error motor [6]              │
  └──────────────────────────────────────────────────────────┘

Thread 3: RIGHT LEG SUBSYSTEM (instantiated but disabled) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Same structure as Thread 2                              │
  │  Motors 6-10 (base_id=6) [2]                             │
  │  Currently disabled in code [2]                          │
  │  Thread exists but control loop inactive [2]             │
  └──────────────────────────────────────────────────────────┘

Thread 4: IMU READER (dedicated pthread) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Scheduling: Default (SCHED_OTHER)                       │
  │  Rate: ~500 Hz (2 ms per 8-frame cycle) [2]              │
  │  Socket: Blocking UDP, port 8887 [2]                     │
  │  Event: epoll for non-blocking CAN frame reception [2]   │
  │                                                          │
  │  Receives 8 CAN frames (0x514-0x51B) per cycle [2]      │
  │  Parses 16 float32 values [2]                            │
  │  Updates 7D state vector (mutex-protected) [2]           │
  │  Shares port 8887 with UdpServer 0 [3]                   │
  └──────────────────────────────────────────────────────────┘

Thread 5: UDPSERVER 0 — LEFT LEG (singleton) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Ports: local 8887, remote 8886 [3]                      │
  │  Motors: IDs 1-5 (device_id < 6) [2]                     │
  │  CAN recv IDs: 0x11-0x15 (device_id + 0x10) [2][3]      │
  │  Callback dispatch: Motor::callback() [2]                │
  │  13-byte CAN-over-UDP frame format [3]                   │
  └──────────────────────────────────────────────────────────┘

Thread 6: UDPSERVER 1 — RIGHT LEG (singleton) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Ports: local 8889, remote 8888 [3]                      │
  │  Motors: IDs 6-10 (device_id >= 6) [2]                   │
  │  CAN recv IDs: 0x16-0x1A [2][3]                          │
  │  Callback dispatch: Motor::callback() [2]                │
  └──────────────────────────────────────────────────────────┘

Thread 7: DRIVERSTATION (UDP receiver) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Event-driven: UDP packet parser [2]                     │
  │  Parses control words + joystick axes [2]                │
  │  No periodic loop — fires on packet arrival              │
  └──────────────────────────────────────────────────────────┘

Thread 8: EVENTLOOP (polled from main loop) [2]
  ┌──────────────────────────────────────────────────────────┐
  │  Polled via m_loop.poll() from robotPeriodic() [2]       │
  │  BooleanEvent edge detection [2]                         │
  │  Button 1: enable left leg [2]                           │
  │  Button 2: disable left leg [2]                          │
  │  Button 3: reboot (disabled) [2]                         │
  │  Button 4: async state update [2]                        │
  └──────────────────────────────────────────────────────────┘

Thread 9: MQTT CLIENT (libwebsockets event loop) [3]
  ┌──────────────────────────────────────────────────────────┐
  │  libwebsockets 4.5.8 [3]                                 │
  │  Broker: 127.0.0.1:1883 [3]                              │
  │  Auth: username/password [2]                             │
  │  Reconnect: automatic [2]                                │
  │  Publishes: RobotStatusWire (890 bytes) [2]              │
  │  Publishes: JSON SenML data logs [2]                     │
  │  Rate: 50 Hz (every robotPeriodic cycle) [2]             │
  └──────────────────────────────────────────────────────────┘
```

---

## Synchronization Mechanisms

All motor and IMU state access is mutex-protected [3]. Subsystem threads use `poll()`-based event notification to avoid busy-waiting [3].

### Synchronous Interfaces

| Interface | Mechanism | Thread Context | Frequency |
|-----------|-----------|:---:|:---:|
| Robot → Legged.robotPeriodic() | Direct method call [2] | Main loop thread | 50 Hz |
| Legged → Controller.calculate() | Direct method call within subsystem pthread [2] | Leg subsystem thread | 200 Hz |
| Legged → Motor.setMitControl() | Direct method call [2] | Leg subsystem thread | 200 Hz |
| Robot → Imu.getStates() | Mutex-protected accessor [2] | Main loop thread | 50 Hz |
| Robot → RobotStatus.publish() | Inline publish from robotPeriodic() [2] | Main loop thread | 50 Hz |

### Asynchronous Interfaces

| Interface | Mechanism | Direction | Trigger |
|-----------|-----------|:---------:|---------|
| EventLoop → Legged.onMessage() | Async FIFO message queue + poll() [2] | DriverStation → Subsystem | Button press [2] |
| UdpServer → Motor.callback() | UDP receive callback [2] | Motor HW → Software | CAN feedback frame |
| ImuReader → Imu.update() | Internal callback within ImuReader pthread [2] | IMU HW → Software | CAN frame arrival |
| Mercury Controller → SHM | Cross-process atomic write [3] | Consumer → Owner (Robot) | Controller write cycle |

### Mutex-Protected State

| State | Mutex Location | Writers | Readers | Contention Risk |
|-------|:---:|:---:|:---:|:---:|
| IMU 7D state vector | Imu::m_state [2] | ImuReader thread (500 Hz) | Main loop (50 Hz), Controller (200 Hz) | Medium — main loop and controller both need orientation data |
| Motor internal state | Motor class [2] | UdpServer callback (async) | Leg subsystem thread (200 Hz) | Low — callback and periodic are in the same thread context via ownership |

### Message Queue Synchronization

Subsystem command dispatch (enable/disable/zero/clear) uses async FIFO message queues with `poll()`-based notification [2]:

```
EventLoop (Thread 8)                    Legged (Thread 2)
       │                                      │
       │  BooleanEvent rising edge [2]         │
       │  Button 1 detected                   │
       │                                      │
       ├──► Write MSG_ENABLE to FIFO          │
       │    (non-blocking write)              │
       │                                      │
       │                                      │  poll() wakes up [2]
       │                                      │  Read MSG_ENABLE from FIFO
       │                                      │
       │                                      ├──► For each motor:
       │                                      │    sendCommand(0xFC) [6]
       │                                      │
       │                                      ├──► CAN frame → UdpServer
       │                                      │    → UDP → Motor HW
       │                                      │
       │                                      └──► Motor state → ENABLED
```

---

## Motor Safety State Machine

Motors implement a state machine with automatic error detection [3]. The Motor class enforces enable/disable commands (0xFC/0xFD) and 500 ms responsiveness timeout before flagging a motor as unresponsive [3].

```
MOTOR STATE MACHINE (per motor):

  ┌────────────────────────┐
  │                        │
  │   DISABLED (0x00)      │◄────────────────────────────────┐
  │   (power-up default)   │                                 │
  │                        │  onMessage(MSG_DISABLE)          │
  │   No control accepted  │  OR safety trigger               │
  │                        │  OR mode → Disabled [2]          │
  └───────────┬────────────┘                                 │
              │                                              │
              │ sendCommand(0xFC) [6]                        │
              │ {0xFF×7, 0xFC}                               │
              ▼                                              │
  ┌────────────────────────┐                                 │
  │                        │                                 │
  │   ENABLED (0x01)       │─────────────────────────────────┤
  │   (normal operation)   │  sendCommand(0xFD) [6]          │
  │                        │  {0xFF×7, 0xFD}                 │
  │   MIT commands active  │                                 │
  │   Feedback monitored   │                                 │
  │   500ms timeout [3]    │                                 │
  │                        │                                 │
  └───────────┬────────────┘                                 │
              │                                              │
              │ Error detected in                            │
              │ feedback D[0] upper nibble [6]               │
              ▼                                              │
  ┌────────────────────────┐                                 │
  │                        │                                 │
  │   ERROR STATE          │─────────────────────────────────┘
  │                        │  sendCommand(0xFB) [6] → clear
  │   0x08: overvoltage    │  then re-enable with 0xFC [6]
  │   0x09: undervoltage   │
  │   0x0A: overcurrent    │
  │   0x0B: MOS overtemp   │
  │   0x0C: coil overtemp  │
  │   0x0D: comm loss      │  ← 500ms timeout [3]
  │   0x0E: overload       │
  │   [2][3]               │
  │                        │
  │   Control suspended    │
  │   Error must be        │
  │   cleared (0xFB) [6]   │
  │   before re-enable     │
  │                        │
  └────────────────────────┘
```

### Motor Feedback Decoding

Motor feedback arrives asynchronously via UdpServer callback chain [2]. The feedback frame is decoded by `parse_motor_state_data()` [6]:

```
Feedback CAN frame (8 bytes) [6]:

  D[0] = motor_id | (error_code << 4)
  D[1:2] = position (16-bit mapped to [-PMAX, PMAX])
  D[3:4] = velocity (12-bit mapped to [-VMAX, VMAX])
  D[4:5] = torque (12-bit mapped to [-TMAX, TMAX])
  D[6] = T_MOS (°C)
  D[7] = T_Rotor (°C)

  Decoding [6]:
  q_uint = (data[1] << 8) | data[2]
  dq_uint = (data[3] << 4) | (data[4] >> 4)
  tau_uint = ((data[4] & 0xF) << 8) | data[5]

  position = uint_to_double(q_uint, -pMax, pMax, 16) [6]
  velocity = uint_to_double(dq_uint, -vMax, vMax, 12) [6]
  torque = uint_to_double(tau_uint, -tMax, tMax, 12) [6]
```

---

## Safety Coordination Sequence Diagram

### Normal Operation (Teleop Mode)

```
DriverStation   Robot Main    Legged        Motor         UdpServer    Mercury
               Loop (20ms)   (5ms) [2]     (×5) [2]     [2]          Controller
    │              │             │            │             │            │
    │ Mode: TELEOP │             │            │             │            │
    ├─────────────►│             │            │             │            │
    │              │ teleopInit()│            │             │            │
    │              ├────────────►│            │             │            │
    │              │             │ start()    │             │            │
    │              │             │ (if SHM    │             │            │
    │              │             │  valid)    │             │            │
    │              │             │            │             │            │
    │ Button 1:    │             │            │             │            │
    │ enable [2]   │             │            │             │            │
    ├─────────────►│             │            │             │            │
    │              │ m_loop      │            │             │            │
    │              │ .poll() [2] │            │             │            │
    │              │             │            │             │            │
    │              │ onMessage   │            │             │            │
    │              │ (ENABLE)    │            │             │            │
    │              ├────FIFO────►│            │             │            │
    │              │             │ 0xFC [6]   │             │            │
    │              │             ├───────────►│             │            │
    │              │             │            │ CAN frame   │            │
    │              │             │            ├────────────►│            │
    │              │             │            │             │ UDP        │
    │              │             │            │             ├───────────►│
    │              │             │            │             │            │
    │              │             │            │             │ SHM write  │
    │              │             │            │             │            │
    │              │             │ read cmd   │             │ commands   │
    │              │             │◄───────────┼─────────────┼────────────┤
    │              │             │ from SHM   │             │            │
    │              │             │            │             │            │
    │              │             │ calculate()│             │            │
    │              │             │ [2]        │             │            │
    │              │             │            │             │            │
    │              │             │ MIT cmd    │             │            │
    │              │             ├───────────►│             │            │
    │              │             │            │ CAN [6]     │            │
    │              │             │            ├────────────►│            │
    │              │             │            │             │───────────►│
    │              │             │            │             │            │
    │              │             │            │ feedback    │            │
    │              │             │            │◄────────────┤            │
    │              │             │            │ callback [2]│            │
    │              │             │◄───────────┤             │            │
    │              │             │ state      │             │            │
    │              │             │ update     │             │            │
    │              │             │            │             │            │
    │              │             │ write SHM  │             │            │
    │              │             │ sensor data│             │            │
    │              │             ├────────────┼─────────────┼───────────►│
    │              │             │            │             │            │
```

### Safety Trigger (Mercury Controller Crash)

```
Mercury         Robot Main     Legged         Motor        UdpServer
Controller      Loop (20ms)    (5ms) [2]      (×5) [2]    [2]
    │               │              │             │            │
    │  CRASH!       │              │             │            │
    ╳               │              │             │            │
                    │              │             │            │
                    │              │ Cmd timestamp│            │
                    │              │ check:      │            │
                    │              │ stale >     │            │
                    │              │ 100ms       │            │
                    │              │             │            │
                    │              │ disableAll  │            │
                    │              │ Motors()    │            │
                    │              ├────────────►│            │
                    │              │ 0xFD [6]   │            │
                    │              │ for each   │            │
                    │              │             │ CAN frame  │
                    │              │             ├───────────►│
                    │              │             │            │
                    │              │ set:        │            │
                    │              │ emergency   │            │
                    │              │ _stop=true  │            │
                    │              │ controllerAlive│          │
                    │              │ = false     │            │
                    │              │             │            │
                    │              │ stop()      │            │
                    │              │ (thread     │            │
                    │              │  pauses)    │            │
                    │              │             │            │
                    │ robotPeriodic():           │            │
                    │ check controllerAlive      │            │
                    │              │             │            │
                    │ SHM remains  │             │            │
                    │ mapped (Robot│             │            │
                    │ still owns)  │             │            │
                    │              │             │            │
                    │ Monitor for  │             │            │
                    │ fresh cmd    │             │            │
                    │ timestamp    │             │            │
                    │              │             │            │
    │ RESTART       │              │             │            │
    │               │              │             │            │
    │ shm_open      │              │             │            │
    │ (O_RDWR,      │              │             │            │
    │  no O_CREAT)  │              │             │            │
    │ mmap()        │              │             │            │
    │ write cmds    │              │             │            │
    │               │              │             │            │
                    │ Detect fresh │             │            │
                    │ cmd timestamp│             │            │
                    │ → SUCCESS    │             │            │
                    │ emergency    │             │            │
                    │ _stop=false  │             │            │
                    │              │             │            │
                    │ setSharedMem │             │            │
                    ├─────────────►│             │            │
                    │              │ start()     │            │
                    │              │ (thread     │            │
                    │              │  resumes)   │            │
                    │              │             │            │
                    │              │ Normal      │            │
                    │              │ operation   │            │
                    │              │ resumes     │            │
```

---

## Safety Layer Summary

| Layer | Actor | Failure Detected | Response | Mechanism |
|:-----:|-------|-----------------|----------|-----------|
| **1** | Motor firmware | Overvoltage, overcurrent, overtemp, overload [2][3] | Reject commands, report error code in D[0] [6] | Hardware state machine, independent of software |
| **2** | Motor timeout | Communication loss (0x0D) [2] | Flag motor unresponsive after 500 ms [3] | Timestamp tracking per motor |
| **3** | Leg subsystem | SHM magic invalid, cmd timestamp stale (>100ms), controller_emergency_stop set | Disable all motors (0xFD) [6], set emergency_stop=true, flag controllerAlive=false | Checks in controllerPeriodic() every 5 ms [2] |
| **4** | DriverStation | Operator presses disable button [2] | Send 0xFD via async message to leg subsystem [2] | EventLoop + BooleanEvent edge detection [2] |
| **5** | Mode transition | Enter Disabled mode [2] | Send 0xFD to all motors, stop controller | FRC-pattern mode callbacks [2] |
| **6** | Robot main loop | controllerAlive=false detected | Monitor for fresh cmd timestamp (controller reattaches to Robot-owned SHM), shm_unlink() on shutdown | robotPeriodic() at 50 Hz [2] |
| **7** | Emergency stop | Combined safety trigger | Disable all 10 motors, set SHM emergency flag, require manual recovery | All layers cooperate |

The motor firmware is the strongest safety layer because it operates independently of all software — it will reject dangerous commands and detect communication loss regardless of what the controller, operator, or actuator process does [2][3]. MIT control parameters are bounded by motor-type limits (DM8009: ±12.5 rad position, ±45 rad/s velocity, ±54 Nm torque) [2], and the `double_to_uint` encoding function clamps values to these bounds before transmission [6].