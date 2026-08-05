
# Robot Startup, Shutdown, Emergency Stop Procedures and Safety Architecture

## System Startup Procedure

The Kuavo controller coordinates 10 Damiao servo motors across two legs and an LPMS-IG1 IMU at hard real-time rates (20 ms main loop, 5 ms control inner loop), preventing unsafe motor states, communication timeouts, and control divergence that could damage hardware or cause the robot to fall [3].

### Startup Sequence

```
POWER ON
    │
    ▼
Phase 1: PROCESS INITIALIZATION
    │
    ├──► main() → StartRobot<Robot>()
    │    C++20, CMake 3.12+ [3]
    │
    ├──► Config::instance() loads config/config.yaml [3]
    │    All network addresses, port mappings, motor types,
    │    leg assignments, IMU parameters, logger settings [3]
    │
    ├──► Hardware vs simulation determined by UDP target IPs
    │    (192.168.4.x for hardware, 127.0.0.1 for simulation) [3]
    │
    ▼
Phase 2: FRAMEWORK CONSTRUCTION
    │
    ├──► Robot::Robot() constructor
    │    TimedRobot default period: 20 ms (50 Hz) [2]
    │
    ├──► Legged subsystem construction (×2)
    │    ControlledSubsystemBase<7, 2, 5> [2]
    │    Spawns dedicated pthread per subsystem instance [2]
    │    Left leg: base_id=1, motors 1-5 [2]
    │    Right leg: base_id=6, motors 6-10 (disabled) [2]
    │
    ├──► Motor construction (×5 per leg)
    │    Device IDs assigned from config [2]
    │    Send CAN ID = device_id [3]
    │    Receive CAN ID = device_id + 0x10 [3]
    │    Initial state: DISABLED (0x00) [2]
    │
    ├──► IMU subsystem construction
    │    ImuReader pthread spawned [2]
    │    Blocking UDP socket on port 8887 [2]
    │    epoll-based CAN frame reception [2]
    │
    ├──► UdpServer singletons created (×2)
    │    Server 0: left leg, ports 8887/8886 [3]
    │    Server 1: right leg, ports 8889/8888 [3]
    │    Port formula: localPort = base + server_id × 2 [2]
    │
    ├──► DriverStation + EventLoop initialized
    │    UDP receiver for Xbox controller [2]
    │    BooleanEvent edge detection [2]
    │    Button mapping: 1=enable, 2=disable, 3=reboot, 4=async [2]
    │
    ├──► MqttClient initialized
    │    libwebsockets 4.5.8 [3]
    │    Broker: 127.0.0.1:1883 [3]
    │    Username/password auth [2]
    │    Non-blocking with automatic reconnection [2]
    │
    ├──► POSIX shared memory attached (consumer)
    │    mercury_shm.h IPC bridge to Mercury Controller [3]
    │    DynaCoRE at /usr/local/include/DynaCoRE [3]
    │    NOTE: Not yet active in main control path [2]
    │
    ▼
Phase 3: MODE INITIALIZATION
    │
    ├──► Robot enters DISABLED mode (power-up default) [2]
    │    Operating modes follow FRC pattern [2][3]:
    │    init() and periodic() callbacks per mode
    │
    ├──► disabledInit() called [2]
    │    All motors remain in DISABLED (0x00) state [2]
    │    No MIT commands dispatched
    │    Controller not running
    │
    ├──► TimedRobot Notifier timer starts [2]
    │    loopFunc() chain begins:
    │    Notifier → loopFunc() → refreshData()
    │    → mode switch → robotPeriodic() [2]
    │
    ├──► robotPeriodic() begins executing at 50 Hz [2]
    │    m_loop.poll() — button events [2]
    │    Telemetry collection (inactive — no motor data)
    │    Subsystem periodic dispatch [2]
    │
    ├──► ImuReader begins receiving CAN frames [2]
    │    8 sequential frames 0x514-0x51B per cycle [2]
    │    16 float32 values parsed [2]
    │    7D state vector updated (mutex-protected) [2]
    │
    ▼
Phase 4: OPERATOR ACTIVATION
    │
    ├──► Operator selects TELEOP or AUTONOMOUS mode
    │    via DriverStation control word [2]
    │
    ├──► teleopInit() or autonomousInit() called [2]
    │    Mode transition to active operation
    │
    ├──► Operator presses Button 1 (enable left leg) [2]
    │    BooleanEvent rising edge detection [2]
    │    EventLoop callback → Legged::onMessage(MSG_ENABLE) [2]
    │    Async FIFO message queue, poll()-based [2]
    │
    ├──► For each of 5 motors per leg:
    │    Motor::sendCommand(0xFC) [6]
    │    Command data: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC} [6]
    │    CAN ID = send_can_id [6]
    │    Via UdpServer → UDP → Motor HW
    │
    ├──► Motors transition: DISABLED (0x00) → ENABLED (0x01) [2]
    │    Confirmed by feedback frame D[0] upper nibble [6]
    │
    ├──► controllerPeriodic() begins active MIT dispatch [2]
    │    200 Hz (5 ms) inner control loop [2]
    │    Controller::calculate() — LQR with precomputed gains [2]
    │    Motor::setMitControl() for each motor [2]
    │    MIT encoding via double_to_uint() [6]
    │
    └──► SYSTEM OPERATIONAL
         Telemetry publishing at 50 Hz [2]
         RobotStatusWire ~890 bytes, magic 0x4B564155 [2]
         JSON SenML data logs [2]
```

---

## Normal Shutdown Procedure

```
OPERATOR INITIATES SHUTDOWN
    │
    ▼
Phase 1: MODE TRANSITION TO DISABLED
    │
    ├──► Operator selects DISABLED mode via DriverStation [2]
    │    OR presses Button 2 (disable left leg) [2]
    │
    ├──► BooleanEvent rising edge → EventLoop callback [2]
    │    → Legged::onMessage(MSG_DISABLE) [2]
    │    Async FIFO, poll()-based notification [2]
    │
    ├──► For each of 5 motors per leg:
    │    Motor::sendCommand(0xFD) [6]
    │    Command data: {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD} [6]
    │    Via UdpServer → UDP → Motor HW
    │
    ├──► Motors transition: ENABLED (0x01) → DISABLED (0x00) [2]
    │
    ├──► controllerPeriodic() stops MIT command dispatch [2]
    │    Thread continues running but does not send commands
    │    Still monitors motor feedback for error detection
    │
    ├──► disabledInit() called [2]
    │    Mode-specific initialization for disabled state
    │
    ▼
Phase 2: TELEMETRY SHUTDOWN
    │
    ├──► RobotStatus continues publishing status = DISABLED [2]
    │    Binary RobotStatusWire (~890 bytes) [2]
    │    JSON SenML logs [2]
    │    MQTT connection maintained [2]
    │
    ▼
Phase 3: PROCESS TERMINATION
    │
    ├──► Robot destructor called
    │    TimedRobot Notifier stopped
    │
    ├──► Legged subsystem destructors
    │    Subsystem pthreads terminated [2]
    │    Motor objects destroyed
    │
    ├──► ImuReader pthread terminated [2]
    │    UDP socket closed
    │
    ├──► UdpServer singletons destroyed [2]
    │    UDP sockets closed
    │
    ├──► MqttClient disconnected [2]
    │    libwebsockets cleanup [3]
    │
    ├──► POSIX shared memory detached [3]
    │    munmap() called
    │
    └──► PROCESS EXIT
```

---

## Emergency Stop Procedure

The motor safety state machine is the last line of defense — it operates independently of all software and will reject dangerous commands regardless of controller or operator actions [3].

```
EMERGENCY STOP TRIGGERED
    │
    │ Triggers:
    │ • Motor error detected (0x08-0x0E) [2][3]
    │ • Communication loss > 500 ms timeout [3]
    │ • Mercury Controller crash (SHM heartbeat stale)
    │ • Operator emergency action
    │ • Software assertion failure
    │
    ▼
Phase 1: IMMEDIATE MOTOR DISABLE
    │
    ├──► disableAllMotors() called
    │    For each of 10 motors (5 per leg):
    │    Motor::sendCommand(0xFD) [6]
    │    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD} [6]
    │
    ├──► Motors transition to DISABLED (0x00) [2]
    │    All MIT commands cease immediately
    │    Motor three-phase terminal voltage → 50% modulation
    │
    ├──► controllerPeriodic() stops dispatching [2]
    │    Thread remains running for monitoring only
    │
    ▼
Phase 2: ERROR REPORTING
    │
    ├──► Motor error code logged from feedback D[0] [6]:
    │    0x08 = Overvoltage [2]
    │    0x09 = Undervoltage [2]
    │    0x0A = Overcurrent [2]
    │    0x0B = MOS overtemperature [2]
    │    0x0C = Coil overtemperature [2]
    │    0x0D = Communication loss (500 ms timeout) [2][3]
    │    0x0E = Overload [2]
    │
    ├──► Telemetry publishes error state [2]
    │    RobotStatusWire with error flags [2]
    │
    ▼
Phase 3: RECOVERY (MANUAL)
    │
    ├──► Operator must clear error:
    │    Motor::sendCommand(0xFB) [6]
    │    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB} [6]
    │    Motor transitions: ERROR → DISABLED (0x00)
    │
    ├──► Operator transitions to DISABLED mode [2]
    │    Verify all motors report DISABLED (0x00)
    │
    ├──► Operator re-enables via Button 1 [2]
    │    Motor::sendCommand(0xFC) → ENABLED (0x01) [6]
    │
    └──► Normal operation resumes
```

---

## Complete Module State Transition Map

### Motor State Machine (per motor)

```
                    ┌────────────────────────┐
                    │                        │
                    │   POWER-ON DEFAULT     │
                    │   (not yet addressed)  │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ First CAN frame received
                                │ or motor firmware ready
                                ▼
                    ┌────────────────────────┐
                    │                        │
           ┌───────│   DISABLED (0x00)      │◄────────────────┐
           │       │                        │                 │
           │       │   No control accepted  │                 │
           │       │   Three-phase at 50%   │                 │
           │       │   modulation [2]       │                 │
           │       │                        │                 │
           │       └───────────┬────────────┘                 │
           │                   │                              │
           │                   │ sendCommand(0xFC) [6]        │
           │                   │ {0xFF×7, 0xFC}               │
           │                   ▼                              │
           │       ┌────────────────────────┐                 │
           │       │                        │                 │
           │       │   ENABLED (0x01)       │─────────────────┤
           │       │                        │  0xFD [6]       │
           │       │   MIT commands active  │  {0xFF×7, 0xFD} │
           │       │   Feedback monitored   │                 │
           │       │   500 ms timeout [3]   │                 │
           │       │                        │                 │
           │       │   MIT bounds [2]:      │                 │
           │       │   pos: ±12.5 rad       │                 │
           │       │   vel: ±45.0 rad/s     │                 │
           │       │   tau: ±54.0 Nm        │                 │
           │       │   Kp: [0, 500]         │                 │
           │       │   Kd: [0, 5]           │                 │
           │       │                        │                 │
           │       └───────────┬────────────┘                 │
           │                   │                              │
           │                   │ Error detected in            │
           │                   │ feedback D[0] upper nibble   │
           │                   │ [6]                          │
           │                   ▼                              │
           │       ┌────────────────────────┐                 │
           │       │                        │                 │
           │       │   ERROR STATE          │─────────────────┘
           │       │                        │  0xFB [6] then
           │       │   0x08: Overvoltage    │  re-enable 0xFC
           │       │   0x09: Undervoltage   │
           │       │   0x0A: Overcurrent    │
           │       │   0x0B: MOS overtemp   │
           │       │   0x0C: Coil overtemp  │
           │       │   0x0D: Comm loss [3]  │
           │       │   0x0E: Overload       │
           │       │   [2][3]               │
           │       │                        │
           │       │   Control suspended    │
           │       │   Must be cleared      │
           │       │   with 0xFB [6]        │
           │       │                        │
           │       └────────────────────────┘
           │
           │ Zero-position calibration
           │ sendCommand(0xFE) [6]
           │ {0xFF×7, 0xFE}
           ▼
    ┌────────────────────────┐
    │   ZERO CALIBRATED      │
    │   Returns to DISABLED  │
    │   with new zero pos    │
    └────────────────────────┘
```

### Robot Operating Mode State Machine

Operating modes follow the FRC pattern with `init()` and `periodic()` callbacks per mode [2][3].

```
                    ┌────────────────────────┐
                    │                        │
                    │   STARTUP              │
                    │   (process init)       │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ Construction complete
                                │ TimedRobot Notifier starts [2]
                                ▼
                    ┌────────────────────────┐
                    │                        │
         ┌─────────│   DISABLED             │◄────────────────┐
         │         │   (power-up default)   │                 │
         │         │                        │                 │
         │         │   disabledInit() [2]   │                 │
         │         │   disabledPeriodic()   │                 │
         │         │   Motors: 0x00         │                 │
         │         │   Controller: OFF      │                 │
         │         │   Telemetry: minimal   │                 │
         │         │                        │                 │
         │         └──────┬───────┬─────────┘                 │
         │                │       │                           │
         │    DS: auto    │       │ DS: teleop                │
         │                ▼       ▼                           │
         │  ┌──────────────┐  ┌──────────────┐               │
         │  │              │  │              │               │
         │  │ AUTONOMOUS   │  │ TELEOPERATION│               │
         │  │              │  │              │               │
         │  │ autoInit()   │  │ teleopInit() │               │
         │  │ autoPeriodic │  │ teleopPeriod │               │
         │  │ [2]          │  │ [2]          │               │
         │  │              │  │              │               │
         │  │ Motors: 0xFC │  │ Motors: 0xFC │               │
         │  │ (after       │  │ (after       │               │
         │  │  operator    │  │  Button 1)   │               │
         │  │  enable)     │  │  [2]         │               │
         │  │              │  │              │               │
         │  │ Controller:  │  │ Controller:  │               │
         │  │ ON (LQR [2]) │  │ ON (LQR)    │               │
         │  │              │  │              │               │
         │  │ Trajectory:  │  │ Joystick:    │               │
         │  │ replay       │  │ active [2]   │               │
         │  │              │  │              │               │
         │  └──────┬───────┘  └──────┬───────┘               │
         │         │                 │                        │
         │         │ DS: disable     │ DS: disable            │
         │         │ or error        │ or Button 2 [2]        │
         │         │                 │ or error               │
         │         └────────┬────────┘                        │
         │                  │                                 │
         │                  ├─────────────────────────────────┘
         │                  │ Normal disable
         │                  │
         │                  │ Error or safety trigger
         │                  ▼
         │  ┌────────────────────────┐
         │  │                        │
         │  │   EMERGENCY STOP       │
         │  │                        │
         │  │   Motors: 0xFD all [6] │
         │  │   Controller: OFF      │
         │  │   Error logged         │
         │  │   Recovery: manual     │
         │  │                        │
         │  │   Requires:            │
         │  │   1. Error clear 0xFB  │
         │  │      [6]               │
         │  │   2. Mode → DISABLED   │
         │  │   3. Re-enable 0xFC    │
         │  │      [6]               │
         │  │                        │
         │  └───────────┬────────────┘
         │              │
         │              │ Manual recovery complete
         └──────────────┘
```

### Legged Subsystem State Machine

`ControlledSubsystemBase` spawns a dedicated pthread per subsystem instance for non-blocking message processing [2].

```
                    ┌────────────────────────┐
                    │                        │
                    │   CONSTRUCTED          │
                    │                        │
                    │   pthread created [2]  │
                    │   Motors allocated ×5  │
                    │   Config loaded [3]    │
                    │   Thread: running but  │
                    │   waiting for mode     │
                    │   transition           │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ Mode → TELEOP or AUTONOMOUS [2]
                                │ + Button 1 (enable) [2]
                                │ → onMessage(MSG_ENABLE) via
                                │   async FIFO + poll() [2]
                                ▼
                    ┌────────────────────────┐
                    │                        │
                    │   ENABLED              │
                    │   (normal operation)   │
                    │                        │
                    │   Motors: 0xFC [6]     │
                    │   controllerPeriodic() │
                    │   running at 5 ms [2]  │
                    │   Controller::         │
                    │   calculate() [2]      │
                    │   MIT dispatch active  │
                    │   Feedback monitored   │
                    │   500 ms timeout [3]   │
                    │                        │
                    └──────┬────────┬────────┘
                           │        │
                Button 2   │        │ Motor error 0x08-0x0E [2]
                [2] or     │        │ or comm loss 0x0D [3]
                mode →     │        │ or SHM stale
                DISABLED   │        │
                           ▼        ▼
                    ┌──────────┐  ┌──────────────┐
                    │          │  │              │
                    │ DISABLED │  │ ERROR        │
                    │          │  │              │
                    │ Motors:  │  │ Motors: 0xFD │
                    │ 0xFD [6] │  │ [6]          │
                    │ Thread:  │  │ Error clear: │
                    │ running  │  │ 0xFB [6]     │
                    │ (waiting │  │ required     │
                    │  for re- │  │ before       │
                    │  enable) │  │ re-enable    │
                    │          │  │              │
                    └──────────┘  └──────────────┘
```

### IMU State Machine

The `ImuReader` runs as a dedicated pthread with a blocking UDP socket on port 8887 [2].

```
                    ┌────────────────────────┐
                    │                        │
                    │   INITIALIZED          │
                    │                        │
                    │   pthread spawned [2]  │
                    │   UDP socket bound     │
                    │   port 8887 [2]        │
                    │   epoll configured [2] │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ First CAN frames received
                                │ from LPMS-IG1 hardware [2]
                                ▼
                    ┌────────────────────────┐
                    │                        │
                    │   RECEIVING            │
                    │   (normal operation)   │
                    │                        │
                    │   8 CAN frames per     │
                    │   cycle (0x514-0x51B)  │
                    │   [2]                  │
                    │                        │
                    │   16 float32 parsed    │
                    │   per cycle [2]        │
                    │                        │
                    │   7D state vector      │
                    │   updated [2]:         │
                    │   [eulerX,Y,Z,         │
                    │    quatW,X,Y,Z]        │
                    │                        │
                    │   Mutex-protected      │
                    │   getStates() [2]      │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ Hardware fault
                                │ Cable disconnect
                                │ No frames received
                                ▼
                    ┌────────────────────────┐
                    │                        │
                    │   STALE                │
                    │   (NO TIMEOUT          │
                    │    IMPLEMENTED) [2]    │
                    │                        │
                    │   getStates() returns  │
                    │   last-known state     │
                    │   indefinitely [2]     │
                    │                        │
                    │   SAFETY GAP:          │
                    │   Unlike motor 500 ms  │
                    │   timeout [3], IMU     │
                    │   has no equivalent    │
                    │   staleness detection  │
                    │                        │
                    └────────────────────────┘
```

### Shared Memory Connection State Machine

POSIX shared memory IPC bridge to Mercury dynamics controller (`mercury_shm.h`) [3]. DynaCoRE integration is referenced but not yet active in the main control path [2].

```
                    ┌────────────────────────┐
                    │                        │
                    │   NOT ATTACHED         │
                    │   (process startup)    │
                    │                        │
                    │   m_shm = nullptr      │
                    │   Mercury Controller   │
                    │   may not be running   │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ Robot::robotInit():
                                │ shm_open("/mercury_robot_ipc")
                                │ + mmap()
                                ▼
                    ┌────────────────────────┐
                    │                        │
                    │   ATTACHED             │
                    │   (SHM mapped)         │
                    │                        │
                    │   Validate:            │
                    │   • mmap != MAP_FAILED │
                    │   • magic field valid  │
                    │   • version compatible │
                    │                        │
                    │   NOTE: Not yet active │
                    │   in main control      │
                    │   path [2]             │
                    │                        │
                    └──────┬────────┬────────┘
                           │        │
                    valid  │        │ invalid / producer crash
                           ▼        ▼
                    ┌──────────┐  ┌──────────────┐
                    │          │  │              │
                    │ ACTIVE   │  │ DISCONNECTED │
                    │          │  │              │
                    │ Commands │  │ Motors: 0xFD │
                    │ read     │  │ munmap()     │
                    │ Sensor   │  │ m_shm=null   │
                    │ data     │  │ Retry loop   │
                    │ written  │  │ in robot-    │
                    │          │  │ Periodic()   │
                    └──────────┘  └──────────────┘
```

### Telemetry State Machine

Telemetry is published every main loop iteration (20 ms / 50 Hz) [2].

```
                    ┌────────────────────────┐
                    │                        │
                    │   INITIALIZED          │
                    │                        │
                    │   MqttClient created   │
                    │   lws 4.5.8 [3]        │
                    │   Broker: 127.0.0.1    │
                    │   :1883 [3]            │
                    │   Auth configured [2]  │
                    │                        │
                    └───────────┬────────────┘
                                │
                                │ MQTT CONNECT
                                ▼
                    ┌────────────────────────┐
                    │                        │
                    │   CONNECTED            │
                    │   (publishing)         │
                    │                        │
                    │   Every 20 ms [2]:     │
                    │   • collect() from     │
                    │     all subsystems [2] │
                    │   • RobotStatusWire    │
                    │     ~890 bytes [2]     │
                    │     magic 0x4B564155   │
                    │   • JSON SenML logs [2]│
                    │                        │
                    └──────┬────────┬────────┘
                           │        │
                    OK     │        │ Broker disconnect
                           ▼        ▼
                    ┌──────────┐  ┌──────────────┐
                    │          │  │              │
                    │ PUBLISH  │  │ RECONNECTING │
                    │ (normal) │  │              │
                    │          │  │ lws auto-    │
                    │          │  │ reconnect [2]│
                    │          │  │              │
                    └──────────┘  └──────────────┘
```

---

## Safety Architecture: Fail-Safe and Fault-Tolerant Processing

### Fail-Safe Layers

The Kuavo safety architecture uses **defense-in-depth** with multiple independent layers. Each layer can independently bring the system to a safe state [3].

```
SAFETY LAYER HIERARCHY (inner = strongest):

    ╔═══════════════════════════════════════════════════╗
    ║  Layer 1: MOTOR FIRMWARE (Hardware)               ║
    ║  Independent of all software [3]                  ║
    ║  Automatic error detection: 0x08-0x0E [2]         ║
    ║  500 ms comm loss timeout [3]                     ║
    ║  Rejects dangerous commands via limits [2]        ║
    ║                                                   ║
    ║  ┌───────────────────────────────────────────┐    ║
    ║  │  Layer 2: PARAMETER CLAMPING (Encoding)   │    ║
    ║  │  double_to_uint() clamps to motor bounds [6]│   ║
    ║  │  pos: ±PMAX, vel: ±VMAX, tau: ±TMAX [2]  │    ║
    ║  │  Kp: [0,500], Kd: [0,5] [6]              │    ║
    ║  │                                           │    ║
    ║  │  ┌───────────────────────────────────┐    │    ║
    ║  │  │  Layer 3: CONTROL LOOP SAFETY     │    │    ║
    ║  │  │  controllerPeriodic() 5 ms [2]    │    │    ║
    ║  │  │  Motor responsiveness 500 ms [3]  │    │    ║
    ║  │  │  Error code monitoring D[0] [6]   │    │    ║
    ║  │  │  SHM heartbeat check              │    ║    ║
    ║  │  │                                   │    │    ║
    ║  │  │  ┌───────────────────────────┐    │    │    ║
    ║  │  │  │  Layer 4: OPERATOR        │    │    │    ║
    ║  │  │  │  Button 2 = disable [2]   │    │    │    ║
    ║  │  │  │  Mode transitions [2]     │    │    │    ║
    ║  │  │  │  EventLoop edge detect [2]│    │    │    ║
    ║  │  │  └───────────────────────────┘    │    │    ║
    ║  │  └───────────────────────────────────┘    │    ║
    ║  └───────────────────────────────────────────┘    ║
    ╚═══════════════════════════════════════════════════╝
```

### Fail-Safe Mechanisms

| Mechanism | Layer | Trigger | Safe State | Recovery |
|-----------|:-----:|---------|:----------:|----------|
| Motor error state machine | 1 (HW) | Overvoltage, overcurrent, overtemp, overload [2] | Motor suspends control | Clear error (0xFB) + re-enable (0xFC) [6] |
| Communication loss timeout | 1 (HW) | No CAN frame for 500 ms [3] | Motor flags comm loss (0x0D) [2] | Re-establish communication + re-enable |
| MIT parameter clamping | 2 (SW) | Command exceeds motor bounds [2] | Value clamped via `limit_min_max()` [6] | Automatic — no recovery needed |
| Motor disable command | 3 (SW) | Error detected by control loop | All motors sent 0xFD [6] | Operator re-enables via Button 1 [2] |
| Operator emergency disable | 4 (Human) | Button 2 pressed [2] | All motors sent 0xFD [6] | Operator re-enables via Button 1 [2] |
| Mode transition to Disabled | 4 (Human) | DriverStation mode change [2] | All motors sent 0xFD [6] | Operator selects Teleop/Auto [2] |

### Fault-Tolerant Processing

| Fault | Detection | Tolerance | Action |
|-------|-----------|:---------:|--------|
| Single motor failure | Error code in feedback D[0] [6] | Partial — other motors continue | Disable failed motor only; log error [2] |
| CAN bus interruption | 500 ms timeout per motor [3] | Graceful degradation | Flag unresponsive motors; continue with responsive ones [2] |
| IMU data loss | **No timeout implemented** [2] | **None — uses stale data** | Controller continues with last-known state (SAFETY GAP) [2] |
| Mercury Controller crash | SHM heartbeat stale | Graceful shutdown | Disable all motors; enter reconnection loop |
| MQTT broker disconnect | libwebsockets auto-reconnect [2] | Full tolerance | Telemetry queued; reconnect automatically [2] |
| DriverStation disconnect | No explicit timeout [2] | **None — last mode persists** | Last mode and button states remain active (SAFETY GAP) |
| UdpServer socket error | Socket error return | None | Motor communication lost; 500 ms timeout fires [3] |

### Identified Safety Gaps

Based on the architecture documentation, these safety gaps exist in the current system:

| Gap | Risk | Mitigation Status |
|-----|------|:------------------:|
| IMU has no staleness timeout — `getStates()` returns last-known state indefinitely [2] | Controller operates on stale orientation data → robot may fall | Not implemented [2] |
| DriverStation has no disconnect timeout — last mode persists [2] | Robot continues in active mode after operator loses connection | Not implemented |
| Mercury Controller SHM integration not yet active in main control path [2] | Cross-process commands not yet operational | Not implemented [2] |
| Right leg subsystem disabled in code [2] | Only single-leg operation verified | Intentional for current phase [2] |
| Emergency stop GPIO described in docs but not visible in software [2] | No hardware-level emergency stop integration | Not implemented |
| Recovery protocol after emergency stop undefined | No formal sequence for safe restart | Not defined |

### MIT Encoding as Safety Layer

The `double_to_uint()` function provides software-level bounds enforcement — even if the Mercury Controller produces a command outside the motor's safe range, the encoding layer clamps it before the CAN frame is sent [6]:

```cpp
uint16_t CanPacketEncoder::double_to_uint(
    double x, double x_min, double x_max, int bits) {
    x = limit_min_max(x, x_min, x_max);  // CLAMP to safe range
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}
```
[6]

The `limit_min_max()` function enforces hard bounds [6]:
```cpp
double CanPacketEncoder::limit_min_max(double x, double min, double max) {
    return std::max(min, std::min(x, max));
}
```
[6]

This ensures that regardless of what the controller computes, the motor never receives a command outside its physical limits (DM8009: ±12.5 rad position, ±45 rad/s velocity, ±54 Nm torque) [2].