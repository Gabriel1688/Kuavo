
# Thread Settings in the Kuavo System

Based on the architecture documentation, the Kuavo controller runs as a **single C++20 process** on an embedded Linux SBC with multiple threads [4]. Here are the thread settings:

## Main Loop Thread

The `TimedRobot` default period is **20 ms (50 Hz)**, with inner control loops running at configurable faster rates [2]. The main loop uses a `Notifier` timer that fires periodically, executing the `loopFunc()` chain: `Notifier fires → IterativeRobotBase::loopFunc() → refreshData() → mode switch → robotPeriodic()` [4]. `ControlledSubsystemBase` spawns a **dedicated pthread per subsystem instance** for non-blocking message processing [2].

## Subsystem Inner Loop Threads

Subsystem inner loops run at **200 Hz (5 ms)** [3]. The `Legged` subsystem uses `controllerPeriodic()` as its inner control loop at this rate, handling motor state queries, LQR computation, and MIT command dispatch [2]. Subsystem threads use **async FIFO message queues with `poll()`-based notification** to avoid blocking the main robot loop [2].

## IMU Reader Thread

The `ImuReader` runs as a **dedicated pthread** with a blocking UDP socket on port 8887 [2]. It uses **epoll** for non-blocking CAN frame reception, parsing 8 sequential CAN frames (IDs 0x514 through 0x51B) per measurement cycle at approximately **500 Hz** [2][4].

## UdpServer Threads

Two `UdpServer` singleton instances manage CAN-over-UDP communication — one per leg [2][4]. Motor device IDs 1-5 route to UDP server 0 (left leg, ports 8887/8886), and IDs 6-10 route to UDP server 1 (right leg, ports 8889/8888) [2][3].

## MQTT / Telemetry Thread

The `MqttClient` uses **libwebsockets 4.5.8** for MQTT publication [3]. It publishes binary `RobotStatusWire` packets (~890 bytes with magic number 0x4B564155) and JSON SenML data logs every main loop iteration at **50 Hz** [2].

## DriverStation and EventLoop Threads

The `DriverStation` is a UDP receiver that parses control words and joystick axes [4]. The `EventLoop` with `BooleanEvent` handles button input dispatch with rising/falling edge detection [2][3].

## Synchronization

All motor and IMU state access is **mutex-protected** [3]. The architecture documentation notes this as an open question — "Shared memory architecture (mentioned in architecture doc) is not yet implemented — currently using mutex-protected state" [4]. Subsystem threads use `poll()`-based event notification to avoid busy-waiting [3].

## Summary Table

| Thread | Rate | Mechanism | Scheduling |
|--------|:----:|-----------|------------|
| Main Robot Loop | 50 Hz (20 ms) | `TimedRobot` Notifier timer [2] | Periodic |
| Leg Subsystem (×2) | 200 Hz (5 ms) | `ControlledSubsystemBase` pthread [2] | `poll()`-based async FIFO |
| IMU Reader | ~500 Hz | Dedicated pthread, epoll [2][4] | Blocking UDP + epoll |
| UdpServer (×2) | Async | Singleton, callback-based [2][4] | Event-driven |
| MqttClient | 50 Hz | libwebsockets event loop [3] | Periodic publish |
| DriverStation | Event-driven | UDP receiver [4] | Event-driven |
| EventLoop | Polled | `BooleanEvent` edge detection [2] | Polled in main loop |

All threads use **POSIX threads with real-time extensions** (`pthread`, `rt`) [3], and the right leg subsystem is currently "instantiated but disabled in code; only the left leg runs the active control loop" [2][4].


# Thread Scheduling, Priority, and Stack Settings — Current vs Proposed

## What the Codebase Currently Defines

The current Kuavo codebase does **not** specify explicit `SCHED_FIFO`, `SCHED_RR` priorities, or stack sizes for any thread. All threads run at **OS-default settings** (`SCHED_OTHER`, default stack ~8MB) [3].

The system uses "POSIX threads + real-time extensions (pthread, rt)" [3], and "all motor and IMU state access is mutex-protected" with "subsystem threads using `poll()`-based event notification to avoid busy-waiting" [3]. The `TimedRobot` default period is 20 ms (50 Hz) with inner control loops at 200 Hz (5 ms) [2][3].

## What Was Proposed in Our Previous Discussion

The proposed thread settings from our architecture revision discussion were:

| # | Thread | Rate | Policy | Priority | Stack | Rationale |
|:---:|--------|:----:|--------|:--------:|:-----:|-----------|
| 1 | Main Robot Loop | 100Hz (10ms) | `SCHED_FIFO` | **75** | 512KB | Supervisory only — mode management, button events, health monitoring, parameter queries at 10Hz. Below all sensor/actuator threads |
| 2 | Left Leg Subsystem | 400Hz (2.5ms) | `SCHED_FIFO` | **90** | 256KB | Highest — motor safety critical. Reads Mercury commands from SHM, encodes MIT frames [9], sends CAN-over-UDP [7]. Both legs at same priority since they manage independent motor groups on separate UDP sockets [2] |
| 3 | Right Leg Subsystem | 400Hz (2.5ms) | `SCHED_FIFO` | **90** | 256KB | Same as left leg — currently "instantiated but disabled in code" [2] |
| 4 | IMU Reader | 500Hz | `SCHED_FIFO` | **80** | 128KB | Parses 8 sequential CAN frames (0x514-0x51B) per cycle [2][4]. Below leg threads but above main loop |
| 5 | Composer | 400Hz (2.5ms) | `SCHED_FIFO` | **85** | 256KB | Merges 3 per-source staging buffers into one consistent SensorData snapshot. Must run after producers publish but before Mercury Controller reads |
| 6 | MQTT Logger | Network-paced | `SCHED_OTHER` | **nice +10** | 512KB | Lowest — best-effort telemetry. Uses libwebsockets 4.5.8 [3] event loop. Must never preempt any RT thread |
| 7 | DriverStation | Event-driven | `SCHED_OTHER` | **0** | 128KB | Non-RT — parses UDP control words from operator console [2][4] |
| 8 | EventLoop | Polled in main | `SCHED_OTHER` | **0** | 128KB | Non-RT — button edge detection via `BooleanEvent` [2][3] |
| 9 | UdpServer 0 (left) | Async callback | `SCHED_FIFO` | **88** | 128KB | Motor feedback reception for left leg (ports 8887/8886) [2][3]. Below leg threads but above Composer |
| 10 | UdpServer 1 (right) | Async callback | `SCHED_FIFO` | **88** | 128KB | Motor feedback reception for right leg (ports 8889/8888) [2][3] |

## Priority Ordering

```
Priority 99 ─── (kernel watchdog, migration)
    │
    90 ─── Left Leg / Right Leg (motor control, 400Hz)
    │
    88 ─── UdpServer 0 / UdpServer 1 (motor feedback callbacks)
    │
    85 ─── Composer (merge 3 sources, 400Hz)
    │
    80 ─── IMU Reader (500Hz)
    │
    75 ─── Main Robot Loop (supervisory, 100Hz)
    │
    ── SCHED_OTHER boundary ──
    │
     0 ─── DriverStation, EventLoop (event-driven, non-RT)
    │
   +10 ─── MQTT Logger (best-effort, nice +10)
```

## Total Stack Memory Budget

| Thread | Count | Stack Each | Subtotal |
|--------|:-----:|:----------:|:--------:|
| Leg Subsystems | 2 | 256 KB | 512 KB |
| Composer | 1 | 256 KB | 256 KB |
| IMU Reader | 1 | 128 KB | 128 KB |
| Main Robot Loop | 1 | 512 KB | 512 KB |
| UdpServer | 2 | 128 KB | 256 KB |
| DriverStation | 1 | 128 KB | 128 KB |
| EventLoop | 1 | 128 KB | 128 KB |
| MQTT Logger | 1 | 512 KB | 512 KB |
| **Total** | **10** | | **2,432 KB (~2.4 MB)** |

This is significantly reduced from the default 10 × 8 MB = 80 MB that Linux allocates without explicit stack size configuration [3].

## Key Prerequisites

These proposed settings require:

1. **`PREEMPT_RT` kernel patch** — without it, `SCHED_FIFO` at priority 90 can still be delayed by non-preemptible kernel sections [3]
2. **Root privileges or `CAP_SYS_NICE`** — required for `SCHED_FIFO` scheduling:
   ```bash
   sudo setcap cap_sys_nice+ep ./kuavo_robot
   ```
3. **Lock-free double buffers** replacing the current mutex-protected state access [3] — otherwise `SCHED_FIFO` creates priority inversion when the main loop (75) holds a mutex while the leg thread (90) waits

## What Changed from Current to Proposed

| Aspect | Current [2][3] | Proposed |
|--------|:-:|:-:|
| Main loop rate | 20ms / 50Hz | **10ms / 100Hz** |
| Inner control rate | 5ms / 200Hz | **2.5ms / 400Hz** |
| IMU rate | ~500Hz | **500Hz (unchanged)** |
| Motor timeout | 500ms | **100ms** |
| Scheduling policy | `SCHED_OTHER` (all threads) | **`SCHED_FIFO` for RT threads** |
| Stack sizes | Default (~8MB each) | **128-512KB per thread** |
| State access | Mutex-protected [3] | **Lock-free double buffers** |
| Thread count | ~9 | **10** (added Composer, replaced telemetry MqttClient with MQTT Logger) |
| Total stack memory | ~72MB (default) | **~2.4MB** |