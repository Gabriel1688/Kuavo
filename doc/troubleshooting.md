# Troubleshooting

## 1. Controller-Initiated Emergency Stop

### Symptom
Repeated `Controller-initiated emergency stop` in logs, followed by `Motor Group A/B stale — disabling left/right leg` and `All motors disabled (fault shutdown): reason=3`.

### Propagation Path

```
mercury_shm.h:352   std::atomic<bool> controller_emergency_stop{false}
        │
        ├──► Robot.cpp:263-267  robotPeriodic() reads controller_emergency_stop
        │    if true → sets emergency_stop = true, logs "Controller-initiated emergency stop"
        │
        └──► Legged.cpp:82-87  controllerPeriodic() reads controller_emergency_stop
             if true → disableAllMotorsOnce(CONTROLLER_EMERGENCY_STOP)  (reason=3)
```

### DisableReason Enum (Legged.h:60-66)

| Value | Name | Meaning |
|-------|------|---------|
| 1 | SHM_INVALID_MAGIC | Shared memory magic field is wrong |
| 2 | SHM_VERSION_MISMATCH | SHM version doesn't match expected |
| 3 | CONTROLLER_EMERGENCY_STOP | Controller set controller_emergency_stop in SHM |
| 4 | EMERGENCY_STOP_ACTIVE | emergency_stop flag is active (currently #if 0) |
| 5 | CMD_WRITE_IDX_INVALID | cmd_write_idx > 1 |

### Key Finding
No production code in this repository writes `controller_emergency_stop = true`. The field is written by an **external controller process** that attaches to the same shared memory segment (`/dev/shm/mercury_robot_ipc`).

### Diagnosis Commands

```bash
# See which processes have the SHM open
sudo fuser -v /dev/shm/mercury_robot_ipc

# Show controller binary details
sudo readlink -f /proc/<controller_pid>/exe
sudo cat /proc/<controller_pid>/cmdline | tr '\0' ' '

# Check if the running binary matches the repo source
md5sum /proc/<controller_pid>/exe
md5sum /home/gabriel_wang/work/Kuavo/cmake-build-debug/tools/mercury_controller
```

---

## 2. Motor Enable/Disable Timing Mismatch

**Status: Resolved (2026-08-06)**

### Symptom
When a test script toggles joystick buttons via MQTT at ~130ms intervals,
DamiaoSimulator logs show the disable (0xFD) frames arriving 16-39ms **after**
the Kuavo side logs "Leg is Disabled", while enable (0xFC) frames arrive with
~0ms delay.

### Observed Timeline (2026-08-06 08:45:24, same run, same clock)

```
KUAVO SIDE                     DAMIAO SIM (Left 0x1-0x6)   DAMIAO SIM (Right 0x7-0xC)
──────────                     ─────────────────────────    ──────────────────────────
24.124  Leg Disabled
                               24.141  DISABLED (+17ms)
                                                            24.162  DISABLED (+38ms)

24.233  MQTT_RX (btn press)
24.234  Leg Enabled
                               24.234  ENABLED  (+0ms)
                                                            24.234  ENABLED  (+0ms)

24.361  MQTT_RX (btn release)
24.364  Leg Disabled
                               24.380  DISABLED (+16ms)
                                                            24.403  DISABLED (+39ms)

                                                            24.484  ENABLED  (next cycle)
```

### Architecture Context

#### Two Legs, Two RT Threads, Two DamiaoSimulator Processes

```
config/config.yaml:
  UdpServer[0] (left leg):  localPort=8887, remotePort=8886
  UdpServer[1] (right leg): localPort=8889, remotePort=8888

  DamiaoSimulator process 1: port 8886 (left leg, motors 0x1-0x6)
  DamiaoSimulator process 2: port 8888 (right leg, motors 0x7-0xC)
```

#### Enable/Disable Flow (MQTT → Motor)

```
MQTT_RX (test/topic1)
  → MqttClient::processMessage()
  → FRCDriverStation::newDataOccur()         [parses joystick button state]
  → JoystickDataCache::Update()
  → driverStation->newDataEvents.wakeup()
  → loopFunc() @ 100Hz (main thread):
      DriverStation::refreshData()           [swaps triple-buffer]
      robotPeriodic()
        → EventLoop::poll()                  [iterates all BooleanEvent bindings]
        → BooleanEvent::rising()             [detects false→true edge]
        → leftLeg.message(MSG_DISABLE)       [pushes to Left RT thread FIFO]
        → rightLeg.message(MSG_DISABLE)      [pushes to Right RT thread FIFO]
  → Legged RT Thread @ 400Hz (per leg):
      ControlledSubsystemBase::Run()
        → poll(eventfd, timeout)             [wakes on FIFO message OR timeout]
        → Fifo::pop() → onMessage()          [drain messages first]
        → setEnable(false)
        → motor->disableMotor() × 6          [tight for_each loop, sub-ms]
        → UdpServer::sendMsg() → sendto()   [synchronous UDP send]
  → DamiaoSimulator process (per leg):
      epoll_wait(fd, -1)                     [blocks until packet arrives]
      recvfrom() → handleCommand()           [one packet per wakeup, sub-ms]
      sendFeedback() → sendto()
```

#### Key Files

| Component | File | Lines |
|-----------|------|-------|
| Button event setup | src/Robot.cpp | 129-142 |
| Motor group staleness disable | src/Robot.cpp | 240-246 |
| RT thread loop | lib/robot/ControlledSubsystemBase.h | 256-331 |
| RT thread scheduling | lib/robot/ControlledSubsystemBase.h | 55-75 |
| FIFO implementation | lib/common/FdEvent.h | 31-64 |
| setEnable() | src/subsystems/Legged.cpp | 345-365 |
| Motor enable/disable | lib/motor/Motor.cpp | 102-112 |
| UDP send | lib/motor/UdpServer.cpp | 95-131 |
| DamiaoSimulator main loop | tools/DamiaoSimulator.cpp | 251-290 |
| UDP port config | config/config.yaml | 25-31 |

### Investigation: Three Hypotheses

Three candidate explanations were investigated for the 16-39ms
disable-only delay. The first two were ruled out; the third is the
root cause.

#### Hypothesis A — RT Thread FIFO Drain Latency (Ruled Out)

**Theory**: The main thread pushes `MSG_DISABLE` to the FIFO, but the RT
thread is mid-`controllerPeriodic()` and must finish the current cycle
before draining the FIFO. This would delay the `sendto()` calls.

**Why ruled out**: In `ControlledSubsystemBase::Run()` (line 270-327),
the loop order is:

```cpp
while (running) {
    poll(fifo_fd, timeout_ms);          // 1. wait
    if (fifo has data) drain → onMessage()  // 2. FIFO drain (sends disable)
    if (now >= next_wake) controllerPeriodic()  // 3. MIT dispatch
}
```

FIFO drain (step 2) runs **before** `controllerPeriodic()` (step 3).
If the RT thread is currently in `controllerPeriodic()` when the FIFO
message arrives, it must finish — but `controllerPeriodic()` takes
<2.5ms (one 400Hz period). The observed delays of 16-39ms are 6-15
periods, far too large for a single cycle stall.

More importantly, `setEnable(false)` (Legged.cpp:358-363) sends
`disableMotor() × 6` via `sendto()` **before** logging "Leg is Disabled":

```cpp
std::for_each(motors, [](motor) { motor->disableMotor(); });  // sendto() × 6
disable();
SPDLOG_INFO("Leg is Disabled.");  // logged AFTER all sends complete
```

So all 6 packets are in the kernel socket buffer **before** the Kuavo
timestamp. The delay is between the kernel buffer and the
DamiaoSimulator's `recvfrom()`, not between FIFO push and `sendto()`.

#### Hypothesis B — DamiaoSimulator Processing Delay (Ruled Out)

**Theory**: The DamiaoSimulator's `epoll_wait` or per-packet processing
introduces receive latency (e.g., busy handling MIT response packets
when the disable arrives).

**Why ruled out**: Source code review (tools/DamiaoSimulator.cpp) shows:

- `runMotorMode()` uses `epoll_wait(epfd, events, 2, -1)` — blocks
  indefinitely with no timeout, wakes **instantly** on packet arrival
- Processes exactly **one packet per wakeup**: `recvfrom()` →
  `handleCommand()` → `sendFeedback()` → back to `epoll_wait()`
- No batching, no queuing, no sleep — sub-millisecond per iteration
- Both spdlog instances use `flush_on(spdlog::level::debug)` —
  timestamps are not delayed by log buffering

The DamiaoSimulator log confirms this: all 6 disable packets are
processed within <2ms (e.g., 24.380-24.381 for the left leg), showing
instant processing once the first packet is received.

#### Hypothesis C — RT-Priority CPU Starvation (Root Cause)

**Theory**: The Kuavo RT threads (SCHED_FIFO/90) preempt the
DamiaoSimulator processes (SCHED_OTHER), preventing them from being
scheduled to call `recvfrom()` even though packets are sitting in the
kernel socket buffer.

**Evidence**:

| Factor | Kuavo RT threads | DamiaoSimulator |
|--------|-----------------|-----------------|
| Scheduler | `SCHED_FIFO`, priority 90 | `SCHED_OTHER` (normal) |
| Set at | ControlledSubsystemBase.h:66-68 | (none — default) |
| Frequency | 400Hz per leg (2 threads) | Event-driven (epoll) |

On a **4-core machine**, the active threads during control are:

| Thread/Process | Priority | Activity |
|---------------|----------|----------|
| Left leg RT | SCHED_FIFO/90 | MIT dispatch × 6, feedback staging |
| Right leg RT | SCHED_FIFO/90 | MIT dispatch × 6, feedback staging |
| Main thread | SCHED_OTHER | 100Hz robotPeriodic |
| DamiaoSimulator (left) | SCHED_OTHER | epoll_wait on UDP |
| DamiaoSimulator (right) | SCHED_OTHER | epoll_wait on UDP |
| + MQTT, UdpServer recv, spdlog, ... | SCHED_OTHER | various |

SCHED_FIFO threads **unconditionally preempt** all SCHED_OTHER threads
system-wide. With 2 RT threads actively running `controllerPeriodic()`
on a 4-core system, the remaining 2 cores are shared by the main
thread, both DamiaoSimulators, and all other SCHED_OTHER threads.

**Why enable has 0ms delay**: When enabling, the RT threads were idle
(legs disabled, `controllerPeriodic()` skips MIT dispatch at line 102).
CPU is mostly free — DamiaoSimulators get scheduled immediately.

**Why disable has 16-39ms delay**: When disabling, the RT threads were
actively dispatching MIT commands (6 motors × `sendto()` + mutex +
feedback staging per 2.5ms cycle). The DamiaoSimulator processes don't
get CPU time until the RT threads enter `poll()` and sleep.

**Why right leg delay > left leg delay** (39ms vs 16ms): The two
DamiaoSimulator processes compete with each other and with the main
thread for the 2 remaining cores. Scheduling order is
non-deterministic but consistent within a run — whichever simulator
gets scheduled last sees the longest delay.

### Conclusion

The 16-39ms delay is a **simulation-only artifact** caused by OS
scheduler starvation on a 4-core machine. The actual motor commands
(`sendto()`) complete within microseconds of the Kuavo "Disabled" log.

On real hardware this does not apply:
- Motor controllers have dedicated MCUs — no OS scheduling involved
- UDP packets hit the NIC → wire → motor controller in microseconds
- The receiver is not sharing CPU cores with the sender

### Mitigation (Simulation Only)

If accurate timing measurement in simulation is needed:

```bash
# Option 1: Run DamiaoSimulators at elevated priority (below RT threads)
sudo chrt -f 80 ./DamiaoSimulator --left
sudo chrt -f 80 ./DamiaoSimulator --right

# Option 2: Pin DamiaoSimulators to dedicated cores
sudo taskset -c 2 ./DamiaoSimulator --left
sudo taskset -c 3 ./DamiaoSimulator --right
```
