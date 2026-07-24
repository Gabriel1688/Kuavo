# Robot-Motor Communication: State Feedback Flow Analysis

## Overview

This document describes the complete data flow of motor state feedback in the Kuavo bipedal robot control system, from the physical motor back to the sensor data consumed by the control loop. It explains why MIT control commands use a **fire-and-forget** pattern (no blocking wait for response) and how the asynchronous feedback path works.

---

## Architecture Summary

The motor communication follows an **asynchronous, lock-free** architecture:

- **Legged thread** (SCHED_FIFO/90, 400Hz) sends MIT commands and reads motor state atomically.
- **UdpServer thread** (SCHED_FIFO/88) receives motor response frames and updates atomic state fields.
- **Composer thread** (SCHED_FIFO/85, 400Hz) reads staging buffers and merges into SensorData.
- State feedback has a **one-cycle delay** (~2.5ms), which is standard in servo control loops.

---

## Complete Data Flow

### 1. MIT Command Dispatch (Legged Thread)

**File:** `src/subsystems/Legged.cpp` — `controllerPeriodic()`

```
Legged::controllerPeriodic() @ 400Hz (2.5ms period)
  → For each motor (6 per leg):
      → Motor::setMitControl(mit)
          → Acquires m_transactionMutex (per-motor, brief hold)
          → Encodes MIT command into 8-byte CAN frame
          → sendMessage(df, 8, 0, false)  // reply=false, fire-and-forget
          → Returns immediately — NO prepareWait() / waitResponse()
```

The `reply=false` parameter means no entry is registered in UdpServer's `m_frameIds` pending-reply map. The response will arrive via the **unsolicited path** in `dispatchMessage()`.

### 2. Motor Response Reception (UdpServer Thread)

**File:** `lib/motor/UdpServer.cpp` — `run()` and `dispatchMessage()`

```
UdpServer::run() @ SCHED_FIFO/88
  → epoll_wait() on UDP socket (100ms timeout)
  → recvfrom() receives 8-byte motor response frame
  → dispatchMessage(frame, length)
```

`dispatchMessage()` routes incoming frames through three paths:

| Path | Condition | Action |
|------|-----------|--------|
| **Parameter response** | `FrameId == 0x7ff` AND `data[2] == 0x33 or 0x55` | Store to MotorParamCache (lock-free atomics), return early |
| **Pending reply** | Frame matches registered `m_frameIds` entry | Call Motor::callback(), signal replyEvent, return early |
| **Unsolicited (state feedback)** | No pending reply match | Derive `deviceId = FrameId - 0x10`, call Motor::callback() |

**MIT control responses arrive via the unsolicited path** because `setMitControl()` sends with `reply=false`, so no pending reply is registered.

### 3. State Parsing and Atomic Storage (Motor)

**File:** `lib/motor/Motor.cpp` — `callback()`, `parseMotorStateData()`, `updateState()`

```
Motor::callback(msg, size)
  → parseMotorParamData(data)  // Try param response first — returns invalid for state frames
  → parseMotorStateData(data)  // Parse 8-byte state feedback
      → Decode: status (4 bits), position (16-bit), velocity (12-bit),
                torque (12-bit), MOS temp, rotor temp
      → Returns StateResult { status, position, velocity, torque, t_mos, t_rotor, valid=true }
  → updateState(status, q, dq, tau, tmos, trotor)
      → m_status.store(status, memory_order_release)
      → m_stateQ.store(q, memory_order_release)
      → m_stateDq.store(dq, memory_order_release)
      → m_stateTau.store(tau, memory_order_release)
      → m_stateTmos.store(tmos, memory_order_release)
      → m_stateTrotor.store(trotor, memory_order_release)
      → m_lastUpdateTime.store(now_ns, memory_order_release)
      → notify()  // Skipped: m_requestPending is false for MIT control
```

All state fields are `std::atomic<>` with a compile-time lock-free guarantee:

```cpp
static_assert(std::atomic<double>::is_always_lock_free,
              "std::atomic<double> must be lock-free for real-time use");
```

### 4. State Reads by Control Loop (Legged Thread)

**File:** `src/subsystems/Legged.cpp` — `controllerPeriodic()`, lines after `setMitControl()` loop

```cpp
for (size_t i = 0; i < motors.size(); i++) {
    stageData.joint_jpos[i]  = motors[i]->getPosition();   // m_stateQ.load(acquire)
    stageData.joint_jvel[i]  = motors[i]->getVelocity();   // m_stateDq.load(acquire)
    stageData.jtorque[i]     = motors[i]->getTorque();      // m_stateTau.load(acquire)
    stageData.mos_temperature[i]   = motors[i]->getStateTmos();
    stageData.rotor_temperature[i] = motors[i]->getStateTrotor();
    stageData.motor_status[i] = static_cast<uint8_t>(motors[i]->getState());
}
```

All reads use `std::memory_order_acquire`, paired with the `memory_order_release` stores in `updateState()`.

### 5. Staging Buffer Publication (Legged Thread → Composer)

**File:** `tools/mercury_shm_v2.h` — `SourceDoubleBuffer<MotorGroupStageData>`

```cpp
m_staging->publish(stageData);  // Lock-free double-buffer swap
```

The Composer thread reads via `motor_group_a_.read()` / `motor_group_b_.read()`, which are also lock-free.

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│  Legged::controllerPeriodic() @ SCHED_FIFO/90, 400Hz (2.5ms)      │
│                                                                     │
│  1. Read command from SHM (lock-free atomic loads)                  │
│  2. For each motor:                                                 │
│     setMitControl(mit) → sendMessage(reply=false) → UDP sendto()   │
│     (returns immediately, no blocking wait)                         │
│  3. For each motor:                                                 │
│     Read state via atomic loads: getPosition(), getVelocity(), ...  │
│     (reads latest state — may be from previous cycle's response)    │
│  4. Populate MotorGroupStageData                                    │
│  5. m_staging->publish(stageData) → lock-free double-buffer swap   │
└─────────────┬───────────────────────────────────────┬───────────────┘
              │ UDP command frame                     │ staging buffer
              ▼                                       ▼
┌──────────────────────────┐    ┌─────────────────────────────────────┐
│  Physical Motor          │    │  Composer @ SCHED_FIFO/85, 400Hz   │
│  (DAMIAO MIT protocol)   │    │                                     │
│  Processes command,      │    │  1. motor_group_a_.read() (lock-free)│
│  sends state feedback    │    │  2. motor_group_b_.read() (lock-free)│
└──────────┬───────────────┘    │  3. imu_stage_.read() (lock-free)   │
           │ UDP response       │  4. Merge into SensorData           │
           ▼                    │  5. Publish to composed buffer      │
┌──────────────────────────┐    └─────────────────────────────────────┘
│  UdpServer::run()        │
│  @ SCHED_FIFO/88         │
│                          │
│  epoll_wait() → recvfrom()
│  → dispatchMessage()     │
│    → unsolicited path    │
│    → Motor::callback()   │
│      → parseMotorState() │
│      → updateState()     │
│        → atomic stores   │
│          (release)       │
└──────────────────────────┘
```

---

## Command-Response Correlation

### There is no sequence-number-based correlation.

The DAMIAO MIT control protocol does not include command sequence numbers. The motor simply echoes its **current state** (position, velocity, torque, temperature) after processing each command. Every response means "here is where I am now," not "here is the result of command X."

### Response identification is by motor ID only

When UdpServer receives a response frame:

1. The CAN frame ID encodes the motor's device ID (e.g., `deviceId + 0x10`).
2. `dispatchMessage()` derives `deviceId = FrameId - 0x10`.
3. The corresponding Motor subscriber's `callback()` is invoked.
4. The state is stored atomically in that Motor instance.

There is no ambiguity about **which motor** sent the response — only about **which command** it corresponds to. Since commands are sent at 400Hz and UDP round-trip is typically < 1ms, the response usually arrives within the same cycle or the next.

---

## One-Cycle Feedback Delay

In a 400Hz control loop (2.5ms period):

| Time | Event |
|------|-------|
| Cycle N, t=0.0ms | `controllerPeriodic()` sends MIT command for all 6 motors |
| Cycle N, t=0.0ms | Immediately reads motor state (from previous cycle's response) |
| Cycle N, t~0.5ms | Motor processes command, sends response via UDP |
| Cycle N, t~1.0ms | UdpServer receives frame, calls `updateState()` with atomic stores |
| Cycle N+1, t=2.5ms | `controllerPeriodic()` reads state (now from Cycle N's response) |

**The state read in Cycle N is the response to Cycle N-1's command (or earlier).**

This one-cycle delay is:
- **Standard** in servo control loops (even industrial EtherCAT drives operate this way).
- **Negligible** at 400Hz (2.5ms is < 1% of typical mechanical time constants).
- **Accounted for** in the PD controller gains (kp, kd), which are tuned assuming this latency.

---

## Why waitResponse() Was Removed from setMitControl()

### The Problem

The original `setMitControl()` called `prepareWait()` and `waitResponse()`, which blocked on a condition variable with a **200ms timeout**:

```cpp
// OLD CODE (removed):
prepareWait();           // sets m_requestPending = true
sendMessage(df, 8, 0, true);
waitResponse();          // blocks up to 200ms on m_requestCv
```

At 400Hz with 6 motors per leg, this could block for **1.2 seconds per cycle** (6 motors x 200ms timeout), completely breaking the 2.5ms deadline.

### Symptoms

- IMU data went critically stale (>200ms) because the Legged thread (SCHED_FIFO/90) was blocked, preventing the entire system from progressing.
- All 12 motors appeared unresponsive (>100ms) because `m_lastUpdateTime` stopped being checked.
- Emergency stop triggered repeatedly.

### The Fix

`setMitControl()` now uses fire-and-forget:

```cpp
// CURRENT CODE:
sendMessage(df, 8, 0, false);  // reply=false, no pending reply registered
// No prepareWait() / waitResponse()
```

The motor state feedback still arrives via the asynchronous path (`UdpServer → callback → updateState → atomic store`). Nothing is lost — `waitResponse()` was just an unnecessary synchronization barrier.

### All Commands Are Now Fire-and-Forget

After removing the synchronous mechanism, every Motor command uses fire-and-forget (`reply=false`):

| Command | Frequency | Response Handling |
|---------|-----------|-------------------|
| `enableMotor()` | Once at startup | State feedback via `callback()` → `updateState()` |
| `disableMotor()` | Once at shutdown | State feedback via `callback()` → `updateState()` |
| `setZeroCommand()` | Once at calibration | State feedback via `callback()` → `updateState()` |
| `clearMotorError()` | On error recovery | State feedback via `callback()` → `updateState()` |
| `setMitControl()` | 400Hz hot path | State feedback via `callback()` → `updateState()` |
| `setPosvelControl()` | Control path | State feedback via `callback()` → `updateState()` |
| `getMotorStatus()` | Infrequent polling | State feedback via `callback()` → `updateState()` |
| `getRegParam()` | 10Hz round-robin | Param response via `callback()` → `setTempParam()` + `MotorParamCache` |
| `writeRegParam()` | Rare configuration | Param response via `callback()` → `setTempParam()` |
| `saveRegParam()` | Rare configuration | Param response via `callback()` → `setTempParam()` |

---

## Removed: Synchronous Request/Response Mechanism

### What Was Removed

The Motor class originally contained a **synchronous request/response mechanism** built on a condition variable. This mechanism was removed entirely because it caused system-wide stalls when called from real-time threads.

**Removed components:**

| Component | Type | Location (former) | Purpose |
|-----------|------|--------------------|---------|
| `prepareWait()` | Method | `Motor.cpp` | Set `m_requestPending = true`, `m_completed = false` |
| `waitResponse()` | Method | `Motor.cpp` | Block on `m_requestCv` with 200ms timeout |
| `notify()` | Method | `Motor.cpp` | Signal `m_requestCv` when response arrived |
| `m_requestMutex` | `std::mutex` | `Motor.h` | Protect `m_completed` and `m_requestPending` flags |
| `m_requestCv` | `std::condition_variable` | `Motor.h` | Blocking wait primitive |
| `m_completed` | `bool` | `Motor.h` | Flag set by `notify()` when response arrived |
| `m_requestPending` | `bool` | `Motor.h` | Flag set by `prepareWait()` to indicate a wait is active |

### How It Worked (Before Removal)

The original command flow was:

```
Motor::setMitControl(mit_param)          // Called from SCHED_FIFO/90 thread @ 400Hz
  → std::lock_guard<std::mutex> txn(m_transactionMutex)   // Lock per-motor mutex
  → prepareWait()                                          // Set m_requestPending = true
      → std::lock_guard<std::mutex> lock(m_requestMutex)
      → m_completed = false
      → m_requestPending = true
  → sendMessage(df, 8, 0, true)                           // Send CAN frame, reply=true
      → HAL_WriteCANPacket() → UdpServer::sendMsg()
      → Register pending reply in m_frameIds map
  → waitResponse()                                         // BLOCK up to 200ms
      → std::unique_lock<std::mutex> lock(m_requestMutex)
      → m_requestCv.wait_for(lock, 200ms, [this]{ return m_completed; })
      → (blocked until notify() is called or timeout)
```

When the motor response arrived:

```
UdpServer::run()                         // SCHED_FIFO/88 thread
  → recvfrom() receives response frame
  → dispatchMessage(frame)
      → Look up pending reply in m_frameIds
      → Call Motor::callback(data, size)
          → parseMotorStateData() → updateState()
              → Atomic stores (m_stateQ, m_stateDq, ...)
              → notify()
                  → std::lock_guard<std::mutex> lock(m_requestMutex)
                  → m_completed = true
                  → m_requestCv.notify_one()    // Unblock waitResponse()
      → Set storage->replyEvent                // Legacy WPI event signaling
```

### Why It Was Removed

**1. Catastrophic blocking in the 400Hz control loop**

`setMitControl()` was called at 400Hz for each of 6 motors per leg. Each call could block for up to 200ms on `waitResponse()`. Worst case: 6 motors × 200ms = **1.2 seconds of blocking** per 2.5ms cycle.

**2. Priority inversion**

The condition variable wait in `waitResponse()` ran on the SCHED_FIFO/90 Legged thread, but the response that would unblock it arrived via the SCHED_FIFO/88 UdpServer thread. Since the Legged thread held `m_transactionMutex` while waiting, and the UdpServer thread needed `m_frameIdsMutex` (which could contend with the send path), this created a priority inversion chain:

```
Legged (FIFO/90) holds m_transactionMutex, waits on m_requestCv
  → Needs UdpServer (FIFO/88) to call notify()
    → UdpServer needs m_frameIdsMutex
      → m_frameIdsMutex may be held by sendMsg() called from Legged thread
        → DEADLOCK / PRIORITY INVERSION
```

**3. Blocking during motor enable/disable stalled the entire system**

`Legged::setEnable(true)` called `enableMotor()` sequentially for all 6 motors, each blocking up to 200ms. This ran on the same SCHED_FIFO/90 thread as `controllerPeriodic()`, so:

- The 400Hz control loop stopped running for up to 1.2 seconds
- IMU data went stale (>200ms) → emergency stop triggered
- All 12 motors reported unresponsive (>100ms)

**4. The mechanism was redundant**

Motor state feedback was **already delivered asynchronously** via `UdpServer → callback() → updateState()` with lock-free atomic stores. The Legged thread reads state via atomic loads (`getPosition()`, `getVelocity()`, etc.) regardless of whether `waitResponse()` was used. The blocking wait added latency without providing any data that wasn't already available through the async path.

### What Replaced It

All Motor commands now use fire-and-forget with `reply=false`:

```cpp
void Motor::enableMotor() {
    std::lock_guard<std::mutex> txn(m_transactionMutex);
    dataframe_t data(dataframe_enable_motor_t{});
    sendMessage(data, 8, 0, false);   // Fire-and-forget, returns immediately
}
```

The response still arrives asynchronously:

```
Motor sends command (UDP sendto)
  → Motor hardware processes command (~0.5ms)
  → Motor sends state feedback frame (UDP)
  → UdpServer::run() receives frame
  → dispatchMessage() → unsolicited path (no pending reply registered)
  → Motor::callback() → parseMotorStateData() → updateState()
  → Atomic stores: m_stateQ, m_stateDq, m_stateTau, m_lastUpdateTime
  → Legged thread reads via atomic loads on next cycle
```

No data is lost. The one-cycle feedback delay (~2.5ms) is standard in servo control loops and is accounted for in the PD controller gains.

---

## State Field Synchronization Summary

| Field | Type | Write Location | Read Location | Synchronization |
|-------|------|----------------|---------------|-----------------|
| `m_stateQ` | `std::atomic<double>` | `updateState()` (release) | `getPosition()` (acquire) | Lock-free atomic |
| `m_stateDq` | `std::atomic<double>` | `updateState()` (release) | `getVelocity()` (acquire) | Lock-free atomic |
| `m_stateTau` | `std::atomic<double>` | `updateState()` (release) | `getTorque()` (acquire) | Lock-free atomic |
| `m_stateTmos` | `std::atomic<int>` | `updateState()` (release) | `getStateTmos()` (acquire) | Lock-free atomic |
| `m_stateTrotor` | `std::atomic<int>` | `updateState()` (release) | `getStateTrotor()` (acquire) | Lock-free atomic |
| `m_status` | `std::atomic<int>` | `updateState()` (release) | `getState()` (acquire) | Lock-free atomic |
| `m_lastUpdateTime` | `std::atomic<uint64_t>` | `updateState()` (release) | `getLastUpdateTime()` (acquire) | Lock-free atomic |
| `m_lastMitParam` | `MITParam` | `setMitControl()` | `getLastMitParam()` | `m_commandMutex` |

---

## Mutex Inventory (Motor Communication Path)

| Mutex | Scope | Used By | Hot Path? | Notes |
|-------|-------|---------|-----------|-------|
| `m_transactionMutex` | Per motor | All Motor command methods | Yes (400Hz) | Serializes commands per motor; hold time is brief (sendto only) |
| `m_commandMutex` | Per motor | `setMitControl()` write, `getLastMitParam()` read | Yes (400Hz) | Protects `m_lastMitParam` struct copy |
| `m_paramMutex` | Per motor | `getParam()`, `setTempParam()` | No (10Hz) | Parameter dictionary access |
| `can->periodicSendsMutex` | Per CAN handle | `HAL_WriteCANPacket()` | Yes (400Hz) | Brief hold during periodic sends map update |
| `m_frameIdsMutex` | Per UdpServer | `sendMsg()`, `dispatchMessage()` | No | Legacy; not acquired when `reply=false` (all commands now) |

---

## Thread Priority Hierarchy

| Thread | Priority | Frequency | Role |
|--------|----------|-----------|------|
| Legged (left/right) | SCHED_FIFO/90 | 400Hz | MIT command dispatch + state read |
| UdpServer (x2) | SCHED_FIFO/88 | Event-driven | Motor response reception + dispatch |
| Composer | SCHED_FIFO/85 | 400Hz | Staging buffer merge → SensorData |
| ImuReader | SCHED_FIFO/80 | Event-driven (~500Hz) | IMU CAN frame accumulation |
| Robot (main loop) | SCHED_FIFO/75 | 100Hz | Staleness monitoring, state machine |
| Logger | SCHED_OTHER | Event-driven | SPSC ring drain → MQTT publish |
