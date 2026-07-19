
# Architecture Review — Consolidated Q&A Record

Below is the complete record of all 18 architecture review questions with your responses, status, and follow-up actions.

---

## Thread Model & Scheduling

### Q1: SCHED_FIFO Priority Starvation Between Leg Threads

**Question:** Both leg threads at `SCHED_FIFO` priority 90 — what happens if one overruns its 2.5ms deadline while the other's next period fires?

**Answer:** No answer yet.

**Status:** 🟡 Open

**Action Required:** Investigate whether `SCHED_RR` should be used instead of `SCHED_FIFO` for the two leg threads at the same priority level. Since both legs manage independent motor groups on separate UDP sockets (left leg IDs 1-5 on ports 8887/8886, right leg IDs 6-10 on ports 8889/8888) [1], they should not contend for the same resource. However, CPU core affinity (`pthread_setaffinity_np`) should be evaluated as a mitigation — pinning each leg thread to a different CPU core would eliminate the starvation risk entirely regardless of scheduling policy.

---

### Q2: Composer Preemption and SPSC Ring Buffer Impact

**Question:** What happens if the composer (priority 85) is preempted by both leg threads (90) and both UdpServers (88) simultaneously?

**Answer:** SPSC ring buffer will lose data. The Mercury Controller will use the previous sensor data from the shared memory double buffer.

**Status:** ✅ Accepted

**Rationale:** The double-buffer design guarantees the controller always reads a complete, consistent snapshot — the last successfully composed one [1]. If the composer misses a cycle, the controller reads a 5ms-old snapshot instead of a 2.5ms-old one. For a bipedal robot at 400Hz, a single missed compose cycle is equivalent to running the controller at 200Hz for one cycle — which is the current rate that already works [2]. The SPSC ring buffer data loss only affects telemetry logging (non-safety-critical), not the control loop.

---

### Q3: PREEMPT_RT Kernel Requirement

**Question:** Has the ARM SBC kernel been verified for `PREEMPT_RT` or `CONFIG_PREEMPT`?

**Answer:** Add the real-time kernel patch.

**Status:** ✅ Action Defined

**Action Required:** Apply the `PREEMPT_RT` patch to the ARM SBC's kernel. The main control loop runs at 100Hz (10ms) and the inner loops at 400Hz (2.5ms) [2]. Without `PREEMPT_RT`, kernel spinlocks and softirqs can introduce unbounded latency that violates the 2.5ms deadline. The existing system uses POSIX threads with real-time extensions (`pthread`, `rt`) [2], which only deliver deterministic scheduling under a fully preemptible kernel.

---

## IMU Rate Mismatch

### Q4: 4:1 IMU-to-Controller Rate Mismatch

**Question:** How does the controller handle 3 consecutive cycles with stale IMU data? Where should prediction/extrapolation be implemented?

**Answer:** Where to implement prediction/extrapolation needs further analysis. Leave as an open question.

**Status:** 🟡 Open

**Context:** The architecture documentation explicitly notes this as an open question: "state estimation rate mismatch (100Hz estimation vs 200Hz control) — prediction/extrapolation not yet implemented" [3]. The new design widens this gap from 2:1 to 4:1 (100Hz IMU vs 400Hz controller). The IMU provides a 7D state vector (Euler angles + quaternion) used for balance [1]. Three options exist for prediction: (a) Kalman filter in the composer thread, (b) linear extrapolation using gyroscope angular velocity in the inner control loop, (c) accept the mismatch and rely on the 100Hz update being sufficient for the robot's dynamics.

---

### Q5: UDP Receive Buffer Overflow at Reduced IMU Rate

**Question:** If the LPMS-IG1 hardware still outputs at 500Hz but the reader drains at 100Hz, does the UDP buffer overflow?

**Answer:** Keep 500Hz if the IMU supports it when choosing the high-precision model.

**Status:** ✅ Decision Made

**Decision:** Maintain the IMU reader at 500Hz rather than reducing to 100Hz. The LPMS-IG1 IMU data arrives as 8 sequential CAN frames (IDs 0x514 through 0x51B) per measurement cycle [1]. Keeping the reader at 500Hz eliminates the rate mismatch concern (Q4) — the IMU-to-controller ratio becomes 500:400 (1.25:1) instead of 100:400 (1:4), which is acceptable. The IMU reader thread remains a dedicated pthread with a blocking UDP socket on port 8887 [1], and its `SCHED_FIFO` priority of 80 ensures it is not starved by lower-priority threads.

**Revised Thread Table:**

| Thread | Original Proposed Rate | Final Rate |
|--------|:---:|:---:|
| IMU Reader | 100Hz | **500Hz (unchanged from current)** |

---

## Shared Memory & Double Buffering

### Q6: `std::atomic<uint32_t>` Lock-Free Guarantee on ARM64

**Question:** Is `std::atomic<uint32_t>::is_always_lock_free` true on the target ARM toolchain?

**Answer:** Not yet verified. Leave as an open question.

**Status:** 🟡 Open

**Action Required:** Add a compile-time assertion to `mercury_shm.hpp`:

```cpp
static_assert(std::atomic<uint32_t>::is_always_lock_free,
              "atomic<uint32_t> must be lock-free on target architecture");
```

If this assertion fails on the ARM toolchain, the double-buffer pattern degrades to a hidden mutex, which could cause priority inversion between the leg threads (priority 90) and the composer (priority 85).

---

### Q7: Motor Count Mismatch (10 vs 12)

**Question:** Kuavo has 10 motors but `Mercury_SensorData` has arrays sized `num_act_joint = 12`. What populates indices 10 and 11?

**Answer:** Update to 12 motors. The source code value of 10 is wrong.

**Status:** ✅ Decision Made

**Decision:** The motor count is being updated to 12 (6 per leg instead of 5 per leg). This changes the motor topology:

| Aspect | Previous (Source Code) [1] | Updated |
|--------|:---:|:---:|
| Motors per leg | 5 | **6** |
| Total motors | 10 | **12** |
| `MOTORS_PER_GROUP` | 5 | **6** |
| Left leg IDs | 1-5 | 1-6 |
| Right leg IDs | 6-10 | 7-12 |
| `num_act_joint` | 12 (was mismatched) | 12 (now correct) |

The `max_can_device` threshold that routes motor IDs to UDP server 0 (left) vs server 1 (right) [1] must be updated from `device_id < 6` to `device_id < 7`.

---

### Q8: ARM Performance Penalty from `#pragma pack(1)`

**Question:** Has the performance impact of packed struct access on ARM been measured?

**Answer:** How to measure it is an open question.

**Status:** 🟡 Open

**Action Required:** Profile the composer thread's cycle time with and without `#pragma pack(1)` on the ARM SBC. The composer reads three staging buffers and merges them into a `SensorData` struct at 400Hz. Unaligned `double` access on ARMv8 is handled in hardware but incurs approximately 2× latency compared to aligned access. If the performance penalty exceeds 10% of the 2.5ms budget (i.e., >250μs), switch to Option B (natural alignment with explicit padding) instead of `#pragma pack(1)`.

---

## Motor Control & Safety

### Q9: Damiao Motor Internal Processing Rate

**Question:** What is the Damiao motor driver's internal control loop rate? If the motor firmware runs slower than 400Hz, commands will be overwritten.

**Answer:** Not an issue. The motor's internal processing rate is higher than 400Hz.

**Status:** ✅ Resolved

**Rationale:** The Damiao motor driver firmware processes commands faster than 400Hz, so sending MIT commands at 400Hz from the inner control loop will not cause command queuing or overwrite issues. The MIT control mode documentation describes the motor's internal current loop and control pipeline [5], which operates at a significantly higher rate than the CAN command input rate.

---

### Q10: CAN Bus Bandwidth Utilization

**Question:** At 400Hz MIT commands + 10Hz parameter queries per leg, can the CAN bus sustain the load?

**Answer:** 1 Mbps CAN bus. There are two CAN buses, each with 6 motors.

**Status:** ✅ Resolved

**Bandwidth Calculation:**

| Parameter | Value |
|-----------|-------|
| CAN bus speed | 1 Mbps (fixed, per Damiao protocol) [5] |
| CAN buses | 2 (one per leg) [1] |
| Motors per bus | 6 (updated from 5) |
| MIT commands per bus | 6 motors × 400Hz = 2,400 frames/sec |
| Parameter queries per bus | 6 motors × 10Hz ÷ 6 (round-robin) = 10 frames/sec |
| Total frames per bus | ~2,410 frames/sec |
| Frame size | 13 bytes × 8 bits = 104 bits + ~44 bits overhead = ~148 bits per CAN frame |
| Bandwidth per bus | 2,410 × 148 = ~357 kbps |
| Utilization | 357 kbps / 1,000 kbps = **~36%** |

At 36% utilization per bus, there is ample headroom. CAN buses are generally considered safe up to 70-80% utilization before arbitration delays become significant.

---

### Q11: Motor Responsiveness Timeout

**Question:** Is 200 missed feedback frames (500ms at 400Hz) the right detection granularity?

**Answer:** Need to change it to 100ms.

**Status:** ✅ Action Defined

**Change:** Reduce the motor responsiveness timeout from 500ms to **100ms**. At 400Hz, 100ms corresponds to 40 missed feedback frames — a more appropriate detection granularity for a bipedal robot that could fall within 200-300ms of losing motor control. The existing 500ms timeout [1][2] was designed for the 200Hz inner loop rate; at 400Hz, the faster loop deserves a proportionally tighter timeout.

---

## MQTT Logger & Telemetry

### Q12: MQTT Network Bandwidth (7.7 Mbps)

**Question:** Can the network between ARM edge and x86 remote host sustain 7.7 Mbps of MQTT traffic?

**Answer:** Need to test with fake data.

**Status:** 🟡 Open — Test Required

**Action Required:** Run a bandwidth test using the existing `test_actuator_logger` with simulated motor data to measure actual MQTT throughput. The test should validate:
- Sustained throughput over 60 seconds
- Packet loss rate at the MQTT broker
- SPSC ring buffer fill level under steady-state conditions
- Impact of network jitter on ring buffer overflow

---

### Q13: SPSC Ring Buffer `dropped_` Counter Data Race

**Question:** The `dropped_` counter is a non-atomic `uint64_t` accessed by both the producer and consumer threads. Is this a data race?

**Answer:** Accept this data race.

**Status:** ✅ Accepted (Known Limitation)

**Rationale:** The `dropped_` counter is a diagnostic metric used only for reporting — it does not affect control flow, safety, or data integrity. A torn read on the counter (which can happen on 32-bit ARM for a 64-bit value) would at worst display an incorrect drop count in the log report. The cost of making it `std::atomic<uint64_t>` is negligible, but the decision to accept the race is pragmatic for the current phase.

---

### Q14: Breaking Change for Existing MQTT Topic Consumers

**Question:** Are there monitoring dashboards or safety systems that depend on the localhost MQTT topics?

**Answer:** No.

**Status:** ✅ Resolved

**Rationale:** No external systems depend on the existing 50Hz telemetry topics (`/telemetry/subsystem/<name>/motor`) [1]. The binary `RobotStatusWire` (magic 0x4B564155, 890 bytes) and JSON SenML data logs [1] can be safely removed and replaced by the new binary MQTT logger without breaking any downstream consumers.

---

## Cross-Process Mercury Controller Bridge

### Q15: 10ms Delay Detecting Mercury Controller Crash

**Question:** Is 10ms of uncontrolled motor commands acceptable if the Mercury Controller crashes?

**Answer:** Acceptable now, no better choice.

**Status:** ✅ Accepted (Known Limitation)

**Rationale:** The heartbeat timeout detection runs in `robotPeriodic()` at 100Hz (10ms), while the inner control loop dispatches MIT commands at 400Hz (2.5ms). If the Mercury Controller crashes, up to 4 stale commands could be dispatched before detection. Given the Mercury Controller communicates via POSIX shared memory [2] and the robot's dynamics are not fast enough for 10ms of stale commands to cause catastrophic failure, this latency is acceptable. The inner control loop also independently checks the command timestamp and will detect stale commands if the shared memory heartbeat ages beyond the configured threshold.

---

### Q16: Bus Voltage Staleness (1-Second-Old Data in SensorData)

**Question:** The Mercury Controller has no way to know that `bus_voltage` was updated 1 second ago while `joint_jpos` was updated 2.5ms ago.

**Answer:** Voltage change is quite slow. It is acceptable.

**Status:** ✅ Accepted

**Rationale:** Bus voltage on a 24-48V power supply [5] changes on a timescale of seconds to minutes — far slower than the 1-second update interval from the parameter query round-robin. The Damiao motor driver's internal protections (overvoltage 0x08, undervoltage 0x09) [1] fire independently of the controller's voltage reading, so safety is not compromised by the staleness. If the Mercury Controller uses bus voltage in a feedforward computation, the 1-second-old value introduces negligible error compared to the voltage's actual rate of change.

---

### Q17: LQR Gain Retuning for 2.5ms Sampling Period

**Question:** Have the LQR gain lookup tables been regenerated for the 2.5ms discrete-time model?

**Answer:** No longer using LQR. The control algorithm has changed.

**Status:** ✅ Resolved (No Longer Applicable)

**Context:** The original architecture uses an LTV-LQR controller with precomputed gain lookup tables indexed by trajectory progress [1]. Since the control algorithm is being replaced (no longer LQR), the concern about discrete-time gain retuning is moot. The new controller running in the Mercury Controller process handles its own gain computation and communicates joint commands via shared memory [2]. The inner control loop acts purely as a command relay and motor driver — it does not compute control gains.

---

## Q18: Prioritized Risk Ranking

**Original Question:** Rank these risks by severity for a bipedal robot that must not fall.

**Final Ranking After Answers:**

| Rank | Risk | Severity | Status | Resolution |
|:---:|------|:---:|:---:|------------|
| 1 | **IMU rate mismatch** (Q4/Q5) | **High** | ✅ Mitigated | Keep IMU at 500Hz (Q5 answer), eliminating the 4:1 mismatch. Prediction/extrapolation remains an open question but is less critical at 500:400 ratio |
| 2 | **LQR gains not retuned** (Q17) | **High** | ✅ Eliminated | No longer using LQR — not applicable |
| 3 | **Motor timeout too long** (Q11) | **Medium** | ✅ Action Defined | Reduce from 500ms to 100ms |
| 4 | **10ms crash detection delay** (Q15) | **Medium** | ✅ Accepted | No better alternative in current architecture |
| 5 | **ARM struct packing penalty** (Q8) | **Medium** | 🟡 Open | Requires profiling on target hardware |
| 6 | **SCHED_FIFO starvation** (Q1) | **Medium** | 🟡 Open | Needs investigation; CPU pinning may resolve |
| 7 | **CAN bandwidth saturation** (Q10) | **Low** | ✅ Resolved | 36% utilization at 1 Mbps with 2 CAN buses |
| 8 | **MQTT ring buffer overflow** (Q12/Q13) | **Low** | 🟡 Test Required | Non-safety-critical; data race accepted for diagnostics |

---

## Summary

| Category | Total | Resolved | Accepted | Open |
|----------|:-----:|:--------:|:--------:|:----:|
| Thread Model | 3 | 1 | 1 | 1 |
| IMU Rate | 2 | 1 | 0 | 1 |
| Shared Memory | 3 | 1 | 0 | 2 |
| Motor Control | 3 | 3 | 0 | 0 |
| MQTT Logger | 3 | 1 | 1 | 1 |
| Cross-Process | 3 | 1 | 2 | 0 |
| **Total** | **17** | **8** | **4** | **5** |

### Key Decisions Made

1. **Motor count updated to 12** (6 per leg) — aligns `num_act_joint` with actual hardware
2. **IMU stays at 500Hz** — eliminates the most severe architectural risk
3. **Motor timeout reduced to 100ms** — faster detection of unresponsive motors
4. **LQR no longer used** — new controller in Mercury process handles gain computation
5. **PREEMPT_RT kernel required** — patch must be applied to the ARM SBC
6. **No existing MQTT consumers** — safe to replace telemetry system entirely
7. **Voltage staleness acceptable** — 1-second-old data is fine for slow-changing parameters

### Open Actions

1. **Q1:** Evaluate CPU core affinity vs `SCHED_RR` for leg thread starvation
2. **Q4:** Decide where to implement IMU prediction/extrapolation (now less critical at 500Hz)
3. **Q6:** Verify `std::atomic<uint32_t>::is_always_lock_free` on the ARM toolchain
4. **Q8:** Profile `#pragma pack(1)` performance impact on ARM
5. **Q12:** Run MQTT bandwidth test with fake data