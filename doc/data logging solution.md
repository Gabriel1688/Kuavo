
# Refined MQTT Logger Design Decisions

## 1. Can the Logger and Actuator Run in One Process?

**Yes — and it is the recommended approach.** Here is why:

The actuator process already runs 4 threads (1 IMU writer, 2 motor group writers, 1 composer). The MQTT logger drains the SPSC ring buffer in a non-blocking loop. Since libwebsockets uses a **single-threaded event loop** via `lws_service()` — which matches the existing epoll-based architecture used by the Damiao simulator [1] — the logger can run as a **5th thread** inside the actuator process.

### Advantages of Co-locating Logger with Actuator

| Aspect | Separate Process | Same Process (Recommended) |
|--------|:---:|:---:|
| Shared memory access | Must call `shm_open` + `mmap` to attach | Direct pointer — **zero IPC overhead** |
| Ring buffer access | Cross-process SPSC via shared memory | In-process SPSC — **no shared memory needed for logging** |
| Process management | Must start/stop 3 processes (controller, actuator, logger) | Start/stop 2 processes (controller, actuator+logger) |
| Failure isolation | Logger crash does not affect actuator | Logger crash in separate thread can be caught via `try/catch` without affecting motor threads |
| Deployment | 3 separate binaries | 2 binaries — simpler for embedded ARM edge devices |

### Thread Layout (Actuator + Logger Combined)

When combined, the actuator process runs 5 threads:

| Thread | Purpose | Rate | Writes To |
|--------|---------|:----:|-----------|
| Thread 1: IMU | Decode IMU CAN-over-UDP frames [1] | 500Hz | `imu_stage` double buffer |
| Thread 2: Motor Group A | Decode feedback from motors 0-5 [2] | 1kHz | `motor_group_a_stage` double buffer |
| Thread 3: Motor Group B | Decode feedback from motors 6-11 [2] | 1kHz | `motor_group_b_stage` double buffer |
| Thread 4: Composer | Read all 3 stages → merge → publish composed snapshot | 1kHz | `composed_buffers` double buffer |
| **Thread 5: MQTT Logger** | **Drain ring buffer → `lws_service()` → publish to broker** | **~5kHz drain, network-paced publish** | **MQTT broker (remote)** |

The ring buffer becomes an **in-process data structure** rather than a shared-memory object. The composer thread pushes `LogRecord` entries into the ring buffer after each compose cycle, and the logger thread pops them in its own `lws_service()` loop. Since the SPSC ring buffer is designed for exactly one producer and one consumer, this is safe without any mutex.

### What Changes

The `SharedMemoryLayout` no longer needs the `SPSCRingBuffer` field — the ring buffer moves from shared memory to a process-local global:

- **Shared memory** contains only: command double buffers, per-source staging double buffers, composed sensor double buffers, watchdog heartbeats, emergency stop flag, and motor CAN ID configuration
- **Process-local** (actuator): SPSC ring buffer for logging, lws MQTT client context, send queue

This simplifies the shared memory layout and reduces its size.

---

## 2. libwebsockets Without TLS

Without TLS, the lws MQTT client configuration simplifies significantly:

- No certificate file paths needed
- No `ssl_connection` flags in `lws_client_connect_info`
- Use `LWSS_FLAG_NONE` instead of `LWSS_USE_SSL`
- Broker URI uses `mqtt://` (port 1883) instead of `mqtts://` (port 8883)

### Connection Setup (No TLS)

The lws context creation uses a minimal `lws_context_creation_info` with no SSL-related fields. The MQTT connection uses plain TCP on port 1883, matching the standard Mosquitto default configuration.

Key differences from TLS mode:

| Parameter | With TLS | Without TLS (Selected) |
|-----------|----------|----------------------|
| Port | 8883 | **1883** |
| Protocol | `mqtts://` | **`mqtt://`** |
| lws SSL flag | `LCCSCF_USE_SSL` | **0 (none)** |
| Certificate | Required (`ca.pem`, `client.pem`) | **Not needed** |
| mbedTLS/OpenSSL | Required build dependency | **Not needed — smaller binary** |
| CPU overhead | ~5-15% for encrypt/decrypt | **Zero** |
| Build command | `-DLWS_WITH_SSL=ON` | **`-DLWS_WITH_SSL=OFF`** |

Building lws without TLS reduces the binary size and eliminates the OpenSSL/mbedTLS dependency — important for the ARM edge device where storage and build toolchain may be constrained.

### Security Consideration

Without TLS, MQTT traffic is unencrypted. This is acceptable when:
- The edge device and remote host are on the same **private network or VPN**
- The data is not sensitive (robot sensor telemetry, not credentials)
- The broker is behind a firewall

If the data traverses the public internet, TLS should be re-enabled in the future by adding the SSL flags to the lws context without changing the application logic.

---

## 3. Subscriber Design: Store to InfluxDB

The remote host subscriber receives binary MQTT messages and writes them to InfluxDB for time-series storage, dashboarding (Grafana), and offline analysis.

### Why InfluxDB

| Aspect | InfluxDB | PostgreSQL/TimescaleDB | MongoDB |
|--------|:---:|:---:|:---:|
| Time-series native | Yes — built for it | Extension (TimescaleDB) | No — document store |
| Write throughput | ~500K points/sec | ~100K rows/sec | ~50K docs/sec |
| Downsampling | Built-in (continuous queries) | Manual | Manual |
| Retention policies | Built-in auto-expiry | Manual | TTL index |
| Grafana integration | Native data source | Requires plugin | Requires plugin |
| Query language | Flux / InfluxQL | SQL | MQL |

At 1kHz control rate with 2 records per iteration (command + sensor), each containing 12 joints of data, the write rate is approximately **2000 records/sec** with **~24 fields per record** = **~48,000 data points/sec**. InfluxDB handles this comfortably.

### InfluxDB Data Model

The binary `LogRecord` payload is deserialized back into `Command` or `SensorData` structs, then mapped to InfluxDB measurements:

**Measurement: `robot_sensor`**

| Field Type | Fields | Source |
|-----------|--------|--------|
| **Tags** (indexed) | `robot_id`, `joint_id` (0-11) | Binary header + loop index |
| **Fields** (values) | `jpos`, `jvel`, `jtorque`, `motor_jpos`, `motor_jvel`, `bus_current`, `bus_voltage`, `motor_current` | `SensorData` struct — decoded from Damiao feedback frame D[1:5] [2] |
| **Fields** (IMU) | `imu_gx`, `imu_gy`, `imu_gz`, `imu_ax`, `imu_ay`, `imu_az` | `SensorData` struct — from IMU stream [1] |
| **Fields** (status) | `motor_status`, `mos_temp`, `rotor_temp` | Damiao D[0] (ID\|ERR<<4), D[6], D[7] [2] |
| **Fields** (contact) | `rfoot_contact`, `lfoot_contact` | `SensorData` struct |
| **Timestamp** | Nanosecond precision | `compose_timestamp_ns` from binary header |

**Measurement: `robot_command`**

| Field Type | Fields | Source |
|-----------|--------|--------|
| **Tags** | `robot_id`, `joint_id` | Binary header + loop index |
| **Fields** | `jpos_cmd`, `jvel_cmd`, `jtorque_cmd`, `kp`, `kd`, `control_mode`, `enabled` | `Command` struct |
| **Timestamp** | Nanosecond precision | `timestamp_ns` from binary header |

### Write Strategy

The subscriber should **batch writes** to InfluxDB rather than writing one point per MQTT message:

- Accumulate points in a local buffer (e.g., 500 points)
- Flush to InfluxDB every 500ms or when the buffer reaches 500 points (whichever comes first)
- Use the InfluxDB **line protocol** over HTTP for fastest ingestion
- Use **asynchronous writes** to avoid blocking the MQTT message callback

This batching reduces HTTP round-trips from ~2000/sec to ~4/sec while maintaining sub-second data freshness in dashboards.

### Retention and Downsampling

| Policy | Duration | Resolution | Purpose |
|--------|----------|:---:|---------|
| **Raw** | 7 days | 1ms (full rate) | Debugging, replay, detailed analysis |
| **Downsampled 1s** | 90 days | 1 second averages | Trend monitoring, Grafana dashboards |
| **Downsampled 1m** | 1 year | 1 minute averages | Long-term fleet health comparison |

InfluxDB continuous queries or tasks automatically downsample raw data into the lower-resolution buckets, and the 7-day retention policy on the raw bucket prevents unbounded storage growth.

### Grafana Dashboard

The InfluxDB data source connects directly to Grafana for real-time dashboarding:

- **Panel 1:** 12-joint position tracking (commanded vs actual) — overlay `jpos_cmd` from `robot_command` with `jpos` from `robot_sensor`
- **Panel 2:** Motor temperatures (MOS + Rotor) per joint — alert when approaching the Damiao overtemp threshold (ERR=0x0B for MOS, ERR=0x0C for coil) [2]
- **Panel 3:** IMU angular velocity (gyroscope) — 3-axis plot
- **Panel 4:** Torque error (commanded vs measured) per joint
- **Panel 5:** Motor status heatmap — color-coded by Damiao error code [2]

---

## Revised Architecture Summary

```
ARM Edge Device                              x86 Remote Host
┌─────────────────────────┐                  ┌─────────────────────────┐
│  Controller Process     │                  │  MQTT Subscriber        │
│  (1kHz control loop)    │                  │  (Python or C++)        │
│                         │                  │                         │
│  Writes: cmd_buffers    │   Shared         │  Receives binary MQTT   │
│  Reads:  composed_bufs  │   Memory         │  Deserializes structs   │
│                         │◄────────────────►│  Batch writes to        │
└─────────────────────────┘   (double        │  InfluxDB (line proto)  │
                              buffer)        └───────────┬─────────────┘
┌─────────────────────────┐                              │
│  Actuator + Logger      │                              ▼
│  (single process)       │                  ┌─────────────────────────┐
│                         │                  │  InfluxDB               │
│  Thread 1: IMU (500Hz)  │                  │  • robot_sensor (raw)   │
│  Thread 2: Motors 0-5   │                  │  • robot_command (raw)  │
│  Thread 3: Motors 6-11  │                  │  • 7d retention (raw)   │
│  Thread 4: Composer     │                  │  • 90d retention (1s)   │
│  Thread 5: MQTT Logger  │   MQTT           │  • 1yr retention (1m)   │
│    lws_service() loop   │──────────────────►                         │
│    No TLS (port 1883)   │   binary payload └───────────┬─────────────┘
│    Topics:              │   robot/sensor/bin            │
│      robot/command/bin  │   robot/command/bin           ▼
│      robot/sensor/bin   │                  ┌─────────────────────────┐
│      robot/status       │                  │  Grafana                │
│                         │                  │  • Joint tracking       │
│  SPSC Ring Buffer       │                  │  • Motor temperatures   │
│  (process-local, not    │                  │  • IMU visualization    │
│   in shared memory)     │                  │  • Torque error         │
└─────────────────────────┘                  │  • Status heatmap       │
        │                                    └─────────────────────────┘
        │ UDP (CAN-over-UDP, 13 bytes) [1]
        ▼
   Damiao Motors (0x01 - 0x0C)
```

### Key Design Points

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Logger + Actuator in one process | **Yes** | Eliminates shared memory for the ring buffer; simpler deployment on ARM edge; direct pointer access to composed data |
| TLS | **Disabled** | Private network assumption; smaller binary; no OpenSSL dependency on ARM edge |
| Topic structure | `robot/command/bin`, `robot/sensor/bin`, `robot/status` | Single robot per edge device; static `const char*` topics; zero allocation in hot path |
| Payload format | Binary (`memcpy` of struct) | Lowest bandwidth (~2.4 MB/s vs ~7 MB/s JSON); zero serialization overhead; consistent with Damiao CAN frame approach [1] |
| Cross-platform safety | `#pragma pack` or explicit padding + `sizeof` assertion | ARM64 and x86-64 are both little-endian; padding differences caught at compile time |
| Remote storage | InfluxDB with batch writes | Native time-series database; built-in retention and downsampling; native Grafana integration |
| Write batching | 500 points or 500ms flush interval | Reduces HTTP round-trips from ~2000/sec to ~4/sec |