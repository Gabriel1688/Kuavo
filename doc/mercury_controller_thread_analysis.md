# Mercury Controller / Kuavo RT Threading Analysis

## 1. RT-Priority CPU Starvation of DamiaoSimulator

### Symptom
When toggling joystick enable/disable via MQTT, `DamiaoSimulator` logs show the disable (0xFD) frames arriving **16-39 ms** after Kuavo logs `Leg is Disabled`, while enable (0xFC) frames arrive with ~0 ms delay.

### Root cause
The Kuavo leg RT threads run at `SCHED_FIFO` priority **90** (`ControlledSubsystemBase.h:66-68`). The `DamiaoSimulator` processes are normal `SCHED_OTHER` processes. On a 4-core host, the RT threads can preempt the `DamiaoSimulator` processes, so packets sit in the kernel socket buffer until the RT threads block in `poll()`.

| Process/Thread | Scheduler | Priority | Activity |
|---|---|---|---|
| Left leg RT | `SCHED_FIFO` | 90 | 400 Hz MIT dispatch + feedback staging |
| Right leg RT | `SCHED_FIFO` | 90 | 400 Hz MIT dispatch + feedback staging |
| UdpServer recv | `SCHED_FIFO` | 88 | UDP feedback receive |
| Kuavo main loop | `SCHED_FIFO` | 75 | 100 Hz `robotPeriodic` |
| DamiaoSimulator (left/right) | `SCHED_OTHER` | 0 | UDP epoll receive |

### Why the asymmetry?
- **Enable (0xFC):** RT threads are idle (leg disabled, minimal `controllerPeriodic` work), so `DamiaoSimulator` is scheduled immediately.
- **Disable (0xFD):** RT threads are busy dispatching MIT commands, so `DamiaoSimulator` is starved until they sleep.

### How to confirm with a remote real-time host
Move the two `DamiaoSimulator` processes to a separate host with a PREEMPT_RT kernel and run them as real-time on isolated cores:

```bash
# Remote RT host
sudo chrt -f 95 taskset -c 2 ./DamiaoSimulator -ids 1,2,3,4,5,6 -local 8886 -remote 8887 -log left.log
sudo chrt -f 95 taskset -c 3 ./DamiaoSimulator -ids 7,8,9,10,11,12 -local 8888 -remote 8889 -log right.log
```

Expected result:
- If the 16-39 ms delay collapses to network latency (<0.5 ms on a local switch), the original delay was CPU starvation on the 4-core control host.
- If the delay persists, the cause is not starvation; look at the Kuavo send path or network buffering.

### Cross-host note
`DamiaoSimulator.cpp` hardcodes `127.0.0.1` for bind and remote feedback. For remote operation it must be patched to accept bind/remote IP arguments, and `config/config.yaml` `udp.client_ip.left/right` must point to the remote host.

## 2. `mercury_controller` / `mercury_service_main` Needs Real-Time Scheduling

The Mercury controller is a separate process but uses POSIX shared memory (`/dev/shm/mercury_robot_ipc`), so it must stay on the same host as Kuavo.

### Why it must be `SCHED_FIFO`
The controller currently has **no** explicit scheduling; it runs as `SCHED_OTHER`. On the same 4-core host as the Kuavo `SCHED_FIFO/90` leg threads, it can be starved. That causes:

- Stale `Mercury_Command` in SHM (`cmd_age_ns > COMMAND_STALE_THRESHOLD_NS`), so `Legged::controllerPeriodic` skips MIT dispatch.
- `Robot::robotPeriodic` heartbeat timeout (`>100 ms`), triggering emergency stop.

### Recommended run command
```bash
sudo chrt -f 85 taskset -c 3 ./mercury_service_main
```

Priority should be **below** the Kuavo leg threads (90) and UdpServer thread (88) but above normal processes. If the controller has its own isolated core, priority matters less as long as it is `SCHED_FIFO`.

### Optional self-scheduling code
```cpp
#include <sched.h>
// in main()
struct sched_param param{};
param.sched_priority = 85;
if (sched_setscheduler(0, SCHED_FIFO, &param) < 0) {
    perror("sched_setscheduler");
}
```

### Controller frequency
The main loop uses `mercury::servo_rate`, which is `1.0/200.0` (200 Hz) in the current code. The Kuavo leg threads read commands at 400 Hz, so they will simply see the same command for two consecutive cycles.

## 3. MQTT Broker Should Run on Another Host

`mosquitto` is a normal `SCHED_OTHER` service. Running it on the control host adds context switches, network interrupts, and CPU load that compete with the RT threads.

### What moving it helps
- Removes broker CPU/interrupt load from the control host.
- Prevents broker preemption by Kuavo RT threads.

### What it does not fix
The Kuavo `MqttClient` thread is still `SCHED_OTHER/0` (`MqttClient.cpp:84-86`) and lives on the Kuavo host. If joystick latency matters, raise it to `SCHED_FIFO/70` (below the main loop at 75):

```cpp
// lib/mqtt/MqttClient.cpp
param.sched_priority = 70;
int ret = pthread_setschedparam(m_thrId, SCHED_FIFO, &param);
```

### Config for remote broker
In `config/config.yaml`:

```yaml
mqtt:
  address: 192.168.1.50
  mqtt_broker: 192.168.1.50
  host: 192.168.1.50
  port: 1883
```

Open `1883/tcp` (and `9001/tcp` for WebSockets) on the broker host.

## 4. Thread Inventory in `mercury_service_main`

At steady state `mercury_service_main` uses **5 explicit threads**:

| # | Thread | File | Purpose |
|---|---|---|---|
| 1 | Main control loop | `mercury_service_main.cpp` | Runs `Mercury_interface::GetCommand()` at `mercury::servo_rate` (200 Hz) |
| 2 | DataManager | `Utils/DataManager.cpp` | Telemetry UDP sender (200 Hz, port 61125) |
| 3 | ExtCtrlReceiver | `Mercury_Controller/ExtCtrlReceiver.cpp` | UDP receiver for joystick/walking setpoints (port 61123) |
| 4 | MercuryShmInterface proactive | `Mercury_Controller/MercuryShmInterface.cpp` | Periodic SHM command writer (200 Hz) |
| 5 | MoCapManager | `Mercury_Controller/MoCapManager.cpp` | Motion-capture UDP receiver |

None of these threads are explicitly set to `SCHED_FIFO` unless the whole process is launched with `chrt`.

## 5. State Estimation Runs in the Main Control Loop

State estimation is **not** a separate thread. It is called synchronously inside the main control loop:

```cpp
mercury_service_main.cpp
  -> Mercury_interface::GetCommand()
       -> state_estimator_->Update(data)
```

`Mercury_StateEstimator::Update()` performs:
- `_JointUpdate(data)`
- IMU orientation estimation (`ori_est_`)
- RBDL model update
- MoCap-based body position/velocity (`body_foot_est_->Update()`)
- CoM velocity smoothing
- Foot contact update

The `MoCapManager` thread only supplies fresh MoCap LED data; the actual estimation runs on the main thread.

## 6. Combining `MercuryShmInterface` Proactive Thread into the Main Loop

`MercuryShmInterface` already supports direct (reactive) SHM writes. To eliminate the proactive thread:

1. Remove the proactive start in `Mercury_interface.cpp`:

```cpp
// Remove this line in Mercury_interface constructor:
// shm_interface_->startProactiveMode(200);
```

2. `writeCommand()` will then use the reactive branch:

```cpp
bool MercuryShmInterface::writeCommand(const Mercury_Command& mercury_cmd) {
    if (proactive_running_.load()) {
        // copy to buffer for proactive thread
    } else {
        // write directly to SHM now
        mercury::Command cmd;
        convertCommandWithDoubleCell(mercury_cmd, cmd);
        uint64_t write_start = mercury::get_monotonic_ns();
        cmd.timestamp_ns = write_start;
        uint32_t wb = 1 - layout_->cmd_write_idx.load(...);
        std::memcpy(&layout_->cmd_buffers[wb], &cmd, sizeof(mercury::Command));
        layout_->cmd_write_idx.store(wb, ...);
        layout_->cmd_sequence.fetch_add(1, ...);
        return true;
    }
}
```

### What is gained
- One fewer thread.
- No `command_mutex_` contention.
- Lower latency: command written in the same cycle it is computed.

### What is lost
The proactive thread could keep refreshing the last command with a current timestamp if the main loop stalled. With direct writes, stale timestamps reflect reality; Kuavo will detect stale commands after ~100 ms and disable the leg. If immediate safety fallback is needed, add a small watchdog thread or write a zero command in the shutdown signal handler.

### Heartbeat
`controller_heartbeat_ns` is updated separately by `updateHeartbeat()` after `readSensorData()` in `GetCommand()`. If you want the heartbeat tied to the command write, add:

```cpp
layout_->controller_heartbeat_ns.store(write_start, std::memory_order_release);
```

inside the reactive `writeCommand` branch.

## 7. `DataManager` Telemetry Is Redundant with SHM

`DataManager` is a `dynacore_pThread` that sends telemetry over UDP to `IP_ADDR:PORT_DATA_RECEIVE` (default `127.0.0.1:61125`) at 200 Hz. The data it sends is mostly already in the SHM that Kuavo reads.

| DataManager variable | SHM field | Notes |
|---|---|---|
| `jpos_command_` (jpos_des) | `Command::jpos_cmd` | SHM contains final value **after** spring compensation: `jpos_cmd = jpos_command_ + torque/spring_const` |
| `jvel_command_` (jvel_des) | `Command::jvel_cmd` | Direct match |
| `sensed_torque_` (torque) | `SensorData::jtorque` | Direct match |
| `motor_current_` | `SensorData::motor_current` | Direct match |
| `bus_current_` | `SensorData::bus_current` | Direct match |
| `bus_voltage_` | `SensorData::bus_voltage` | Direct match |
| `torque_command_` (command) | `Command::jtorque_cmd` | Direct match |
| `running_time_` | *not in SHM* | Can be derived from `Command::timestamp_ns` or `Command::sequence` |

### Conclusion
`DataManager` is **not needed for control**. It can be disabled by not calling `DataManager::GetDataManager()->start()` in `Mercury_interface::_Initialization()`. Any external logger can read the same data from:

- `/dev/shm/mercury_robot_ipc` locally, or
- MQTT binary topics `robot/command/bin` and `robot/sensor/bin` remotely.

## 8. `ExtCtrlReceiver` — External Walking Setpoint Receiver

`ExtCtrlReceiver` is a UDP listener for joystick/teleop walking commands:

```cpp
void ExtCtrlReceiver::run(){
    while (true){
        COMM::receive_data(socket_, PORT_EXT_CTRL,
                           &des_loc_, sizeof(ExtCtrl::Location), IP_ADDR_MYSELF);
        sp_->des_location_[0] = des_loc_.x;
        sp_->des_location_[1] = des_loc_.y;
    }
}
```

The packet type is:

```cpp
namespace ExtCtrl {
    typedef struct { double x; double y; } Location;
}
```

It writes to `Mercury_StateProvider::des_location_`, which the walking planner consumes as `pl_param.des_loc`. It is **not** a real-time thread, so the sender does not need real-time scheduling.

## 9. Feeding Kuavo Joystick Input to `ExtCtrlReceiver`

Because `ExtCtrlReceiver` is non-RT, you can feed it from a non-RT source. The recommended approach is a separate UDP bridge.

### Option A — Separate Python/C++ bridge (cleanest)

A normal-priority process subscribes to the MQTT joystick topic and sends `ExtCtrl::Location` packets to the controller on **port 61123**.

```python
import socket
import json
import struct
import paho.mqtt.client as mqtt

CONTROLLER_IP = '127.0.0.1'
CONTROLLER_PORT = 61123
SCALE = 0.5  # meters at full stick deflection
MQTT_BROKER = '192.168.1.50'
TOPIC = 'test/topic1'

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def on_message(client, userdata, msg):
    data = json.loads(msg.payload)
    x = -data['axes'][1] * SCALE   # forward/back
    y =  data['axes'][0] * SCALE   # left/right
    sock.sendto(struct.pack('dd', x, y), (CONTROLLER_IP, CONTROLLER_PORT))

client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER)
client.subscribe(TOPIC)
client.loop_forever()
```

### Option B — Non-RT worker thread inside Kuavo `Robot`

Keep network I/O out of the `SCHED_FIFO/75` main thread. Add a worker thread that reads a lock-free atomic setpoint and sends UDP.

In `Robot.h`:

```cpp
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include <sys/socket.h>

struct ExtCtrlLoc { double x; double y; };
std::atomic<ExtCtrlLoc> m_extCtrlLoc{ {0.0, 0.0} };
std::thread m_extCtrlThread;
std::atomic<bool> m_extCtrlRunning{false};
int m_extCtrlSock = -1;
sockaddr_in m_extCtrlAddr{};
```

Start in `Robot::robotInit()`:

```cpp
m_extCtrlSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
m_extCtrlAddr.sin_family = AF_INET;
m_extCtrlAddr.sin_port = htons(61123);
m_extCtrlAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);  // or remote controller IP

m_extCtrlRunning = true;
m_extCtrlThread = std::thread([this]() {
    ExtCtrlLoc loc;
    while (m_extCtrlRunning) {
        loc = m_extCtrlLoc.load();
        sendto(m_extCtrlSock, &loc, sizeof(loc), 0,
               (sockaddr*)&m_extCtrlAddr, sizeof(m_extCtrlAddr));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
    }
});
```

Update setpoint in `Robot::driveWithJoystick()`:

```cpp
void Robot::driveWithJoystick(bool fieldRelative) {
    const double maxStep = 0.5;  // meters at full stick
    double x = -m_controller.getLeftY() * maxStep;  // forward
    double y =  m_controller.getLeftX() * maxStep;  // lateral
    m_extCtrlLoc.store({x, y});
}
```

Join on shutdown:

```cpp
m_extCtrlRunning = false;
if (m_extCtrlThread.joinable()) m_extCtrlThread.join();
```

### Option C — Direct send from `driveWithJoystick` (quick test only)

```cpp
void Robot::driveWithJoystick(bool fieldRelative) {
    ExtCtrlLoc loc;
    loc.x = -m_controller.getLeftY() * 0.5;
    loc.y =  m_controller.getLeftX() * 0.5;
    sendto(m_extCtrlSock, &loc, sizeof(loc), 0,
           (sockaddr*)&m_extCtrlAddr, sizeof(m_extCtrlAddr));
}
```

This runs in the **RT main thread** and is not recommended for production.

### Note on `x, y` semantics

`ExtCtrlReceiver` writes the received values directly to `sp_->des_location_`. Depending on the walking planner, this is either:

- an **absolute target position** in meters, or
- a **velocity-like target**.

If your joystick axes represent velocity, integrate them over time before sending. If they represent a direct position setpoint, scale to meters and send directly.

### Remote controller

`ExtCtrlReceiver` binds to `INADDR_ANY`, so sending to the controller host's IP on port `61123` works. Change `127.0.0.1` in the examples to the controller's IP.

## 10. Recommended Deployment Layout

| Process | Host | Scheduling | Notes |
|---|---|---|---|
| Kuavo `main` (Robot) | Control host | `SCHED_FIFO/75` main loop, `SCHED_FIFO/90` leg threads | Keep only RT work |
| `mercury_service_main` | Control host | `SCHED_FIFO/85`, pinned to core 3 | Must be same host for SHM |
| `MqttClient` thread | Control host | `SCHED_FIFO/70` (optional) | For low joystick latency |
| DamiaoSimulator left/right | Remote RT host | `SCHED_FIFO/95`, isolated cores | Avoids RT starvation |
| MQTT broker (`mosquitto`) | Separate host | Normal | Offload non-RT I/O |
| Data logger/visualizer | Separate host | Normal | Read MQTT binary topics or SHM |

### Threads that can be disabled if not used
- `DataManager` — telemetry sender (redundant with SHM/MQTT).
- `ExtCtrlReceiver` — only needed if you use UDP teleop.
- `MoCapManager` — only needed if you have motion capture.
- `MercuryShmInterface` proactive thread — can be merged into main loop (Section 6).

Reducing these threads lowers context-switch and scheduling pressure on the control host, which is the main goal of the RT-priority starvation investigation.