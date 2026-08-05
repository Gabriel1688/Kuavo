# Installation and Controller/Robot Connection

## 1. Controller Service

The controller is managed by a systemd unit file so it starts automatically on boot.

### 1.1 Install the service (system-wide)

Build the controller binary, then copy the service artifacts:

```bash
# Build the controller binary
cd /path/to/Kuavo
cmake --build cmake-build-debug --target mercury_controller

# Install the controller binary and wrapper
sudo cp cmake-build-debug/tools/mercury_controller /usr/local/bin/
sudo chmod +x /usr/local/bin/mercury_controller
sudo cp services/mercury-controller /usr/local/bin/
sudo chmod +x /usr/local/bin/mercury-controller

# Install the systemd unit file
sudo cp services/mercury-controller.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mercury-controller
sudo systemctl start mercury-controller
```

### 1.2 User service (no sudo)

If you do not have passwordless sudo, you can run the controller under your user session:

```bash
mkdir -p ~/.config/systemd/user
cp services/mercury-controller.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable mercury-controller
systemctl --user start mercury-controller
```

You must set `MERCURY_CONTROLLER` to the built binary path inside the user unit file, e.g.:

```ini
Environment="MERCURY_CONTROLLER=/home/<user>/work/Kuavo/cmake-build-debug/tools/mercury_controller"
```

### 1.3 Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `MERCURY_CONTROLLER` | `/usr/local/bin/mercury_controller` | Path to the controller binary |
| `MERCURY_CONTROLLER_FREQ` | `200` | Control loop frequency (Hz) |
| `MERCURY_CONTROLLER_DUR` | `100000` | Maximum run duration (seconds) |
| `MERCURY_CONTROLLER_JOINTS` | `12` | Number of active joints |

Override them with a systemd drop-in file:

```bash
sudo systemctl edit --full mercury-controller
# or, for user service:
systemctl --user edit --full mercury-controller
```

### 1.4 Manage the service

```bash
# Check status
sudo systemctl status mercury-controller
# journal logs
sudo journalctl -u mercury-controller -f

# Stop / restart
sudo systemctl stop mercury-controller
sudo systemctl restart mercury-controller
```

For the user service, drop `sudo` and add `--user`:

```bash
systemctl --user status mercury-controller
systemctl --user restart mercury-controller
```

### 1.5 Debug build quick install

To run the controller directly from the CMake build directory without installing the binary system-wide, create a user service that points at the debug build:

```bash
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/mercury-controller.service << 'EOF'
[Unit]
Description=Mercury controller SHM producer (debug build)
After=network.target

[Service]
Type=simple
ExecStart=/home/gabriel_wang/work/Kuavo/cmake-build-debug/tools/mercury_controller -freq 200 -dur 100000 -joints 12
Restart=on-failure
RestartSec=2
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable mercury-controller
systemctl --user start mercury-controller
```

This is the fastest way to start the controller for development and testing.

## 2. Robot–Controller Connection

The Robot creates and owns the shared memory segment.  The controller waits for the Robot to publish it and then attaches as a consumer.

### 2.1 Shared memory segment

The **Kuavo Robot** creates a POSIX shared memory object named:

```
/dev/shm/mercury_robot_ipc
```

It initializes:
- `version` = `5`
- `num_joints`, `control_freq_hz`, `robot_id`, `motor_can_ids`
- `lifecycle_state` = `RUNNING` (release)
- `magic` = `0x4D455243` ("MERC") (release, written last as the readiness sentinel)

The controller opens the existing segment with `shm_open(O_RDWR)` (no `O_CREAT`), validates `fstat` size, `magic`, `version`, and `lifecycle_state == RUNNING` before proceeding.

### 2.2 Kuavo Robot systemd unit

The Robot's systemd service should include SHM cleanup directives to handle stale segments from a previous crash:

```ini
ExecStartPre=/bin/sh -c '/usr/bin/test -e /dev/shm/mercury_robot_ipc && /bin/rm -f /dev/shm/mercury_robot_ipc || true'
ExecStopPost=/bin/sh -c '/usr/bin/test -e /dev/shm/mercury_robot_ipc && /bin/rm -f /dev/shm/mercury_robot_ipc || true'
Restart=on-failure
```

### 2.3 Robot startup

In `Robot::robotInit()` the Robot creates and initializes the SHM segment directly:

- Calls `shm_open(O_CREAT | O_RDWR)` + `ftruncate(sizeof(SharedMemoryLayout))`
- Initializes all fields (version, joints, frequencies, zeroed buffers)
- Writes `lifecycle_state = RUNNING` and `magic = SHM_MAGIC` as final release-ordered stores
- Exits on `shm_open`/`ftruncate`/`mmap` failure
- Then calls `attachSharedMemory()` to start IMU, Composer, Logger, and leg threads

### 2.4 Controller startup

The mercury controller attaches as a consumer:

- Polls `shm_open(O_RDWR)` every 100ms until the Robot's SHM segment appears
- Validates `fstat` size >= `sizeof(SharedMemoryLayout)`
- Validates `magic`, `version`, `lifecycle_state == RUNNING`
- If any check fails, re-enters the retry loop

### 2.5 Runtime validation

**Robot (`robotPeriodic()`):**
- Checks command-timestamp staleness: reads `cmd_buffers[cmd_write_idx].timestamp_ns`, skips if zero (controller never connected), sets `emergency_stop` if age > 100ms
- Propagates `controller_emergency_stop` to `emergency_stop`

**Controller (main loop):**
- Checks `compose_timestamp_ns` staleness: if age > 100ms, detaches (`munmap`) and re-enters the `shm_open` retry loop

### 2.6 Manual leg re-enable

After reattachment, the legs stay disabled. The operator must explicitly enable them:
- Button press mapped to `MSG_ENABLE_SUBSYSTEM` for left/right leg
- `setEnable(true)` on the `Legged` subsystem re-arms the fault-disable flag and re-enables the motors

The Robot only resumes motor control after this explicit operator action.

### 2.7 Failure scenarios

| Event | Robot behavior | Controller behavior |
|---|---|---|
| Robot graceful shutdown | Sets `lifecycle_state = SHUTTING_DOWN` then `TERMINATED`, calls `shm_unlink` | Detects stale compose data or invalid lifecycle, detaches, retries `shm_open` |
| Robot crash (`kill -9`) | SHM remains until `ExecStopPost` cleans it up; next Robot start creates fresh | Controller detects stale compose data, detaches, retries |
| Controller graceful shutdown (SIGTERM) | Detects stale `cmd.timestamp_ns` (>100ms), sets `emergency_stop`, disables motors | `munmap` only (no `shm_unlink`) |
| Controller crash (`kill -9`) | Same as graceful: stale command timestamp triggers motor disable within 100ms | Nothing — heartbeat was the old mechanism |
| Controller restart | Legs stay disabled until operator re-enables | Re-attaches to existing Robot SHM after up to 100ms retry |

## 3. Real-time Thread Scheduling

The robot uses `SCHED_FIFO` for several threads (UdpServers, IMU reader, Composer, leg subsystems, main loop). If `pthread_setschedparam` fails you will see warnings such as `failed to set SCHED_FIFO/90: Operation not permitted`. Use one of the following setups.

### 3.1 Non-sudo mode (file capabilities)

Grant `CAP_SYS_NICE` to the binary:

```bash
sudo setcap cap_sys_nice+ep cmake-build-debug/Kuavo
./cmake-build-debug/Kuavo
```

**Important:** `setcap` puts the loader in secure-execution mode. If `LD_LIBRARY_PATH` is needed to find libraries (e.g. `/usr/local/lib` or `/opt/grpc/lib`), the variable is ignored and capabilities may be dropped. Instead, register those paths in the trusted linker cache:

```bash
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/usrlocal.conf
# only if you have libraries under /opt/grpc/lib:
echo '/opt/grpc/lib' | sudo tee /etc/ld.so.conf.d/opt-grpc.conf
sudo ldconfig
```

After `ldconfig`, verify the libraries resolve without `LD_LIBRARY_PATH`:

```bash
ldd cmake-build-debug/Kuavo | grep -E 'Mercury_Controller|dynacore'
```

### 3.2 Sudo mode

If you prefer not to set file capabilities, run as root and explicitly preserve the library path:

```bash
sudo LD_LIBRARY_PATH=/usr/local/lib:/opt/grpc/lib ./cmake-build-debug/Kuavo
```

Or, if you already ran the `ldconfig` steps in section 3.1:

```bash
sudo ./cmake-build-debug/Kuavo
```

### 3.3 Cgroup v1 real-time bandwidth

Even with `cap_sys_nice` or `sudo`, the kernel can still deny `SCHED_FIFO` if the process cgroup has no real-time bandwidth allocated. On RHEL/CentOS 8 this is common for `user.slice`:

```bash
cat /sys/fs/cgroup/cpu,cpuacct/user.slice/cpu.rt_runtime_us
```

If the output is `0`, all `SCHED_FIFO` requests in your session will fail. Allow RT scheduling temporarily:

```bash
sudo bash -c 'echo 950000 > /sys/fs/cgroup/cpu,cpuacct/user.slice/cpu.rt_runtime_us'
```

To make this persistent across reboots, create a systemd one-shot unit:

```bash
sudo tee /etc/systemd/system/mercury-rt-bandwidth.service << 'EOF'
[Unit]
Description=Allow real-time scheduling for user sessions
After=local-fs.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo 950000 > /sys/fs/cgroup/cpu,cpuacct/user.slice/cpu.rt_runtime_us'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

Then enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now mercury-rt-bandwidth
```

Confirm the value:

```bash
cat /sys/fs/cgroup/cpu,cpuacct/user.slice/cpu.rt_runtime_us
# expected: 950000
```

## 4. Damiao Simulator Services

For testing without real motor hardware, run one Damiao simulator process per leg as a systemd service.

### 4.1 Build and install the simulator binary

```bash
cd /home/gabriel_wang/work/Kuavo
cmake --build cmake-build-debug --target DamiaoSimulator
sudo cp cmake-build-debug/tools/DamiaoSimulator /usr/local/bin/
sudo cp services/damiao-simulator /usr/local/bin/
sudo chmod +x /usr/local/bin/damiao-simulator
```

### 4.2 Install the left and right leg unit files

```bash
sudo cp services/damiao-simulator-left.service /etc/systemd/system/
sudo cp services/damiao-simulator-right.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now damiao-simulator-left
sudo systemctl enable --now damiao-simulator-right
```

The wrapper reads the environment variables from each unit file and passes them to `DamiaoSimulator`:

| Service | Motor IDs | Local port | Remote port | Simulated leg |
|---|---|---|---|---|
| `damiao-simulator-left` | `1,2,3,4,5,6` | `8886` | `8887` | Left |
| `damiao-simulator-right` | `7,8,9,10,11,12` | `8888` | `8889` | Right |

### 4.3 Manage the simulators

```bash
sudo systemctl status damiao-simulator-left damiao-simulator-right
sudo journalctl -u damiao-simulator-left -f
sudo journalctl -u damiao-simulator-right -f

# Stop / restart
sudo systemctl restart damiao-simulator-left
sudo systemctl restart damiao-simulator-right
```

To override the motor IDs or ports for one service, edit the unit:

```bash
sudo systemctl edit damiao-simulator-left
```

## 5. IMU Simulator Service

The IMU simulator is managed by a systemd unit file so it starts automatically on boot for testing without real IMU hardware.

### 5.1 Install the service (system-wide)

Build the IMU simulator binary, then copy the service artifacts:

```bash
# Build the IMU simulator binary
cd /path/to/Kuavo
cmake --build cmake-build-debug --target ImuSimulator

# Install the IMU simulator binary and wrapper
sudo cp cmake-build-debug/tools/ImuSimulator /usr/local/bin/
sudo chmod +x /usr/local/bin/ImuSimulator
sudo cp services/imu-simulator /usr/local/bin/
sudo chmod +x /usr/local/bin/imu-simulator

# Install the systemd unit file
sudo cp services/imu-simulator.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable imu-simulator
sudo systemctl start imu-simulator
```

### 5.2 User service (no sudo)

If you do not have passwordless sudo, you can run the IMU simulator under your user session:

```bash
mkdir -p ~/.config/systemd/user
cp services/imu-simulator.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable imu-simulator
systemctl --user start imu-simulator
```

You must set `IMU_SIMULATOR` to the built binary path inside the user unit file, e.g.:

```ini
Environment="IMU_SIMULATOR=/home/<user>/work/Kuavo/cmake-build-debug/tools/ImuSimulator"
```

### 5.3 Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `IMU_SIMULATOR` | `/usr/local/bin/ImuSimulator` | Path to the IMU simulator binary |
| `IMU_SIMULATOR_FREQ` | `200` | Streaming frequency (Hz) |
| `IMU_SIMULATOR_BASE` | `0x514` | CAN base ID (hex) |
| `IMU_SIMULATOR_LOCAL` | `8891` | Local UDP port bound by the simulator |
| `IMU_SIMULATOR_REMOTE` | `8890` | Remote UDP port (ImuReader listens here) |

Override them with a systemd drop-in file:

```bash
sudo systemctl edit --full imu-simulator
# or, for user service:
systemctl --user edit --full imu-simulator
```

### 5.4 Manage the service

```bash
# Check status
sudo systemctl status imu-simulator
# journal logs
sudo journalctl -u imu-simulator -f

# Stop / restart
sudo systemctl stop imu-simulator
sudo systemctl restart imu-simulator
```

For the user service, drop `sudo` and add `--user`:

```bash
systemctl --user status imu-simulator
systemctl --user restart imu-simulator
```

### 5.5 Debug build quick install

To run the IMU simulator directly from the CMake build directory without installing the binary system-wide, create a user service that points at the debug build:

```bash
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/imu-simulator.service << 'EOF'
[Unit]
Description=LPMS-IG1 IMU simulator (debug build)
After=network.target

[Service]
Type=simple
ExecStart=/home/gabriel_wang/work/Kuavo/cmake-build-debug/tools/ImuSimulator -freq 200 -base_id 0x514 -local 8891 -remote 8890
Restart=on-failure
RestartSec=2
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable imu-simulator
systemctl --user start imu-simulator
```

This is the fastest way to start the IMU simulator for development and testing.

## 6. MQTT Broker on Raspberry Pi

The Robot uses MQTT over WebSockets. On the Raspberry Pi, install and run Mosquitto as the broker.

### 6.1 Install Mosquitto

```bash
sudo apt-get update
sudo apt-get install -y mosquitto mosquitto-clients
```

### 6.2 Enable and start the broker

```bash
sudo systemctl enable --now mosquitto
systemctl status mosquitto
```

### 6.3 Configure WebSockets (optional but required for browser clients)

Edit `/etc/mosquitto/mosquitto.conf`:

```bash
sudo tee -a /etc/mosquitto/mosquitto.conf << 'EOF'
listener 1883
listener 9001
protocol websockets
allow_anonymous true
EOF
```

Restart Mosquitto:

```bash
sudo systemctl restart mosquitto
```

### 6.4 Quick test on the Pi

Open two terminals on the Pi.

Terminal 1 (subscribe):

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t test/topic -v
```

Terminal 2 (publish):

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t test/topic -m "hello from pi"
```

Terminal 1 should display:

```
test/topic hello from pi
```

### 6.5 Allow remote connections

If you publish from another host, edit the config and ensure the listener binds to all interfaces:

```bash
listener 1883 0.0.0.0
listener 9001 0.0.0.0
protocol websockets
allow_anonymous true
```

Then restart Mosquitto and open the firewall:

```bash
sudo systemctl restart mosquitto
sudo ufw allow 1883/tcp
sudo ufw allow 9001/tcp
```

## 7. Mercury WBLC Service (IJRR_WBLC Integration)

The Mercury WBLC (Whole-Body Controller) from IJRR_WBLC provides advanced whole-body control algorithms integrated with Kuavo via shared memory. This service runs Mercury's WBC controller and communicates with Kuavo through the `/dev/shm/mercury_robot_ipc` shared memory segment.

### 7.1 Build the Mercury WBLC service

```bash
# Navigate to the IJRR_WBLC project
cd /home/gabriel_wang/work/IJRR_WBLC

# Configure and build the project
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake .. -DCMAKE_BUILD_TYPE=Debug
cmake --build . --target mercury_service
```

The mercury_service executable will be created at:
```
/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/mercury_service
```

### 7.2 Install the service (system-wide)

```bash
# Copy the service file
sudo cp /home/gabriel_wang/work/IJRR_WBLC/mercury-service.service /etc/systemd/system/

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable mercury-service
sudo systemctl start mercury-service
```

### 7.3 User service (no sudo)

If you do not have passwordless sudo, you can run the Mercury WBLC service under your user session:

```bash
mkdir -p ~/.config/systemd/user
cp /home/gabriel_wang/work/IJRR_WBLC/mercury-service.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable mercury-service
systemctl --user start mercury-service
```

You must update the `ExecStart` and `WorkingDirectory` paths inside the user unit file, e.g.:

```ini
WorkingDirectory=/home/gabriel_wang/work/IJRR_WBLC
ExecStart=/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/mercury_service
```

### 7.4 Service configuration

The Mercury WBLC service uses the following configuration:

| Parameter | Value | Description |
|---|---|---|
| **Control Frequency** | 400 Hz | Matches Kuavo's Composer thread frequency |
| **Joint Mapping** | 6→12 | Mercury's 6 joints mapped to Kuavo's 12 motor slots via double-cell duplication |
| **Shared Memory** | `/dev/shm/mercury_robot_ipc` | Attaches to Kuavo's existing shared memory |
| **MIT Gains** | kp=50.0, kd=5.0 | Default MIT mode gains for Phase 1 |

### 7.5 Integration details

**Shared Memory Architecture:**
- Mercury WBLC acts as SHM owner/producer or attaches to existing Kuavo SHM
- Uses double-buffered command and sensor data structures
- Implements 6→12 joint expansion via double-cell command mapping
- Implements 12→6 joint reduction for sensor data conversion

**Joint Mapping Strategy:**
- Left leg: Mercury[0-2] → Kuavo[0-5] (2 motors per joint)
- Right leg: Mercury[3-5] → Kuavo[6-11] (2 motors per joint)
- Each Mercury joint command written to 2 consecutive Kuavo motor slots

**Timing:**
- Mercury's servo_rate adapted from 1500Hz to 400Hz to match Kuavo
- Control loop maintains precise 400Hz timing using monotonic clock

### 7.6 Manage the service

```bash
# Check status
sudo systemctl status mercury-service
# journal logs
sudo journalctl -u mercury-service -f

# Stop / restart
sudo systemctl stop mercury-service
sudo systemctl restart mercury-service
```

For the user service, drop `sudo` and add `--user`:

```bash
systemctl --user status mercury-service
systemctl --user restart mercury-service
```

### 7.7 Debug build quick install

To run the Mercury WBLC service directly from the CMake build directory without installing system-wide:

```bash
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/mercury-service.service << 'EOF'
[Unit]
Description=Mercury Whole-Body Controller Service with Kuavo Shared Memory Integration
After=network.target

[Service]
Type=simple
User=gabriel_wang
WorkingDirectory=/home/gabriel_wang/work/IJRR_WBLC
ExecStart=/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/mercury_service
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=mercury-service

# Security settings
NoNewPrivileges=true
PrivateTmp=true

# Environment
LD_LIBRARY_PATH=/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/DynaController/Mercury_Controller:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/Filter:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/RobotSystems/Mercury:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/WBC/WBLC:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/Planner/PIPM_FootPlacementPlanner:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/ExternalSource/rbdl:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/ExternalSource/urdf:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/ExternalSource/Optimizer/Goldfarb:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/Utils:/home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/ExternalSource/ParamHandler

[Install]
WantedBy=multi-user.target
EOF

systemctl --user daemon-reload
systemctl --user enable mercury-service
systemctl --user start mercury-service
```

### 7.8 Manual testing

For quick testing without systemd, run the service directly:

```bash
cd /home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug
./mercury_service

# Or with timeout for testing
timeout 30 ./mercury_service
```

### 7.9 Troubleshooting

**Shared memory permission denied:**
```bash
# Check if shared memory exists
ls -l /dev/shm/mercury_robot_ipc

# If owned by root and inaccessible, the service will attach to existing SHM
# or create new SHM as the current user
```

**Service fails to start:**
```bash
# Check detailed logs
sudo journalctl -u mercury-service -n 50

# Verify library dependencies
ldd /home/gabriel_wang/work/IJRR_WBLC/cmake-build-debug/mercury_service
```

**Timing issues:**
- Ensure Mercury's servo_rate is set to 1.0/400.0 in `RobotSystems/Mercury/Mercury_Definition.h`
- Verify system has sufficient CPU for 400Hz control loop
- Check for other processes consuming CPU resources

**Integration with Kuavo:**
- Mercury WBLC will attach to existing Kuavo SHM if available
- If Kuavo SHM doesn't exist, Mercury WBLC will create it
- Both systems must use compatible SHM version (currently version 4)
