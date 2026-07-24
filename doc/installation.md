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

The Robot does not create the shared memory segment. It waits for the controller service to publish it and then attaches as a consumer.

### 2.1 Shared memory segment

The controller creates a POSIX shared memory object named:

```
/dev/shm/mercury_robot_ipc
```

It publishes:
- `magic` = `0x4D455243` ("MERC")
- `version` = `3`
- `lifecycle_state` = `RUNNING`
- `controller_heartbeat_ns` updated every control cycle

### 2.2 Robot startup attach

In `Robot::robotInit()` the Robot calls `tryAttachSharedMemory()` in a loop:

- Polls every `100 ms`
- Times out after `30 s`
- Exits if no valid SHM appears within the timeout

This means the Robot tolerates the controller service taking up to 30 seconds to become ready. Once attached, `Robot::attachSharedMemory()` starts the IMU, Composer, Logger, and leg threads.

### 2.3 Runtime validation and reconnection

`Robot::robotPeriodic()` validates the SHM every cycle:
- `magic` matches `SHM_MAGIC`
- `version` matches `SHM_VERSION`
- `lifecycle_state` is `RUNNING`
- `controller_heartbeat_ns` is fresh (stale threshold: `100 ms`)

If any check fails, `Robot::detachSharedMemory()`:
- Disables both legs
- Pauses and clears leg SHM pointers
- Stops IMU, Composer, Logger
- Unmaps the SHM

Then `robotPeriodic()` retries `shm_open` every `10 cycles` (`100 ms`) until the controller service is back.

### 2.4 Manual leg re-enable

After reattachment, the legs stay disabled. The operator must explicitly enable them:
- Button press mapped to `MSG_ENABLE_SUBSYSTEM` for left/right leg
- `setEnable(true)` on the `Legged` subsystem re-arms the fault-disable flag and re-enables the motors

The Robot only resumes motor control after this explicit operator action.

### 2.5 Failure scenarios

| Event | Controller behavior | Robot behavior |
|---|---|---|
| Graceful shutdown (SIGTERM) | Sets `SHUTTING_DOWN` -> `emergency_stop=true` -> `TERMINATED`, then unlinks SHM | Detects lifecycle change, detaches, retries `shm_open` |
| Crash (`kill -9`) | Heartbeat stops | Detects stale heartbeat (>100 ms), detaches, disables motors, retries `shm_open` |
| Controller restart | Recreates SHM | Reattaches after up to 100 ms retry; legs stay disabled until operator enables |

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
