/*
# Reduced IMU Simulator: Acc + Gyro + Quaternion Only

## Revised Data Layout

The LPMS-IG1 supports configurable data channels via the `SET_TRANSMIT_DATA` command [1]. The full 32-bit float channel assignment table includes accelerometer (channels 1-3), gyroscope (channels 7-9), magnetometer (channels 25-27), quaternion (channels 34-37), and Euler angles (channels 38-40) [1]. By disabling magnetometer and Euler angle transmission, only **10 float32 values** remain, requiring **5 CAN frames** instead of 8.

### Original Layout (16 floats, 8 frames) [2]

| CAN ID | Slot 0 | Slot 1 | Status |
|:------:|--------|--------|:------:|
| 0x514 | accX | accY | ✅ Keep |
| 0x515 | accZ | gyroX | ✅ Keep |
| 0x516 | gyroY | gyroZ | ✅ Keep |
| 0x517 | magX | magY | ❌ Remove |
| 0x518 | magZ | eulerX | ❌ Remove |
| 0x519 | eulerY | eulerZ | ❌ Remove |
| 0x51A | quatW | quatX | ✅ Keep |
| 0x51B | quatY | quatZ | ✅ Keep |

### Revised Layout (10 floats, 5 frames)

With magnetometer and Euler removed, the remaining channels are repacked sequentially into 5 CAN frames starting from the same base ID (0x514) [1][2]:

| CAN ID | Offset | Slot 0 | Slot 1 | Unit | Channel [1] |
|:------:|:------:|--------|--------|------|:-----------:|
| 0x514 | 0 | accX | accY | g | 1, 2 |
| 0x515 | 1 | accZ | gyroX | g, dps | 3, 7 |
| 0x516 | 2 | gyroY | gyroZ | dps | 8, 9 |
| 0x517 | 3 | quatW | quatX | — | 34, 35 |
| 0x518 | 4 | quatY | quatZ | — | 36, 37 |

This gives the ImuReader a CAN ID range of `[0x514, 0x518]` — 5 frames instead of 8 [2][4].

The 7D state vector exposed to the Mercury Controller remains `[eulerX, eulerY, eulerZ, quatW, quatX, quatY, quatZ]` [2], but since Euler angles are no longer transmitted, they must be **computed from the quaternion** on the receiver side.

## Complete Implementation

*/

/**
 * @file imu_simulator.cpp
 * @brief LPMS-IG1 IMU Simulator — CAN-over-UDP (Reduced: Acc+Gyro+Quat)
 *
 * Simulates an LPMS-IG1 IMU sensor streaming 5 sequential CAN frames
 * per measurement cycle at 500Hz using 32-bit float precision.
 * Transmits only accelerometer, gyroscope, and quaternion — magnetometer
 * and Euler angles are disabled.
 *
 * Data channels enabled (per SET_TRANSMIT_DATA bitmask [1]):
 *   Bit 0: Accelerometer raw data (channels 1-3: accX/Y/Z in g)
 *   Bit 2: GyroI raw data (channels 7-9: gyroX/Y/Z in dps)
 *   Bit 11: Quaternion orientation (channels 34-37: quatW/X/Y/Z)
 *   Bit 8 (mag): DISABLED
 *   Bit 12 (euler): DISABLED
 *
 * CAN frame layout (5 frames × 2 float32 = 10 values per cycle):
 *   Frame 0x514: accX, accY          (g)
 *   Frame 0x515: accZ, gyroX         (g, dps)
 *   Frame 0x516: gyroY, gyroZ        (dps)
 *   Frame 0x517: quatW, quatX        (unitless)
 *   Frame 0x518: quatY, quatZ        (unitless)
 *
 * CAN-over-UDP frame format (13 bytes, same as DamiaoSimulator [7]):
 *   Byte 0:    DLC (0x08)
 *   Bytes 1-4: CAN ID (big-endian, 4 bytes)
 *   Bytes 5-12: CAN Data (8 bytes = 2 × float32, little-endian)
 *
 * Architecture context:
 *   - IMU shares UDP port 8887 with UdpServer 0 (left leg motors) [2][3]
 *   - ImuReader filters by CAN ID range [base..base+N) [4]
 *   - Motor receive IDs are 0x11-0x1C — no overlap with IMU 0x514+ [2]
 *   - DamiaoSimulator IMU mode sends messages with 250us inter-frame [7]
 *
 * Build:
 *   g++ -O2 -std=c++20 -pthread -lrt -lm -o imu_simulator imu_simulator.cpp
 *
 * Usage:
 *   ./imu_simulator -local 8886 -remote 8887
 *   ./imu_simulator -freq 500 -base_id 0x514 -local 8886 -remote 8887
 */

#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace std::chrono_literals;

// ============================================================
// IMU Configuration
// ============================================================

// Sequential CAN mode start ID — default 0x514 (1300 decimal) [1]
static constexpr uint32_t DEFAULT_CAN_BASE_ID = 0x514;

// Reduced frame count: acc(3) + gyro(3) + quat(4) = 10 floats = 5 frames
// Original was 8 frames for 16 floats [2]
static constexpr int NUM_CAN_FRAMES = 5;

// 2 float32 values per CAN frame (32-bit precision mode) [1]
static constexpr int FLOATS_PER_FRAME = 2;

// Total float32 values per cycle: 5 frames × 2 = 10
static constexpr int TOTAL_FLOATS = NUM_CAN_FRAMES * FLOATS_PER_FRAME;

// CAN-over-UDP frame size [7]
static constexpr int CAN_FRAME_SIZE = 13;

// Default streaming frequency [1]
static constexpr int DEFAULT_FREQ_HZ = 500;

// Inter-frame delay: 250us between messages [7]
// "std::this_thread::sleep_for(250us)" [7]
static constexpr int INTER_FRAME_US = 250;

// ============================================================
// Simulated IMU Sensor State
// ============================================================

struct ImuState {
    // Accelerometer (g) — channels 1-3 [1]
    float accX  = 0.0f;
    float accY  = 0.0f;
    float accZ  = 1.0f;   // Gravity along Z at rest

    // Gyroscope (degrees per second) — channels 7-9 [1]
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;

    // Quaternion (unitless) — channels 34-37 [1]
    float quatW = 1.0f;
    float quatX = 0.0f;
    float quatY = 0.0f;
    float quatZ = 0.0f;

    /**
     * Update simulated IMU state with gentle sinusoidal motion.
     *
     * Euler angles are computed internally for quaternion generation
     * but NOT transmitted — the receiver must derive Euler from
     * quaternion if needed.
     *
     * Euler angle definition [1]:
     *   Roll: rotation around global X, -180° to 180°
     *   Pitch: rotation around global Y, -90° to 90°
     *   Yaw: rotation around global Z
     *   ZYX global type (aerospace sequence) [1]
     */
    void update(uint64_t cycle, double freq_hz) {
        double t = static_cast<double>(cycle) / freq_hz;

        // Internal Euler angles for quaternion computation (not transmitted)
        double roll  = 5.0 * std::sin(2.0 * M_PI * 0.3 * t);   // ±5 deg
        double pitch = 3.0 * std::sin(2.0 * M_PI * 0.5 * t);   // ±3 deg
        double yaw   = 2.0 * std::sin(2.0 * M_PI * 0.1 * t);   // ±2 deg

        // Quaternion from Euler (ZYX aerospace sequence) [1]
        double r = roll  * M_PI / 180.0 / 2.0;
        double p = pitch * M_PI / 180.0 / 2.0;
        double y = yaw   * M_PI / 180.0 / 2.0;

        double cr = std::cos(r), sr = std::sin(r);
        double cp = std::cos(p), sp = std::sin(p);
        double cy = std::cos(y), sy = std::sin(y);

        quatW = static_cast<float>(cr * cp * cy + sr * sp * sy);
        quatX = static_cast<float>(sr * cp * cy - cr * sp * sy);
        quatY = static_cast<float>(cr * sp * cy + sr * cp * sy);
        quatZ = static_cast<float>(cr * cp * sy - sr * sp * cy);

        // Normalize quaternion to unit length
        float norm = std::sqrt(quatW*quatW + quatX*quatX +
                               quatY*quatY + quatZ*quatZ);
        if (norm > 0.0f) {
            quatW /= norm;
            quatX /= norm;
            quatY /= norm;
            quatZ /= norm;
        }

        // Gyroscope — angular velocity (dps) [1]
        gyroX = static_cast<float>(
            5.0 * 2.0 * M_PI * 0.3 * std::cos(2.0 * M_PI * 0.3 * t));
        gyroY = static_cast<float>(
            3.0 * 2.0 * M_PI * 0.5 * std::cos(2.0 * M_PI * 0.5 * t));
        gyroZ = static_cast<float>(
            2.0 * 2.0 * M_PI * 0.1 * std::cos(2.0 * M_PI * 0.1 * t));

        // Accelerometer — gravity + tilt effect (g) [1]
        float roll_rad  = static_cast<float>(roll * M_PI / 180.0);
        float pitch_rad = static_cast<float>(pitch * M_PI / 180.0);
        accX = -std::sin(pitch_rad);
        accY =  std::sin(roll_rad) * std::cos(pitch_rad);
        accZ =  std::cos(roll_rad) * std::cos(pitch_rad);
    }

    /**
     * Pack 10 float32 values into the reduced canonical order:
     *
     *   Frame 0x514: slot[0]=accX,  slot[1]=accY
     *   Frame 0x515: slot[2]=accZ,  slot[3]=gyroX
     *   Frame 0x516: slot[4]=gyroY, slot[5]=gyroZ
     *   Frame 0x517: slot[6]=quatW, slot[7]=quatX
     *   Frame 0x518: slot[8]=quatY, slot[9]=quatZ
     *
     * Magnetometer (channels 25-27) and Euler (channels 38-40)
     * are NOT included [1].
     */
    void pack(float values[TOTAL_FLOATS]) const {
        values[0] = accX;    values[1] = accY;     // Frame 0x514
        values[2] = accZ;    values[3] = gyroX;    // Frame 0x515
        values[4] = gyroY;   values[5] = gyroZ;    // Frame 0x516
        values[6] = quatW;   values[7] = quatX;    // Frame 0x517
        values[8] = quatY;   values[9] = quatZ;    // Frame 0x518
    }
};

// ============================================================
// CAN-over-UDP Frame Builder
// ============================================================

/**
 * Build a 13-byte CAN-over-UDP frame [7].
 *
 * Format matches the DamiaoSimulator [7]:
 *   Byte 0:    DLC (0x08)
 *   Bytes 1-4: CAN ID (big-endian)
 *   Bytes 5-8: float32 slot 0 (little-endian)
 *   Bytes 9-12: float32 slot 1 (little-endian)
 */
static void build_can_frame(uint8_t frame[CAN_FRAME_SIZE],
                             uint32_t can_id,
                             float value0, float value1) {
    frame[0] = 0x08;  // DLC [7]

    // CAN ID big-endian [7]
    frame[1] = (can_id >> 24) & 0xFF;
    frame[2] = (can_id >> 16) & 0xFF;
    frame[3] = (can_id >> 8)  & 0xFF;
    frame[4] = can_id & 0xFF;

    // 2 × float32 little-endian [1]
    std::memcpy(&frame[5], &value0, sizeof(float));
    std::memcpy(&frame[9], &value1, sizeof(float));
}

// ============================================================
// IMU Simulator
// ============================================================

class ImuSimulator {
public:
    ImuSimulator(int localPort, int remotePort,
                 uint32_t baseCanId, int freqHz)
        : localPort_(localPort),
          remotePort_(remotePort),
          baseCanId_(baseCanId),
          freqHz_(freqHz) {
        cyclePeriodUs_ = 1'000'000 / freqHz;
    }

    ~ImuSimulator() {
        if (sockfd_ >= 0) close(sockfd_);
    }

    /**
     * Initialize UDP socket following DamiaoSimulator pattern [7]:
     *   - Non-blocking (fcntl O_NONBLOCK)
     *   - Bind to localhost on localPort
     *   - Remote address = localhost on remotePort
     */
    bool init() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sockfd_ < 0) {
            perror("Error creating socket");
            return false;
        }

        fcntl(sockfd_, F_SETFL, O_NONBLOCK);  // [7]

        struct sockaddr_in clientAddr;
        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddr.sin_family = AF_INET;
        clientAddr.sin_port = htons(localPort_);
        clientAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (bind(sockfd_, (struct sockaddr*)&clientAddr,
                 sizeof(clientAddr)) < 0) {
            perror("Error binding socket");
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        memset(&remoteAddr_, 0, sizeof(remoteAddr_));
        remoteAddr_.sin_family = AF_INET;
        remoteAddr_.sin_port = htons(remotePort_);
        remoteAddr_.sin_addr.s_addr = inet_addr("127.0.0.1");

        printf("IMU Simulator (Reduced: Acc+Gyro+Quat)\n");
        printf("  CAN base ID:    0x%03X (range: 0x%03X - 0x%03X)\n",
               baseCanId_, baseCanId_, baseCanId_ + NUM_CAN_FRAMES - 1);
        printf("  Frequency:      %d Hz\n", freqHz_);
        printf("  Precision:      32-bit float\n");
        printf("  Frames/cycle:   %d (%d float32 values)\n",
               NUM_CAN_FRAMES, TOTAL_FLOATS);
        printf("  Channels:       acc(3) + gyro(3) + quat(4)\n");
        printf("  Disabled:       magnetometer, Euler angles\n");
        printf("  Inter-frame:    %d us\n", INTER_FRAME_US);
        printf("  Local port:     %d\n", localPort_);
        printf("  Remote port:    %d\n", remotePort_);
        printf("  Frame size:     %d bytes\n\n", CAN_FRAME_SIZE);

        return true;
    }

    /**
     * Run the IMU streaming loop at the configured frequency.
     *
     * Each cycle [7]:
     *   1. Update simulated IMU state
     *   2. Pack 10 float32 values (acc+gyro+quat)
     *   3. Send 5 CAN frames with 250us between each
     *   4. Sleep until next cycle
     *
     * Total time per cycle at 500Hz = 2000us [7]:
     *   5 frames × 250us inter-frame = 1000us sending
     *   ~1000us remaining for computation + sleep
     */
    void run() {
        printf("Streaming at %d Hz (Ctrl+C to stop)...\n\n", freqHz_);

        uint64_t cycle = 0;
        uint64_t totalFramesSent = 0;
        auto startTime = std::chrono::steady_clock::now();

        while (running_) {
            auto cycleStart = std::chrono::steady_clock::now();

            // Update sensor state
            state_.update(cycle, static_cast<double>(freqHz_));

            // Pack values: acc(3) + gyro(3) + quat(4) = 10 floats
            float values[TOTAL_FLOATS];
            state_.pack(values);

            // Send 5 sequential CAN frames
            for (int f = 0; f < NUM_CAN_FRAMES; f++) {
                uint32_t canId = baseCanId_ + f;
                float v0 = values[f * FLOATS_PER_FRAME];
                float v1 = values[f * FLOATS_PER_FRAME + 1];

                uint8_t frame[CAN_FRAME_SIZE];
                build_can_frame(frame, canId, v0, v1);

                ssize_t sent = sendto(sockfd_, frame, CAN_FRAME_SIZE, 0,
                                      (struct sockaddr*)&remoteAddr_,
                                      sizeof(struct sockaddr_in));
                if (sent == CAN_FRAME_SIZE) {
                    totalFramesSent++;
                }

                // Inter-frame delay [7]
                if (f < NUM_CAN_FRAMES - 1) {
                    std::this_thread::sleep_for(
                        std::chrono::microseconds(INTER_FRAME_US));
                }
            }

            cycle++;

            // Periodic status
            if (cycle % (freqHz_ * 2) == 0) {
                auto elapsed = std::chrono::steady_clock::now() - startTime;
                double secs = std::chrono::duration<double>(elapsed).count();

                // Compute Euler from quaternion for display only
                // (NOT transmitted over CAN) [1]
                float w = state_.quatW, x = state_.quatX;
                float y = state_.quatY, z = state_.quatZ;
                float roll  = std::atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                              * 180.0f / M_PI;
                float pitch = std::asin(std::max(-1.0f,
                              std::min(1.0f, 2*(w*y - z*x))))
                              * 180.0f / M_PI;
                float yaw   = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                              * 180.0f / M_PI;

                printf("  [%.1fs] cycles=%lu  frames=%lu  rate=%.1f Hz\n"
                       "          acc=[%.3f, %.3f, %.3f] g\n"
                       "          gyro=[%.2f, %.2f, %.2f] dps\n"
                       "          quat=[%.4f, %.4f, %.4f, %.4f]\n"
                       "          euler=[%.2f, %.2f, %.2f] deg (derived)\n",
                       secs, cycle, totalFramesSent, cycle / secs,
                       state_.accX, state_.accY, state_.accZ,
                       state_.gyroX, state_.gyroY, state_.gyroZ,
                       state_.quatW, state_.quatX,
                       state_.quatY, state_.quatZ,
                       roll, pitch, yaw);
            }

            // Sleep until next cycle
            auto cycleEnd = std::chrono::steady_clock::now();
            auto cycleElapsed = std::chrono::duration_cast<
                std::chrono::microseconds>(cycleEnd - cycleStart).count();
            int64_t sleepUs = cyclePeriodUs_ - cycleElapsed;

            if (sleepUs > 0) {
                struct timespec ts;
                ts.tv_sec = 0;
                ts.tv_nsec = sleepUs * 1000;
                clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, nullptr);
            }
        }

        auto totalElapsed = std::chrono::steady_clock::now() - startTime;
        double totalSecs = std::chrono::duration<double>(totalElapsed).count();
        printf("\nIMU Simulator stopped.\n");
        printf("  Total cycles:      %lu\n", cycle);
        printf("  Total frames:      %lu\n", totalFramesSent);
        printf("  Duration:          %.2f seconds\n", totalSecs);
        printf("  Effective rate:    %.1f Hz\n", cycle / totalSecs);
        printf("  Frame throughput:  %.1f frames/sec\n",
               totalFramesSent / totalSecs);
    }

    void stop() { running_ = false; }

private:
    int sockfd_ = -1;
    int localPort_;
    int remotePort_;
    uint32_t baseCanId_;
    int freqHz_;
    int cyclePeriodUs_;
    struct sockaddr_in remoteAddr_{};
    ImuState state_;
    volatile bool running_ = true;
};

// ============================================================
// Main
// ============================================================

static ImuSimulator* g_simulator = nullptr;
static void signal_handler(int) {
    if (g_simulator) g_simulator->stop();
}

static void printUsage(const char* prog) {
    printf("LPMS-IG1 IMU Simulator (Reduced: Acc+Gyro+Quat)\n\n");
    printf("Usage: %s [options]\n", prog);
    printf("  -local PORT      Local UDP port (default: 8886)\n");
    printf("  -remote PORT     Remote UDP port (default: 8887)\n");
    printf("  -freq HZ         Streaming frequency (default: 500)\n");
    printf("  -base_id ID      CAN base ID in hex (default: 0x514)\n");
    printf("  -h, --help       Show this help\n");
    printf("\nCAN frame layout (5 frames, 10 floats per cycle):\n");
    printf("  0x514: accX, accY          (g)\n");
    printf("  0x515: accZ, gyroX         (g, dps)\n");
    printf("  0x516: gyroY, gyroZ        (dps)\n");
    printf("  0x517: quatW, quatX        (unitless)\n");
    printf("  0x518: quatY, quatZ        (unitless)\n");
    printf("\nDisabled channels: magnetometer, Euler angles\n");
    printf("  Euler angles must be derived from quaternion on receiver\n");
}

int main(int argc, char* argv[]) {
    int localPort  = 8886;
    int remotePort = 8887;   // Shares with UdpServer 0 [2][3]
    int freqHz     = DEFAULT_FREQ_HZ;
    uint32_t baseCanId = DEFAULT_CAN_BASE_ID;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-local") == 0 && i + 1 < argc) {
            localPort = atoi(argv[++i]);
            if (localPort <= 0 || localPort > 65535) {
                fprintf(stderr, "Error: invalid local port\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-remote") == 0 && i + 1 < argc) {
            remotePort = atoi(argv[++i]);
            if (remotePort <= 0 || remotePort > 65535) {
                fprintf(stderr, "Error: invalid remote port\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-freq") == 0 && i + 1 < argc) {
            freqHz = atoi(argv[++i]);
            if (freqHz <= 0 || freqHz > 1000) {
                fprintf(stderr, "Error: frequency must be 1-1000 Hz\n");
                return 1;
            }
        } else if (strcmp(argv[i], "-base_id") == 0 && i + 1 < argc) {
            baseCanId = static_cast<uint32_t>(
                strtoul(argv[++i], nullptr, 0));
        } else if (strcmp(argv[i], "-h") == 0 ||
                   strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Error: unknown argument: %s\n", argv[i]);
            printUsage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    ImuSimulator sim(localPort, remotePort, baseCanId, freqHz);
    g_simulator = &sim;

    if (!sim.init()) return 1;
    sim.run();

    return 0;
}
/*


## Receiver-Side Impact

The `ImuReader` on the Kuavo actuator side must be updated to match the reduced frame count [2][4]:

| Parameter | Original [2][4] | Revised |
|-----------|:---:|:---:|
| CAN ID range | 0x514 – 0x51B (8 frames) | 0x514 – 0x518 (5 frames) |
| Floats per cycle | 16 | 10 |
| Frame count check | `frame_counter == 8` | `frame_counter == 5` |
| Euler angles | Parsed from frames 0x518-0x519 | **Computed from quaternion** |
| Magnetometer | Parsed from frame 0x517 | **Not available** |

### Euler Derivation from Quaternion

Since Euler angles (channels 38-40) are no longer transmitted, the receiver must compute them from the quaternion using the ZYX aerospace sequence defined in the LPMS-IG1 manual [1]:

```cpp
// ZYX global type (aerospace sequence) [1]
// Roll: rotation around global X, -180° to 180°
// Pitch: rotation around global Y, -90° to 90°
float roll  = atan2(2*(quatW*quatX + quatY*quatZ),
                    1 - 2*(quatX*quatX + quatY*quatY));
float pitch = asin(max(-1.0f, min(1.0f,
                    2*(quatW*quatY - quatZ*quatX))));
float yaw   = atan2(2*(quatW*quatZ + quatX*quatY),
                    1 - 2*(quatY*quatY + quatZ*quatZ));
```

### Updated ImuStageData Struct

The `ImuStageData` struct in `mercury_shm.hpp` should reflect the reduced data set:

```cpp
struct ImuStageData {
    double imu_acc[3];       // accX, accY, accZ (from frames 0x514-0x515)
    double imu_ang_vel[3];   // gyroX, gyroY, gyroZ (from frames 0x515-0x516)
    double quat[4];          // quatW, quatX, quatY, quatZ (from frames 0x517-0x518)
    // Euler angles derived from quaternion — NOT from CAN frames
    double euler[3];         // roll, pitch, yaw (computed, not transmitted)
    uint64_t timestamp_ns;
    uint64_t sequence;
};
```

## Build & Run

```bash
# Build
g++ -O2 -std=c++20 -pthread -lrt -lm -o imu_simulator imu_simulator.cpp

# Run with defaults (500Hz, base ID 0x514, ports 8886→8887)
./imu_simulator

# Run alongside DamiaoSimulator for full system test
# Terminal 1: Motor simulator (left leg)
./damiao_simulator -mode motor -local 8886 -remote 8887

# Terminal 2: IMU simulator (shares remote port 8887 with motors)
./imu_simulator -local 8890 -remote 8887

# Run at reduced frequency for debugging
./imu_simulator -freq 100
```

## Key Differences from Full-Channel Version

| Aspect | Full (Original) | Reduced (This Version) |
|--------|:---:|:---:|
| CAN frames per cycle | 8 | **5** |
| Float values per cycle | 16 | **10** |
| CAN ID range | 0x514 – 0x51B | **0x514 – 0x518** |
| Channels transmitted | acc, gyro, mag, euler, quat | **acc, gyro, quat only** |
| Euler source | Direct from CAN (channels 38-40) [1] | **Derived from quaternion** |
| Magnetometer | Transmitted (channels 25-27) [1] | **Not available** |
| Cycle time budget | 8 × 250μs = 2000μs | **5 × 250μs = 1000μs** |
| Bandwidth per cycle | 8 × 13 = 104 bytes | **5 × 13 = 65 bytes** |
| SET_TRANSMIT_DATA bits [1] | 0, 2, 8, 11, 12 enabled | **0, 2, 11 enabled** |
    */