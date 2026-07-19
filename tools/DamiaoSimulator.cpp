/**
 * @file damiao_multi_simulator.cpp
 * @brief Damiao Multi-Motor Simulator — Supports up to 6 motors
 *
 * Each motor has its own CAN ID, state, and feedback generation.
 * All motors share a single UDP socket (CAN bus model).
 *
 * Protocol references:
 *   [1] Original damiao_simulator.cpp
 *   [2] 达妙驱动控制协议 V1.4
 *   [3] openarm_can/dm_motor_control.cpp
 *
 * Usage:
 *   # Single motor (backward compatible)
 *   ./damiao_multi_simulator -ids 1
 *
 *   # 6 motors with IDs 1-6
 *   ./damiao_multi_simulator -ids 1,2,3,4,5,6
 *
 *   # Custom IDs
 *   ./damiao_multi_simulator -ids 0x01,0x02,0x03,0x04,0x05,0x06
 */

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <memory>
using namespace spdlog;
using namespace std::chrono_literals;
void setupLogger() ;
static constexpr size_t MAX_MOTORS = 12;
static constexpr size_t CAN_FRAME_SIZE = 13;

// ============================================================
// Motor Error/Status Codes [2]
// ============================================================
enum class MotorStatus : uint8_t {
    DISABLED     = 0x00,
    ENABLED      = 0x01,
    OVERVOLTAGE  = 0x08,
    UNDERVOLTAGE = 0x09,
    OVERCURRENT  = 0x0A,
    MOS_OVERTEMP = 0x0B,
    COIL_OVERTEMP= 0x0C,
    COMM_LOST    = 0x0D,
    OVERLOAD     = 0x0E,
};

// ============================================================
// Motor Limit Parameters [2][3]
// ============================================================
struct LimitParam {
    float pMax;
    float vMax;
    float tMax;
};

static const LimitParam DEFAULT_LIMITS = {12.5f, 45.0f, 18.0f};

// ============================================================
// Per-Motor State
// ============================================================
struct MotorState {
    uint16_t    canId      = 0x01;
    uint16_t    masterId   = 0x00;
    MotorStatus status     = MotorStatus::DISABLED;

    double position    = 0.0;
    double velocity    = 0.0;
    double torque      = 0.0;
    int    mosFetTemp  = 35;
    int    rotorTemp   = 40;

    double targetPos     = 0.0;
    double targetVel     = 0.0;
    double targetTorque  = 0.0;
    double kp            = 0.0;
    double kd            = 0.0;

    LimitParam limits    = DEFAULT_LIMITS;
    bool zeroSaved       = false;

    // Parameter storage for read/write commands [3]
    std::map<uint8_t, float> parameters;

    void initDefaults() {
        parameters[0x05] = limits.pMax;
        parameters[0x06] = limits.vMax;
        parameters[0x07] = static_cast<float>(canId);
        parameters[0x08] = static_cast<float>(masterId);
        parameters[0x09] = 10000.0f;
        parameters[0x0A] = limits.tMax;
        parameters[0x0B] = 100.0f;
    }
};

// ============================================================
// Protocol Helpers [2][3]
// ============================================================

static uint16_t double_to_uint(double x, double x_min, double x_max, int bits) {
    x = std::max(x_min, std::min(x, x_max));
    double span = x_max - x_min;
    return static_cast<uint16_t>(((x - x_min) / span) * ((1 << bits) - 1));
}

static double uint_to_double(uint16_t x, double min_val, double max_val, int bits) {
    double span = max_val - min_val;
    return (static_cast<double>(x) / ((1 << bits) - 1)) * span + min_val;
}

static float bytes_to_float(const uint8_t* bytes) {
    float v;
    std::memcpy(&v, bytes, sizeof(float));
    return v;
}

static uint32_t bytes_to_uint32(const uint8_t* bytes) {
    uint32_t v;
    std::memcpy(&v, bytes, sizeof(uint32_t));
    return v;
}

static bool is_int_param(int rid) {
    return (7 <= rid && rid <= 10) || (13 <= rid && rid <= 16) ||
           (35 <= rid && rid <= 36);
}

static uint32_t extract_can_id(const uint8_t* msg) {
    return (static_cast<uint32_t>(msg[1]) << 24) |
           (static_cast<uint32_t>(msg[2]) << 16) |
           (static_cast<uint32_t>(msg[3]) << 8)  |
           static_cast<uint32_t>(msg[4]);
}

static void set_can_id(uint8_t* msg, uint32_t canId) {
    msg[1] = (canId >> 24) & 0xFF;
    msg[2] = (canId >> 16) & 0xFF;
    msg[3] = (canId >> 8)  & 0xFF;
    msg[4] = canId & 0xFF;
}

// ============================================================
// Command Type Enum
// ============================================================
enum class CommandType {
    UNKNOWN,
    ENABLE,
    DISABLE,
    SAVE_ZERO,
    CLEAR_ERROR,
    MIT_CONTROL,
    POSVEL_CONTROL,
    VELOCITY_CONTROL,
    POSFORCE_CONTROL,
    QUERY_PARAM,
    WRITE_PARAM,
    REFRESH,
};

// ============================================================
// Multi-Motor Simulator
// ============================================================
class DamiaoMultiSimulator {
public:
    DamiaoMultiSimulator(int localPort, int remotePort,
                         const std::vector<uint16_t>& motorIds)
        : localPort_(localPort), remotePort_(remotePort) {

        if (motorIds.size() > MAX_MOTORS) {
            std::cerr << "Error: max " << MAX_MOTORS
                      << " motors supported, got " << motorIds.size() << std::endl;
            std::exit(1);
        }

        // Initialize each motor with unique CAN ID and MasterID
        for (size_t i = 0; i < motorIds.size(); i++) {
            MotorState motor;
            motor.canId    = motorIds[i];
            motor.masterId = motorIds[i];  // MasterID defaults to CAN ID [2]
            motor.initDefaults();
            motors_[motorIds[i]] = motor;
            SPDLOG_INFO(" Motor {}:  CAN_ID=[0x{}] MasterID=[0x{}]", i, motorIds[i], motorIds[i]);
            std::cout << "  Motor " << i << ": CAN_ID=0x"
                      << std::hex << motorIds[i]
                      << " MasterID=0x" << motorIds[i]
                      << std::dec << std::endl;
        }
    }

    ~DamiaoMultiSimulator() {
        if (sockfd_ >= 0) close(sockfd_);
    }

    // ========================================================
    // Socket Initialization [1]
    // ========================================================
    bool init() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sockfd_ < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return false;
        }

        fcntl(sockfd_, F_SETFL, O_NONBLOCK);

        struct sockaddr_in clientAddr;
        memset(&clientAddr, 0, sizeof(clientAddr));
        clientAddr.sin_family = AF_INET;
        clientAddr.sin_port = htons(localPort_);
        clientAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

        if (bind(sockfd_, (struct sockaddr*)&clientAddr, sizeof(clientAddr)) < 0) {
            std::cerr << "Error binding to port " << localPort_ << std::endl;
            close(sockfd_);
            return false;
        }

        memset(&remoteAddr_, 0, sizeof(remoteAddr_));
        remoteAddr_.sin_family = AF_INET;
        remoteAddr_.sin_port = htons(remotePort_);
        remoteAddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        SPDLOG_INFO("Multi-motor simulator initialized: motors=[{}] local=[{}] remote=[{}]", motors_.size(), localPort_, remotePort_);
        return true;
    }

    // ========================================================
    // Motor Mode — epoll command-response [1]
    // ========================================================
    void runMotorMode() {
        int epfd = epoll_create(2);
        struct epoll_event ev;
        ev.data.fd = sockfd_;
        ev.events = EPOLLIN;
        epoll_ctl(epfd, EPOLL_CTL_ADD, sockfd_, &ev);

        struct epoll_event events[2];
        bool isConnected = true;
        SPDLOG_INFO("waiting for commands [{}] motors ...", motors_.size());

        while (isConnected) {
            int ready = epoll_wait(epfd, events, 2, -1);
            if (ready < 0) {
                perror("epoll_wait error");
                break;
            }
            if (ready == 0) continue;

            for (int i = 0; i < ready; i++) {
                if (events[i].data.fd == sockfd_) {
                    uint8_t msg[CAN_FRAME_SIZE];
                    memset(msg, 0, CAN_FRAME_SIZE);

                    ssize_t numBytes = recvfrom(sockfd_, msg, CAN_FRAME_SIZE, 0,
                                                NULL, NULL);
                    if (numBytes < 1) {
                        if (numBytes == 0) isConnected = false;
                        break;
                    }

                    printFrame("-->", msg, CAN_FRAME_SIZE);
                    handleCommand(msg, static_cast<size_t>(numBytes));
                }
            }
        }

        close(epfd);
        SPDLOG_INFO("connection closed");
    }

private:
    int sockfd_ = -1;
    int localPort_;
    int remotePort_;
    struct sockaddr_in remoteAddr_;

    // Motor instances keyed by CAN ID
    std::map<uint16_t, MotorState> motors_;

    // ========================================================
    // Motor Lookup — Route command to correct motor
    // ========================================================

    /**
     * Find which motor a CAN ID belongs to.
     * Handles mode offsets: +0x100 (PosVel), +0x200 (Vel), +0x300 (PosForce) [2][3]
     */
    MotorState* findMotorByCanId(uint32_t canId) {
        // Direct match (MIT mode, enable/disable/save/clear)
        auto it = motors_.find(static_cast<uint16_t>(canId));
        if (it != motors_.end()) return &it->second;

        // Check mode offsets [3]
        for (auto& [baseId, motor] : motors_) {
            if (canId == baseId + 0x100 ||   // PosVel [3]
                canId == baseId + 0x200 ||   // Velocity [3]
                canId == baseId + 0x300) {   // PosForce [3]
                return &motor;
            }
        }

        return nullptr;
    }

    /**
     * Find motor by CAN ID embedded in parameter command data bytes.
     * For 0x7FF commands, motor ID is in data[0:1] (little-endian) [3]
     */
    MotorState* findMotorByParamData(const uint8_t* data) {
        uint16_t motorId = static_cast<uint16_t>(data[0]) |
                           (static_cast<uint16_t>(data[1]) << 8);
        auto it = motors_.find(motorId);
        if (it != motors_.end()) return &it->second;

        // Also try matching by send_can_id (which may differ from base canId)
        for (auto& [baseId, motor] : motors_) {
            if (motorId == baseId) return &motor;
        }
        return nullptr;
    }

    // ========================================================
    // Command Identification [1][2][3]
    // ========================================================

    struct ParsedCommand {
        CommandType type  = CommandType::UNKNOWN;
        MotorState* motor = nullptr;
        uint32_t    canId = 0;
    };

    ParsedCommand identifyCommand(uint8_t* msg, size_t len) {
        ParsedCommand result;
        if (len < CAN_FRAME_SIZE) return result;

        result.canId = extract_can_id(msg);
        const uint8_t* data = &msg[5];

        // Check control commands (D[0:6]=0xFF, D[7]=cmd) [2]
        if (data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFF &&
            data[3] == 0xFF && data[4] == 0xFF && data[5] == 0xFF &&
            data[6] == 0xFF) {
            result.motor = findMotorByCanId(result.canId);
            if (!result.motor) return result;

            switch (data[7]) {
                case 0xFC: result.type = CommandType::ENABLE;      break;
                case 0xFD: result.type = CommandType::DISABLE;     break;
                case 0xFE: result.type = CommandType::SAVE_ZERO;   break;
                case 0xFB: result.type = CommandType::CLEAR_ERROR;  break;
            }
            return result;
        }

        // Parameter commands (CAN ID = 0x7FF) [3]
        if (result.canId == 0x7FF) {
            result.motor = findMotorByParamData(data);
            if (!result.motor) {
                SPDLOG_ERROR("Param cmd: motor ID 0x{} not found", data[0] | (data[1] << 8));
                return result;
            }
            if (data[2] == 0x33)      result.type = CommandType::QUERY_PARAM;
            else if (data[2] == 0x55) result.type = CommandType::WRITE_PARAM;
            else if (data[2] == 0xCC) result.type = CommandType::REFRESH;
            return result;
        }

        // Mode-specific commands by CAN ID offset [2][3]
        result.motor = findMotorByCanId(result.canId);
        if (!result.motor) return result;

        uint16_t baseId = result.motor->canId;
        if (result.canId == baseId) {
            result.type = CommandType::MIT_CONTROL;
        } else if (result.canId == baseId + 0x100) {
            result.type = CommandType::POSVEL_CONTROL;
        } else if (result.canId == baseId + 0x200) {
            result.type = CommandType::VELOCITY_CONTROL;
        } else if (result.canId == baseId + 0x300) {
            result.type = CommandType::POSFORCE_CONTROL;
        }

        return result;
    }

    // ========================================================
    // Command Handler Dispatch
    // ========================================================

    static double trunc6(double v) {
        const double scale = 1e6;
        return std::trunc(v * scale) / scale;
    }

    void handleCommand(uint8_t* msg, size_t len) {
        ParsedCommand parsed = identifyCommand(msg, len);

        if (!parsed.motor) {
            std::cerr << "  No motor found for CAN ID 0x"
                      << std::hex << parsed.canId << std::dec << std::endl;
            return;
        }

        MotorState& motor = *parsed.motor;
        const uint8_t* data = &msg[5];

        std::ostringstream cmdLog;
        cmdLog << "Motor [0x" << std::hex << static_cast<int>(motor.canId) << std::dec << "]";

        switch (parsed.type) {
            case CommandType::ENABLE:
                motor.status = MotorStatus::ENABLED;
                cmdLog << ", ENABLED";
                break;

            case CommandType::DISABLE:
                motor.status = MotorStatus::DISABLED;
                motor.targetPos = motor.targetVel = motor.targetTorque = 0;
                cmdLog << ", DISABLED";
                break;

            case CommandType::SAVE_ZERO:
                motor.position = 0.0;
                motor.zeroSaved = true;
                cmdLog << ", ZERO SAVED";
                break;

            case CommandType::CLEAR_ERROR:
                motor.status = MotorStatus::DISABLED;
                cmdLog << ", ERRORS CLEARED";
                break;

            case CommandType::MIT_CONTROL:
                handleMitControl(motor, data, cmdLog);
                break;

            case CommandType::POSVEL_CONTROL:
                handlePosVelControl(motor, data, cmdLog);
                break;

            case CommandType::VELOCITY_CONTROL:
                handleVelocityControl(motor, data, cmdLog);
                break;

            case CommandType::POSFORCE_CONTROL:
                handlePosForceControl(motor, data, cmdLog);
                break;

            case CommandType::QUERY_PARAM:
                handleQueryParam(motor, data, cmdLog);
                return;  // Query sends its own response format

            case CommandType::WRITE_PARAM:
                handleWriteParam(motor, data, cmdLog);
                return;

            case CommandType::REFRESH:
                cmdLog << ", REFRESH";
                break;

            default:
                std::cerr << "Motor [0x" << std::hex << static_cast<int>(motor.canId)
                          << std::dec << "] UNKNOWN COMMAND" << std::endl;
                return;
        }

        SPDLOG_INFO("{}", cmdLog.str());

        // Send motor feedback frame [2]
        sendFeedback(motor);
    }

    // ========================================================
    // Control Command Handlers
    // ========================================================

    void handleMitControl(MotorState& motor, const uint8_t* data, std::ostringstream& out) {
        if (motor.status != MotorStatus::ENABLED) {
            out << ", MIT rejected (not enabled)";
            return;
        }

        // Decode MIT frame [2][3]
        uint16_t q_uint   = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        uint16_t dq_uint  = (static_cast<uint16_t>(data[2]) << 4) | (data[3] >> 4);
        uint16_t kp_uint  = (static_cast<uint16_t>(data[3] & 0x0F) << 8) | data[4];
        uint16_t kd_uint  = (static_cast<uint16_t>(data[5]) << 4) | (data[6] >> 4);
        uint16_t tau_uint = (static_cast<uint16_t>(data[6] & 0x0F) << 8) | data[7];

        motor.targetPos    = uint_to_double(q_uint, -motor.limits.pMax, motor.limits.pMax, 16);
        motor.targetVel    = uint_to_double(dq_uint, -motor.limits.vMax, motor.limits.vMax, 12);
        motor.kp           = uint_to_double(kp_uint, 0, 500, 12);
        motor.kd           = uint_to_double(kd_uint, 0, 5, 12);
        motor.targetTorque = uint_to_double(tau_uint, -motor.limits.tMax, motor.limits.tMax, 12);

        updatePhysics(motor);
        out << std::fixed << std::setprecision(6)
            << ", MIT p=" << trunc6(motor.targetPos)
            << " v=" << trunc6(motor.targetVel)
            << " t=" << trunc6(motor.targetTorque);
    }

    void handlePosVelControl(MotorState& motor, const uint8_t* data, std::ostringstream& out) {
        if (motor.status != MotorStatus::ENABLED) {
            out << ", PosVel rejected (not enabled)";
            return;
        }

        motor.targetPos = bytes_to_float(&data[0]);   // D[0:3] float LE [2][3]
        motor.targetVel = bytes_to_float(&data[4]);   // D[4:7] float LE [2][3]

        updatePhysics(motor);
        out << std::fixed << std::setprecision(6)
            << ", PosVel p=" << trunc6(motor.targetPos)
            << " v=" << trunc6(motor.targetVel);
    }

    void handleVelocityControl(MotorState& motor, const uint8_t* data, std::ostringstream& out) {
        if (motor.status != MotorStatus::ENABLED) {
            out << ", Vel rejected (not enabled)";
            return;
        }

        motor.targetVel = bytes_to_float(&data[0]);   // D[0:3] float LE [2][3]

        updatePhysics(motor);
        out << std::fixed << std::setprecision(6)
            << ", Vel v=" << trunc6(motor.targetVel);
    }

    void handlePosForceControl(MotorState& motor, const uint8_t* data, std::ostringstream& out) {
        if (motor.status != MotorStatus::ENABLED) {
            out << ", PosForce rejected (not enabled)";
            return;
        }

        motor.targetPos = bytes_to_float(&data[0]);   // D[0:3] float LE [3]
        uint16_t vel_u = data[4] | (static_cast<uint16_t>(data[5]) << 8);
        uint16_t i_u   = data[6] | (static_cast<uint16_t>(data[7]) << 8);

        motor.targetVel    = vel_u / 100.0;             // Scaled by 100 [3]
        motor.targetTorque = (i_u / 10000.0) * motor.limits.tMax;  // Per-unit [3]

        updatePhysics(motor);
        out << std::fixed << std::setprecision(6)
            << ", PosForce p=" << trunc6(motor.targetPos)
            << " v=" << trunc6(motor.targetVel)
            << " t=" << trunc6(motor.targetTorque);
    }

    // ========================================================
    // Parameter Command Handlers [3]
    // ========================================================

    void handleQueryParam(MotorState& motor, const uint8_t* data, std::ostringstream& out,
                          const char* op = "QUERY") {
        uint8_t rid = data[3];
        float value = 0.0f;

        auto it = motor.parameters.find(rid);
        if (it != motor.parameters.end()) value = it->second;

        out << std::fixed << std::setprecision(6)
            << ", " << op << " RID=" << static_cast<int>(rid)
            << " val=" << trunc6(value);
        SPDLOG_INFO("{}", out.str());
        std::cout << out.str() << std::endl;

        // Build response [3]
        std::array<uint8_t, CAN_FRAME_SIZE> resp = {};
        resp[0] = 0x08;
        set_can_id(resp.data(), motor.masterId);
        resp[5] = data[0];  // Echo motor ID [3]
        resp[6] = data[1];
        resp[7] = 0x33;     // Query response marker [3]
        resp[8] = rid;

        if (is_int_param(rid)) {
            uint32_t iv = static_cast<uint32_t>(value);
            std::memcpy(&resp[9], &iv, 4);
        } else {
            std::memcpy(&resp[9], &value, 4);
        }

        printFrame("<--", resp.data(), CAN_FRAME_SIZE);
        sendResponse(resp.data(), CAN_FRAME_SIZE);
    }

    void handleWriteParam(MotorState& motor, const uint8_t* data, std::ostringstream& out) {
        uint8_t rid = data[3];
        float value;

        if (is_int_param(rid)) {
            value = static_cast<float>(bytes_to_uint32(&data[4]));
        } else {
            value = bytes_to_float(&data[4]);
        }

        motor.parameters[rid] = value;
        handleQueryParam(motor, data, out, "WRITE");
    }

    // ========================================================
    // Motor Physics Simulation
    // ========================================================

    void updatePhysics(MotorState& motor) {
        const double alpha = 0.1;

        motor.position += alpha * (motor.targetPos - motor.position);
        motor.velocity += alpha * (motor.targetVel - motor.velocity);
        motor.torque   += alpha * (motor.targetTorque - motor.torque);

        motor.position = std::max(-static_cast<double>(motor.limits.pMax),
                        std::min(motor.position, static_cast<double>(motor.limits.pMax)));
        motor.velocity = std::max(-static_cast<double>(motor.limits.vMax),
                        std::min(motor.velocity, static_cast<double>(motor.limits.vMax)));
        motor.torque   = std::max(-static_cast<double>(motor.limits.tMax),
                        std::min(motor.torque, static_cast<double>(motor.limits.tMax)));

        motor.mosFetTemp = 35 + static_cast<int>(std::abs(motor.torque) * 2.0);
        motor.rotorTemp  = 40 + static_cast<int>(std::abs(motor.torque) * 1.5);
    }

    // ========================================================
    // Feedback Frame Generation [2]
    // ========================================================

    void buildFeedbackFrame(const MotorState& motor,
                            std::array<uint8_t, CAN_FRAME_SIZE>& frame) {
        frame[0] = 0x08;
        set_can_id(frame.data(), motor.masterId);

        // D[0]: ID | (ERR << 4) [2]
        frame[5] = (motor.canId & 0xFF) |
                   (static_cast<uint8_t>(motor.status) << 4);

        // D[1:2]: Position (16-bit) [2]
        uint16_t pos_u = double_to_uint(motor.position,
            -motor.limits.pMax, motor.limits.pMax, 16);
        frame[6] = (pos_u >> 8) & 0xFF;
        frame[7] = pos_u & 0xFF;

        // D[3:4]: Velocity (12-bit) | D[4:5]: Torque (12-bit) [2]
        uint16_t vel_u = double_to_uint(motor.velocity,
            -motor.limits.vMax, motor.limits.vMax, 12);
        uint16_t tau_u = double_to_uint(motor.torque,
            -motor.limits.tMax, motor.limits.tMax, 12);

        frame[8]  = (vel_u >> 4) & 0xFF;
        frame[9]  = ((vel_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F);
        frame[10] = tau_u & 0xFF;

        // D[6]: MOS temp, D[7]: Rotor temp [2]
        frame[11] = static_cast<uint8_t>(std::max(0, std::min(255, motor.mosFetTemp)));
        frame[12] = static_cast<uint8_t>(std::max(0, std::min(255, motor.rotorTemp)));
    }

    void sendFeedback(const MotorState& motor) {
        std::array<uint8_t, CAN_FRAME_SIZE> frame;
        buildFeedbackFrame(motor, frame);
        printFrame("<--", frame.data(), CAN_FRAME_SIZE);
        sendResponse(frame.data(), CAN_FRAME_SIZE);
    }

    // ========================================================
    // UDP Communication [1]
    // ========================================================

    void sendResponse(const uint8_t* data, size_t len) {
        sendto(sockfd_, data, len, 0,
               (struct sockaddr*)&remoteAddr_, sizeof(struct sockaddr_in));
    }

    void printFrame(const char* dir, const uint8_t* data, size_t len) {
        std::ostringstream oss;
        oss << dir;
        for (size_t i = 0; i < len; i++) {
            oss << " 0x" << std::hex << std::uppercase
                << std::setw(2) << std::setfill('0')
                << static_cast<int>(data[i]);
        }
        std::string frameStr = oss.str();
        SPDLOG_INFO("{}", frameStr);
    }
};

// ============================================================
// CLI Argument Parsing
// ============================================================

static std::vector<uint16_t> parseMotorIds(const std::string& idStr) {
    std::vector<uint16_t> ids;
    std::stringstream ss(idStr);
    std::string token;

    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        size_t start = token.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        token = token.substr(start);

        uint16_t id = static_cast<uint16_t>(
            std::strtoul(token.c_str(), nullptr, 0));
        if (id == 0) {
            SPDLOG_WARN("motor ID 0 is reserved");
            continue;
        }
        ids.push_back(id);
    }

    return ids;
}

static void printUsage(const char* prog) {
    std::cout << "Usage: " << prog
              << " [-mode motor|imu]"
              << " [-local port]"
              << " [-remote port]"
              << " [-ids id1,id2,...,id6]"
              << "\n\nExamples:\n"
              << "  " << prog << " -ids 1\n"
              << "  " << prog << " -ids 1,2,3,4,5,6\n"
              << "  " << prog << " -ids 0x01,0x02,0x03 -mode imu\n"
              << "  " << prog << " -ids 1,2,3 -local 8886 -remote 8887\n"
              << std::endl;
}

// ============================================================
// Main
// ============================================================

int main(int argc, char* argv[]) {
    int localPort        = 8886;   // [1]
    int remotePort       = 8887;   // [1]
    std::string idString = "1";    // Default: single motor with ID 1
    setupLogger();
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-local") == 0 && i + 1 < argc) {
            localPort = atoi(argv[++i]);
            if (localPort <= 0 || localPort > 65535) {
                SPDLOG_ERROR("invalid local port {}",localPort);
                return 1;
            }
        } else if (strcmp(argv[i], "-remote") == 0 && i + 1 < argc) {
            remotePort = atoi(argv[++i]);
            if (remotePort <= 0 || remotePort > 65535) {
                SPDLOG_ERROR("invalid remote port {}",remotePort);
                return 1;
            }
        } else if (strcmp(argv[i], "-ids") == 0 && i + 1 < argc) {
            idString = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            SPDLOG_ERROR("unknown argument: {}", argv[i]);
            printUsage(argv[0]);
            return 1;
        }
    }

    std::vector<uint16_t> motorIds = parseMotorIds(idString);
    if (motorIds.empty()) {
        SPDLOG_ERROR("no valid motor IDs provided");
        printUsage(argv[0]);
        return 1;
    }

    if (motorIds.size() > MAX_MOTORS) {
        SPDLOG_ERROR("max {}  motors supported",MAX_MOTORS);
        return 1;
    }

    DamiaoMultiSimulator sim(localPort, remotePort, motorIds);

    if (!sim.init()) return 1;

    sim.runMotorMode();
    return 0;
}

void setupLogger() {
    std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v";
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("../logs/damiao_simulator.log", 100 * 1024, 3);
    auto logger = std::make_shared<spdlog::logger>("multi_sink", spdlog::sinks_init_list{console_sink, file_sink});
    console_sink->set_level(spdlog::level::debug);
    file_sink->set_level(spdlog::level::debug);
    logger->set_level(spdlog::level::debug);
    logger->flush_on(spdlog::level::debug);
    console_sink->set_pattern(pattern);
    file_sink->set_pattern(pattern);
    spdlog::set_default_logger(logger);
}
/*
| Aspect | Current (1 Motor) [1] | Multi-Motor (6 Motors) |
|--------|:---:|:---:|
| Motor state | Single `responses` map | `std::map<uint16_t, MotorState>` keyed by CAN ID |
| Command routing | Direct lookup `responses.find(command)` [1] | First extract CAN ID → find motor → then handle command |
| CAN ID in response | `msg[4] += 0x10` or `msg[5] += 0x10` [1] | Each motor has its own `masterId` for feedback |
| UDP socket | One socket shared | **Same** — CAN is a shared bus; one socket serves all motors |
| IMU mode | 8 fixed responses [1] | 8 responses **per motor** = up to 48 messages per cycle |
 *
 */
/*
Command Routing Logic
Per the protocol, the CAN ID determines which motor is addressed :
*Incoming CAN ID → Which Motor?
─────────────────────────────────────────────
CAN_ID = 0x01        → Motor 1 (MIT mode)
CAN_ID = 0x01 + 0x100 → Motor 1 (PosVel mode)
CAN_ID = 0x01 + 0x200 → Motor 1 (Velocity mode)
CAN_ID = 0x01 + 0x300 → Motor 1 (PosForce mode)
CAN_ID = 0x02        → Motor 2 (MIT mode)
CAN_ID = 0x02 + 0x100 → Motor 2 (PosVel mode)
...
CAN_ID = 0x7FF       → Parameter command (motor ID in data[0:1])
 *
 */
/*
 *
Key Design Decisions
DECISION
RATIONALE
Single UDP socket for all motors
CAN is a shared bus — all motors on one bus share the same physical medium . One socket accurately models this
std::map<uint16_t, MotorState> keyed by CAN ID
O(log N) lookup per command; with max 6 motors, performance is irrelevant but correctness matters
Mode offset scanning in findMotorByCanId()
A command with CAN ID 0x101 must be routed to motor 0x01 (PosVel mode = base + 0x100)
Parameter commands use data[0:1] for motor ID
When CAN ID is 0x7FF, the motor identifier is embedded in the first two data bytes
Each motor has independent MotorState
Position, velocity, torque, temperature, and status are per-motor — no shared state
IMU mode distributes time across all motors
Original sends 8 messages in 2ms (250μs each) ; multi-motor divides the 2ms cycle by motor count
 */
/*
# Single motor (backward compatible with original) [1]
./damiao_multi_simulator -ids 1

# 6-DOF robotic arm: 6 motors with IDs 1-6
./damiao_multi_simulator -ids 1,2,3,4,5,6

# Hex IDs (e.g., matching real hardware configuration)
./damiao_multi_simulator -ids 0x01,0x02,0x03,0x04,0x05,0x06

# IMU mode with 3 motors
./damiao_multi_simulator -mode imu -ids 1,2,3

# Custom ports
./damiao_multi_simulator -ids 1,2,3,4,5,6 -local 9000 -remote 9001
| Command | CAN ID | Data Pattern | Simulator Response |
|---------|:------:|-------------|-------------------|
| Enable [2] | `CAN_ID` | `FF FF FF FF FF FF FF FC` | Feedback with status=0x01 (ENABLED) |
| Disable [2] | `CAN_ID` | `FF FF FF FF FF FF FF FD` | Feedback with status=0x00 (DISABLED) |
| Save Zero [2] | `CAN_ID` | `FF FF FF FF FF FF FF FE` | Feedback with position=0 |
| Clear Error [2] | `CAN_ID` | `FF FF FF FF FF FF FF FB` | Feedback with status=0x00 |
| MIT Control [2] | `CAN_ID` | MIT-encoded 8 bytes | Feedback with updated position/velocity/torque |
| PosVel [2][3] | `CAN_ID+0x100` | `pos(4B) vel(4B)` float LE | Feedback with updated state |
| Velocity [2][3] | `CAN_ID+0x200` | `vel(4B)` float LE | Feedback with updated state |
| PosForce [3] | `CAN_ID+0x300` | `pos(4B) vlim(2B) ilim(2B)` | Feedback with updated state |
| Query Param [3] | `0x7FF` | `id_lo id_hi 0x33 RID 00 00 00 00` | Parameter value response |
| Write Param [3] | `0x7FF` | `id_lo id_hi 0x55 RID val(4B)` | Echo with 0x33 confirmation |
| Refresh [3] | `0x7FF` | `id_lo id_hi 0xCC 00 00 00 00 00` | Feedback frame |
 */