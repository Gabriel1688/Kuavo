# Sending MIT Control Commands to Damiao Motor Actuator 
## Overview
The MIT (Impedance) control mode is the most flexible control mode for Damiao motors, allowing simultaneous control of position, velocity, and torque with configurable PD gains . Before sending MIT control commands, the motor must be enabled by sending the enable command (0xFC) . The entire process involves four stages: float-to-integer conversion, bit packing, CAN-over-UDP framing, and feedback decoding.

Prerequisites: Enable the Motor
After power-on self-check, the motor must receive an "enable" command before it will accept any control commands . The enable frame uses the motor's CAN ID and a fixed 8-byte data payload:

| D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|------|------|------|------|------|------|------|------|
| 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFF | 0xFC |

CANPacket CanPacketEncoder::create_enable_command(const Motor& motor) {
return {motor.get_send_can_id(), pack_command_data(0xFC)};
}

std::vector<uint8_t> CanPacketEncoder::pack_command_data(uint8_t cmd) {
return {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
}
The existing simulator recognizes this command by checking msg[12] == 0xFC (the last byte of the 13-byte UDP-CAN frame) and responds with a pre-built echo response .

Important: The motor's disable state is the power-up default. In disable state, the motor's three-phase terminal voltage waveforms are identical, all at 50% modulation of the supply voltage . The disable command uses the same frame format but with 0xFD as the last byte .

## MIT Control Frame Format
The MIT control frame packs 5 floating-point parameters into exactly 8 bytes of CAN data using bit-level packing . The frame ID equals the motor's configured CAN ID directly — no offset is added (unlike PosVel mode which adds +0x100 or Velocity mode which adds +0x200) .

### Byte Layout
| Byte | D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|------|------|------|------|------|------|------|------|------|
| Content | p_des[15:8] | p_des[7:0] | v_des[11:4] | v_des[3:0] \| Kp[11:8] | Kp[7:0] | Kd[11:4] | Kd[3:0] \| t_ff[11:8] | t_ff[7:0] |

## Parameter Specifications
| Parameter | Bits | Range | Default Limits | Description |
|-----------|:----:|-------|:---:|-------------|
| **p_des** | 16 | [-P_MAX, P_MAX] | ±12.5 rad | Position setpoint |
| **v_des** | 12 | [-V_MAX, V_MAX] | ±45.0 rad/s | Velocity setpoint |
| **Kp** | 12 | [0, 500] | Fixed | Position proportional gain |
| **Kd** | 12 | [0, 5] | Fixed | Position derivative gain |
| **t_ff** | 12 | [-T_MAX, T_MAX] | ±18.0 Nm | Feedforward torque |

Standard CAN data is limited to 8 bytes per frame. The MIT command format combines Position (16 bits), Velocity (12 bits), Kp (12 bits), Kd (12 bits), and Torque (12 bits) = 64 bits = 8 bytes .

Warning: The P_MAX, V_MAX, and T_MAX values can be adjusted via the debugging assistant, but the values used when sending control commands must match the values configured in the motor driver. Otherwise, the control commands will be proportionally scaled incorrectly .

Warning: When performing position control via MIT mode, Kd must not be set to zero, otherwise the motor will oscillate or lose control .

## Stage 1: Float-to-Integer Conversion
Each floating-point parameter must be linearly mapped to its corresponding unsigned integer range. The protocol defines two conversion functions :

### float_to_uint (Encoding — Controller Side)
int float_to_uint(float x, float x_min, float x_max, int bits) {
float span = x_max - x_min;
float offset = x_min;
return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

### uint_to_float (Decoding — Feedback Side)
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
float span = x_max - x_min;
float offset = x_min;
return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

The openarm library implements these with clamping as :

uint16_t CanPacketEncoder::double_to_uint(double x, double x_min, double x_max, int bits) {
x = limit_min_max(x, x_min, x_max);
double span = x_max - x_min;
double data_norm = (x - x_min) / span;
return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

double CanPacketDecoder::uint_to_double(uint16_t x, double min, double max, int bits) {
double span = max - min;
double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
return data_norm * span + min;
}

### Conversion Example
For a motor with default limits (P_MAX=12.5, V_MAX=45, T_MAX=18), the protocol specification gives this explicit example for velocity mapping :

If the motor's current velocity is 25.0 rad/s with V_MAX=45 rad/s, the encoded data is:
VEL = 25.0 / (45 - (-45)) × 2^12 + 2^11 = 3185 = 0xC71

Full conversion table for a sample command:
| Parameter | Value | Min | Max | Bits | Calculation | Encoded |
|-----------|-------|-----|-----|:----:|-------------|---------|
| p_des | 5.0 rad | -12.5 | 12.5 | 16 | (5.0+12.5)/25.0 × 65535 | 45874 = 0xB332 |
| v_des | 25.0 rad/s | -45.0 | 45.0 | 12 | (25.0+45.0)/90.0 × 4095 | 3185 = 0xC71 |
| Kp | 50.0 | 0 | 500 | 12 | 50.0/500.0 × 4095 | 409 = 0x199 |
| Kd | 1.0 | 0 | 5 | 12 | 1.0/5.0 × 4095 | 819 = 0x333 |
| t_ff | 0.0 Nm | -18.0 | 18.0 | 12 | (0.0+18.0)/36.0 × 4095 | 2047 = 0x7FF |

### Stage 2: Bit Packing into 8 Bytes
After converting all 5 parameters to integers, they are packed by bit position into exactly 8 bytes. The protocol specification defines the MIT control sending function as :
void ctrl_motor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel,
float _KP, float _KD, float _torq) {
uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    hcan->pTxMsg->StdId = id;
    hcan->pTxMsg->IDE = CAN_ID_STD;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 0x08;
    hcan->pTxMsg->Data[0] = (pos_tmp >> 8);
    hcan->pTxMsg->Data[1] = pos_tmp;
    hcan->pTxMsg->Data[2] = (vel_tmp >> 4);
    hcan->pTxMsg->Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    hcan->pTxMsg->Data[4] = kp_tmp;
    hcan->pTxMsg->Data[5] = (kd_tmp >> 4);
    hcan->pTxMsg->Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    hcan->pTxMsg->Data[7] = tor_tmp;

    HAL_CAN_Transmit(hcan, 100);
}

The openarm library implements the same packing logic as :
std::vector<uint8_t> CanPacketEncoder::pack_mit_control_data(
MotorType motor_type, const MITParam& mit_param) {
uint16_t kp_uint  = double_to_uint(mit_param.kp, 0, 500, 12);
uint16_t kd_uint  = double_to_uint(mit_param.kd, 0, 5, 12);

    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(motor_type)];
    uint16_t q_uint   = double_to_uint(mit_param.q,
                            -(double)limits.pMax, (double)limits.pMax, 16);
    uint16_t dq_uint  = double_to_uint(mit_param.dq,
                            -(double)limits.vMax, (double)limits.vMax, 12);
    uint16_t tau_uint = double_to_uint(mit_param.tau,
                            -(double)limits.tMax, (double)limits.tMax, 12);

    return {
        static_cast<uint8_t>((q_uint >> 8) & 0xFF),                           // D[0]: p_des[15:8]
        static_cast<uint8_t>(q_uint & 0xFF),                                  // D[1]: p_des[7:0]
        static_cast<uint8_t>(dq_uint >> 4),                                   // D[2]: v_des[11:4]
        static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)), // D[3]: v_des[3:0]|Kp[11:8]
        static_cast<uint8_t>(kp_uint & 0xFF),                                 // D[4]: Kp[7:0]
        static_cast<uint8_t>(kd_uint >> 4),                                   // D[5]: Kd[11:4]
        static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)), // D[6]: Kd[3:0]|t_ff[11:8]
        static_cast<uint8_t>(tau_uint & 0xFF)                                 // D[7]: t_ff[7:0]
    };
}

And the MIT command creation function sets the CAN ID to the motor's base CAN ID without any offset :
CANPacket CanPacketEncoder::create_mit_control_command(const Motor& motor,
const MITParam& mit_param) {
return {motor.get_send_can_id(),
pack_mit_control_data(motor.get_motor_type(), mit_param)};
}

### Stage 3: Wrap in CAN-over-UDP Frame and Send
The existing Damiao simulator packages CAN frames in 13-byte UDP datagrams :
Byte:  [0]   [1]  [2]  [3]  [4]   [5]  [6]  [7]  [8]  [9]  [10] [11] [12]
Field: DLC   ----CAN ID (4B)----   ----------CAN DATA (8B)----------------

The simulator uses a non-blocking UDP socket on localhost with default ports 8886 (local) and 8887 (remote) . Here is the complete send function that combines all previous stages:
void send_mit_command(int sockfd, struct sockaddr_in* remote_addr,
uint16_t can_id,
double p_des, double v_des,
double kp, double kd, double t_ff,
double p_max, double v_max, double t_max) {
// Stage 1: Float-to-integer conversion [2]
uint16_t pos_uint = double_to_uint(p_des, -p_max, p_max, 16);
uint16_t vel_uint = double_to_uint(v_des, -v_max, v_max, 12);
uint16_t kp_uint  = double_to_uint(kp, 0, 500, 12);
uint16_t kd_uint  = double_to_uint(kd, 0, 5, 12);
uint16_t tau_uint = double_to_uint(t_ff, -t_max, t_max, 12);

    // Stage 2 + 3: Bit packing [2] + UDP-CAN framing [1]
    uint8_t frame[13];
    frame[0] = 0x08;  // DLC = 8 bytes [1]

    // CAN ID (big-endian, 4 bytes) [1]
    frame[1] = 0x00;
    frame[2] = 0x00;
    frame[3] = (can_id >> 8) & 0xFF;
    frame[4] = can_id & 0xFF;

    // MIT control data (8 bytes) [2]
    frame[5]  = (pos_uint >> 8) & 0xFF;                              // D[0]: p_des[15:8]
    frame[6]  = pos_uint & 0xFF;                                     // D[1]: p_des[7:0]
    frame[7]  = (vel_uint >> 4) & 0xFF;                              // D[2]: v_des[11:4]
    frame[8]  = ((vel_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF);   // D[3]: v_des[3:0]|Kp[11:8]
    frame[9]  = kp_uint & 0xFF;                                      // D[4]: Kp[7:0]
    frame[10] = (kd_uint >> 4) & 0xFF;                               // D[5]: Kd[11:4]
    frame[11] = ((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF);   // D[6]: Kd[3:0]|t_ff[11:8]
    frame[12] = tau_uint & 0xFF;                                     // D[7]: t_ff[7:0]

    // Send via UDP [1]
    sendto(sockfd, frame, 13, 0,
           (struct sockaddr*)remote_addr, sizeof(struct sockaddr_in));
}

### Stage 4: Receive and Decode Feedback
After sending an MIT control frame, the motor responds with a feedback frame on its MasterID . The feedback frame format is the same for all three control modes (MIT, PosVel, Velocity) :
| Byte | D[0] | D[1] | D[2] | D[3] | D[4] | D[5] | D[6] | D[7] |
|------|------|------|------|------|------|------|------|------|
| Content | ID \| ERR<<4 | POS[15:8] | POS[7:0] | VEL[11:4] | VEL[3:0] \| T[11:8] | T[7:0] | T_MOS | T_Rotor |

Where :

ID: Controller ID (low bits of CAN ID)
ERR: Status/error code (0=disabled, 1=enabled, 8=overvoltage, 9=undervoltage, A=overcurrent, B=MOS overtemp, C=coil overtemp, D=communication lost, E=overload)
POS: 16-bit position mapped to [-P_MAX, P_MAX]
VEL: 12-bit velocity mapped to [-V_MAX, V_MAX]
T: 12-bit torque mapped to [-T_MAX, T_MAX]
T_MOS: MOS average temperature in °C
T_Rotor: Motor coil average temperature in °C
The protocol specification defines the feedback decoding function (same for all three modes) :

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan) {
p_int  = (_hcan->pRxMsg->Data[1] << 8) | _hcan->pRxMsg->Data[2];
v_int  = (_hcan->pRxMsg->Data[3] << 4) | (_hcan->pRxMsg->Data[4] >> 4);
t_int  = ((_hcan->pRxMsg->Data[4] & 0xF) << 8) | _hcan->pRxMsg->Data[5];
position = uint_to_float(p_int, P_MIN, P_MAX, 16);  // (-12.5, 12.5)
velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);  // (-45.0, 45.0)
torque   = uint_to_float(t_int, T_MIN, T_MAX, 12);  // (-18.0, 18.0)
}

The openarm library implements the same decoding :
StateResult CanPacketDecoder::parse_motor_state_data(const Motor& motor,
const std::vector<uint8_t>& data) {
uint16_t q_uint   = (static_cast<uint16_t>(data[1]) << 8) | data[2];
uint16_t dq_uint  = (static_cast<uint16_t>(data[3]) << 4) |
(static_cast<uint16_t>(data[4]) >> 4);
uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0xf) << 8) | data[5];
int t_mos   = static_cast<int>(data[6]);
int t_rotor = static_cast<int>(data[7]);

    LimitParam limits = MOTOR_LIMIT_PARAMS[static_cast<int>(motor.get_motor_type())];
    double recv_q   = uint_to_double(q_uint, -limits.pMax, limits.pMax, 16);
    double recv_dq  = uint_to_double(dq_uint, -limits.vMax, limits.vMax, 12);
    double recv_tau = uint_to_double(tau_uint, -limits.tMax, limits.tMax, 12);

    return {recv_q, recv_dq, recv_tau, t_mos, t_rotor, true};
}

The complete receive and decode function for the CAN-over-UDP frame:
bool receive_mit_feedback(int sockfd,
double p_max, double v_max, double t_max,
uint8_t& motor_id, uint8_t& error_code,
double& position, double& velocity, double& torque,
int& t_mos, int& t_rotor) {
uint8_t frame[13];
ssize_t n = recvfrom(sockfd, frame, 13, 0, nullptr, nullptr);
if (n < 13) return false;

    // CAN data starts at byte 5 in the UDP-CAN frame [1]
    const uint8_t* data = &frame[5];

    // D[0]: ID | ERR<<4 [2]
    motor_id   = data[0] & 0x0F;
    error_code = (data[0] >> 4) & 0x0F;

    // D[1:2]: Position (16-bit) [2]
    uint16_t p_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];

    // D[3:4]: Velocity (12-bit) [2]
    uint16_t v_uint = (static_cast<uint16_t>(data[3]) << 4) |
                      (static_cast<uint16_t>(data[4]) >> 4);

    // D[4:5]: Torque (12-bit) [2]
    uint16_t t_uint = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];

    // D[6]: T_MOS, D[7]: T_Rotor [2]
    t_mos   = static_cast<int>(data[6]);
    t_rotor = static_cast<int>(data[7]);

    // Convert back to floating point [2]
    position = uint_to_double(p_uint, -p_max, p_max, 16);
    velocity = uint_to_double(v_uint, -v_max, v_max, 12);
    torque   = uint_to_double(t_uint, -t_max, t_max, 12);

    return true;
}

## Simulator Command Recognition
The existing simulator identifies MIT mode commands by checking msg[7] == 0x33 (byte 7 of the 13-byte UDP frame, which corresponds to D of the CAN data) . When an MIT command is recognized, the simulator looks up the pre-built response, updates the CAN ID by adding 0x10 to msg[5], and sends the response back :
unsigned int command = 0x0;
if ((msg[12] == 0xFC) || (msg[12] == 0xFD) || (msg[12] == 0xFE)) {
command = msg[12];
} else if (msg[7] == 0x33) {
command = 0x33;                    // MIT mode identified
} else if (msg[7] == 0xCC) {
command = 0xCC;                    // PosVel mode
} else if ((msg[5] == 0x7F) && (msg[6] == 0xFF)) {
command = 0xFF;                    // Read parameters
}

auto itmap = responses.find(command);
if (itmap != responses.end()) {
if ((command == 0xcc) || (command == 0x33)) {
msg[5] += 0x10;
itmap->second[05] = msg[5];   // Update low byte of CAN ID
} else {
msg[4] += 0x10;
itmap->second[04] = msg[4];
}
sendto(_sockfd, itmap->second.data(), 13, 0,
(struct sockaddr*)&servAddr, sizeof(struct sockaddr_in));
}

## Complete Usage Example
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// Default Damiao motor limits [2]
static constexpr double P_MAX = 12.5;   // rad
static constexpr double V_MAX = 45.0;   // rad/s
static constexpr double T_MAX = 18.0;   // Nm

int main() {
// Setup UDP socket (same as simulator) [1]
int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
fcntl(sockfd, F_SETFL, O_NONBLOCK);

    struct sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(8887);              // Local port [1]
    local_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    bind(sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr));

    struct sockaddr_in remote_addr{};
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(8886);             // Simulator port [1]
    remote_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    uint16_t motor_can_id = 0x01;

    // Step 1: Send enable command [2]
    uint8_t enable_frame[13] = {
        0x08,                                       // DLC [1]
        0x00, 0x00, 0x00, motor_can_id,            // CAN ID [1]
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC  // Enable [2]
    };
    sendto(sockfd, enable_frame, 13, 0,
           (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    // Wait for enable response
    usleep(10000);
    uint8_t resp[13];
    recvfrom(sockfd, resp, 13, 0, nullptr, nullptr);

    // Step 2: Send MIT control command
    // Target: position=5.0 rad, velocity=0.0 rad/s
    //         Kp=50.0, Kd=1.0, torque_ff=0.0 Nm
    send_mit_command(sockfd, &remote_addr, motor_can_id,
                     5.0,    // p_des (rad) [2]
                     0.0,    // v_des (rad/s) [2]
                     50.0,   // Kp [0,500] [2]
                     1.0,    // Kd [0,5] [2]
                     0.0,    // t_ff (Nm) [2]
                     P_MAX, V_MAX, T_MAX);

    // Step 3: Receive and decode feedback [2]
    usleep(10000);
    uint8_t motor_id, error_code;
    double position, velocity, torque;
    int t_mos, t_rotor;

    if (receive_mit_feedback(sockfd, P_MAX, V_MAX, T_MAX,
                             motor_id, error_code,
                             position, velocity, torque,
                             t_mos, t_rotor)) {
        printf("Feedback:\n");
        printf("  Motor ID:    0x%02X\n", motor_id);
        printf("  Status:      0x%02X (%s)\n", error_code,
               error_code == 0x01 ? "ENABLED" : "OTHER");
        printf("  Position:    %.4f rad\n", position);
        printf("  Velocity:    %.4f rad/s\n", velocity);
        printf("  Torque:      %.4f Nm\n", torque);
        printf("  MOS Temp:    %d C\n", t_mos);
        printf("  Rotor Temp:  %d C\n", t_rotor);
    }

    // Step 4: Disable motor when done [2]
    uint8_t disable_frame[13] = {
        0x08,
        0x00, 0x00, 0x00, motor_can_id,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD  // Disable [2]
    };
    sendto(sockfd, disable_frame, 13, 0,
           (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    close(sockfd);
    return 0;
}

## Summary of CAN ID Usage Across Modes
For reference, the CAN ID offset differs by control mode. MIT mode uses the base CAN ID with no offset
| Mode | Send CAN ID | Data Format | Reference |
|------|:---:|-------------|-----------|
| **MIT Control** | `CAN_ID` (no offset) | 5 params packed in 8 bytes [2] | `create_mit_control_command` [3] |
| **PosVel** | `CAN_ID + 0x100` | pos(4B float) + vel(4B float) [2] | `create_posvel_control_command` [3] |
| **Velocity** | `CAN_ID + 0x200` | vel(4B float), DLC=4 [2] | `create_vel_control_command` [3] |
| **PosForce** | `CAN_ID + 0x300` | pos(4B) + vlim(2B) + ilim(2B) [3] | `create_posforce_control_command` [3] |
| **Enable** | `CAN_ID` | `0xFF×7 + 0xFC` [2] | `create_enable_command` [3] |
| **Disable** | `CAN_ID` | `0xFF×7 + 0xFD` [2] | `create_disable_command` [3] |
| **Save Zero** | `CAN_ID` | `0xFF×7 + 0xFE` [2] | `create_set_zero_command` [3] |
| **Query Param** | `0x7FF` | id(2B) + 0x33 + RID + 0×4B [3] | `create_query_param_command` [3] |
| **Refresh** | `0x7FF` | id(2B) + 0xCC + 0×5B [3] | `create_refresh_command` [3] |
