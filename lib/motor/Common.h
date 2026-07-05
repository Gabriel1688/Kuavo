#pragma once

#include "../../../dummy/include/types.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8
typedef unsigned char __u8;
#define CAN_SFF_MASK 0x000007FFU
#define MAX_CAN_DEVICE 6

enum class MotorType : uint8_t {
    DM8009 = 0,
    DM10010L = 1,
    COUNT = 2
};

enum class ControlMode : uint8_t {
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    TORQUE_POS = 4
};

enum class RID : uint8_t {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
    COUNT = 82
};

typedef struct {
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float cur_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;

// Limit parameters structure for different motor types
struct LimitParam {
    double pMax;// Position limit (rad)
    double vMax;// Velocity limit (rad/s)
    double tMax;// Torque limit (Nm)
};
// Limit parameters for each motor type [pMax, vMax, tMax]
inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)>
    MOTOR_LIMIT_PARAMS = {{
        {12.5, 45, 54},// DM8009
        {12.5, 25, 200}// DM10010L
    }};
