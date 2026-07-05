#pragma once

#include <array>
#include <stdint.h>
#include <stddef.h>

extern "C" {

/*********CAN Application ID Defines***********/
const int CMD_API_GET_MOTOR_STATUS = 0x01;                // Get Motor Status
const int CMD_API_GET_MOTOR_PARAMETERS = 0x02;            // Get Motor Register Parameters
const int CMD_API_SAVE_MOTOR_PARAMETERS = 0x03;           // Save Motor Register Parameters to flash
const int CMD_API_WRITE_MOTOR_PARAMETERS = 0x04;          // Write Motor Register Parameters

//https://github.com/Gabriel1688/dummy/blob/403d62d1c7ea570c233f986e2b6118dd3a53d0c8/firmware/dummy-35motor-fw/UserApp/protocols/interface_can.cpp

#define PACKED __attribute__((__packed__))
struct PACKED dataframe_clear_error_t {
    uint8_t data[8];
    constexpr dataframe_clear_error_t() : data{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb} {
    }
};

struct PACKED dataframe_enable_motor_t {
    uint8_t data[8];
    constexpr dataframe_enable_motor_t() : data{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc} {
    }
};

struct PACKED dataframe_disable_motor_t {
    uint8_t data[8];
    constexpr dataframe_disable_motor_t() : data{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd} {
    }
};

struct PACKED dataframe_set_zero_position_t {
    uint8_t data[8];
    constexpr dataframe_set_zero_position_t() : data{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe} {
    }
};

struct PACKED dataframe_get_motor_status_t {
    int16_t can_id;
    uint8_t cmd[2];
    uint8_t rsvd[4];
};

typedef struct PACKED {
    int16_t can_id;
    uint8_t cmd;
    uint8_t reg_id;
    uint8_t data[4];
} dataframe_reg_param_t;

typedef struct PACKED {
    std::array<uint8_t, 4> v_des;
    std::array<uint8_t, 4> reserved;
} dataframe_vel_set_point_t;

typedef struct PACKED {
    std::array<uint8_t, 4> p_des;
    int16_t v_des;
    int16_t i_des;
} dataframe_pose_with_torque_param_t;

union dataframe_t {
    dataframe_clear_error_t clearError;
    dataframe_enable_motor_t enableMotor;
    dataframe_disable_motor_t disableMotor;
    dataframe_set_zero_position_t setZeroPosition;
    dataframe_vel_set_point_t velSetPoint;
    dataframe_get_motor_status_t updateMotorStatus;
    dataframe_reg_param_t registerParam;
    dataframe_pose_with_torque_param_t poseWithTorqueParam;

    constexpr dataframe_t(dataframe_clear_error_t clearError) : clearError(clearError) {};
    constexpr dataframe_t(dataframe_enable_motor_t enableMotor) : enableMotor(enableMotor) {};
    constexpr dataframe_t(dataframe_disable_motor_t disableMotor) : disableMotor(disableMotor) {};
    constexpr dataframe_t(dataframe_set_zero_position_t setZeroPosition) : setZeroPosition(setZeroPosition) {};
    constexpr dataframe_t(dataframe_get_motor_status_t updateMotorStatus) : updateMotorStatus(updateMotorStatus) {};
    dataframe_t(dataframe_vel_set_point_t velSetPoint) : velSetPoint(velSetPoint) {};
    dataframe_t(dataframe_reg_param_t registerParam) : registerParam(registerParam) {};
    dataframe_t(dataframe_pose_with_torque_param_t poseWithTorqueParam) : poseWithTorqueParam(poseWithTorqueParam) {};

    dataframe_t() {};
    uint8_t data[8];
};

struct ParamResult {
    int rid;
    double value;
    bool valid;
};

struct StateResult {
    int state;
    double position;
    double velocity;
    double torque;
    int t_mos;
    int t_rotor;
    bool valid;
};

struct MITParam {
    double kp;
    double kd;
    double q;
    double dq;
    double tau;
};

struct PosVelParam {
    double q;
    double dq;
};
#undef PACKED
} //extern "C"
