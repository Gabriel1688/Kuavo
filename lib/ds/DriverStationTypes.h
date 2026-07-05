#pragma once

#include <stdint.h>

#define HAL_IO_CONFIG_DATA_SIZE 32
#define HAL_SYS_STATUS_DATA_SIZE 44
#define HAL_USER_STATUS_DATA_SIZE \
    (984 - HAL_IO_CONFIG_DATA_SIZE - HAL_SYS_STATUS_DATA_SIZE)

#define HALFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Input 17
#define HALFRC_NetworkCommunication_DynamicType_DSEnhancedIO_Output 18
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Header 19
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Extra1 20
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Vertices1 21
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Extra2 22
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Vertices2 23
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Joystick 24
#define HALFRC_NetworkCommunication_DynamicType_Kinect_Custom 25

struct ControlWord_t {
    uint32_t enabled: 1;
    uint32_t autonomous: 1;
    uint32_t test: 1;
    uint32_t eStop: 1;
    uint32_t fmsAttached: 1;
    uint32_t dsAttached: 1;
    uint32_t control_reserved: 26;
};
//struct JoystickAxes_t {
//    uint16_t count;
//    int16_t axes[1];
//};
//
//struct JoystickPOV_t {
//    uint16_t count;
//    int16_t povs[1];
//};

struct HAL_ControlWord {
    uint32_t enabled: 1;
    uint32_t autonomous: 1;
    uint32_t test: 1;
    uint32_t eStop: 1;
    uint32_t fmsAttached: 1;
    uint32_t dsAttached: 1;
    uint32_t control_reserved: 26;
};
typedef struct HAL_ControlWord HAL_ControlWord;

/**
 * The maximum number of axes that will be stored in a single HAL_JoystickAxes
 * struct. This is used for allocating buffers, not bounds checking, since there
 * are usually less axes in practice.
 */
#define HAL_kMaxJoystickAxes 12
/**
 * The maximum number of POVs that will be stored in a single HAL_JoystickPOVs
 * struct. This is used for allocating buffers, not bounds checking, since there
 * are usually less POVs in practice.
 */
#define HAL_kMaxJoystickPOVs 12
/**
 * The maximum number of joysticks.
 */
#define HAL_kMaxJoysticks 6

struct HAL_JoystickAxes {
    int16_t count;
    float axes[HAL_kMaxJoystickAxes];
    uint8_t raw[HAL_kMaxJoystickAxes];
};
typedef struct HAL_JoystickAxes HAL_JoystickAxes;

struct HAL_JoystickPOVs {
    int16_t count;
    int16_t povs[HAL_kMaxJoystickPOVs];
};
typedef struct HAL_JoystickPOVs HAL_JoystickPOVs;

struct HAL_JoystickButtons {
    uint32_t buttons;
    uint8_t count;
};
typedef struct HAL_JoystickButtons HAL_JoystickButtons;

struct HAL_JoystickDescriptor {
    uint8_t isXbox;
    uint8_t type;
    char name[256];
    uint8_t axisCount;
    uint8_t axisTypes[HAL_kMaxJoystickAxes];
    uint8_t buttonCount;
    uint8_t povCount;
};
typedef struct HAL_JoystickDescriptor HAL_JoystickDescriptor;
