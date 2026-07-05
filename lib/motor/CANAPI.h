// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "common/Synchronization.h"
#include <map>
#include <mutex>
#include <stdint.h>
#include <string>

using HAL_CANHandle = int32_t;
extern std::mutex canHandlesMutex;
enum HAL_CANDeviceType : int32_t {
    /// Broadcast.
    HAL_CAN_Dev_kBroadcast = 0,
    /// Robot arm controller.
    HAL_CAN_Dev_karmController = 1,
    /// Robot gripper controller.
    HAL_CAN_Dev_kgripperController = 2,
    /// Robot gyros Motor controller.
    HAL_CAN_Dev_kgyroSensor = 3,
    /// Motion controller.
    HAL_CAN_Dev_kMotionController = 4
};

enum HAL_CANManufacturer : int32_t {
    /// Broadcast.
    HAL_CAN_Man_kBroadcast = 0,
    /// Dummy Robot.
    HAL_CAN_Man_Dummy = 1,
    /// DAMIAO.
    HAL_CAN_Man_Dm = 2
};
struct CANFrameId {
    uint8_t deviceId;
    uint32_t forwardCANId;   /* Input CAN frame Id */
    uint32_t replyCANId;     /* Response CAN frame Id */
    HAL_CANHandle hanlde;    /* CANStorage handle of this message */
    CANFrameId() : deviceId(0), forwardCANId(0), replyCANId(0), hanlde(0) {};
};

namespace {
    struct Receives {
        uint64_t lastTimeStamp;
        uint8_t data[8];
        uint8_t length;
    };

    struct CANStorage {
        HAL_CANManufacturer manufacturer;
        HAL_CANDeviceType deviceType;
        uint8_t deviceId;
        uint8_t masterId;    // reply frame id for Damiao protocol
        std::mutex periodicSendsMutex;
        std::mutex receivesMutex;
        std::map<int32_t, Receives> receives;
        std::map<int32_t, int32_t> periodicSends;
        wpi::Event replyEvent{false, false};
    };
}// namespace

/**
 * @defgroup hal_canapi CAN API Functions
 * @ingroup hal_capi
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes a CAN device.
 *
 * These follow the FIRST standard CAN layout.
 * https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
 *
 * @param[in] manufacturer the can manufacturer
 * @param[in] deviceId     the device ID (0-63)
 * @param[in] deviceType   the device type
 * @param[out] status      Error status variable. 0 on success.
 * @return the created CAN handle
 */
HAL_CANHandle HAL_InitializeCAN(HAL_CANManufacturer manufacturer,
                                int32_t deviceId, HAL_CANDeviceType deviceType,
                                int32_t *status);

/**
 * Frees a CAN device
 *
 * @param handle the CAN handle
 */
void HAL_CleanCAN(HAL_CANHandle handle);

/**
 * Writes a packet to the CAN device with a specific ID.
 *
 * This ID is 10 bits.
 *
 * @param[in] handle  the CAN handle
 * @param[in] data    the data to write (0-8 bytes)
 * @param[in] length  the length of data (0-8)
 * @param[in] apiId   the ID to write (0-1023 bits)
 * @param[out] status Error status variable. 0 on success.
 */
void HAL_WriteCANPacket(HAL_CANHandle handle, const uint8_t *data,
                        int32_t length, int32_t apiId, int32_t *status, bool reply = false);

/**
 * Writes a repeating packet to the CAN device with a specific ID.
 *
 * This ID is 10 bits.
 *
 * The RoboRIO will automatically repeat the packet at the specified interval
 *
 * @param[in] handle   the CAN handle
 * @param[in] data     the data to write (0-8 bytes)
 * @param[in] length   the length of data (0-8)
 * @param[in] apiId    the ID to write (0-1023)
 * @param[in] repeatMs the period to repeat in ms
 * @param[out] status  Error status variable. 0 on success.
 */
void HAL_WriteCANPacketRepeating(HAL_CANHandle handle, const uint8_t *data,
                                 int32_t length, int32_t apiId,
                                 int32_t repeatMs, int32_t *status, bool reply = false);

/**
 * Stops a repeating packet with a specific ID.
 *
 * This ID is 10 bits.
 *
 * @param[in] handle  the CAN handle
 * @param[in] apiId   the ID to stop repeating (0-1023)
 * @param[out] status Error status variable. 0 on success.
 */
void HAL_StopCANPacketRepeating(HAL_CANHandle handle, int32_t apiId, int32_t *status, bool reply = false);

/**
 * Reads a new CAN packet.
 *
 * This will only return properly once per packet received. Multiple calls
 * without receiving another packet will return an error code.
 *
 * @param[in] handle             the CAN handle
 * @param[in] apiId              the ID to read (0-1023)
 * @param[out] data              the packet data (8 bytes)
 * @param[out] length            the received length (0-8 bytes)
 * @param[out] receivedTimestamp the packet received timestamp in ms (based off
 *                               of CLOCK_MONOTONIC)
 * @param[out] status            Error status variable. 0 on success.
 */
void HAL_ReadCANPacketNew(HAL_CANHandle handle, int32_t apiId, uint8_t *data,
                          int32_t *length, uint64_t *receivedTimestamp,
                          int32_t *status, bool reply = false);

void HAL_ReadCANPacketLatest(HAL_CANHandle handle, int32_t apiId, uint8_t *data,
                             int32_t *length, uint64_t *receivedTimestamp,
                             int32_t *status);
void HAL_ReadCANPacketTimeout(HAL_CANHandle handle, int32_t apiId,
                              uint8_t *data, int32_t *length,
                              uint64_t *receivedTimestamp, int32_t timeoutMs,
                              int32_t *status);

#ifdef __cplusplus
}// extern "C"
#endif
/** @} */