#include "CAN.h"
#include "UdpServer.h"
#include "common/Config.h"
#include "spdlog/spdlog.h"
#include <atomic>

CAN::CAN(int deviceId) {
    int32_t status = 0;
    m_handle = HAL_InitializeCAN(kTeamManufacturer, deviceId, kTeamDeviceType, &status);
}

CAN::CAN(int deviceId, int deviceManufacturer, int deviceType) {
    int32_t status = 0;
    m_handle = HAL_InitializeCAN(
        static_cast<HAL_CANManufacturer>(deviceManufacturer), deviceId,
        static_cast<HAL_CANDeviceType>(deviceType), &status);
}

void CAN::registrateCallback(const int32_t deviceId, const client_observer_t<uint8_t> &observer) {
    static std::atomic<bool> warned{false};
    const int maxCanDevice = Config::instance().motor().maxCanDevice;

    if (maxCanDevice != MAX_CAN_DEVICE && !warned.exchange(true)) {
        SPDLOG_WARN("Configured max_can_device ({}) differs from compile-time default ({}). Using configured value.",
                    maxCanDevice, MAX_CAN_DEVICE);
    }

    if (deviceId < maxCanDevice) {
        UdpServer::getInstance(0).subscribe(deviceId, observer);
        UdpServer::getInstance(0).bindDevicesToServer(deviceId);
    } else {
        UdpServer::getInstance(1).subscribe(deviceId, observer);
        UdpServer::getInstance(1).bindDevicesToServer(deviceId);
    }
}

void CAN::writePacket(const uint8_t *data, int length, int apiId, bool reply) {
    int32_t status = 0;
    HAL_WriteCANPacket(m_handle, data, length, apiId, &status, reply);
}

void CAN::writePacketRepeating(const uint8_t *data, int length, int apiId, int repeatMs) {
    int32_t status = 0;
    HAL_WriteCANPacketRepeating(m_handle, data, length, apiId, repeatMs, &status);
}

void CAN::stopPacketRepeating(int apiId) {
    int32_t status = 0;
    HAL_StopCANPacketRepeating(m_handle, apiId, &status);
}

bool CAN::readPacketNew(int apiId, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketNew(m_handle, apiId, data->data, &data->length, &data->timestamp, &status);
    if (status != 0) {
        return false;
    }
    return true;
}

bool CAN::readPacketLatest(int apiId, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketLatest(m_handle, apiId, data->data, &data->length, &data->timestamp, &status);
    if (status != 0) {
        return false;
    } else {
        return true;
    }
}

bool CAN::readPacketTimeout(int apiId, int timeoutMs, CANData *data) {
    int32_t status = 0;
    HAL_ReadCANPacketTimeout(m_handle, apiId, data->data, &data->length, &data->timestamp, timeoutMs, &status);
    if (status != 0) {
        return false;
    } else {
        return true;
    }
}
