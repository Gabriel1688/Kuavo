
#pragma once

#include "CANAPI.h"
#include "Common.h"
#include <stdint.h>

struct CANData {
    /** Contents of the CAN packet. */
    uint8_t data[8];
    /** Length of packet in bytes. */
    int32_t length;
    /** CAN frame timestamp in milliseconds. */
    uint64_t timestamp;
};

/**
 * High level class for interfacing with CAN devices conforming to
 * the standard CAN spec.
 *
 * No packets that can be sent gets blocked by the RoboRIO, so all methods
 * work identically in all robot modes.
 *
 * All methods are thread save, however the buffer objects passed in
 * by the user need to not be modified for the duration of their calls.
 */
class CAN {
public:
    /**
     * Create a new CAN communication interface with the specific device ID.
     * This uses the team manufacturer and device types.
     * The device ID is 6 bits (0-63)
     *
     * @param deviceId The device id
     */
    explicit CAN(int deviceId);

    /**
     * Create a new CAN communication interface with a specific device ID,
     * manufacturer and device type. The device ID is 6 bits, the
     * manufacturer is 8 bits, and the device type is 5 bits.
     *
     * @param deviceId           The device ID
     * @param deviceManufacturer The device manufacturer
     * @param deviceType         The device type
     */
    CAN(int deviceId, int deviceManufacturer, int deviceType);

    CAN(CAN &&) = default;

    CAN &operator=(CAN &&) = default;

    /**
     * Write a packet to the CAN device with a specific ID. This ID is 10 bits.
     *
     * @param data The data to write (8 bytes max)
     * @param length The data length to write
     * @param apiId The API ID to write.
     */
    void writePacket(const uint8_t *data, int length, int apiId, bool reply = false);

    /**
     * Write a repeating packet to the CAN device with a specific ID. This ID is
     * 10 bits. The RoboRIO will automatically repeat the packet at the specified
     * interval
     *
     * @param data The data to write (8 bytes max)
     * @param length The data length to write
     * @param apiId The API ID to write.
     * @param repeatMs The period to repeat the packet at.
     */
    void writePacketRepeating(const uint8_t *data, int length, int apiId, int repeatMs);

    /**
     * Write an RTR frame to the CAN device with a specific ID. This ID is 10
     * bits. The length by spec must match what is returned by the responding
     * device
     *
     * @param length The length to request (0 to 8)
     * @param apiId The API ID to write.
     */
    void writeRTRFrame(int length, int apiId);

    /**
     * Stop a repeating packet with a specific ID. This ID is 10 bits.
     *
     * @param apiId The API ID to stop repeating
     */
    void stopPacketRepeating(int apiId);

    /**
     * Read a new CAN packet. This will only return properly once per packet
     * received. Multiple calls without receiving another packet will return
     * false.
     *
     * @param apiId The API ID to read.
     * @param data Storage for the received data.
     * @return True if the data is valid, otherwise false.
     */
    bool readPacketNew(int apiId, CANData *data);

    /**
     * Read a CAN packet. The will continuously return the last packet received,
     * without accounting for packet age.
     *
     * @param apiId The API ID to read.
     * @param data Storage for the received data.
     * @return True if the data is valid, otherwise false.
     */
    bool readPacketLatest(int apiId, CANData *data);

    /**
     * Read a CAN packet. The will return the last packet received until the
     * packet is older then the requested timeout. Then it will return false.
     *
     * @param apiId The API ID to read.
     * @param timeoutMs The timeout time for the packet
     * @param data Storage for the received data.
     * @return True if the data is valid, otherwise false.
     */
    bool readPacketTimeout(int apiId, int timeoutMs, CANData *data);

    void registrateCallback(const int32_t deviceId, const client_observer_t<uint8_t> &observer);

    /// Team manufacturer.
    static constexpr HAL_CANManufacturer kTeamManufacturer = static_cast<HAL_CANManufacturer>(2);//HAL_CAN_Man_Dummy;

    /// Team device type.
    static constexpr HAL_CANDeviceType kTeamDeviceType = static_cast<HAL_CANDeviceType>(4);// HAL_CAN_Dev_kMotionController;
private:
    HAL_CANHandle m_handle;
};
