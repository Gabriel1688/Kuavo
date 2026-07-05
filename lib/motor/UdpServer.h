#pragma once

#include "CANAPI.h"
#include "Common.h"
#include <arpa/inet.h>
#include <atomic>
#include <errno.h>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>

#pragma pack(1)
struct CANFrame {
    uint8_t FrameHeader = 0x08;// 0：标准帧 0： 数据帧, DLC = xx;
    uint32_t FrameId = 0x01;   // CAN ID 使用电机ID作为CAN ID
    uint8_t data[CAN_MAX_DLEN] = {0};
    void modify(const uint32_t id, const uint8_t *send_data, uint8_t dataSize) {
        FrameId = __builtin_bswap32(id);//change to big endian format.
        std::copy(send_data, send_data + dataSize, data);
    }
} __attribute__((aligned(8)));
#pragma pack()

class UdpServer {
private:
    int m_sockfd;
    std::atomic<bool> m_isClosed;
    sockaddr_in m_server;
    sockaddr_in m_clientLeft, m_clientRight;
    std::map<int32_t, client_observer_t<uint8_t>> m_subscribers;
    std::mutex m_subscribersMtx;
    pthread_t m_threadId;
    std::mutex m_frameIdsMutex;
    std::map<int32_t, HAL_CANHandle> m_frameIds;// keep the reply frameId and handle of device.
    std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> *m_canHandles;
    std::map<int, sockaddr_in *> m_deviceIPs;//IP address associate to specific CAN DeviceIds;

    void dispatchMessage(const CANFrame &frame, size_t msgSize);
    bool init(const std::string address, uint16_t localPort, uint16_t remotePort);
    void run();
    static void *EntryOfThread(void *argv);

    std::string m_interface;
    int m_port;
    std::string m_address;
    int m_socketFd;

    bool initialize_socket(const std::string &interface);

    bool is_initialized() const { return m_socketFd >= 0; }
    // Private constructor
public:
    //1. Get instance with specific ID
    static UdpServer &getInstance(int id = 0) {
        // Thread-safe in C++11 and later
        static std::mutex instances_mtx;
        static std::map<int, UdpServer> instances;
        std::lock_guard<std::mutex> lock(instances_mtx);
        if (instances.find(id) == instances.end()) {
            instances.emplace(std::piecewise_construct,
                              std::forward_as_tuple(id),
                              std::forward_as_tuple(id));
        }
        return instances.at(id);
    }
    UdpServer(int id) { m_serverId = id; };

    // 2. Delete copy constructor and assignment operator
    UdpServer(const UdpServer &) = delete;
    UdpServer &operator=(const UdpServer &) = delete;

    const std::string &getInterface() const { return m_interface; }
    void start();

    /**
     * Sends a CAN message.
    *
    * @param[in] messageID the CAN ID to send
    * @param[in] data      the data to send (0-8 bytes)
    * @param[in] dataSize  the size of the data to send (0-8 bytes)
    * @param[out] status    Error status variable. 0 on success.
    */
    void sendMsg(CANFrameId frameId, const uint8_t *data, uint8_t dataSize, bool reply, int32_t *status);
    void subscribe(const int32_t deviceId, const client_observer_t<uint8_t> &observer);
    void setCanHandles(std::map<HAL_CANHandle, std::shared_ptr<CANStorage>> *p_canHandles) {
        m_canHandles = p_canHandles;
    }

    void unsubscribe(const int32_t deviceId, const client_observer_t<uint8_t> &observer);
    void bindDevicesToServer(int deviceId);
    sockaddr_in *getClientAddrByDeviceId(int deviceId);
    bool close();
    ~UdpServer() = default;

    // Map to store different instances using smart pointers
    static std::map<int, std::unique_ptr<UdpServer>> instances_;
    static std::mutex instances_mutex_;

private:
    // 3. Private constructor prevents direct instantiation
    int m_serverId;
};
