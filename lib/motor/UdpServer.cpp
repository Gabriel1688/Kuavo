#include "UdpServer.h"
#include "common/Config.h"
#include "spdlog/fmt/ranges.h"
#include "spdlog/spdlog.h"
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>

bool UdpServer::init(const std::string address, uint16_t localPort, uint16_t remotePort) {
    // bind socket to UDP Server port.
    m_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int status = inet_aton(address.c_str(), &m_server.sin_addr);
    if (!status) {
        // if hostname is not in IP strings and dots format, try resolve it
        struct hostent *host;
        struct in_addr **addrList;
        if ((host = gethostbyname(address.c_str())) == nullptr) {
            throw std::runtime_error("Failed to resolve hostname");
        }
        addrList = (struct in_addr **) host->h_addr_list;
        m_server.sin_addr = *addrList[0];
    }
    m_server.sin_family = AF_INET;
    m_server.sin_port = htons(localPort);
    status = bind(m_sockfd, (struct sockaddr *) &m_server, sizeof(m_server));
    if (status == -1) {
        SPDLOG_ERROR("Error binding socket to local address {}:{}, errno:{}.", address, localPort, strerror(errno));
        m_isClosed = true;
        return false;
    }
    /* Disable socket blocking */
    fcntl(m_sockfd, F_SETFL, O_NONBLOCK);

    /* Initialize the UDP client for left right leg*/
    const auto &cfg = Config::instance().udp();
    inet_aton(cfg.clientIpLeft.c_str(), &m_clientLeft.sin_addr);
    m_clientLeft.sin_family = AF_INET;
    m_clientLeft.sin_port = htons(remotePort);

    inet_aton(cfg.clientIpRight.c_str(), &m_clientRight.sin_addr);
    m_clientRight.sin_family = AF_INET;
    m_clientRight.sin_port = htons(remotePort);

    m_isClosed = false;
    SPDLOG_INFO("UdpServer[{}] listening on port {}, remote port {}", m_serverId, localPort, remotePort);
    return true;
}

void UdpServer::start() {
    const auto &udpCfg = Config::instance().udp();
    int localPort = udpCfg.baseLocalPort + m_serverId * 2;
    int remotePort = udpCfg.baseRemotePort + m_serverId * 2;
    if (init(udpCfg.serverIp, localPort, remotePort) == false) {
        SPDLOG_ERROR("Failed to initialize sockets for UdpServer[{}]", m_serverId);
    } else {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 128 * 1024);
        int rc = pthread_create(&m_threadId, &attr, EntryOfThread, this);
        pthread_attr_destroy(&attr);
        if (rc != 0) {
            SPDLOG_ERROR("Failed to initialize thread for UdpServer[{}]", m_serverId);
        } else {
            struct sched_param param{};
            param.sched_priority = 88;
            if (pthread_setschedparam(m_threadId, SCHED_FIFO, &param) != 0) {
                SPDLOG_WARN("UdpServer[{}]: failed to set SCHED_FIFO/88: {}",
                            m_serverId, strerror(errno));
            } else {
                SPDLOG_INFO("UdpServer[{}]: SCHED_FIFO priority 88", m_serverId);
            }
        }
    }
}

/*static*/
void *UdpServer::EntryOfThread(void *argv) {
    UdpServer *server = static_cast<UdpServer *>(argv);
    server->run();
    return (void *) server;
}

void UdpServer::sendMsg(CANFrameId frameId, const uint8_t *data, uint8_t dataSize, bool reply,
                        __attribute__((unused)) int32_t *status) {
    sockaddr_in *client = getClientAddrByDeviceId(frameId.deviceId);
    CANFrame frame;
    frame.modify(frameId.forwardCANId, data, dataSize);
    auto p = reinterpret_cast<const uint8_t *>(&frame);
    SPDLOG_TRACE("<------ {:04x} : {:#04x}", frame.FrameId, fmt::join(p, p + 13, " "));

    // Register pending reply BEFORE sending to prevent race with receive thread
    if (reply) {
        std::scoped_lock lock(m_frameIdsMutex);
        if (m_frameIds.find(frameId.replyCANId) == m_frameIds.end()) {
            m_frameIds.insert(std::make_pair(frameId.replyCANId, frameId.hanlde));
            SPDLOG_TRACE("Wait reply from device [{0:d}],FrameId : [{1:04x}].",
                         frameId.deviceId, frameId.replyCANId);
        }
    }

    const size_t numBytesSent = sendto(m_sockfd, (uint8_t *) &frame, 13, 0, (struct sockaddr *) client, sizeof(m_server));
    if (numBytesSent < dataSize) {
        // Remove pending reply on send failure
        if (reply) {
            std::scoped_lock lock(m_frameIdsMutex);
            m_frameIds.erase(frameId.replyCANId);
        }
        if (numBytesSent <= 0) {
            SPDLOG_ERROR("Failed to send data to client, error : {} ", strerror(errno));
        } else {
            SPDLOG_ERROR("Only {} bytes out of {} was sent to client", numBytesSent, dataSize);
        }
    }
}

void UdpServer::subscribe(const int32_t deviceId, const client_observer_t<uint8_t> &observer) {
    std::lock_guard<std::mutex> lock(m_subscribersMtx);
    m_subscribers.insert(std::make_pair(deviceId, observer));
}

sockaddr_in *UdpServer::getClientAddrByDeviceId(int deviceId) {
    auto iter = m_deviceIPs.find(deviceId);
    if (iter == m_deviceIPs.end()) {
        SPDLOG_ERROR("Device with device ID {:d} does not exist", deviceId);
        return nullptr;
    }
    return iter->second;
}

void UdpServer::bindDevicesToServer(int deviceId) {
    std::lock_guard<std::mutex> lock(m_subscribersMtx);
    if (deviceId < Config::instance().motor().maxCanDevice) {
        m_deviceIPs.insert(std::make_pair(deviceId, &m_clientLeft));
    } else {
        m_deviceIPs.insert(std::make_pair(deviceId, &m_clientRight));
    }
}

/*
 * Publish message to observer.
 */
void UdpServer::dispatchMessage(const CANFrame &frame, size_t msgSize) {
    auto FrameId = __builtin_bswap32(frame.FrameId);

    if (FrameId == 0x7ff) {
        FrameId = frame.data[0];// overwrite it with the real canId because data[0] is the low byte of CAN ID.
    }

    // Try pending reply lookup first (release lock before callback)
    std::function<void(const uint8_t *, size_t)> handler;
    std::shared_ptr<CANStorage> storage;
    {
        std::scoped_lock lock(m_frameIdsMutex);
        auto itmap = m_frameIds.find(FrameId);
        if (itmap != m_frameIds.end()) {
            auto handle = itmap->second;
            m_frameIds.erase(itmap);// always consume the pending reply entry
            {
                std::scoped_lock canLock(canHandlesMutex);
                auto canIt = m_canHandles->find(handle);
                if (canIt == m_canHandles->end()) {
                    SPDLOG_WARN("dispatchMessage: handle {} not found in m_canHandles", handle);
                    return;
                }
                storage = canIt->second;
            }
            auto subscriber = m_subscribers.find(storage->deviceId);
            if (subscriber != m_subscribers.end()) {
                handler = subscriber->second.packetHandler;
            }
        }
    }

    if (handler) {
        handler(frame.data, 8);
        storage->replyEvent.set();
        auto pr = reinterpret_cast<const uint8_t *>(&frame);
        SPDLOG_TRACE("------> {:04x} : {:#04x}", frame.FrameId, fmt::join(pr, pr + 13, " "));
        return;
    }

    // No pending reply - deliver to subscriber by deriving deviceId from masterId
    // DAMIAO protocol: response CAN ID (masterId) = deviceId + 0x10
    int32_t derivedDeviceId = static_cast<int32_t>(FrameId) - 0x10;
    {
        std::lock_guard<std::mutex> lock(m_subscribersMtx);
        auto subscriber = m_subscribers.find(derivedDeviceId);
        if (subscriber != m_subscribers.end()) {
            handler = subscriber->second.packetHandler;
        }
    }

    if (handler) {
        handler(frame.data, 8);
        auto pr = reinterpret_cast<const uint8_t *>(&frame);
        SPDLOG_TRACE("unsolicited --> {:04x} : {:#04x}", frame.FrameId, fmt::join(pr, pr + 13, " "));
    } else {
        auto pd = reinterpret_cast<const uint8_t *>(&frame);
        SPDLOG_TRACE("drop--> {:04x} : {:#04x}", frame.FrameId, fmt::join(pd, pd + 13, " "));
    }
}

void UdpServer::run() {
    /* Initialize variables for epoll */
    struct epoll_event ev;
    int epfd = epoll_create(2);
    ev.data.fd = m_sockfd;
    ev.events = EPOLLIN;
    epoll_ctl(epfd, EPOLL_CTL_ADD, m_sockfd, &ev);

    struct epoll_event events[1];
    SPDLOG_INFO("UdpServer ::receiveTask is running.");
    while (!m_isClosed) {
        int ready = epoll_wait(epfd, events, 1, -1);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            perror("epoll_wait error.");
            return;
        } else {
            for (int i = 0; i < ready; i++) {
                if (events[i].data.fd == m_sockfd) {
                    CANFrame frame;
                    const size_t length = recvfrom(m_sockfd, (uint8_t *) &frame, sizeof(frame) - 1, 0, nullptr, nullptr);
                    if (length < 1) {
                        if (length == 0) {
                            SPDLOG_INFO("receive empty package");
                        } else {
                            SPDLOG_ERROR("Failed to receive data {}", strerror(errno));
                        }
                    } else {
                        dispatchMessage(frame, length);
                    }
                }
            }
        }
    }
}

bool UdpServer::close() {
    if (m_isClosed) {
        SPDLOG_INFO("server is already closed");
        return false;
    }
    m_isClosed = true;
    void *result;
    if (pthread_join(m_threadId, &result) != 0) {
        SPDLOG_ERROR("Failed to join thread.");
        return false;
    }

    const bool closeFailed = (::close(m_sockfd) == -1);
    if (closeFailed) {
        SPDLOG_ERROR("failed to close socket, error:{} ", strerror(errno));
        return false;
    }
    return true;
}
