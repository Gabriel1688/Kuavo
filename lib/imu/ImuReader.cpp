#include "ImuReader.h"
#include "common/Config.h"
#include "spdlog/spdlog.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

// The CAN-over-UDP bridge uses the same frame format as UdpServer:
//   [1B header] [4B CAN-ID big-endian] [8B payload]
// Total = 13 bytes per frame.
#pragma pack(push, 1)
struct ImuCanFrame {
    uint8_t header;
    uint32_t canIdBE; // big-endian
    uint8_t data[8];
};
#pragma pack(pop)
static_assert(sizeof(ImuCanFrame) == 13, "unexpected ImuCanFrame size");

// ── construction / destruction ──────────────────────────────────────────────

ImuReader::ImuReader() = default;

ImuReader::~ImuReader() {
    shutdown();
}

// ── public API ──────────────────────────────────────────────────────────────

void ImuReader::start() {
    const auto &cfg = Config::instance().imu();
    m_serverIp = cfg.serverIp.empty() ? "127.0.0.1" : cfg.serverIp;
    m_localPort = cfg.localPort;
    m_baseId = cfg.baseId;
    SPDLOG_INFO("ImuReader: base CAN ID 0x{:03X}, {} frames, UDP {}:{}",
                m_baseId, m_numFrames, m_serverIp, m_localPort);

    m_shutdown = false;

    // Configure real-time thread: SCHED_FIFO priority 80, 128 KiB stack
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 128 * 1024);
    m_threadCreated = (pthread_create(&m_threadId, &attr, threadEntry, this) == 0);
    pthread_attr_destroy(&attr);
    if (!m_threadCreated) {
        SPDLOG_ERROR("ImuReader: failed to create thread");
    } else {
        // Set thread name for debugging
        pthread_setname_np(m_threadId, "imu-reader");
        
        struct sched_param param{};
        param.sched_priority = 80;
        int ret = pthread_setschedparam(m_threadId, SCHED_FIFO, &param);
        if (ret != 0) {
            SPDLOG_WARN("ImuReader: failed to set SCHED_FIFO/80: {}",
                        strerror(ret));
        } else {
            SPDLOG_INFO("ImuReader: SCHED_FIFO priority 80");
        }
    }
}

void ImuReader::shutdown() {
    m_shutdown = true;
    if (!m_threadCreated)
        return;
    void *res;
    pthread_join(m_threadId, &res);
    m_threadCreated = false;
}

// ── thread ──────────────────────────────────────────────────────────────────

/*static*/
void *ImuReader::threadEntry(void *arg) {
    static_cast<ImuReader *>(arg)->run();
    return nullptr;
}

void ImuReader::run() {
    // ── create + bind UDP socket ────────────────────────────────────────────
    int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        SPDLOG_ERROR("ImuReader: socket() failed: {}", strerror(errno));
        return;
    }

    // Allow address reuse so restarts don't fail with EADDRINUSE
    int optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(m_localPort));
    inet_aton(m_serverIp.c_str(), &addr.sin_addr);

    if (bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        SPDLOG_ERROR("ImuReader: bind({}:{}) failed: {}",
                     m_serverIp, m_localPort, strerror(errno));
        ::close(sockfd);
        return;
    }
    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    // ── epoll ───────────────────────────────────────────────────────────────
    int epfd = epoll_create1(0);
    struct epoll_event ev {};
    ev.data.fd = sockfd;
    ev.events = EPOLLIN;
    epoll_ctl(epfd, EPOLL_CTL_ADD, sockfd, &ev);

    SPDLOG_INFO("ImuReader: listening on {}:{}", m_serverIp, m_localPort);

    struct epoll_event events[1];
    while (!m_shutdown.load(std::memory_order_relaxed)) {
        int ready = epoll_wait(epfd, events, 1, 100 /* 100ms timeout */);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            SPDLOG_ERROR("ImuReader: epoll_wait error: {}", strerror(errno));
            break;
        }
        if (ready == 0) continue; // timeout, no data
        // Drain all available frames
        for (;;) {
            ImuCanFrame frame;
            ssize_t len = recvfrom(sockfd, &frame, sizeof(frame), 0, nullptr, nullptr);
            if (len < static_cast<ssize_t>(sizeof(frame))) {
                if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    break; // no more data
                if (len > 0)
                    SPDLOG_TRACE("ImuReader: short frame ({} bytes)", len);
                break;
            }

            uint32_t canId = __builtin_bswap32(frame.canIdBE);
            SPDLOG_TRACE("ImuReader: receive frame (0x{0:x} )", canId);
            // Filter: only accept [baseCanId .. baseCanId + numFrames)
            if (canId < static_cast<uint32_t>(m_baseId) ||
                canId >= static_cast<uint32_t>(m_baseId) + static_cast<uint32_t>(m_numFrames))
                continue;

            int offset = static_cast<int>(canId - static_cast<uint32_t>(m_baseId));

            // Extract two little-endian float32 values from the 8-byte payload
            float f0, f1;
            std::memcpy(&f0, &frame.data[0], sizeof(float));
            std::memcpy(&f1, &frame.data[4], sizeof(float));

            // New measurement cycle starts with the first CAN frame
            if (offset == 0) {
                m_frameCount = 0;
            }

            // Populate the per-cycle accumulator from the first three frames.
            // Frames 0x514-0x516 carry accelerometer and gyroscope data that
            // feeds the Composer (and ultimately the Mercury Controller).
            switch (offset) {
            case 0:
                m_accumulator.imu_acc[0] = static_cast<double>(f0);
                m_accumulator.imu_acc[1] = static_cast<double>(f1);
                break;
            case 1:
                m_accumulator.imu_acc[2] = static_cast<double>(f0);
                m_accumulator.imu_ang_vel[0] = static_cast<double>(f1);
                break;
            case 2:
                m_accumulator.imu_ang_vel[1] = static_cast<double>(f0);
                m_accumulator.imu_ang_vel[2] = static_cast<double>(f1);
                break;
            default:
                // Magnetometer, Euler, and quaternion frames are parsed above
                // for diagnostic purposes but are not copied into ImuStageData;
                // the Mercury Controller computes orientation from raw sensors.
                break;
            }

            ++m_frameCount;
            SPDLOG_TRACE("ImuReader: CAN 0x{:03X} offset={} [{:.4f}, {:.4f}]", canId, offset, f0, f1);

            if (m_frameCount == m_numFrames) {
                m_accumulator.timestamp_ns = mercury::get_monotonic_ns();
                m_accumulator.sequence = ++m_sequence;
                constexpr double dt = 1.0 / kRateHz;
                for (int i = 0; i < 3; ++i) {
                    m_accumulator.imu_inc[i] = m_accumulator.imu_ang_vel[i] * dt;
                }

                auto* stage = m_stage.load(std::memory_order_acquire);
                if (stage) {
                    stage->publish(m_accumulator);
                } else {
                    SPDLOG_TRACE("ImuReader: staging buffer not set, skipping publish");
                }

                m_frameCount = 0;
            }
        }
    }

    ::close(sockfd);
    ::close(epfd);
    SPDLOG_INFO("ImuReader: thread exiting");
}
