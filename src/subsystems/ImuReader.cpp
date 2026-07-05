#include "ImuReader.h"
#include "common/Config.h"
#include "spdlog/spdlog.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

// The CAN-over-UDP bridge uses the same frame format as UdpServer:
//   [1B header] [4B CAN-ID big-endian] [8B payload]
// Total = 13 bytes per frame.
#pragma pack(push, 1)
struct ImuCanFrame {
    uint8_t header;
    uint32_t canIdBE;// big-endian
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
    m_numFrames = cfg.numFrames > 0 ? std::min(cfg.numFrames, kMaxFrames)
                                    : kMaxFrames;
    m_baseCanId = static_cast<uint32_t>(cfg.startId + (cfg.imuId - 1) * 8);

    SPDLOG_INFO("ImuReader: base CAN ID 0x{:03X}, {} frames, UDP {}:{}",
                m_baseCanId, m_numFrames, m_serverIp, m_localPort);

    m_shutdown = false;
    m_threadCreated = (pthread_create(&m_threadId, nullptr, threadEntry, this) == 0);
    if (!m_threadCreated) {
        SPDLOG_ERROR("ImuReader: failed to create thread");
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

void ImuReader::getFloats(float *out) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::memcpy(out, m_data, sizeof(m_data));
}

void ImuReader::getDoubles(double *out) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int i = 0; i < kMaxFloats; ++i)
        out[i] = static_cast<double>(m_data[i]);
}

float ImuReader::getFloat(int idx) const {
    if (idx < 0 || idx >= kMaxFloats)
        return 0.0f;
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_data[idx];
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
        int ready = epoll_wait(epfd, events, 1, 100 /*ms timeout*/);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            SPDLOG_ERROR("ImuReader: epoll_wait error: {}", strerror(errno));
            break;
        }
        if (ready == 0)
            continue;// timeout — check m_shutdown flag

        // Drain all available frames
        for (;;) {
            ImuCanFrame frame;
            ssize_t len = recvfrom(sockfd, &frame, sizeof(frame), 0, nullptr, nullptr);
            if (len < static_cast<ssize_t>(sizeof(frame))) {
                if (len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    break;// no more data
                if (len > 0)
                    SPDLOG_TRACE("ImuReader: short frame ({} bytes)", len);
                break;
            }

            uint32_t canId = __builtin_bswap32(frame.canIdBE);

            // Filter: only accept [baseCanId .. baseCanId + numFrames)
            if (canId < m_baseCanId || canId >= m_baseCanId + static_cast<uint32_t>(m_numFrames))
                continue;

            int offset = static_cast<int>(canId - m_baseCanId);
            int slot = offset * 2;// 2 floats per frame

            // Extract two little-endian float32 values from the 8-byte payload
            float f0, f1;
            std::memcpy(&f0, &frame.data[0], sizeof(float));
            std::memcpy(&f1, &frame.data[4], sizeof(float));

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_data[slot] = f0;
                m_data[slot + 1] = f1;
            }

            // First frame of a new cycle → bump counter + mark data available
            if (offset == 0) {
                m_cycleCount.fetch_add(1, std::memory_order_relaxed);
                m_hasData.store(true, std::memory_order_release);
            }

            SPDLOG_TRACE("ImuReader: CAN 0x{:03X} offset={} [{:.4f}, {:.4f}]",
                         canId, offset, f0, f1);
        }
    }

    ::close(sockfd);
    ::close(epfd);
    SPDLOG_INFO("ImuReader: thread exiting");
}
