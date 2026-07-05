#include "spdlog/fmt/ranges.h"
#include "spdlog/spdlog.h"
#include <arpa/inet.h>
#include <array>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <map>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
using namespace std;
using namespace std::chrono_literals;

static void printUsage(const char *progName) {
    cerr << "Usage: " << progName << " [-mode <motor|imu>] [-local <port>] [-remote <port>]" << endl;
    cerr << "  -mode  <motor|imu>  Mode to use (default: motor)" << endl;
    cerr << "  -local  <port>  Local port to bind to (default: 8886)" << endl;
    cerr << "  -remote <port>  Remote port to send responses to (default: 8887)" << endl;
    cerr << "Example: " << progName << " -mode motor -local 8886 -remote 8887" << endl;
}

std::map<int, std::array<uint8_t, 13>> responses;
int main(int argc, char *argv[]) {
    std::string mode = "motor";
    int localPort = 8886;
    int remotePort = 8887;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-mode") == 0 && i + 1 < argc) {
            mode = argv[++i];
        } else if (strcmp(argv[i], "-local") == 0 && i + 1 < argc) {
            localPort = atoi(argv[++i]);
            if (localPort <= 0 || localPort > 65535) {
                cerr << "Error: invalid local port number" << endl;
                printUsage(argv[0]);
                return 1;
            }
        } else if (strcmp(argv[i], "-remote") == 0 && i + 1 < argc) {
            remotePort = atoi(argv[++i]);
            if (remotePort <= 0 || remotePort > 65535) {
                cerr << "Error: invalid remote port number" << endl;
                printUsage(argv[0]);
                return 1;
            }
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else {
            cerr << "Error: unknown argument: " << argv[i] << endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    SPDLOG_INFO("UdpClient starting as [{}] with local port: {}, remote port: {}", mode, localPort, remotePort);
    //open datagram oriented socket with internet address
    //also keep track of the socket descriptor
    int _sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_sockfd < 0) {
        cerr << "Error establishing the server socket" << endl;
        exit(1);
    }
    /* Disable socket blocking */
    fcntl(_sockfd, F_SETFL, O_NONBLOCK);

    // Set up local address structure
    struct sockaddr_in clientAddr;
    memset(&clientAddr, 0, sizeof(clientAddr));
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(localPort);             // Local port to bind to
    clientAddr.sin_addr.s_addr = inet_addr("127.0.0.1");// Local IP to bind to
    socklen_t clientAddrSize = sizeof(clientAddr);

    // Bind socket to local address
    if (bind(_sockfd, (struct sockaddr *) &clientAddr, sizeof(clientAddr)) < 0) {
        std::cerr << "Error binding socket to local address" << std::endl;
        close(_sockfd);
        exit(1);
    }

    //setup a socket and connection tools
    sockaddr_in servAddr;
    bzero((char *) &servAddr, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(remotePort);
    servAddr.sin_addr.s_addr = inet_addr("127.0.0.1");// Server IP to send to

    //also keep track of the amount of data sent as well
    uint8_t response[] = {0x11, 0x70, 0x81, 0x7F, 0xF8, 0x01, 0x1C, 0x1B};
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFC, {0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}));
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFD, {0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}));
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFE, {0x08, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}));
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0x33, {0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x33, 0x07, 0xFD, 0x7F, 0x00, 0x00}));
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xCC, {0x08, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0xCC, 0x00, 0xFD, 0x7F, 0x00, 0x00}));
    responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFF, {0x08, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x03}));
    /* Initialize variables for epoll */
    struct epoll_event ev;
    int epfd = epoll_create(2);
    ev.data.fd = _sockfd;
    ev.events = EPOLLIN;
    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, _sockfd, &ev);
    struct epoll_event events[2];
    bool _isConnected = true;
    if (mode == "motor") {
        while (_isConnected) {
            int ready = epoll_wait(epfd, events, 2, -1);//20 milliseconds
            if (ready < 0) {
                perror("epoll_wait error.");
                exit(0);
            } else if (ready == 0) {
                continue; /* timeout, no data coming */
                continue;
            } else {
                for (int i = 0; i < ready; i++) {
                    if (events[i].data.fd == _sockfd) {
                        uint8_t msg[13];
                        memset(msg, 0, 13);

                        //handle the new connection with client address
                        const size_t numOfBytesReceived = recvfrom(_sockfd, msg, 13, 0, NULL, NULL);
                        if (numOfBytesReceived < 1) {
                            std::string errorMsg;
                            if (numOfBytesReceived == 0) {
                                errorMsg = "Server closed connection";
                            } else {
                                errorMsg = strerror(errno);
                            }
                            _isConnected = false;
                        } else {
                            SPDLOG_INFO("------> {:#04x}", fmt::join(msg, msg + 13, " "));

                            unsigned int command = 0x0;
                            if ((msg[12] == 0xFC) || (msg[12] == 0xFD) || (msg[12] == 0xFE)) {
                                command = msg[12];
                            } else if (msg[7] == 0x33) {
                                command = 0x33;
                            } else if (msg[7] == 0xCC) {
                                command = 0xCC;
                            } else if ((msg[5] == 0x7F) && (msg[6] == 0xFF)) {
                                command = 0xFF;
                            }

                            auto itmap = responses.find(command);
                            if (itmap != responses.end()) {
                                if ((command == 0xcc) || (command == 0x33)) {
                                    msg[5] += 0x10;
                                    itmap->second[05] = msg[5];// low byte of canId
                                } else {
                                    msg[4] += 0x10;
                                    itmap->second[04] = msg[4];// low byte of canId
                                }
                                sendto(_sockfd, itmap->second.data(), 13, 0, (struct sockaddr *) &servAddr, sizeof(struct sockaddr_in));
                                SPDLOG_INFO("<------ {:#04x}", fmt::join(itmap->second.begin(), itmap->second.end(), " "));
                            }
                        }
                    }
                }
            }//read and reply
        }
    } else {
        //TODO:: need to build the mock data for  IMU
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFC, {0x08, 0x00, 0x00, 0x05, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFD, {0x08, 0x00, 0x00, 0x05, 0x15, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFE, {0x08, 0x00, 0x00, 0x05, 0x16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0x33, {0x08, 0x00, 0x00, 0x05, 0x17, 0x00, 0x00, 0x33, 0x07, 0xFD, 0x7F, 0x00, 0x00}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xCC, {0x08, 0x00, 0x00, 0x05, 0x18, 0x00, 0x00, 0xCC, 0x00, 0xFD, 0x7F, 0x00, 0x00}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFF, {0x08, 0x00, 0x00, 0x05, 0x19, 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x03}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xCC, {0x08, 0x00, 0x00, 0x05, 0x1a, 0x00, 0x00, 0xCC, 0x00, 0xFD, 0x7F, 0x00, 0x00}));
        responses.insert(std::make_pair<int, std::array<uint8_t, 13>>(0xFF, {0x08, 0x00, 0x00, 0x05, 0x1b, 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x08, 0x03}));
        // IMU update info in 500Hz, total 8 messages be sent in 2ms.
        while (_isConnected) {
            std::for_each(
                responses.begin(), responses.end(), [&servAddr, &_sockfd](const std::pair<int, std::array<uint8_t, 13>> &message) {
                    sendto(_sockfd, message.second.data(), 13, 0, (struct sockaddr *) &servAddr, sizeof(struct sockaddr_in));
                    std::this_thread::sleep_for(250us);
                    SPDLOG_TRACE("<------ {:#04x}", fmt::join(message.second.begin(), message.second.end(), " "));
                });
        }
    }// mode == "motor"

    //we need to close the socket descriptors after we're all done
    close(epfd);
    close(_sockfd);
    cout << "Connection closed..." << endl;
    return 0;
}