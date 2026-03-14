#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include "gripper_sdk/params.hpp"

namespace gripper {

class UdpLogger {
public:
    explicit UdpLogger(const char* host = "127.0.0.1", int port = 9870)
    {
        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) { perror("[UdpLogger] socket"); return; }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        inet_pton(AF_INET, host, &addr.sin_addr);
        ::connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        std::printf("[UdpLogger] streaming to %s:%d\n", host, port);
    }
    ~UdpLogger() { if (sock_ >= 0) ::close(sock_); }
    UdpLogger(const UdpLogger&) = delete;
    UdpLogger& operator=(const UdpLogger&) = delete;

    void send(double t_s,
              const int32_t pos[NUM_MOTORS],
              const int32_t goal[NUM_MOTORS],
              const int16_t cur[NUM_MOTORS],
              const int32_t vel[NUM_MOTORS])
    {
        if (sock_ < 0) return;
        char buf[2048];
        int n = std::snprintf(buf, sizeof(buf),
            "{\"t\":%.4f"
            ",\"pos\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
            ",\"goal\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
            ",\"cur\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
            ",\"vel\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
            "}\n",
            t_s,
            pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],
            pos[6],pos[7],pos[8],pos[9],pos[10],
            goal[0],goal[1],goal[2],goal[3],goal[4],
            goal[5],goal[6],goal[7],goal[8],goal[9],goal[10],
            cur[0],cur[1],cur[2],cur[3],cur[4],
            cur[5],cur[6],cur[7],cur[8],cur[9],cur[10],
            vel[0],vel[1],vel[2],vel[3],vel[4],
            vel[5],vel[6],vel[7],vel[8],vel[9],vel[10]);
        ::send(sock_, buf, n, 0);
    }

private:
    int sock_ = -1;
};

}  // namespace gripper
