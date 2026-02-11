// udp_sender.h
#pragma once
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

#include "common/MathUtils.h"
#include "sensors/ImuSim.h"
#include "control/OuterLoop.h"
#include "guidance/ModeManager.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socklen_t = int;
using socket_handle_t = SOCKET;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
using socket_handle_t = int;
#endif

class UdpSender {
public:
    UdpSender(const std::string& receiver_ip, uint16_t receiver_port);
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;
    UdpSender(UdpSender&&) noexcept;
    UdpSender& operator=(UdpSender&&) noexcept;

    void resetSeq(uint32_t seq = 0);

    // The only call you want long-term:
    // Pass the objects that will still exist even after you delete print vectors.
    bool sendFromSim(
        double t,
        double dt,
        double Hz,
        const Vec<15>& navState,
        const ModeManager& MM,
        const OuterLoop& outer,
        const ImuSim& imu,
        const bool& armed);

private:
    bool sendJson_(const nlohmann::json& j);
    void closeSocket_() noexcept;
    static void platformStartup_();
    static void platformCleanup_();

private:
    socket_handle_t sock_ = INVALID_SOCKET;
    sockaddr_in dst_{};
    uint32_t seq_ = 0;

#ifdef _WIN32
    static bool wsa_started_;
#endif
};
