#include "udp_sender.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

// Constructor
UDPSender::UDPSender(const std::string& ip, int port)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0)
        throw std::runtime_error("UDPSender: Failed to create socket");

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr) <= 0)
        throw std::runtime_error("UDPSender: Invalid IP address");
}

// Destructor
UDPSender::~UDPSender()
{
    if (sock_ >= 0)
        close(sock_);
}

// Template must stay in header OR be explicitly instantiated
void UDPSender::send(const TelemetryPacket& packet) const
{
    sendto(sock_,
        &packet,
        sizeof(TelemetryPacket),
        0,
        reinterpret_cast<const sockaddr*>(&addr_),
        sizeof(addr_));
}