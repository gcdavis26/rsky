#pragma once
#include <string>
#include <netinet/in.h>
#include "telemetry_packet.h"

class UDPSender
{
public:
    UDPSender(const std::string& ip, int port);
    ~UDPSender();

    void send(const TelemetryPacket& packet) const;

private:
    int sock_;
    sockaddr_in addr_;
};