// udp_sender.cpp
#include "telemetry/udp_sender.h"
#include <stdexcept>
#include <utility>

using json = nlohmann::json;

#ifdef _WIN32
bool UdpSender::wsa_started_ = false;
#endif

static inline json vec3ToJson(const Vec<3>& v) {
    return json::array({ v(0), v(1), v(2) });
}

void UdpSender::platformStartup_() {
#ifdef _WIN32
    if (!wsa_started_) {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
        wsa_started_ = true;
    }
#endif
}

void UdpSender::platformCleanup_() {
#ifdef _WIN32
    // no WSACleanup() here (safe/simple)
#endif
}

UdpSender::UdpSender(const std::string& receiver_ip, uint16_t receiver_port) {
    platformStartup_();

    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) throw std::runtime_error("socket() failed");

    dst_ = {};
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(receiver_port);

    if (inet_pton(AF_INET, receiver_ip.c_str(), &dst_.sin_addr) != 1) {
        closeSocket_();
        throw std::runtime_error("inet_pton failed for receiver IP: " + receiver_ip);
    }
}

UdpSender::~UdpSender() {
    closeSocket_();
    platformCleanup_();
}

UdpSender::UdpSender(UdpSender&& other) noexcept { *this = std::move(other); }

UdpSender& UdpSender::operator=(UdpSender&& other) noexcept {
    if (this == &other) return *this;
    closeSocket_();
    sock_ = other.sock_; other.sock_ = INVALID_SOCKET;
    dst_ = other.dst_;
    seq_ = other.seq_; other.seq_ = 0;
    return *this;
}

void UdpSender::closeSocket_() noexcept {
    if (sock_ == INVALID_SOCKET) return;
#ifdef _WIN32
    ::closesocket(sock_);
#else
    ::close(sock_);
#endif
    sock_ = INVALID_SOCKET;
}

void UdpSender::resetSeq(uint32_t seq) { seq_ = seq; }

bool UdpSender::sendJson_(const json& j) {
    const std::string payload = j.dump();
    if (payload.size() > 1200) return false;

    const int sent = ::sendto(
        sock_, payload.data(), static_cast<int>(payload.size()), 0,
        reinterpret_cast<sockaddr*>(&dst_), static_cast<socklen_t>(sizeof(dst_))
    );
    return (sent != SOCKET_ERROR);
}

// =======================EDIT HERE IF YOU WANT TO ADD===============================

bool UdpSender::sendFromSim(
    double t, double dt, double Hz,
    const Vec<15>& navState,
    const ModeManager& MM,
    const OuterLoop& outer,
    const ImuSim& imu,
    const bool& armed,
    const double NIS)
{
    // ---- Extract what MATLAB expects ----
    const Vec<3> euler_est = navState.segment<3>(0);
    const Vec<3> pos_est = navState.segment<3>(3);
    const Vec<3> vel_est = navState.segment<3>(6);

    // ModeManager outputs
    const Vec<3> pos_cmd = MM.out.posCmd;

    // OuterLoop outputs (commanded attitude)
    const Vec<3> euler_cmd = outer.out.attCmd;

    // IMU outputs
    const Vec<3> omega_est = imu.imu.gyro;
    const Vec<3> accel = imu.imu.accel;

    json j;
    j["seq"] = seq_++;
    j["t"] = t;
    j["dt"] = dt;
    j["Hz"] = Hz;

    // If you do NOT have phase in ModeManager, set 0 or add it later.
    // If you DO have it, replace with static_cast<int>(MM.out.phase)
    j["phase"] = static_cast<int>(MM.out.phase);
    j["mode"] = static_cast<int>(MM.out.mode);
    j["armed"] = armed;
    j["EKF_Health"] = NIS;

    j["pos_cmd"] = vec3ToJson(pos_cmd);
    j["pos_est"] = vec3ToJson(pos_est);
    j["vel_est"] = vec3ToJson(vel_est);
    j["euler_est"] = vec3ToJson(euler_est);
    
    j["omega_est"] = vec3ToJson(omega_est);
    j["accel"] = vec3ToJson(accel);

    // MATLAB prefers euler_cmd if present
    j["euler_cmd"] = vec3ToJson(euler_cmd);

    return sendJson_(j);
}
