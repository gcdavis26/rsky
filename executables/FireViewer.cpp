// FireViewer.cpp
// Raylib 3D viewer matching the environment shown by udp_receiver.m
// Handles "state" telemetry packets and "vision" fire/blob grid packets on UDP :8080.
// Legacy packets without a "type" field (basic sim output) are also accepted.

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER

#include <WinSock2.h>
#pragma comment(lib, "ws2_32.lib")

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif

#include <raylib.h>
#include <rlgl.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <deque>
#include <cstring>

using json = nlohmann::json;

// -----------------------------------------------------------------------
// Constants matching the MATLAB viewer environment
// -----------------------------------------------------------------------
static constexpr float kVolE    = 4.572f;   // East extent  [m]
static constexpr float kVolN    = 9.144f;   // North extent [m]
static constexpr float kVolH    = 5.0f;     // Height       [m]

static constexpr int   kGridRows = 100;     // North cells
static constexpr int   kGridCols = 50;      // East cells
static constexpr float kGridRes  = 0.1f;    // Cell size    [m]

// Mission phase / nav mode name tables (0-based enum, matches MATLAB)
static const char* kPhaseNames[] = {
    "Takeoff", "Sweep", "GoToTarget", "HoverAtTarget",
    "DescendToTarget", "Drop", "GoToLand", "DescendToLand", "Terminate"
};
static constexpr int kNumPhases = 9;

static const char* kModeNames[] = { "Manual", "Waypoint", "Sweep" };
static constexpr int kNumModes  = 3;

// -----------------------------------------------------------------------
// Shared data structures
// -----------------------------------------------------------------------
struct DroneState {
    float x  = 0, y  = 0, z  = 0;          // NED position  [m]
    float vx = 0, vy = 0, vz = 0;          // NED velocity  [m/s]
    float roll  = 0, pitch  = 0, yaw  = 0; // Euler ZYX est [rad]
    float rollC = 0, pitchC = 0, yawC = 0; // Euler ZYX cmd [rad]
    float t             = 0;
    float hz            = 0;
    float ekfHealth     = 0;
    float battVoltageMv = 0;
    float battCurrentMa = 0;
    int   phase = 0;
    int   mode  = 0;
    int   armed = 0;
    bool  haveData = false;
};

// Grid cell values: 0 = empty, 1 = fire, 2 = blob
struct SharedData {
    DroneState drone;
    uint8_t    grid[kGridRows][kGridCols];
    SharedData() { std::memset(grid, 0, sizeof(grid)); }
};

static std::atomic<bool> g_running{ true };
static SharedData         g_shared;
static std::mutex         g_mutex;

// -----------------------------------------------------------------------
// UDP listener thread
// -----------------------------------------------------------------------
static void listenerLoop(SOCKET sock) {
    char buf[65536];
    sockaddr_in from{};
    int fromLen = sizeof(from);

    while (g_running.load()) {
        int n = ::recvfrom(sock, buf, (int)sizeof(buf) - 1, 0,
                           reinterpret_cast<sockaddr*>(&from), &fromLen);
        if (n <= 0) { if (!g_running.load()) break; continue; }
        buf[n] = '\0';

        try {
            auto j = json::parse(buf);
            std::lock_guard<std::mutex> lk(g_mutex);

            // Determine packet type; accept legacy packets without "type"
            std::string type = j.value("type", "state");

            if (type == "state") {
                DroneState& s = g_shared.drone;
                s.x  = j["pos_est"][0].get<float>();
                s.y  = j["pos_est"][1].get<float>();
                s.z  = j["pos_est"][2].get<float>();
                s.vx = j["vel_est"][0].get<float>();
                s.vy = j["vel_est"][1].get<float>();
                s.vz = j["vel_est"][2].get<float>();
                s.roll  = j["euler_est"][0].get<float>();
                s.pitch = j["euler_est"][1].get<float>();
                s.yaw   = j["euler_est"][2].get<float>();
                if (j.contains("euler_cmd")) {
                    s.rollC  = j["euler_cmd"][0].get<float>();
                    s.pitchC = j["euler_cmd"][1].get<float>();
                    s.yawC   = j["euler_cmd"][2].get<float>();
                }
                s.t             = j.value("t",                  0.0f);
                s.hz            = j.value("Hz",                 0.0f);
                s.ekfHealth     = j.value("EKF_Health",         0.0f);
                s.battVoltageMv = j.value("battery_voltage_mv", 0.0f);
                s.battCurrentMa = j.value("battery_current_ma", 0.0f);
                s.phase         = j.value("phase", 0);
                s.mode          = j.value("mode",  0);
                s.armed         = j.value("armed", 0);
                s.haveData      = true;

            } else if (type == "vision") {
                if (j.contains("fire_cells")) {
                    for (auto& cell : j["fire_cells"]) {
                        int idx = cell.get<int>();
                        int r = idx / kGridCols, c = idx % kGridCols;
                        if (r >= 0 && r < kGridRows && c >= 0 && c < kGridCols)
                            g_shared.grid[r][c] = 1;
                    }
                }
                if (j.contains("blob_cells")) {
                    for (auto& cell : j["blob_cells"]) {
                        int idx = cell.get<int>();
                        int r = idx / kGridCols, c = idx % kGridCols;
                        if (r >= 0 && r < kGridRows && c >= 0 && c < kGridCols)
                            g_shared.grid[r][c] = 2;
                    }
                }
            }
        } catch (...) {}
    }
}

// -----------------------------------------------------------------------
// Coordinate helpers
// NED (N,E,D) -> Raylib world (x=E, y=-D, z=-N) so Y is up
// -----------------------------------------------------------------------
static inline Vector3 nedToWorld(float n, float e, float d) {
    return { e, -d, -n };
}

// -----------------------------------------------------------------------
// Draw the volume boundary box in world space
// -----------------------------------------------------------------------
static void drawVolumeBox() {
    const float ex = kVolE, hy = kVolH, nz = -kVolN;
    const Color c = Color{ 70, 75, 95, 255 };

    Vector3 v[8] = {
        {0,  0,  0 }, {ex, 0,  0 }, {ex, 0,  nz}, {0,  0,  nz},
        {0,  hy, 0 }, {ex, hy, 0 }, {ex, hy, nz}, {0,  hy, nz},
    };
    DrawLine3D(v[0],v[1],c); DrawLine3D(v[1],v[2],c);
    DrawLine3D(v[2],v[3],c); DrawLine3D(v[3],v[0],c);
    DrawLine3D(v[4],v[5],c); DrawLine3D(v[5],v[6],c);
    DrawLine3D(v[6],v[7],c); DrawLine3D(v[7],v[4],c);
    DrawLine3D(v[0],v[4],c); DrawLine3D(v[1],v[5],c);
    DrawLine3D(v[2],v[6],c); DrawLine3D(v[3],v[7],c);
}

// -----------------------------------------------------------------------
// Draw subtle ground grid aligned to the flight volume
// -----------------------------------------------------------------------
static void drawGroundGrid() {
    const Color minor = Color{ 35, 38, 48, 255 };
    const Color major = Color{ 55, 60, 75, 255 };

    const float maxE = 4.572f;   // East limit (meters)
    const float maxN = 9.144f;   // North limit (meters)
    const float gridStep = 0.1524f; // example: 6 inches in meters

    int lineIndex = 0;

    // Vertical lines (constant East, varying North)
    for (float xe = 0.0f; xe <= maxE + 1e-6f; xe += gridStep, ++lineIndex) {
        Color col = (lineIndex % 10 == 0) ? major : minor;
        DrawLine3D({xe, 0, 0}, {xe, 0, -maxN}, col);
    }

    lineIndex = 0;

    // Horizontal lines (constant North, varying East)
    for (float zn = 0.0f; zn <= maxN + 1e-6f; zn += gridStep, ++lineIndex) {
        Color col = (lineIndex % 10 == 0) ? major : minor;
        DrawLine3D({0, 0, -zn}, {maxE, 0, -zn}, col);
    }
}

// -----------------------------------------------------------------------
// Draw fire / blob grid cells as thin coloured slabs on the ground plane
// -----------------------------------------------------------------------
static void drawFireGrid(const uint8_t grid[kGridRows][kGridCols]) {
    const Color fireCellCol = Color{ 230, 155,  15, 210 };
    const Color blobCellCol = Color{ 215,  35,  35, 230 };
    const float sz   = kGridRes;
    const float slab = 0.004f;

    for (int r = 0; r < kGridRows; ++r) {
        for (int c = 0; c < kGridCols; ++c) {
            uint8_t val = grid[r][c];
            if (val == 0) continue;
            float xe = (c + 0.0f) * sz;
            float zn = -(r + 0.0f) * sz;
            Color col = (val == 1) ? fireCellCol : blobCellCol;
            DrawCube({xe, -slab * 0.0f, zn}, sz, slab, sz, col);
        }
    }
}

// -----------------------------------------------------------------------
// Draw world-frame origin axes
// -----------------------------------------------------------------------
static void drawOriginAxes() {
    const float L = 0.5f;
    DrawLine3D({0,0,0}, {L, 0,  0}, RED  );  // East
    DrawLine3D({0,0,0}, {0, L,  0}, GREEN);  // Up
    DrawLine3D({0,0,0}, {0, 0, -L}, BLUE );  // North
}

// -----------------------------------------------------------------------
// Draw drone model with X-frame arms and body-axis triad
// armSpan = 0.5 m tip-to-tip, matching MATLAB viewer
// Body axes: red=forward(x_b), green=right(y_b), blue=down(z_b)
// In raylib body frame: forward=-Z, right=+X, down=-Y
// -----------------------------------------------------------------------
static void drawDrone(const DroneState& s, Vector3 pos) {
    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    rlRotatef(-s.yaw   * RAD2DEG, 0, 1,  0);
    rlRotatef( s.pitch * RAD2DEG, 1, 0,  0);
    rlRotatef(-s.roll  * RAD2DEG, 0, 0,  1);

    DrawCube({0,0,0}, 0.20f, 0.05f, 0.20f, LIGHTGRAY);
    DrawCubeWires({0,0,0}, 0.20f, 0.05f, 0.20f, RAYWHITE);

    // armSpan=0.5m: each rotor offset L = 0.177m per axis
    const float L       = 0.177f;
    const float rotorR  = 0.060f;
    const float rotorH  = 0.008f;
    const float rotorY  = 0.016f;
    const float baseY   = rotorY - rotorH * 0.5f;
    const int   rotSeg  = 20;

    auto rotor = [&](float x, float z, Color col) {
        DrawCylinder     ({x, baseY, z}, rotorR, rotorR, rotorH, rotSeg, col);
        DrawCylinderWires({x, baseY, z}, rotorR, rotorR, rotorH, rotSeg, BLACK);
    };
    rotor( L, -L, RED   );  // M1 front-right
    rotor(-L, -L, GREEN );  // M4 front-left
    rotor( L,  L, YELLOW);  // M3 rear-right
    rotor(-L,  L, BLUE  );  // M2 rear-left

    const float armW = 0.014f;
    DrawCube({ L*0.5f, 0, -L*0.5f}, L*1.414f, armW, armW, DARKGRAY);
    DrawCube({-L*0.5f, 0, -L*0.5f}, L*1.414f, armW, armW, DARKGRAY);
    DrawCube({ L*0.5f, 0,  L*0.5f}, L*1.414f, armW, armW, DARKGRAY);
    DrawCube({-L*0.5f, 0,  L*0.5f}, L*1.414f, armW, armW, DARKGRAY);

    const float axLen = 0.28f;
    DrawLine3D({0,0,0}, {0,     0,    -axLen}, RED  );  // x_b forward
    DrawLine3D({0,0,0}, {axLen, 0,     0    }, GREEN);  // y_b right
    DrawLine3D({0,0,0}, {0,    -axLen, 0    }, BLUE );  // z_b down

    rlPopMatrix();
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n"; return 1;
    }

    SOCKET sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "socket() failed\n"; WSACleanup(); return 1;
    }

    BOOL reuse = TRUE;
    ::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&reuse), sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(8080);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "bind() failed: " << WSAGetLastError() << "\n";
        ::closesocket(sock); WSACleanup(); return 1;
    }

    std::thread listener(listenerLoop, sock);

    InitWindow(1280, 720, "FireSim 3D Viewer");
    {
        int mon = GetCurrentMonitor();
        SetWindowSize(GetMonitorWidth(mon), GetMonitorHeight(mon));
        ToggleFullscreen();
    }
    SetTargetFPS(60);

    const Vector3 volCenter = { kVolE * 0.5f, kVolH * 0.3f, -kVolN * 0.5f };

    Camera3D camera{};
    camera.up         = { 0.0f, 1.0f, 0.0f };
    camera.fovy       = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float camYaw      = 0.6f;
    float camPitch    = 0.45f;
    float camDist     = 16.0f;
    Vector3 camTarget = volCenter;

    std::deque<Vector3> trail;
    const size_t kMaxTrail    = 600;
    const float  kMinTrailDst = 0.01f;

    SharedData local;

    while (!WindowShouldClose()) {

        if (IsKeyPressed(KEY_F11)) ToggleFullscreen();
        if (IsKeyPressed(KEY_R)) {
            camYaw = 0.6f; camPitch = 0.45f; camDist = 16.0f;
            camTarget = volCenter;
        }

        {
            std::lock_guard<std::mutex> lk(g_mutex);
            local = g_shared;
        }

        const DroneState& s = local.drone;
        Vector3 dronePos = nedToWorld(s.x, s.y, s.z);

        if (s.haveData) {
            bool add = trail.empty();
            if (!add) {
                Vector3& b = trail.back();
                float dx = dronePos.x-b.x, dy = dronePos.y-b.y, dz = dronePos.z-b.z;
                if (sqrtf(dx*dx + dy*dy + dz*dz) > kMinTrailDst) add = true;
            }
            if (add) {
                trail.push_back(dronePos);
                if (trail.size() > kMaxTrail) trail.pop_front();
            }
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 d = GetMouseDelta();
            camYaw   -= d.x * 0.005f;
            camPitch += d.y * 0.005f;
            if (camPitch >  1.50f) camPitch =  1.50f;
            if (camPitch < -0.10f) camPitch = -0.10f;
        }
        {
            float w = GetMouseWheelMove();
            if (w != 0.0f) {
                camDist *= (1.0f - w * 0.10f);
                if (camDist <  1.0f) camDist =  1.0f;
                if (camDist > 60.0f) camDist = 60.0f;
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 d = GetMouseDelta();
            float rx = cosf(camYaw), rz = -sinf(camYaw);
            camTarget.x -= rx * d.x * 0.012f;
            camTarget.z -= rz * d.x * 0.012f;
            camTarget.y += d.y * 0.012f;
        }

        camera.target   = camTarget;
        camera.position = {
            camTarget.x + camDist * cosf(camPitch) * sinf(camYaw),
            camTarget.y + camDist * sinf(camPitch),
            camTarget.z + camDist * cosf(camPitch) * cosf(camYaw)
        };

        BeginDrawing();
        ClearBackground(Color{ 18, 22, 30, 255 });

        BeginMode3D(camera);

        drawGroundGrid();
        drawVolumeBox();
        drawFireGrid(local.grid);
        drawOriginAxes();

        if (s.haveData) {
            DrawLine3D(dronePos, {dronePos.x, 0.0f, dronePos.z}, GRAY);
            DrawSphere({dronePos.x, 0.0f, dronePos.z}, 0.045f,
                       Color{210, 165, 15, 200});
        }

        if (trail.size() >= 2) {
            const float n = (float)trail.size();
            for (size_t i = 1; i < trail.size(); ++i) {
                float a = (float)i / n;
                DrawLine3D(trail[i-1], trail[i], Fade(ORANGE, 0.15f + 0.85f * a));
            }
        }

        drawDrone(s, dronePos);

        EndMode3D();

        // ---- 2D HUD ----
DrawRectangle(0, 0, 900, 380, Color{0, 0, 0, 165});  // larger box
DrawText("FireSim 3D Viewer", 14, 10, 34, RAYWHITE); // larger title

char line[256];
if (s.haveData) {
    int   pi = (s.phase >= 0 && s.phase < kNumPhases) ? s.phase : 0;
    int   mi = (s.mode  >= 0 && s.mode  < kNumModes ) ? s.mode  : 0;
    Color armColor = s.armed ? Color{60,220,60,255} : Color{220,75,75,255};

    int y = 60;        // starting y (pushed down for bigger title)
    int dy = 30;       // vertical spacing
    int fs = 24;       // font size (increased)

    std::snprintf(line, sizeof(line),
        "t: %7.2f s    Rate: %6.1f Hz", s.t, s.hz);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "%s | Phase: %s | Mode: %s",
        s.armed ? "ARMED" : "DISARMED", kPhaseNames[pi], kModeNames[mi]);
    DrawText(line, 14, y, fs, armColor);

    y += dy;
    std::snprintf(line, sizeof(line),
        "EKF Health: %.3f", s.ekfHealth);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "Battery:  V = %7.1f mV    I = %7.1f mA",
        s.battVoltageMv, s.battCurrentMa);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "pos [NED]: % 7.3f  % 7.3f  % 7.3f  m",
        s.x, s.y, s.z);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "vel [NED]: % 7.3f  % 7.3f  % 7.3f  m/s",
        s.vx, s.vy, s.vz);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "eul est:   % 7.4f  % 7.4f  % 7.4f  rad  (R P Y)",
        s.roll, s.pitch, s.yaw);
    DrawText(line, 14, y, fs, RAYWHITE);

    y += dy;
    std::snprintf(line, sizeof(line),
        "eul cmd:   % 7.4f  % 7.4f  % 7.4f  rad",
        s.rollC, s.pitchC, s.yawC);
    DrawText(line, 14, y, fs, Color{170, 170, 255, 255});

} else {
    DrawText("Waiting for telemetry on UDP :8080 ...", 14, 70, 24, YELLOW);
    DrawText("(start the drone firmware or sim)",      14, 105, 22, LIGHTGRAY);
}

DrawText("RMB: orbit   MMB: pan   Scroll: zoom   R: reset cam   F11: fullscreen",
         14, 340, 20, LIGHTGRAY);

DrawFPS(GetScreenWidth() - 110, 10);

EndDrawing();
    }

    g_running.store(false);
    ::closesocket(sock);
    if (listener.joinable()) listener.join();
    CloseWindow();
    WSACleanup();
    return 0;
}
