/*
 * manus.cpp -- Manus glove -> Hand Runner teleoperation (h9 hand only)
 *
 * Reads the Manus glove joint angles (Manus SDK, Integrated mode), retargets
 * them to the 9 joints of the h9 hand and sends them to the Hand Runner
 * (runner.c) as "jointpos" commands over UDP. One runner drives one hand, so
 * the left and the right glove go to different UDP ports.
 *
 *   glove --(Manus SDK)--> manus --(UDP "jointpos d1 ... d9")--> runner --(CAN)--> hand
 *
 * Build:
 *   see build.sh (the Manus SDK is in ./manus-sdk)
 *   g++ -O2 -std=c++17 -W -Wall -DMANUS_SIM -o manus-sim manus.cpp -pthread
 *        (no glove, no SDK: a synthetic open/close motion; for testing with
 *         runner-sim)
 *
 * Run (after the runners are started):
 *   ./manus [-l <port>] [-r <port>] [-a <ip>] [-d <calib dir>] [-n] [-v]
 *
 *   -l  UDP command port of the LEFT  hand runner, 0 = do not send (default 10000)
 *   -r  UDP command port of the RIGHT hand runner, 0 = do not send (default 10001)
 *   -a  IP address of the runners                                (default 127.0.0.1)
 *   -d  directory of the glove calibration files (.mcal)         (default calib)
 *   -n  do not load the calibration files (use the SDK defaults)
 *   -v  verbose: print every message sent
 *
 *   e.g.  ./runner L2 -p 10000 -c 0 &
 *         ./runner R2 -p 10001 -c 1 &
 *         ./manus
 *
 * The glove pose is sent as it is, SEND_HZ times per second, from the first
 * glove data on: there is no start-up motion, so the hand moves quickly to the
 * glove pose when the first command arrives. If the glove data stops (glove
 * off, out of range) nothing is sent and the runner holds the last pose.
 * On exit (Ctrl-C) nothing more is sent: the hand holds its last pose. Send
 * "zero" / "quit" to the runner (runner-test) to move it back / stop it.
 */
#ifndef MANUS_SIM
#include "ManusSDK.h"
#include "ManusSDKTypes.h"
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr int    H9_N        = 9;        /* h9 joints: thumb 3, index 2, middle 2, ring 2 */
constexpr int    GLOVE_N     = 20;       /* glove joints: 5 fingers x 4                   */
constexpr int    SEND_HZ     = 30;       /* jointpos commands per second                  */
constexpr double STALE_TIME  = 1.0;      /* [s] no glove data for this long -> stop       */

constexpr const char* kLeftCalib  = "donghyukLeftMetaglove.mcal";
constexpr const char* kRightCalib = "donghyukRightMetaglove.mcal";

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_exit{false};
void HandleSignal(int) { g_exit.store(true); }

/* ------------------------------------------------------------------------ */
/* glove data (written by the SDK callback thread, read by the main loop)    */
/* ------------------------------------------------------------------------ */
struct Glove {
    float             raw[GLOVE_N] = {0};   /* [rad], thumb..pinky x (spread, MCP, PIP, DIP) */
    bool              seen = false;
    Clock::time_point stamp;                /* time of the last update */
};

std::mutex g_mutex;
Glove      g_glove[2];                      /* 0 = left, 1 = right */

/* ------------------------------------------------------------------------ */
/* retargeting: 20 glove angles [rad] -> 9 h9 joint angles [deg]             */
/*                                                                           */
/* h9 joint order (same as the runner): thumb1 thumb2 thumb3 index1 index2   */
/* middle1 middle2 ring1 ring2, positive = flexion. The same for both sides  */
/* (the runner takes care of left / right).                                  */
/* ------------------------------------------------------------------------ */
void RetargetH9(const float* g, double* deg)
{
    static const struct { int src; double gain, offset; } map[H9_N] = {
        { 0, 1.2, 0.5 },    /* thumb1  <- thumb spread */
        { 2, 1.0, 0.2 },    /* thumb2  <- thumb PIP    */
        { 3, 1.0, 0.2 },    /* thumb3  <- thumb DIP    */
        { 5, 1.2, 0.0 },    /* index1  <- index MCP    */
        { 6, 1.5, 0.0 },    /* index2  <- index PIP    */
        { 9, 1.2, 0.0 },    /* middle1 <- middle MCP   */
        {10, 1.5, 0.0 },    /* middle2 <- middle PIP   */
        {13, 1.2, 0.0 },    /* ring1   <- ring MCP     */
        {14, 1.5, 0.0 },    /* ring2   <- ring PIP     */
    };
    for (int i = 0; i < H9_N; i++) {
        double q = map[i].gain*(double)g[map[i].src] + map[i].offset;      /* [rad] */
        q = q < 0.0 ? 0.0 : (q > 1.57 ? 1.57 : q);                         /* 0 .. 90 deg */
        deg[i] = q*180.0/M_PI;
    }
}

/* ------------------------------------------------------------------------ */
/* one side: UDP sender                                                      */
/* ------------------------------------------------------------------------ */
struct Hand {
    char        name;                       /* 'L' or 'R' */
    int         port = 0;                   /* 0 = this side is not used */
    sockaddr_in addr{};
    bool        active = false;             /* glove data is coming in */
};

int  g_sock = -1;
bool g_verbose = false;

void SendTo(const Hand& s, const char* msg, int len)
{
    if (sendto(g_sock, msg, (size_t)len, 0, reinterpret_cast<const sockaddr*>(&s.addr), sizeof(s.addr)) < 0)
        std::perror("sendto");
    if (g_verbose) std::printf("%c: %s\n", s.name, msg);
}

void StepSide(Hand& s, const Glove& g, Clock::time_point now)
{
    if (s.port == 0) return;

    const bool fresh = g.seen && std::chrono::duration<double>(now - g.stamp).count() < STALE_TIME;
    if (fresh != s.active) {
        if (fresh) std::printf("[%c] glove data received, following the glove\n", s.name);
        else       std::printf("[%c] no glove data, the hand holds its pose\n", s.name);
        s.active = fresh;
    }
    if (!fresh) return;

    double deg[H9_N];
    RetargetH9(g.raw, deg);

    char msg[256];
    int  len = std::snprintf(msg, sizeof(msg), "jointpos");
    for (int i = 0; i < H9_N; i++) len += std::snprintf(msg + len, sizeof(msg) - (size_t)len, " %.2f", deg[i]);
    SendTo(s, msg, len);
}

/* ------------------------------------------------------------------------ */
/* Manus SDK                                                                 */
/* ------------------------------------------------------------------------ */
#ifndef MANUS_SIM
std::atomic<bool>     g_connected{false};
std::atomic<uint32_t> g_gloveId[2] = {{0}, {0}};       /* 0 = left, 1 = right */

void OnConnected(const ManusHost* const)    { std::printf("[manus] connected\n");    g_connected.store(true);  }
void OnDisconnected(const ManusHost* const) { std::printf("[manus] disconnected\n"); g_connected.store(false); }

void OnLandscape(const Landscape* const land)
{
    if (land == nullptr) return;
    uint32_t id[2] = {0, 0};
    for (uint32_t i = 0; i < land->gloveDevices.gloveCount; i++) {
        const GloveLandscapeData& g = land->gloveDevices.gloves[i];
        if (id[0] == 0 && g.side == Side_Left)  id[0] = g.id;
        if (id[1] == 0 && g.side == Side_Right) id[1] = g.id;
    }
    g_gloveId[0].store(id[0]);
    g_gloveId[1].store(id[1]);
}

/* ErgonomicsData.data: 20 left hand angles then 20 right hand angles [deg] */
void OnErgonomics(const ErgonomicsStream* const ergo)
{
    if (ergo == nullptr) return;
    const auto now = Clock::now();
    for (uint32_t i = 0; i < ergo->dataCount; i++) {
        const ErgonomicsData& d = ergo->data[i];
        if (d.isUserID) continue;
        for (int side = 0; side < 2; side++) {
            const uint32_t id = g_gloveId[side].load();
            if (id == 0 || d.id != id) continue;
            std::lock_guard<std::mutex> lk(g_mutex);
            for (int k = 0; k < GLOVE_N; k++) g_glove[side].raw[k] = d.data[20*side + k]*(float)(M_PI/180.0);
            g_glove[side].seen  = true;
            g_glove[side].stamp = now;
        }
    }
}

bool LoadCalibration(uint32_t gloveId, const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) { std::fprintf(stderr, "[calib] not found: %s\n", path.c_str()); return false; }
    const std::streamsize size = file.tellg();
    if (size <= 0) { std::fprintf(stderr, "[calib] empty: %s\n", path.c_str()); return false; }
    file.seekg(0, std::ios::beg);
    std::vector<unsigned char> bytes((size_t)size);
    if (!file.read(reinterpret_cast<char*>(bytes.data()), size)) { std::fprintf(stderr, "[calib] read failed: %s\n", path.c_str()); return false; }

    SetGloveCalibrationReturnCode result;
    const SDKReturnCode rc = CoreSdk_SetGloveCalibration(gloveId, bytes.data(), (uint32_t)size, &result);
    if (rc != SDKReturnCode_Success) {
        std::fprintf(stderr, "[calib] SetGloveCalibration failed: %s rc=%d result=%d\n", path.c_str(), (int)rc, (int)result);
        return false;
    }
    std::printf("[calib] loaded '%s' into glove 0x%x\n", path.c_str(), gloveId);
    return true;
}

bool InitSdk()
{
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode_Success) { std::fprintf(stderr, "CoreSdk_InitializeIntegrated failed\n"); return false; }
    if (CoreSdk_RegisterCallbackForOnConnect(OnConnected)           != SDKReturnCode_Success ||
        CoreSdk_RegisterCallbackForOnDisconnect(OnDisconnected)     != SDKReturnCode_Success ||
        CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape)     != SDKReturnCode_Success ||
        CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomics)   != SDKReturnCode_Success) {
        std::fprintf(stderr, "CoreSdk_RegisterCallback* failed\n");
        return false;
    }

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side_Right;
    vuh.up         = AxisPolarity_PositiveZ;
    vuh.view       = AxisView_XToViewer;
    vuh.unitScale  = 1.0f;
    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode_Success) {
        std::fprintf(stderr, "CoreSdk_InitializeCoordinateSystemWithVUH failed\n");
        return false;
    }

    ManusHost host;
    ManusHost_Init(&host);
    const SDKReturnCode rc = CoreSdk_ConnectToHost(host);
    if (rc != SDKReturnCode_Success) { std::fprintf(stderr, "CoreSdk_ConnectToHost failed: %d\n", (int)rc); return false; }
    return true;
}
#endif /* !MANUS_SIM */

void Usage(const char* prog)
{
    std::fprintf(stderr,
        "usage: %s [-l <port>] [-r <port>] [-a <ip>] [-d <calib dir>] [-n] [-v]\n"
        "  -l  UDP command port of the LEFT  hand runner, 0 = do not send (default 10000)\n"
        "  -r  UDP command port of the RIGHT hand runner, 0 = do not send (default 10001)\n"
        "  -a  IP address of the runners (default 127.0.0.1)\n"
        "  -d  directory of the glove calibration files (default calib)\n"
        "  -n  do not load the calibration files\n"
        "  -v  verbose: print every message sent\n", prog);
}

bool ParsePort(const char* s, int* out)
{
    char* end; errno = 0;
    const long v = std::strtol(s, &end, 10);
    if (errno || end == s || *end || v < 0 || v > 65535) return false;
    *out = (int)v;
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Hand side[2];
    side[0].name = 'L'; side[0].port = 10000;
    side[1].name = 'R'; side[1].port = 10001;
    const char* ip = "127.0.0.1";
    std::string calibDir = "calib";
    bool skipCalib = false;

    int opt;
    while ((opt = getopt(argc, argv, "l:r:a:d:nvh")) != -1) {
        switch (opt) {
        case 'l': if (!ParsePort(optarg, &side[0].port)) { Usage(argv[0]); return 64; } break;
        case 'r': if (!ParsePort(optarg, &side[1].port)) { Usage(argv[0]); return 64; } break;
        case 'a': ip = optarg; break;
        case 'd': calibDir = optarg; break;
        case 'n': skipCalib = true; break;
        case 'v': g_verbose = true; break;
        case 'h': Usage(argv[0]); return 0;
        default:  Usage(argv[0]); return 64;
        }
    }
    if (optind != argc) { Usage(argv[0]); return 64; }
    if (side[0].port == 0 && side[1].port == 0) { std::fprintf(stderr, "manus: both sides are disabled\n"); return 64; }
    if (side[0].port == side[1].port) { std::fprintf(stderr, "manus: the left and the right runner need different ports\n"); return 64; }

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) { std::perror("socket"); return 1; }
    for (Hand& s : side) {
        s.addr.sin_family = AF_INET;
        s.addr.sin_port   = htons((uint16_t)s.port);
        if (inet_pton(AF_INET, ip, &s.addr.sin_addr) != 1) { std::fprintf(stderr, "manus: bad IP address '%s'\n", ip); return 64; }
        if (s.port) std::printf("manus: %c glove -> %s:%d\n", s.name, ip, s.port);
        else        std::printf("manus: %c glove not used\n", s.name);
    }

    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

#ifdef MANUS_SIM
    (void)skipCalib;
    std::printf("manus: SIMULATION build, no glove is used\n");
#else
    if (!InitSdk()) return 2;
    bool calibDone[2] = {skipCalib, skipCalib};
    const char* calibFile[2] = {kLeftCalib, kRightCalib};
#endif
    std::printf("manus: sending at %d Hz, Ctrl-C to stop\n", SEND_HZ);
    std::fflush(stdout);

    const auto period = std::chrono::nanoseconds(1000000000LL/SEND_HZ);
    const auto start  = Clock::now();
    auto next = start;
    while (!g_exit.load()) {
        const auto now = Clock::now();

#ifdef MANUS_SIM
        {   /* synthetic glove: all fingers open and close every 4 s */
            const double t = std::chrono::duration<double>(now - start).count();
            const float  v = (float)(0.5*(1.0 - std::cos(2.0*M_PI*t/4.0)));
            std::lock_guard<std::mutex> lk(g_mutex);
            for (Glove& g : g_glove) {
                for (int k = 0; k < GLOVE_N; k++) g.raw[k] = (k % 4 == 0) ? 0.0f : v;
                g.seen = true; g.stamp = now;
            }
        }
#else
        /* load the calibration of each glove once, when the glove shows up */
        if (g_connected.load()) {
            for (int i = 0; i < 2; i++) {
                const uint32_t id = g_gloveId[i].load();
                if (calibDone[i] || id == 0) continue;
                LoadCalibration(id, calibDir + "/" + calibFile[i]);
                calibDone[i] = true;
            }
        }
#endif

        Glove glove[2];
        { std::lock_guard<std::mutex> lk(g_mutex); glove[0] = g_glove[0]; glove[1] = g_glove[1]; }
        for (int i = 0; i < 2; i++) StepSide(side[i], glove[i], now);
        std::fflush(stdout);

        next += period;
        std::this_thread::sleep_until(next);
    }

#ifndef MANUS_SIM
    CoreSdk_ShutDown();
#endif
    close(g_sock);
    std::printf("manus: stopped, the hand holds its last pose\n");
    return 0;
}
