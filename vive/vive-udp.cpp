// Minimal Vive Tracker -> UDP 6D pose streamer.
//
// Connects to a running SteamVR instance via OpenVR (background app — does
// not start SteamVR, will simply fail if it is not already up), polls the
// tracking pose at a fixed rate, and emits one UDP datagram per generic
// tracker per poll:
//
//   "T<idx> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f"
//        ^idx ^px    ^py    ^pz    ^qx    ^qy    ^qz    ^qw
//
// Position is in meters (TrackingUniverse_Standing). Quaternion is unit.
// Idx is the OpenVR device index at poll time (stable for the duration of
// a SteamVR session, but not across restarts).
//
// Defaults: poll 100 Hz, send to 127.0.0.1:6634.
// Override poll period (ms) with -tN.

#include "openvr.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr const char*    kUdpIp        = "127.0.0.1";
constexpr unsigned short kUdpPort      = 6634;
constexpr int            kDefaultPollMs = 10;  // 100 Hz

std::atomic<bool> g_exit{false};

int          g_udpSocket = -1;
sockaddr_in  g_udpAddr{};

bool InitUdp(const char* ip, unsigned short port)
{
    g_udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udpSocket < 0)
    {
        std::perror("socket");
        return false;
    }
    std::memset(&g_udpAddr, 0, sizeof(g_udpAddr));
    g_udpAddr.sin_family = AF_INET;
    g_udpAddr.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip, &g_udpAddr.sin_addr) <= 0)
    {
        std::fprintf(stderr, "inet_pton failed for %s\n", ip);
        close(g_udpSocket);
        g_udpSocket = -1;
        return false;
    }
    std::printf("[udp] -> %s:%u\n", ip, port);
    return true;
}

void SendUdpDatagram(const char* buf, int len)
{
    if (g_udpSocket < 0 || len <= 0) return;
    sendto(g_udpSocket, buf, len, 0,
           reinterpret_cast<sockaddr*>(&g_udpAddr), sizeof(g_udpAddr));
}

// Stdout-only — appends "tr=<eTrackingResult> bad=<count>", coloring tr red
// when the tracker is not in Running_OK so OutOfRange/Fallback episodes pop
// visually. UDP datagrams stay clean (OK frames only); this is the
// diagnostic side-channel.
void PrintTrackerLine(const char* buf, int len, int tr, long bad_count)
{
    if (len > 0) std::fwrite(buf, 1, static_cast<size_t>(len), stdout);
    const bool ok = (tr == vr::TrackingResult_Running_OK);
    if (ok) std::fprintf(stdout, " tr=%d bad=%ld\n", tr, bad_count);
    else    std::fprintf(stdout, " \033[31mtr=%d\033[0m bad=%ld\n", tr, bad_count);
}

// Convert OpenVR row-major 3x4 pose matrix to (position, quaternion-xyzw).
// Source: standard mat3 -> quat algorithm (Mike Day / Shoemake variant).
struct Pose { float px, py, pz, qx, qy, qz, qw; };

Pose MatToPose(const vr::HmdMatrix34_t& m)
{
    Pose p;
    p.px = m.m[0][3];
    p.py = m.m[1][3];
    p.pz = m.m[2][3];

    const float m00 = m.m[0][0], m01 = m.m[0][1], m02 = m.m[0][2];
    const float m10 = m.m[1][0], m11 = m.m[1][1], m12 = m.m[1][2];
    const float m20 = m.m[2][0], m21 = m.m[2][1], m22 = m.m[2][2];

    const float trace = m00 + m11 + m22;
    if (trace > 0.0f)
    {
        const float s = 0.5f / std::sqrt(trace + 1.0f);
        p.qw = 0.25f / s;
        p.qx = (m21 - m12) * s;
        p.qy = (m02 - m20) * s;
        p.qz = (m10 - m01) * s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const float s = 2.0f * std::sqrt(1.0f + m00 - m11 - m22);
        p.qw = (m21 - m12) / s;
        p.qx = 0.25f * s;
        p.qy = (m01 + m10) / s;
        p.qz = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        const float s = 2.0f * std::sqrt(1.0f + m11 - m00 - m22);
        p.qw = (m02 - m20) / s;
        p.qx = (m01 + m10) / s;
        p.qy = 0.25f * s;
        p.qz = (m12 + m21) / s;
    }
    else
    {
        const float s = 2.0f * std::sqrt(1.0f + m22 - m00 - m11);
        p.qw = (m10 - m01) / s;
        p.qx = (m02 + m20) / s;
        p.qy = (m12 + m21) / s;
        p.qz = 0.25f * s;
    }
    return p;
}

std::string GetSerial(vr::IVRSystem* sys, vr::TrackedDeviceIndex_t idx)
{
    char buf[256] = {0};
    sys->GetStringTrackedDeviceProperty(
        idx, vr::Prop_SerialNumber_String, buf, sizeof(buf), nullptr);
    return std::string(buf);
}

void HandleSignal(int /*sig*/) { g_exit.store(true); }

bool ParsePollArg(const char* arg, int& outMs)
{
    if (std::strncmp(arg, "-t", 2) != 0) return false;
    char* end = nullptr;
    const long v = std::strtol(arg + 2, &end, 10);
    if (end == arg + 2 || *end != '\0' || v < 0 || v > 60'000) return false;
    outMs = static_cast<int>(v);
    return true;
}

void PrintUsage(const char* prog)
{
    std::fprintf(stderr,
        "usage: %s [-tN]\n"
        "  -tN    poll period in ms (default %d, i.e. %d Hz)\n",
        prog, kDefaultPollMs, 1000 / kDefaultPollMs);
}

} // namespace

int main(int argc, char* argv[])
{
    int pollMs = kDefaultPollMs;
    for (int i = 1; i < argc; ++i)
    {
        const char* a = argv[i];
        if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0)
        {
            PrintUsage(argv[0]);
            return 0;
        }
        if (!ParsePollArg(a, pollMs))
        {
            std::fprintf(stderr, "unrecognized argument: %s\n", a);
            PrintUsage(argv[0]);
            return 64;
        }
    }
    std::printf("[main] poll every %d ms\n", pollMs);

    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    if (!InitUdp(kUdpIp, kUdpPort)) return 1;

    vr::EVRInitError initErr = vr::VRInitError_None;
    vr::IVRSystem* sys = vr::VR_Init(&initErr, vr::VRApplication_Background);
    if (initErr != vr::VRInitError_None || sys == nullptr)
    {
        std::fprintf(stderr, "VR_Init failed: %s\n",
                     vr::VR_GetVRInitErrorAsEnglishDescription(initErr));
        return 2;
    }
    std::printf("[openvr] initialized as background app\n");

    // First-pass: enumerate trackers so we log their serials.
    bool anyTracker = false;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
    {
        if (sys->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker)
        {
            std::printf("[openvr] tracker idx=%u serial=%s\n",
                        i, GetSerial(sys, i).c_str());
            anyTracker = true;
        }
    }
    if (!anyTracker)
    {
        std::printf("[openvr] no GenericTracker present yet — will keep polling\n");
    }

    std::printf("[main] running, Ctrl-C to stop\n");

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];

    // Per-device count of frames whose eTrackingResult was not Running_OK.
    // Survives only for the lifetime of this run (indices reset on SteamVR
    // restart anyway).
    long bad_count[vr::k_unMaxTrackedDeviceCount] = {0};

    using clock_t = std::chrono::steady_clock;
    auto next = clock_t::now() + std::chrono::milliseconds(pollMs);

    while (!g_exit.load())
    {
        sys->GetDeviceToAbsoluteTrackingPose(
            vr::TrackingUniverseStanding,
            0.0f,                       // no prediction
            poses,
            vr::k_unMaxTrackedDeviceCount);

        for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
        {
            if (sys->GetTrackedDeviceClass(i) != vr::TrackedDeviceClass_GenericTracker)
                continue;
            const vr::TrackedDevicePose_t& tp = poses[i];
            // Drop only frames with no usable matrix at all. OOR/Fallback frames
            // still have an (IMU-derived) matrix — keep printing them so the
            // user can monitor frequency, but skip UDP send.
            if (!tp.bDeviceIsConnected || !tp.bPoseIsValid) continue;

            const int  tr = tp.eTrackingResult;
            const bool ok = (tr == vr::TrackingResult_Running_OK);
            if (!ok) ++bad_count[i];

            const Pose p = MatToPose(tp.mDeviceToAbsoluteTracking);
            char buf[160];
            const int len = std::snprintf(buf, sizeof(buf),
                "T%u %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",
                i, p.px, p.py, p.pz, p.qx, p.qy, p.qz, p.qw);

            if (ok) SendUdpDatagram(buf, len);
            PrintTrackerLine(buf, len, tr, bad_count[i]);
        }

        std::this_thread::sleep_until(next);
        next += std::chrono::milliseconds(pollMs);
        const auto now = clock_t::now();
        if (next < now) next = now + std::chrono::milliseconds(pollMs);
    }

    vr::VR_Shutdown();
    if (g_udpSocket >= 0) close(g_udpSocket);
    std::printf("[main] bye\n");
    return 0;
}
