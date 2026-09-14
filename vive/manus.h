// manus.h - Manus glove reader (SDK init, calibration upload, joint snapshot).
//
// Split out of vmaster.cpp for the same reason retarget.h was: more than one
// binary now needs the glove. vmaster couples it to Vive/OpenVR; vhand takes
// the glove alone. Both must read the SAME joint angles, or retarget gains
// tuned in one would not hold in the other.
//
// Usage:
//     std::string err;
//     if (!manus::CheckProfile(n, &err)) { ...; return 1; }  // before the thread
//     manus::SetProfile(n);                   // 0 = upload nothing
//     std::thread t(manus::ThreadFn, std::ref(g_exit));
//     ...
//     float L[20], R[20]; bool haveL, haveR;
//     manus::Snapshot(L, R, haveL, haveR);    // 20 joints/side, radians
//
// ThreadFn owns SDK init and runs until the caller's stop flag goes true; join
// it after setting the flag. Snapshot() is the only reader — the SDK callbacks
// write under a mutex.
//
// The 20 floats per side are the SDK's ergonomics angles converted deg -> rad,
// laid out finger-major (thumb, index, middle, ring, little) x 4 joints. That
// is the `_q[20]` retarget.h consumes.

#pragma once

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <atomic>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <climits>
#include <unistd.h>

namespace manus {

constexpr int kSleepMs = 10;

// Per-user glove calibration profiles, uploaded into the glove at connect.
// These hold hand geometry and sensor offsets for ONE person's hand — the raw
// joint angles (and so the retarget gains tuned against them) only mean what
// they say while this calibration is loaded.
//
// `-pN` picks calib/pN/{left,right}.mcal. Keep this table in sync with
// calib/README.md, which is the copy humans are expected to read:
//
//   p0   nothing uploaded. This is the default when -p is absent, and it is
//        what the old `-n` did. NOT a clean slate — see the warning below.
//   p1   Lee Donghyuk (RCCL)   — the former donghyuk*Metaglove.mcal pair
//   p2   Choi Taewon        — made 2026-09-08 with SDKClient_Linux
//   p3+  free slots; make the folder and it is picked up by -p3
//
// WARNING: CoreSdk_SetGloveCalibration() writes into the glove hardware, so p0
// leaves whatever the previous run uploaded still loaded. "No profile" means
// "keep what is in there", not "reset". Pass -pN explicitly when it matters.
constexpr const char* kLeftCalib  = "left.mcal";
constexpr const char* kRightCalib = "right.mcal";


// Where calib/ lives. Resolved against the BINARY, not the cwd: kida-gui spawns
// vmaster with cwd=<repo root>, and a plain "./calib" then points at
// <repo>/calib, which does not exist. Overridable for odd layouts.
inline std::string CalibDir()
{
    if (const char* e = std::getenv("KIDA_CALIB_DIR"))
        if (*e) return e;
    char buf[PATH_MAX];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "./calib";                 // 못 읽으면 예전 동작으로
    buf[n] = '\0';
    const std::string exe(buf);
    const size_t slash = exe.rfind('/');
    return (slash == std::string::npos ? std::string(".")
                                       : exe.substr(0, slash)) + "/calib";
}

inline std::atomic<bool>     g_connected{false};
inline std::atomic<uint32_t> g_leftId{0};
inline std::atomic<uint32_t> g_rightId{0};
inline std::atomic<bool>     g_calibLoaded{false};
inline std::atomic<int>      g_profile{0};      // 0 = upload nothing

inline std::mutex g_mutex;
inline bool       g_have_left  = false;
inline bool       g_have_right = false;
inline float      g_left_joints[20]  = {0};
inline float      g_right_joints[20] = {0};

inline void OnConnected(const ManusHost* const)
{
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    g_connected.store(true);
}

inline void OnDisconnected(const ManusHost* const)
{
    g_connected.store(false);
}

inline void OnLandscape(const Landscape* const land)
{
    if (!land) return;
    uint32_t leftId = 0, rightId = 0;
    for (uint32_t i = 0; i < land->gloveDevices.gloveCount; ++i) {
        const GloveLandscapeData& g = land->gloveDevices.gloves[i];
        if (leftId  == 0 && g.side == Side::Side_Left)  leftId  = g.id;
        if (rightId == 0 && g.side == Side::Side_Right) rightId = g.id;
    }
    g_leftId.store(leftId);
    g_rightId.store(rightId);
}

inline void UpdateGlobals(const ErgonomicsData& d, bool isLeft)
{
    const int   t_DataOffset = isLeft ? 0 : 20;
    const float kDeg2Rad     = static_cast<float>(M_PI / 180.0);

    float joints[20];
    for (int finger = 0; finger < 5; ++finger) {
        const int base = t_DataOffset + finger * 4;
        joints[finger * 4 + 0] = d.data[base + 0] * kDeg2Rad;
        joints[finger * 4 + 1] = d.data[base + 1] * kDeg2Rad;
        joints[finger * 4 + 2] = d.data[base + 2] * kDeg2Rad;
        joints[finger * 4 + 3] = d.data[base + 3] * kDeg2Rad;
    }

    std::lock_guard<std::mutex> lk(g_mutex);
    if (isLeft) {
        std::memcpy(g_left_joints, joints, sizeof(joints));
        g_have_left = true;
    } else {
        std::memcpy(g_right_joints, joints, sizeof(joints));
        g_have_right = true;
    }
}

inline void OnErgonomics(const ErgonomicsStream* const ergo)
{
    if (!ergo) return;
    const uint32_t L = g_leftId.load();
    const uint32_t R = g_rightId.load();
    for (uint32_t i = 0; i < ergo->dataCount; ++i) {
        const ErgonomicsData& d = ergo->data[i];
        if (d.isUserID) continue;
        if (d.id == L && L != 0) UpdateGlobals(d, true);
        if (d.id == R && R != 0) UpdateGlobals(d, false);
    }
}

inline std::string ProfilePath(int n, const char* fn)
{
    return CalibDir() + "/p" + std::to_string(n) + "/" + fn;
}

inline bool LoadOneCalib(uint32_t id, const std::string& path)
{
    if (id == 0) return false;
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) { std::fprintf(stderr, "[calib] not found: %s\n", path.c_str()); return false; }
    const std::streamsize sz = f.tellg();
    if (sz <= 0) { std::fprintf(stderr, "[calib] empty: %s\n", path.c_str()); return false; }
    f.seekg(0, std::ios::beg);
    std::vector<unsigned char> bytes(static_cast<size_t>(sz));
    if (!f.read(reinterpret_cast<char*>(bytes.data()), sz)) return false;

    SetGloveCalibrationReturnCode r;
    const SDKReturnCode rc = CoreSdk_SetGloveCalibration(id, bytes.data(), static_cast<uint32_t>(sz), &r);
    if (rc != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "[calib] set FAILED: %s rc=%d res=%d\n", path.c_str(), (int)rc, (int)r);
        return false;
    }
    std::fprintf(stderr, "[calib] loaded '%s' into glove 0x%x (%lld bytes)\n", path.c_str(), id, static_cast<long long>(sz));
    return true;
}

inline bool InitSdk()
{
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "InitializeIntegrated failed\n"); return false;
    }
    if (CoreSdk_RegisterCallbackForOnConnect(OnConnected) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForOnDisconnect(OnDisconnected) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForLandscapeStream(OnLandscape) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomics) != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "register callbacks failed\n"); return false;
    }

    CoordinateSystemVUH vuh;
    CoordinateSystemVUH_Init(&vuh);
    vuh.handedness = Side::Side_Right;
    vuh.up         = AxisPolarity::AxisPolarity_PositiveZ;
    vuh.view       = AxisView::AxisView_XToViewer;
    vuh.unitScale  = 1.0f;
    if (CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true) != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "InitializeCoordinateSystemWithVUH failed\n"); return false;
    }

    ManusHost empty;
    ManusHost_Init(&empty);
    if (CoreSdk_ConnectToHost(empty) != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "ConnectToHost failed\n"); return false;
    }
    return true;
}

// Thread body. `stop` is the caller's exit flag — set it, then join.
inline void ThreadFn(const std::atomic<bool>& stop)
{
    if (!InitSdk()) {
        std::fprintf(stderr, "[manus] init failed; thread exiting\n");
        return;
    }
    while (!stop.load()) {
        const int prof = g_profile.load();
        if (prof > 0 && g_connected.load() && !g_calibLoaded.load()) {
            const uint32_t L = g_leftId.load();
            const uint32_t R = g_rightId.load();
            if (L != 0 || R != 0) {
                LoadOneCalib(L, ProfilePath(prof, kLeftCalib));
                LoadOneCalib(R, ProfilePath(prof, kRightCalib));
                // Set even on failure: the retry would run every kSleepMs and
                // bury the log. CheckProfile() below is what catches the common
                // case (wrong number / missing file) before we ever get here.
                g_calibLoaded.store(true);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kSleepMs));
    }
    CoreSdk_ShutDown();
}

inline void SetProfile(int n) { g_profile.store(n); }

// Call from main BEFORE starting the thread. The upload itself only happens
// once a glove connects, on the background thread, so without this check a
// typo'd -pN would drift on silently as "whatever was uploaded last time"
// (see the p0 warning above) instead of failing.
inline bool CheckProfile(int n, std::string* err)
{
    if (n < 0)  { *err = "profile number must be >= 0"; return false; }
    if (n == 0) return true;                      // nothing to read
    for (const char* fn : {kLeftCalib, kRightCalib}) {
        const std::string path = ProfilePath(n, fn);
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f)             { *err = "not found: "  + path; return false; }
        if (f.tellg() <= 0) { *err = "empty file: " + path; return false; }
    }
    return true;
}
inline bool Connected()          { return g_connected.load(); }

// Latest glove joints. `haveL`/`haveR` stay false until that side has streamed
// at least once, so callers can tell "glove at zero" from "no glove".
inline void Snapshot(float L[20], float R[20], bool& haveL, bool& haveR)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    haveL = g_have_left;
    haveR = g_have_right;
    if (haveL) std::memcpy(L, g_left_joints,  sizeof(float) * 20);
    if (haveR) std::memcpy(R, g_right_joints, sizeof(float) * 20);
}

} // namespace manus
