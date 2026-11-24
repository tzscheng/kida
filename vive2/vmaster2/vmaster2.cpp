// vmaster2.cpp - VIVE Ultimate Tracker + Manus glove teleop master.
// Derived from kida/vive/vmaster.cpp; the original directory is unchanged.
// Manus retargeting, clutch controls and KIDA/logger wire formats are retained.
// Ultimate poses arrive through SDK callbacks, without OpenVR or SteamVR.
// See README.md for tracker selection, map yaw, controls and validation.

#include "ManusSDK.h"
#include "ManusSDKTypes.h"
#include "ultimate_input.h"
#include "retarget.h"

#include <zmq.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <getopt.h>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <curses.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <spawn.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

extern char** environ;

namespace {

// ============================================================================
// constants
// ============================================================================

constexpr const char* kSlaveEndpoint  = "ipc:///dev/shm/default";
constexpr const char* kLoggerEndpoint = "ipc:///dev/shm/logger";

constexpr int   kManusSleepMs   = 10;
constexpr int   kLoopPeriodMs   = 33;        // ~30 Hz teleop

std::string g_resourceDir = ".";
std::string g_slaveEndpoint = kSlaveEndpoint;
std::string g_loggerEndpoint = kLoggerEndpoint;
bool g_dryRun = false;

std::string ResourcePath(const char* relative) { return g_resourceDir + "/" + relative; }
constexpr const char* kLeftCalib  = "donghyukLeftMetaglove.mcal";
constexpr const char* kRightCalib = "donghyukRightMetaglove.mcal";

// ============================================================================
// shared state
// ============================================================================

std::atomic<bool> g_exit{false};

std::atomic<bool>     g_manus_connected{false};
std::atomic<uint32_t> g_manus_leftId{0};
std::atomic<uint32_t> g_manus_rightId{0};
std::atomic<bool>     g_manus_calibLoaded{false};
std::atomic<bool>     g_manus_skipCalib{false};

std::mutex g_manus_mutex;
bool       g_manus_have_left  = false;
bool       g_manus_have_right = false;
float      g_manus_left_joints[20]  = {0};
float      g_manus_right_joints[20] = {0};

// ============================================================================
// 4x4 / 3x3 row-major matrix helpers (replaces tact.T_*, np.linalg.inv)
// ============================================================================

using Mat4 = std::array<double, 16>;

inline double& M(Mat4& A, int r, int c) { return A[r * 4 + c]; }
inline double  M(const Mat4& A, int r, int c) { return A[r * 4 + c]; }

Mat4 MatIdentity()
{
    Mat4 A{};
    for (int i = 0; i < 4; ++i) M(A, i, i) = 1.0;
    return A;
}

Mat4 MatMul(const Mat4& A, const Mat4& B)
{
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += M(A, i, k) * M(B, k, j);
            M(C, i, j) = s;
        }
    return C;
}




Mat4 T_rot_z(double t)
{
    Mat4 T = MatIdentity();
    const double c = std::cos(t), s = std::sin(t);
    M(T, 0, 0) =  c; M(T, 0, 1) = -s;
    M(T, 1, 0) =  s; M(T, 1, 1) =  c;
    return T;
}


// 3x3 row-major rotation helpers, used for the attach-time rotation offset.
using Mat3 = std::array<double, 9>;

inline double& M3(Mat3& A, int r, int c) { return A[r * 3 + c]; }
inline double  M3(const Mat3& A, int r, int c) { return A[r * 3 + c]; }

Mat3 Mat3Identity()
{
    Mat3 A{};
    M3(A, 0, 0) = M3(A, 1, 1) = M3(A, 2, 2) = 1.0;
    return A;
}

Mat3 Mat3FromMat4(const Mat4& T)
{
    Mat3 R{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M3(R, i, j) = M(T, i, j);
    return R;
}

Mat3 Mat3Mul(const Mat3& A, const Mat3& B)
{
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += M3(A, i, k) * M3(B, k, j);
            M3(C, i, j) = s;
        }
    return C;
}

Mat3 Mat3Transpose(const Mat3& A)
{
    Mat3 R{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M3(R, i, j) = M3(A, j, i);
    return R;
}

// Extrinsic XYZ Tait-Bryan angles from a 3x3 rotation.
// Matches tact.rotation_to_euler(...) with default eulerseq='xyz'.
void RotationToEulerXYZ(const Mat3& R, double& rx, double& ry, double& rz)
{
    const double r20 = M3(R, 2, 0);
    const double clamp_r20 = std::max(-1.0, std::min(1.0, r20));
    ry = std::asin(-clamp_r20);
    if (std::abs(clamp_r20) < 0.999999) {
        rx = std::atan2(M3(R, 2, 1), M3(R, 2, 2));
        rz = std::atan2(M3(R, 1, 0), M3(R, 0, 0));
    } else {
        rx = std::atan2(-M3(R, 1, 2), M3(R, 1, 1));
        rz = 0.0;
    }
}

// ============================================================================
// Manus side (background thread)
// ============================================================================

void OnManusConnected(const ManusHost* const)
{
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);
    g_manus_connected.store(true);
}

void OnManusDisconnected(const ManusHost* const)
{
    g_manus_connected.store(false);
}

void OnManusLandscape(const Landscape* const land)
{
    if (!land) return;
    uint32_t leftId = 0, rightId = 0;
    for (uint32_t i = 0; i < land->gloveDevices.gloveCount; ++i) {
        const GloveLandscapeData& g = land->gloveDevices.gloves[i];
        if (leftId  == 0 && g.side == Side::Side_Left)  leftId  = g.id;
        if (rightId == 0 && g.side == Side::Side_Right) rightId = g.id;
    }
    g_manus_leftId.store(leftId);
    g_manus_rightId.store(rightId);
}

void UpdateManusGlobals(const ErgonomicsData& d, bool isLeft)
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

    std::lock_guard<std::mutex> lk(g_manus_mutex);
    if (isLeft) {
        std::memcpy(g_manus_left_joints, joints, sizeof(joints));
        g_manus_have_left = true;
    } else {
        std::memcpy(g_manus_right_joints, joints, sizeof(joints));
        g_manus_have_right = true;
    }
}

void OnManusErgonomics(const ErgonomicsStream* const ergo)
{
    if (!ergo) return;
    const uint32_t L = g_manus_leftId.load();
    const uint32_t R = g_manus_rightId.load();
    for (uint32_t i = 0; i < ergo->dataCount; ++i) {
        const ErgonomicsData& d = ergo->data[i];
        if (d.isUserID) continue;
        if (d.id == L && L != 0) UpdateManusGlobals(d, true);
        if (d.id == R && R != 0) UpdateManusGlobals(d, false);
    }
}

bool LoadOneCalib(uint32_t id, const std::string& fn)
{
    if (id == 0) return false;
    const std::string path = ResourcePath("calib") + "/" + fn;
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
        std::fprintf(stderr, "[calib] set failed: %s rc=%d res=%d\n", fn.c_str(), (int)rc, (int)r);
        return false;
    }
    std::fprintf(stderr, "[calib] loaded '%s' into glove 0x%x (%lld bytes)\n", path.c_str(), id, static_cast<long long>(sz));
    return true;
}

bool InitManusSdk()
{
    if (CoreSdk_InitializeIntegrated() != SDKReturnCode::SDKReturnCode_Success) {
        std::fprintf(stderr, "InitializeIntegrated failed\n"); return false;
    }
    if (CoreSdk_RegisterCallbackForOnConnect(OnManusConnected) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForOnDisconnect(OnManusDisconnected) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForLandscapeStream(OnManusLandscape) != SDKReturnCode::SDKReturnCode_Success || CoreSdk_RegisterCallbackForErgonomicsStream(OnManusErgonomics) != SDKReturnCode::SDKReturnCode_Success) {
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

void ManusThreadFn()
{
    if (!InitManusSdk()) {
        std::fprintf(stderr, "[manus] init failed; thread exiting\n");
        return;
    }
    while (!g_exit.load()) {
        if (!g_manus_skipCalib.load() && g_manus_connected.load() && !g_manus_calibLoaded.load()) {
            const uint32_t L = g_manus_leftId.load();
            const uint32_t R = g_manus_rightId.load();
            if (L != 0 || R != 0) {
                LoadOneCalib(L, kLeftCalib);
                LoadOneCalib(R, kRightCalib);
                g_manus_calibLoaded.store(true);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kManusSleepMs));
    }
    CoreSdk_ShutDown();
}

// ============================================================================
// Glove retargeting (was glove_retarget_task in python)
// ============================================================================
// Pure RetargetSide() lives in retarget.h so a future binary can share it.

void RetargetAll(int g, float* q)
{
    float L[20] = {0}, R[20] = {0};
    bool haveL = false, haveR = false;
    {
        std::lock_guard<std::mutex> lk(g_manus_mutex);
        haveL = g_manus_have_left;
        haveR = g_manus_have_right;
        if (haveL) std::memcpy(L, g_manus_left_joints,  sizeof(L));
        if (haveR) std::memcpy(R, g_manus_right_joints, sizeof(R));
    }
    if (g == 0) {
        if (haveL) RetargetSide_h9(L, q);
        if (haveR) RetargetSide_h9(R, q + 20);
    } else if (g == 1) {
        if (haveL) RetargetSide_dg5f('L', L, q);
        if (haveR) RetargetSide_dg5f('R', R, q + 20);
    } else if (g == 2) {
        if (haveL) RetargetSide_dg5s('L', L, q);
        if (haveR) RetargetSide_dg5s('R', R, q + 20);
    } else if (g == 3) {
        if (haveL) RetargetSide_h12(L, q);
        if (haveR) RetargetSide_h12(R, q + 20);
    }
}

// ============================================================================
// Attach-time origin capture (same position and rotation clutch as vive/vmaster).
// ============================================================================

void CaptureOrigin(const double raw[3],const Mat3& rotation,const Mat3& bias,
                   double offset[3],Mat3& rotation_offset)
{
    for (int k=0;k<3;++k) offset[k]=-raw[k];
    rotation_offset=Mat3Mul(Mat3Transpose(rotation),bias);
}

// ============================================================================
// ncurses UI: 4-row split — status / vive / manus / log
// (Captures stdout/stderr via pipe so SDK chatter goes to the log pane.)
// ============================================================================

WINDOW* g_winStatus = nullptr;
WINDOW* g_winVive   = nullptr;
WINDOW* g_winManus  = nullptr;
WINDOW* g_winLog    = nullptr;

FILE*   g_ttyIn  = nullptr;
FILE*   g_ttyOut = nullptr;
SCREEN* g_screen = nullptr;

int g_origStdout  = -1;
int g_origStderr  = -1;
int g_logPipeRead = -1;

std::mutex              g_logMtx;
std::deque<std::string> g_logLines;
constexpr size_t        kMaxLogLines = 1000;

std::thread       g_logReader;
std::atomic<bool> g_logReaderStop{false};

void PushLogLine(std::string s)
{
    std::lock_guard<std::mutex> lk(g_logMtx);
    g_logLines.push_back(std::move(s));
    while (g_logLines.size() > kMaxLogLines) g_logLines.pop_front();
}

void LogReaderThread()
{
    char buf[4096];
    std::string acc;
    while (!g_logReaderStop.load()) {
        const ssize_t n = read(g_logPipeRead, buf, sizeof(buf));
        if (n < 0) { if (errno == EINTR) continue; break; }
        if (n == 0) break;
        for (ssize_t i = 0; i < n; ++i) {
            const char c = buf[i];
            if (c == '\n') { PushLogLine(std::move(acc)); acc.clear(); }
            else if (c != '\r') acc.push_back(c);
        }
    }
    if (!acc.empty()) PushLogLine(std::move(acc));
}

bool InitUi()
{
    g_ttyIn  = std::fopen("/dev/tty", "r");
    g_ttyOut = std::fopen("/dev/tty", "w");
    if (!g_ttyIn || !g_ttyOut) { std::fprintf(stderr, "open /dev/tty failed\n"); return false; }

    g_screen = newterm(nullptr, g_ttyOut, g_ttyIn);
    if (!g_screen) { std::fprintf(stderr, "newterm failed\n"); return false; }
    set_term(g_screen);
    cbreak(); noecho(); curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    set_escdelay(25);
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_RED, -1);
    }

    int rows = 0, cols = 0;
    getmaxyx(stdscr, rows, cols);

    int hStatus = 3;
    int hVive   = 6;
    int hManus  = 11;
    int hLog    = rows - hStatus - hVive - hManus;
    if (hLog < 3) {
        hStatus = 3;
        hVive   = std::max(3, (rows - hStatus) / 4);
        hManus  = std::max(3, (rows - hStatus) / 3);
        hLog    = std::max(3, rows - hStatus - hVive - hManus);
    }

    g_winStatus = newwin(hStatus, cols, 0, 0);
    g_winVive   = newwin(hVive,   cols, hStatus, 0);
    g_winManus  = newwin(hManus,  cols, hStatus + hVive, 0);
    g_winLog    = newwin(hLog,    cols, hStatus + hVive + hManus, 0);
    if (!g_winStatus || !g_winVive || !g_winManus || !g_winLog) return false;

    int p[2];
    if (pipe(p) != 0) { std::fprintf(stderr, "pipe failed: %s\n", std::strerror(errno)); return false; }
    g_logPipeRead = p[0];
    const int w = p[1];
    g_origStdout = dup(STDOUT_FILENO);
    g_origStderr = dup(STDERR_FILENO);
    dup2(w, STDOUT_FILENO);
    dup2(w, STDERR_FILENO);
    close(w);
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::setvbuf(stderr, nullptr, _IOLBF, 0);
    g_logReader = std::thread(LogReaderThread);

    return true;
}

void ShutdownUi()
{
    if (g_origStdout >= 0) { dup2(g_origStdout, STDOUT_FILENO); close(g_origStdout); g_origStdout = -1; }
    if (g_origStderr >= 0) { dup2(g_origStderr, STDERR_FILENO); close(g_origStderr); g_origStderr = -1; }
    g_logReaderStop.store(true);
    if (g_logReader.joinable()) g_logReader.join();
    if (g_logPipeRead >= 0) { close(g_logPipeRead); g_logPipeRead = -1; }
    if (g_winStatus) { delwin(g_winStatus); g_winStatus = nullptr; }
    if (g_winVive)   { delwin(g_winVive);   g_winVive   = nullptr; }
    if (g_winManus)  { delwin(g_winManus);  g_winManus  = nullptr; }
    if (g_winLog)    { delwin(g_winLog);    g_winLog    = nullptr; }
    if (g_screen)    { endwin(); delscreen(g_screen); g_screen = nullptr; }
    if (g_ttyOut)    { std::fclose(g_ttyOut); g_ttyOut = nullptr; }
    if (g_ttyIn)     { std::fclose(g_ttyIn);  g_ttyIn  = nullptr; }
}

void DrawBorder(WINDOW* w, const char* title)
{
    werase(w);
    box(w, 0, 0);
    mvwprintw(w, 0, 2, " %s ", title);
}

void DrawStatus(int t, bool ee_attach, bool hand_attach, bool log_on, long cnt, int last_key)
{
    DrawBorder(g_winStatus, "STATUS");
    char keybuf[16];
    if      (last_key < 0)             std::snprintf(keybuf, sizeof(keybuf), "-");
    else if (last_key == 27)           std::snprintf(keybuf, sizeof(keybuf), "ESC");
    else if (last_key == ' ')          std::snprintf(keybuf, sizeof(keybuf), "SPC");
    else if (last_key >= 33 && last_key < 127) std::snprintf(keybuf, sizeof(keybuf), "%c", last_key);
    else                               std::snprintf(keybuf, sizeof(keybuf), "0x%02X", last_key);
    mvwprintw(g_winStatus, 1, 2, "type:%d  ee:%d  hand:%d  log:%d  cnt:%ld  key:%s", t, ee_attach ? 1 : 0, hand_attach ? 1 : 0, log_on ? 1 : 0, cnt, keybuf);
    wnoutrefresh(g_winStatus);
}

void DrawVive(int t, double xyz[2][3], double rpy[2][3], double xyz_raw[2][3], double rpy_raw[2][3], double jawpos, const std::array<ultimate::Sample,2>& samples)
{
    DrawBorder(g_winVive, "ULTIMATE 6D POSE (m / rad)");
    if (t == 0 || t == 1) {
        mvwprintw(g_winVive, 1, 2, "%s  xyz:%6.3f %6.3f %6.3f   rpy:%6.3f %6.3f %6.3f   abs-xyz:%6.3f %6.3f %6.3f   abs-rpy:%6.3f %6.3f %6.3f", t == 0 ? "L" : "R", xyz[t][0], xyz[t][1], xyz[t][2], rpy[t][0], rpy[t][1], rpy[t][2], xyz_raw[t][0], xyz_raw[t][1], xyz_raw[t][2], rpy_raw[t][0], rpy_raw[t][1], rpy_raw[t][2]);
    } else if (t == 2) {
        mvwprintw(g_winVive, 1, 2, "L  xyz:%6.3f %6.3f %6.3f   rpy:%6.3f %6.3f %6.3f   abs-xyz:%6.3f %6.3f %6.3f   abs-rpy:%6.3f %6.3f %6.3f", xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], xyz_raw[0][0], xyz_raw[0][1], xyz_raw[0][2], rpy_raw[0][0], rpy_raw[0][1], rpy_raw[0][2]);
        mvwprintw(g_winVive, 2, 2, "R  xyz:%6.3f %6.3f %6.3f   rpy:%6.3f %6.3f %6.3f   abs-xyz:%6.3f %6.3f %6.3f   abs-rpy:%6.3f %6.3f %6.3f", xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2], xyz_raw[1][0], xyz_raw[1][1], xyz_raw[1][2], rpy_raw[1][0], rpy_raw[1][1], rpy_raw[1][2]);
    } else if (t == 5) {
        mvwprintw(g_winVive, 1, 2, "   xyz:%6.3f %6.3f %6.3f   rpy:%6.3f %6.3f %6.3f   abs-xyz:%6.3f %6.3f %6.3f   abs-rpy:%6.3f %6.3f %6.3f   jaw=%5.3f", xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], xyz_raw[0][0], xyz_raw[0][1], xyz_raw[0][2], rpy_raw[0][0], rpy_raw[0][1], rpy_raw[0][2], jawpos);
    }
    for (int slot=0;slot<2;++slot) {
        if (!((slot==0&&(t==0||t==2||t==5))||(slot==1&&(t==1||t==2)))) continue;
        const auto& sample=samples[slot];
        mvwprintw(g_winVive,3+slot,2,"%s ID:%u  %-24s  battery:%u%%  serial:%s",
            slot==0?"L":"R",sample.id,ultimate::health(sample),sample.data.Battery,sample.serial.c_str());
    }
    wnoutrefresh(g_winVive);
}

void DrawManusSide(int row, const char* label, const float* qs, int n)
{
    // DG5F (n=20): 8, 8, 4 across three rows, grouped by 4 with extra spacing.
    // H9 (n=9): original 10-per-row layout, tight spacing.
    mvwprintw(g_winManus, row, 2, "%s", label);
    const bool wide = (n > 10);
    for (int i = 0; i < n; ++i) {
        int r, c;
        if (wide) {
            const int rowIdx = i / 8;
            const int colIdx = i % 8;
            r = row + 1 + rowIdx;
            c = 2 + colIdx * 8 + (colIdx / 4) * 2;
        } else {
            r = row + 1 + i / 10;
            c = 2 + (i % 10) * 8;
        }
        mvwprintw(g_winManus, r, c, "%7.3f", qs[i]);
    }
}

void DrawManus(int t, int n_joint, const float* q)
{
    DrawBorder(g_winManus, "MANUS JOINTS");
    if (n_joint <= 0) {
        mvwprintw(g_winManus, 1, 2, "(arm-only mode)");
    } else if (t == 0 || t == 1) {
        DrawManusSide(1, t == 0 ? "L-HAND:" : "R-HAND:", q + 20 * t, n_joint);
    } else if (t == 2) {
        const int valRows = (n_joint > 10) ? ((n_joint + 7) / 8) : ((n_joint + 9) / 10);
        DrawManusSide(1, "L-HAND:", q,      n_joint);
        DrawManusSide(1 + 1 + valRows + 1, "R-HAND:", q + 20, n_joint);
    }
    wnoutrefresh(g_winManus);
}

void DrawLog()
{
    DrawBorder(g_winLog, "LOG");
    int rows = 0, cols = 0;
    getmaxyx(g_winLog, rows, cols);
    const int contentRows = rows - 2;
    if (contentRows <= 0) { wnoutrefresh(g_winLog); return; }
    std::lock_guard<std::mutex> lk(g_logMtx);
    const int n = static_cast<int>(g_logLines.size());
    const int start = std::max(0, n - contentRows);
    for (int i = 0; i < contentRows && start + i < n; ++i) {
        const std::string& s = g_logLines[start + i];
        const int show = std::min(static_cast<int>(s.size()), cols - 4);
        const bool isLogger = s.rfind("[logger]", 0) == 0;
        if (isLogger) wattron(g_winLog, COLOR_PAIR(1));
        mvwaddnstr(g_winLog, 1 + i, 2, s.c_str(), show);
        if (isLogger) wattroff(g_winLog, COLOR_PAIR(1));
    }
    wnoutrefresh(g_winLog);
}

// ============================================================================
// ZMQ helpers
// ============================================================================

void* g_zmqCtx        = nullptr;
// Data sockets: per-frame teleop stream. CONFLATE on — only the latest pose
// matters, intermediate drops are fine.
void* g_slaveSock     = nullptr;
void* g_loggerSock    = nullptr;
// Control sockets: 1-shot commands (randomize, rest/home/init, quit,
// log-on/log-off). CONFLATE off — every command must be delivered in order,
// otherwise a control message can be silently overwritten by a subsequent
// data-socket task frame. Both data and control PUSH connect to the SAME
// endpoint; the peer's PULL socket fair-queues across the two connections,
// so no consumer-side changes are needed.
void* g_slaveCtlSock  = nullptr;
void* g_loggerCtlSock = nullptr;

bool InitOnePush(void*& sock, const char* endpoint, bool conflate)
{
    sock = zmq_socket(g_zmqCtx, ZMQ_PUSH);
    if (!sock) return false;
    int one = 1, zero = 0;
    if (conflate) {
        // Data socket: drop stale teleop frames, never queue against a slow peer.
        // LINGER=0 — on close, residual task frames may be discarded.
        zmq_setsockopt(sock, ZMQ_IMMEDIATE, &one,  sizeof(one));
        zmq_setsockopt(sock, ZMQ_CONFLATE,  &one,  sizeof(one));
        zmq_setsockopt(sock, ZMQ_LINGER,    &zero, sizeof(zero));
    } else {
        // Control socket keeps ZMQ implementation defaults (queue depth = SNDHWM,
        // no conflate, no immediate). LINGER=200ms so zmq_close blocks until a
        // final "quit" is actually flushed to the peer's kernel pipe — capped
        // at 200ms in case the peer is gone. Sends are still ZMQ_DONTWAIT.
        const int linger_ms = 200;
        zmq_setsockopt(sock, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));
    }
    if (zmq_connect(sock, endpoint) != 0) {
        std::fprintf(stderr, "zmq_connect %s failed: %s\n", endpoint, zmq_strerror(zmq_errno()));
        return false;
    }
    return true;
}

bool InitZmq()
{
    if (g_dryRun) return true;
    g_zmqCtx = zmq_ctx_new();
    if (!g_zmqCtx) return false;
    if (!InitOnePush(g_slaveSock,     g_slaveEndpoint.c_str(),  true))  return false;
    if (!InitOnePush(g_loggerSock,    g_loggerEndpoint.c_str(), true))  return false;
    if (!InitOnePush(g_slaveCtlSock,  g_slaveEndpoint.c_str(),  false)) return false;
    if (!InitOnePush(g_loggerCtlSock, g_loggerEndpoint.c_str(), false)) return false;
    return true;
}

void ShutdownZmq()
{
    if (g_slaveSock)     zmq_close(g_slaveSock);
    if (g_loggerSock)    zmq_close(g_loggerSock);
    if (g_slaveCtlSock)  zmq_close(g_slaveCtlSock);
    if (g_loggerCtlSock) zmq_close(g_loggerCtlSock);
    if (g_zmqCtx)        zmq_ctx_term(g_zmqCtx);
    g_slaveSock = g_loggerSock = g_slaveCtlSock = g_loggerCtlSock = nullptr;
    g_zmqCtx = nullptr;
}

void SendPush(void* sock, const char* data, size_t len)
{
    if (g_dryRun) {
        std::printf("[dry-run] %.*s\n",static_cast<int>(len),data);
        return;
    }
    if (!sock || len == 0) return;
    (void)zmq_send(sock, data, len, ZMQ_DONTWAIT); // EAGAIN ignored, like python
}

// ============================================================================
// subprocess helpers (spawn ./logger)
// ============================================================================

pid_t SpawnDetached(const std::vector<std::string>& argv)
{
    if (argv.empty()) return -1;
    std::vector<char*> cargs;
    cargs.reserve(argv.size() + 1);
    for (auto& s : argv) cargs.push_back(const_cast<char*>(s.c_str()));
    cargs.push_back(nullptr);

    pid_t pid = -1;
    const int rc = posix_spawn(&pid, cargs[0], nullptr, nullptr, cargs.data(), environ);
    if (rc != 0) {
        std::fprintf(stderr, "spawn '%s' failed: %s\n", cargs[0], std::strerror(rc));
        return -1;
    }
    return pid;
}

// Returns true iff `pid` is still running. If it has already terminated,
// reaps it and reports the exit status via stderr.
bool IsChildAlive(pid_t pid, const char* name)
{
    if (pid <= 0) return false;
    int status = 0;
    const pid_t r = waitpid(pid, &status, WNOHANG);
    if (r == 0) return true;
    if (r < 0) {
        std::fprintf(stderr, "waitpid('%s') failed: %s\n", name, std::strerror(errno));
        return false;
    }
    if (WIFEXITED(status)) {
        std::fprintf(stderr, "'%s' exited early with code %d\n", name, WEXITSTATUS(status));
    } else if (WIFSIGNALED(status)) {
        std::fprintf(stderr, "'%s' killed by signal %d\n", name, WTERMSIG(status));
    } else {
        std::fprintf(stderr, "'%s' terminated abnormally\n", name);
    }
    return false;
}

// ============================================================================
// Wire formatting (kept identical to vive/vmaster)
// ============================================================================

std::string FormatTeleopCommand(int t,int g,int n_joint,const double xyz[2][3],
    const double rpy[2][3],const float q[40],double jawpos,bool ee_attach,bool hand_attach)
{
    // Format arm + hand commands.
    char arm_cmd[256]  = {0};
    char hand_cmd[512] = {0};
    bool have_arm  = false;
    bool have_hand = false;

    if (t == 0 || t == 1) {
        const int s = t;
        std::snprintf(arm_cmd, sizeof(arm_cmd), "task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f", xyz[s][0], xyz[s][1], xyz[s][2], rpy[s][0], rpy[s][1], rpy[s][2]);
        have_arm = true;
        if (g >= 0) {
            int n = std::snprintf(hand_cmd, sizeof(hand_cmd), "joint");
            for (int i = 0; i < n_joint; ++i)
                n += std::snprintf(hand_cmd + n, sizeof(hand_cmd) - n, "%8.3f", q[20 * t + i]);
            have_hand = true;
        }
    } else if (t == 2) {
        std::snprintf(arm_cmd, sizeof(arm_cmd), "task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f", xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2]);
        have_arm = true;
        if (g >= 0) {
            int n = std::snprintf(hand_cmd, sizeof(hand_cmd), "joint");
            for (int i = 0; i < n_joint; ++i)
                n += std::snprintf(hand_cmd + n, sizeof(hand_cmd) - n, "%8.3f", q[i]);
            n += std::snprintf(hand_cmd + n, sizeof(hand_cmd) - n, ", joint");
            for (int i = 0; i < n_joint; ++i)
                n += std::snprintf(hand_cmd + n, sizeof(hand_cmd) - n, "%8.3f", q[i + 20]);
            have_hand = true;
        }
    } else if (t == 5) {
        std::snprintf(arm_cmd, sizeof(arm_cmd), "task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f", xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], jawpos);
        have_arm = true;
    }

    // Send when at least one of the two attach flags is on. Substitute
    // "none" for whichever side is detached, so the slave keeps a
    // consistent two-part frame and can ignore the unused side.
    // For -t5 (no hand part) and gripper-less -t0/1/2, hand_attach is
    // moot — the gating reduces to ee_attach alone.
    const bool send_arm  = have_arm  && ee_attach;
    const bool send_hand = have_hand && hand_attach;
    if (send_arm || send_hand) {
        char cmd[800];
        int  n = 0;
        if (send_arm) n += std::snprintf(cmd + n, sizeof(cmd) - n, "%s", arm_cmd);
        else          n += std::snprintf(cmd + n, sizeof(cmd) - n, "none");
        if (have_hand) {
            if (send_hand) n += std::snprintf(cmd + n, sizeof(cmd) - n, ", %s", hand_cmd);
            else           n += std::snprintf(cmd + n, sizeof(cmd) - n, ", none");
        }
        return std::string(cmd,static_cast<size_t>(n));
    }

    return {};
}

// CLI parsing
struct Args {
    int t=-1,g=-1,b=1;
    bool l=false,n=false,dry_run=false;
    bool left_id_set=false,right_id_set=false,yaw_set=false;
    uint32_t left_id=0,right_id=1;
    std::string left_serial,right_serial;
    std::string slave_endpoint=kSlaveEndpoint,logger_endpoint=kLoggerEndpoint;
    double yaw_degrees=0,seconds=0;
};

void PrintUsage(const char* prog)
{
    std::fprintf(stderr,
        "usage: %s -tN [-gN] [-bN] [-l] [-n] [options]\n"
        "  -tN  0=kida-left, 1=kida-right, 2=kida-both, 5=gos10\n"
        "  -gN  0=H9, 1=DG5F-M, 2=DG5F-S, 3=H12 (omit for arm-only)\n"
        "  -bN  legacy heading preset: 1=0 deg, 2=+30 deg (no base stations)\n"
        "  -l   start the copied logger; -n skip Manus glove calibration\n"
        "  --left-id N / --right-id N       SDK slots, defaults 0 / 1\n"
        "  --left-serial S / --right-serial S  select by serial instead of ID\n"
        "  --yaw-deg N    rotate SDK map into robot frame about +Z\n"
        "  --dry-run      receive/show input and commands, no ZMQ sends\n"
        "  --seconds N    exit after N seconds\n"
        "  --slave-endpoint URI / --logger-endpoint URI (default original IPC)\n"
        "Keys: a arm clutch, b hand clutch, h home, i init, r rest, / randomize,\n"
        "      c log, z/x jaw, 1/2 identify tracker, q quit slave+master, ESC master only.\n",
        prog);
}

bool ParseArgs(int argc,char** argv,Args& a)
{
    static const option options[]={
        {"left-id",required_argument,nullptr,1000},
        {"right-id",required_argument,nullptr,1001},
        {"left-serial",required_argument,nullptr,1002},
        {"right-serial",required_argument,nullptr,1003},
        {"yaw-deg",required_argument,nullptr,1004},
        {"dry-run",no_argument,nullptr,1005},
        {"seconds",required_argument,nullptr,1006},
        {"slave-endpoint",required_argument,nullptr,1007},
        {"logger-endpoint",required_argument,nullptr,1008},
        {"help",no_argument,nullptr,'h'},
        {nullptr,0,nullptr,0}};
    opterr=0;
    int opt;
    while ((opt=getopt_long(argc,argv,"t:g:b:lnh",options,nullptr))!=-1) {
        if (opt=='h') { PrintUsage(argv[0]); std::exit(0); }
        if (opt=='l') { a.l=true; continue; }
        if (opt=='n') { a.n=true; continue; }
        if (opt==1005) { a.dry_run=true; continue; }
        if (opt==1002||opt==1003||opt==1007||opt==1008) {
            if (!optarg||!*optarg) return false;
            if (opt==1002) a.left_serial=optarg;
            if (opt==1003) a.right_serial=optarg;
            if (opt==1007) a.slave_endpoint=optarg;
            if (opt==1008) a.logger_endpoint=optarg;
            continue;
        }
        if (opt==1004||opt==1006) {
            char* end=nullptr; errno=0;
            double value=std::strtod(optarg,&end);
            if (errno||end==optarg||*end||!std::isfinite(value)||(opt==1006&&value<=0)) {
                std::fprintf(stderr,"invalid numeric option: %s\n",optarg); return false;
            }
            if (opt==1004) { a.yaw_degrees=value; a.yaw_set=true; }
            else a.seconds=value;
            continue;
        }
        if (opt=='t'||opt=='g'||opt=='b'||opt==1000||opt==1001) {
            char* end=nullptr; errno=0;
            long value=std::strtol(optarg,&end,10);
            if (errno||end==optarg||*end||value<std::numeric_limits<int>::min()||value>std::numeric_limits<int>::max()) {
                std::fprintf(stderr,"invalid integer option: %s\n",optarg); return false;
            }
            if (opt=='t') a.t=int(value);
            if (opt=='g') a.g=int(value);
            if (opt=='b') a.b=int(value);
            if (opt==1000||opt==1001) {
                if (value<0||value>=long(vive::GetNumTrackers())) {
                    std::fprintf(stderr,"Tracker ID must be within SDK slots.\n"); return false;
                }
                if (opt==1000) { a.left_id=uint32_t(value); a.left_id_set=true; }
                else { a.right_id=uint32_t(value); a.right_id_set=true; }
            }
            continue;
        }
        PrintUsage(argv[0]); return false;
    }
    if (optind<argc || (a.t!=0&&a.t!=1&&a.t!=2&&a.t!=5) ||
        a.g< -1||a.g>3 || (a.b!=1&&a.b!=2)) {
        std::fprintf(stderr,"Invalid operation, gripper, heading preset or positional argument.\n"); return false;
    }
    if ((a.left_id_set&&!a.left_serial.empty())||(a.right_id_set&&!a.right_serial.empty())) {
        std::fprintf(stderr,"Choose an ID or serial for each side.\n"); return false;
    }
    if (a.t==2 && ((a.left_serial.empty()&&a.right_serial.empty()&&a.left_id==a.right_id)||
        (!a.left_serial.empty()&&a.left_serial==a.right_serial))) {
        std::fprintf(stderr,"Left and right must use distinct trackers.\n"); return false;
    }
    if (a.dry_run&&a.l) {
        std::fprintf(stderr,"--dry-run cannot start the logger (-l).\n"); return false;
    }
    if (!a.yaw_set) a.yaw_degrees=a.b==1?0:30;
    return true;
}

// ============================================================================
// signals
// ============================================================================

void HandleSignal(int) { g_exit.store(true); }

} // namespace

// ============================================================================
// main
// ============================================================================

int main(int argc, char** argv)
{
    Args arg;
    if (!ParseArgs(argc, argv, arg)) return 64;
    if (arg.t < 0) {
        std::fprintf(stderr, "choose operation type (-tN)\n");
        PrintUsage(argv[0]);
        return 64;
    }
    
    char executable[4096];
    const ssize_t executable_size=readlink("/proc/self/exe",executable,sizeof(executable)-1);
    if (executable_size<0) { std::perror("readlink /proc/self/exe"); return 1; }
    executable[executable_size]='\0';
    g_resourceDir=std::string(executable).substr(0,std::string(executable).find_last_of('/'));
    g_dryRun=arg.dry_run; g_slaveEndpoint=arg.slave_endpoint; g_loggerEndpoint=arg.logger_endpoint;

    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    // Validate helper binaries before InitUi() so any error message reaches
    // the real terminal instead of the (about-to-be-torn-down) ncurses log pane.
    if (arg.l && access(ResourcePath("logger").c_str(), X_OK) != 0) {
        std::fprintf(stderr, "logger missing/not executable beside vmaster2: %s\n", std::strerror(errno));
        return 1;
    }

    if (!InitUi()) {
        std::fprintf(stderr, "InitUi failed\n");
        ShutdownUi();
        return 1;
    }

    ultimate::Trackers trackers;
    if (!trackers.start()) {
        std::fprintf(stderr,"Ultimate SDK server failed. Check dongle/USB access and stop other SDK servers.\n");
        ShutdownUi(); return 2;
    }
    std::array<ultimate::Selection,2> selections{{{arg.left_id,arg.left_serial},{arg.right_id,arg.right_serial}}};
    std::array<ultimate::Sample,2> samples;
    std::array<uint64_t,2> attached_generation{};
    const auto active=[&](int slot) {
        return slot==0?(arg.t==0||arg.t==2||arg.t==5):(arg.t==1||arg.t==2);
    };

    // Hand target joint vector — same layout as the python q[40].
    float q[40] = {0};
    int   n_joint = 0;
    if      (arg.g == 0) n_joint = 9;
    else if (arg.g == 1 || arg.g == 2) n_joint = 20;
    else if (arg.g == 3) n_joint = 12;
    // arg.g == -1: arm-only mode

    // 1. Manus thread (replaces UDP receiver)
    g_manus_skipCalib.store(arg.n);
    std::thread manusThread;
    if (arg.g >= 0) manusThread = std::thread(ManusThreadFn);

    // 2. Optionally spawn ./logger.
    char tflag[16], gflag[16];
    std::snprintf(tflag, sizeof(tflag), "-t%d", arg.t);
    std::snprintf(gflag, sizeof(gflag), "-g%d", arg.g);

    pid_t loggerPid = -1;
    if (arg.l) {
        std::vector<std::string> loggerArgv = {ResourcePath("logger"), tflag};
        loggerArgv.push_back("--command-endpoint");
        loggerArgv.push_back(arg.logger_endpoint);
        const std::string python=ResourcePath("../../.venv/bin/python");
        if (access(python.c_str(),X_OK)==0) {
            loggerArgv.insert(loggerArgv.begin(),{python,"-u"});
        }
        if (arg.g >= 0) loggerArgv.push_back(gflag);
        loggerPid = SpawnDetached(loggerArgv);
        if (loggerPid <= 0) {
            std::fprintf(stderr, "failed to start ./logger\n");
            g_exit.store(true);
            if (manusThread.joinable()) manusThread.join();
            ShutdownUi();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!IsChildAlive(loggerPid, "./logger")) {
            std::fprintf(stderr, "./logger did not start normally, exiting\n");
            g_exit.store(true);
            if (manusThread.joinable()) manusThread.join();
            ShutdownUi();
            return 1;
        }
    }

    // 3. ZMQ
    if (!InitZmq()) {
        g_exit.store(true);
        if (manusThread.joinable()) manusThread.join();
        ShutdownUi();
        return 1;
    }

    std::printf("[ultimate] SDK %s; slots L=%u / R=%u; map yaw=%.1f deg\n",
        vive::GetVersion(),arg.left_id,arg.right_id,arg.yaw_degrees);
    std::printf("[ultimate] Waiting for TRACKING. Keys 1/2 identify L/R. Press a to attach.\n");
    if (arg.dry_run) std::puts("[dry-run] All outgoing ZMQ sends disabled.");

    // 6. Per-arm state
    double offset[2][3] = {{0, 0, 0}, {0, 0, 0}};
    double _xyz[2][3]   = {{0, 0, 0}, {0, 0, 0}};
    double xyz[2][3]    = {{0, 0, 0}, {0, 0, 0}};
    double rpy[2][3]    = {{0, 0, 0}, {0, 0, 0}};
    double rpy_raw[2][3] = {{0, 0, 0}, {0, 0, 0}};   // before R_off applied; UI only
    double bias[2][3]   = {{0, 0, 0}, {0, 0, 0}};
    double scale[2]     = {1.0, 1.0};
    // Rotation analogue of (offset, bias) for position. R_out = R_m · R_off,
    // with R_off = R_m_at_attach^T · R_bias captured on attach. R_bias holds
    // the slave's last commanded orientation (identity when unknown — first
    // attach after start may still jump; subsequent attach/detach cycles will
    // be smooth).
    Mat3 R_m_cur[2] = {Mat3Identity(), Mat3Identity()};
    Mat3 R_off[2]   = {Mat3Identity(), Mat3Identity()};
    Mat3 R_bias[2]  = {Mat3Identity(), Mat3Identity()};
    Mat3 R_out[2]   = {Mat3Identity(), Mat3Identity()};
    if (arg.t == 0) {
        bias[0][0] = 0.30; bias[0][1] =  0.23; bias[0][2] = -0.40; scale[0] = 1.0;
    } else if (arg.t == 1) {
        bias[1][0] = 0.30; bias[1][1] = -0.23; bias[1][2] = -0.40; scale[1] = 1.0;
    } else if (arg.t == 2) {
        bias[0][0] = 0.30; bias[0][1] =  0.23; bias[0][2] = -0.40; scale[0] = 1.0;
        bias[1][0] = 0.30; bias[1][1] = -0.23; bias[1][2] = -0.40; scale[1] = 1.0;
    } else if (arg.t == 5) {
        bias[0][0] = 0.50; bias[0][1] = -0.15; bias[0][2] =  0.30; scale[0] = 2.0;
    }

    bool   ee_attach   = false;
    bool   hand_attach = false;
    bool   log_on      = false;
    double jawpos   = 0.1;
    long   cnt      = 0;
    int    last_key = -1;

    // Ultimate is already X-forward/Y-left/Z-up. Only map-to-robot yaw
    // remains; applying the old OpenVR Y-up / tracker-mount transforms here
    // would rotate the wrong axes. The clutch absorbs each mounting rotation.
    const Mat4 T01=T_rot_z(arg.yaw_degrees*M_PI/180.0);
    const auto save_and_capture=[&](int slot,bool save) {
        if (save) {
            for (int k=0;k<3;++k) bias[slot][k]=xyz[slot][k];
            R_bias[slot]=R_out[slot];
        }
        CaptureOrigin(_xyz[slot],R_m_cur[slot],R_bias[slot],offset[slot],R_off[slot]);
    };

    std::printf("[main] running, ESC or Ctrl-C to stop\n");

    using clock_t = std::chrono::steady_clock;
    auto next = clock_t::now() + std::chrono::milliseconds(kLoopPeriodMs);

    const auto begin=clock_t::now();

    while (!g_exit.load()&&!trackers.stopped.load()) {
        if (arg.seconds>0&&std::chrono::duration<double>(clock_t::now()-begin).count()>=arg.seconds) break;
        bool tracking_ready=true, discontinuity=false;
        for (int slot=0;slot<2;++slot) if (active(slot)) {
            samples[slot]=trackers.select(selections[slot]);
            if (!ultimate::usable(samples[slot])) tracking_ready=false;
            if (samples[slot].generation!=attached_generation[slot]) discontinuity=true;
        }
        if (arg.t==2 && samples[0].connected && samples[1].connected && samples[0].id==samples[1].id)
            tracking_ready=false;
        if (ee_attach&&(!tracking_ready||discontinuity)) {
            for (int slot=0;slot<2;++slot) if (active(slot)) save_and_capture(slot,true);
            ee_attach=false;
            std::puts("[ultimate] Tracking lost or relocalized: arm detached. Press a after TRACKING returns.");
        }
        for (int slot=0;slot<2;++slot) if (active(slot)&&ultimate::usable(samples[slot])) {
            const Mat4 T03=MatMul(T01,ultimate::pose_matrix(samples[slot].data));
            for (int k=0;k<3;++k) {
                _xyz[slot][k]=M(T03,k,3);
                xyz[slot][k]=scale[slot]*(_xyz[slot][k]+offset[slot][k])+bias[slot][k];
            }
            R_m_cur[slot]=Mat3FromMat4(T03);
            R_out[slot]=Mat3Mul(R_m_cur[slot],R_off[slot]);
            RotationToEulerXYZ(R_out[slot],rpy[slot][0],rpy[slot][1],rpy[slot][2]);
            RotationToEulerXYZ(R_m_cur[slot],rpy_raw[slot][0],rpy_raw[slot][1],rpy_raw[slot][2]);
        }
        const int key = getch();
        if (key != ERR) {
            last_key = key;
            if      (key == '/')  SendPush(g_slaveCtlSock, "randomize", 9);
            else if (key == 'r') {
                //SendPush(g_slaveCtlSock, "rest", 4);
		if (arg.t == 0 || arg.t == 1) SendPush(g_slaveCtlSock, "rest, home", 10);
                else if (arg.t == 2)          SendPush(g_slaveCtlSock, "rest, home, home", 16);
                ee_attach = false; hand_attach = false;
            }
            else if (key == 27) {
                // ZMQ_LINGER on the control socket guarantees delivery during
                // ShutdownZmq(); no explicit sleep needed.
                SendPush(g_loggerCtlSock, "quit", 4);
                break;
            }
            else if (key == 'z') jawpos = 0.10; //{ jawpos += 0.02; if (jawpos > 0.10) jawpos = 0.10; }
            else if (key == 'x') jawpos = 0.02; //{ jawpos -= 0.02; if (jawpos < 0.0)  jawpos = 0.0;  }
            else if (key == 'q') {
                SendPush(g_slaveCtlSock,  "quit", 4);
                SendPush(g_loggerCtlSock, "quit", 4);
                g_exit.store(true);
            }
            else if (key == 'c') {
                log_on = !log_on;
                if (log_on) SendPush(g_loggerCtlSock, "log-on",  6);
                else        SendPush(g_loggerCtlSock, "log-off", 7);
            }
            else if (key == 'b') {
                hand_attach = !hand_attach;
            }
            else if (key=='1'||key=='2') {
                int slot=key=='1'?0:1;
                if (active(slot)&&samples[slot].connected) {
                    std::string echo="Echo "+std::to_string(samples[slot].id);
                    trackers.SendCommand(echo.c_str());
                }
            }
            else if (key == 'a') {
                if (!ee_attach&&!tracking_ready) {
                    std::puts("[ultimate] Cannot attach: every required tracker must be distinct and TRACKING.");
                } else {
                    for (int slot=0;slot<2;++slot) if (active(slot)) {
                        save_and_capture(slot,ee_attach);
                        attached_generation[slot]=samples[slot].generation;
                    }
                    ee_attach=!ee_attach;
                }
            }
            else if (key == 'h') {
                SendPush(g_slaveCtlSock, "home", 4);
                jawpos = 0.1;
                // robot returned to home: restore bias to per-mode home values,
                // and reset offset so the displayed xyz stays at the new bias.
                // R_bias defaults to identity — replace with the slave's actual
                // EE orientation at home if/when those values are known.
                if (arg.t == 0) {
                    bias[0][0] = 0.30; bias[0][1] =  0.23; bias[0][2] = -0.40;
                    offset[0][0] = -_xyz[0][0]; offset[0][1] = -_xyz[0][1]; offset[0][2] = -_xyz[0][2];
                    R_bias[0] = Mat3Identity();
                    R_off[0]  = Mat3Mul(Mat3Transpose(R_m_cur[0]), R_bias[0]);
                } else if (arg.t == 1) {
                    bias[1][0] = 0.30; bias[1][1] = -0.23; bias[1][2] = -0.40;
                    offset[1][0] = -_xyz[1][0]; offset[1][1] = -_xyz[1][1]; offset[1][2] = -_xyz[1][2];
                    R_bias[1] = Mat3Identity();
                    R_off[1]  = Mat3Mul(Mat3Transpose(R_m_cur[1]), R_bias[1]);
                } else if (arg.t == 2) {
                    bias[0][0] = 0.30; bias[0][1] =  0.23; bias[0][2] = -0.40;
                    bias[1][0] = 0.30; bias[1][1] = -0.23; bias[1][2] = -0.40;
                    offset[0][0] = -_xyz[0][0]; offset[0][1] = -_xyz[0][1]; offset[0][2] = -_xyz[0][2];
                    offset[1][0] = -_xyz[1][0]; offset[1][1] = -_xyz[1][1]; offset[1][2] = -_xyz[1][2];
                    R_bias[0] = Mat3Identity();
                    R_bias[1] = Mat3Identity();
                    R_off[0]  = Mat3Mul(Mat3Transpose(R_m_cur[0]), R_bias[0]);
                    R_off[1]  = Mat3Mul(Mat3Transpose(R_m_cur[1]), R_bias[1]);
                } else if (arg.t == 5) {
                    bias[0][0] = 0.50; bias[0][1] = -0.15; bias[0][2] =  0.30;
                    offset[0][0] = -_xyz[0][0]; offset[0][1] = -_xyz[0][1]; offset[0][2] = -_xyz[0][2];
                    R_bias[0] = Mat3Identity();
                    R_off[0]  = Mat3Mul(Mat3Transpose(R_m_cur[0]), R_bias[0]);
                }
            }
            else if (key == 'i') {
                if (arg.t == 0 || arg.t == 1) SendPush(g_slaveCtlSock, "init, home", 10);
                else if (arg.t == 2)          SendPush(g_slaveCtlSock, "init, home, home", 16);
                ee_attach = false; hand_attach = false;
            }
            // Match the original keyboard behavior: skip send/draw on a key press.
            // for this iteration. ESC handler above already breaks the loop.
            continue;
        }

        // Manus: snapshot + retarget into q[40].
        if (arg.g >= 0) RetargetAll(arg.g, q);

        const std::string command=FormatTeleopCommand(arg.t,arg.g,n_joint,xyz,rpy,q,jawpos,ee_attach,hand_attach);
        if (!command.empty()) {
            SendPush(g_slaveSock,command.data(),command.size());
            SendPush(g_loggerSock,command.data(),command.size());
        }

        DrawStatus(arg.t, ee_attach, hand_attach, log_on, cnt, last_key);
        DrawVive(arg.t, xyz, rpy, _xyz, rpy_raw, jawpos, samples);
        DrawManus(arg.t, n_joint, q);
        DrawLog();
        doupdate();

        std::this_thread::sleep_until(next);
        next += std::chrono::milliseconds(kLoopPeriodMs);
        const auto now = clock_t::now();
        if (next < now) next = now + std::chrono::milliseconds(kLoopPeriodMs);

        ++cnt;
    }

    g_exit.store(true);
    if (manusThread.joinable()) manusThread.join();
    trackers.stop();
    if (loggerPid>0) SendPush(g_loggerCtlSock,"quit",4);
    ShutdownZmq();
    ShutdownUi();
    std::printf("\n[main] bye\n");
    return 0;
}



/*
// Clamp Gosung (-t5) EE position into its dexterous workspace so the
// downstream model.ik in gos.py converges. axis bbox + horizontal radial
// shell around the base z-axis; ranges sized from gos.yml link reach and
// the box.yml pick region plus a small margin.
void ClampGosWorkspace(double xyz[3])
{
    constexpr double X_MIN =  0.20, X_MAX = 0.78;
    constexpr double Y_MIN = -0.45, Y_MAX = 0.35;
    constexpr double Z_MIN =  0.05, Z_MAX = 0.60;
    constexpr double R_MIN =  0.28, R_MAX = 0.75;
    xyz[0] = std::max(X_MIN, std::min(X_MAX, xyz[0]));
    xyz[1] = std::max(Y_MIN, std::min(Y_MAX, xyz[1]));
    xyz[2] = std::max(Z_MIN, std::min(Z_MAX, xyz[2]));
    const double r = std::hypot(xyz[0], xyz[1]);
    if (r < R_MIN) {
        if (r > 1e-6) { const double s = R_MIN / r; xyz[0] *= s; xyz[1] *= s; }
        else          { xyz[0] = R_MIN; xyz[1] = 0.0; }
    } else if (r > R_MAX) {
        const double s = R_MAX / r; xyz[0] *= s; xyz[1] *= s;
    }
}
*/
