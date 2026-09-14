// vhand.cpp - Manus glove only teleop master, for retargeting work.
//
// vmaster couples the glove to a Vive tracker: it initializes OpenVR and quits
// if no tracker is present (or if the tracker count does not match -t), so you
// cannot tune retarget.h without SteamVR and the full rig. vhand drops the arm
// half entirely — glove -> retarget.h -> hand `joint` command, nothing else.
// It shares manus.h and retarget.h with vmaster, so the numbers it prints are
// the same ones vmaster would send.
//
// Pair it with ../hand-run (tact sim showing the hand alone):
//
//     ./hand-run -g 2 -t 2 &        # in the repo root
//     cd vive && ./vhand -g 2 -t 2
//
// Edit retarget.h -> ./build.sh -> restart vhand; the sim keeps running.
//
// The `-t` here must match hand-run's `-t`: it decides how many comma-separated
// parts the command carries, and the slave splits on that. With `-A` the
// message gets a leading "none" arm part instead, which is what kida-run /
// single-run expect — that lets the same binary drive the full robot sim
// without moving the arms.
//
// CLI:
//   -tN   side:    0=left  1=right  2=both              (default 2)
//   -gN   gripper: 0=H9    1=DG5F-M  2=DG5F-S           (default 2)
//   -A    prepend a "none" arm part (drive kida-run / single-run instead)
//   -n    skip Manus glove calibration (use SDK defaults)
//   -e    slave endpoint (default ipc:///dev/shm/default)
//
// Keys:
//   a  attach on/off (stream joints)   h  home    z  zero
//   d  deg/rad display                 p  dump current pose to the log
//   q / ESC  quit

#include "manus.h"
#include "retarget.h"

#include <zmq.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <curses.h>
#include <getopt.h>
#include <unistd.h>

namespace {

constexpr const char* kDefaultEndpoint = "ipc:///dev/shm/default";
constexpr int         kLoopPeriodMs    = 33;    // ~30 Hz, same as vmaster

std::atomic<bool> g_exit{false};

void HandleSignal(int) { g_exit.store(true); }

// ============================================================================
// ncurses UI: status / glove(raw) / hand(retargeted) / log
// ============================================================================
// Same stdout/stderr capture as vmaster, and for the same reason: the Manus SDK
// logs on its own schedule (dongle connect, license dump), and those writes
// would land on top of the panes. Curses is driven through /dev/tty so it keeps
// working after stdout is redirected into the log pipe.

WINDOW* g_winStatus = nullptr;
WINDOW* g_winRaw    = nullptr;
WINDOW* g_winOut    = nullptr;
WINDOW* g_winLog    = nullptr;

FILE*   g_ttyIn  = nullptr;
FILE*   g_ttyOut = nullptr;
SCREEN* g_screen = nullptr;

int g_origStdout  = -1;
int g_origStderr  = -1;
int g_logPipeRead = -1;

std::mutex              g_logMtx;
std::deque<std::string> g_logLines;
constexpr size_t        kMaxLogLines = 200;

std::thread       g_logReader;
std::atomic<bool> g_logReaderStop{false};

void Log(const std::string& s)
{
    std::lock_guard<std::mutex> lk(g_logMtx);
    g_logLines.push_back(s);
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
            if (c == '\n') { Log(acc); acc.clear(); }
            else if (c != '\r') acc.push_back(c);
        }
    }
    if (!acc.empty()) Log(acc);
}

// Rows a 20-value block needs: 8 per row, so 3 rows (h9's 9 values fit in 2).
int ValueRows(int n) { return (n + 7) / 8; }

bool InitUi(int nsides, int njoint)
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

    int rows = 0, cols = 0;
    getmaxyx(stdscr, rows, cols);

    // One label row + its values, per side, plus the box border.
    const int hBlock  = 2 + nsides * (1 + ValueRows(njoint));
    const int hStatus = 3;
    int hRaw = hBlock, hOut = hBlock;
    int hLog = rows - hStatus - hRaw - hOut;
    if (hLog < 3) { hLog = 3; hRaw = hOut = std::max(3, (rows - hStatus - hLog) / 2); }

    g_winStatus = newwin(hStatus, cols, 0, 0);
    g_winRaw    = newwin(hRaw,    cols, hStatus, 0);
    g_winOut    = newwin(hOut,    cols, hStatus + hRaw, 0);
    g_winLog    = newwin(hLog,    cols, hStatus + hRaw + hOut, 0);
    if (!g_winStatus || !g_winRaw || !g_winOut || !g_winLog) return false;

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
    if (g_winRaw)    { delwin(g_winRaw);    g_winRaw    = nullptr; }
    if (g_winOut)    { delwin(g_winOut);    g_winOut    = nullptr; }
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

// One labelled block of `n` values, 8 per row with a gap every 4 (= one finger).
void DrawValues(WINDOW* w, int row, const char* label, const float* v, int n, bool deg)
{
    const float k = deg ? static_cast<float>(180.0 / M_PI) : 1.0f;
    mvwprintw(w, row, 2, "%s", label);
    for (int i = 0; i < n; ++i) {
        const int r = row + 1 + i / 8;
        const int c = 2 + (i % 8) * 8 + ((i % 8) / 4) * 2;
        mvwprintw(w, r, c, "%7.2f", v[i] * k);
    }
}

void DrawStatus(int t, int g, bool attach, bool deg, bool armPart, bool haveL, bool haveR, long cnt, int key)
{
    DrawBorder(g_winStatus, "VHAND (glove only, no vive)");
    char keybuf[16];
    if      (key < 0)                std::snprintf(keybuf, sizeof(keybuf), "-");
    else if (key == 27)              std::snprintf(keybuf, sizeof(keybuf), "ESC");
    else if (key >= 33 && key < 127) std::snprintf(keybuf, sizeof(keybuf), "%c", key);
    else                             std::snprintf(keybuf, sizeof(keybuf), "0x%02X", key);
    mvwprintw(g_winStatus, 1, 2,
              "t:%d  g:%d  attach:%d  arm-part:%d  unit:%s  glove L:%d R:%d  sdk:%d  cnt:%ld  key:%s",
              t, g, attach ? 1 : 0, armPart ? 1 : 0, deg ? "deg" : "rad",
              haveL ? 1 : 0, haveR ? 1 : 0, manus::Connected() ? 1 : 0, cnt, keybuf);
    wnoutrefresh(g_winStatus);
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
        mvwaddnstr(g_winLog, 1 + i, 2, s.c_str(), std::min<int>(s.size(), cols - 4));
    }
    wnoutrefresh(g_winLog);
}

// ============================================================================
// ZMQ
// ============================================================================
// Same two-socket split as vmaster: CONFLATE for the per-frame joint stream
// (stale frames are worthless), plain PUSH for one-shot commands so a 'home'
// can never be overwritten by the next glove frame.

void* g_ctx     = nullptr;
void* g_data    = nullptr;
void* g_control = nullptr;

bool InitOnePush(void*& sock, const char* endpoint, bool conflate)
{
    sock = zmq_socket(g_ctx, ZMQ_PUSH);
    if (!sock) return false;
    int one = 1, zero = 0;
    if (conflate) {
        zmq_setsockopt(sock, ZMQ_IMMEDIATE, &one,  sizeof(one));
        zmq_setsockopt(sock, ZMQ_CONFLATE,  &one,  sizeof(one));
        zmq_setsockopt(sock, ZMQ_LINGER,    &zero, sizeof(zero));
    } else {
        const int linger_ms = 200;
        zmq_setsockopt(sock, ZMQ_LINGER, &linger_ms, sizeof(linger_ms));
    }
    if (zmq_connect(sock, endpoint) != 0) {
        std::fprintf(stderr, "zmq_connect %s failed: %s\n", endpoint, zmq_strerror(zmq_errno()));
        return false;
    }
    return true;
}

bool InitZmq(const char* endpoint)
{
    g_ctx = zmq_ctx_new();
    if (!g_ctx) return false;
    return InitOnePush(g_data, endpoint, true) && InitOnePush(g_control, endpoint, false);
}

void ShutdownZmq()
{
    if (g_data)    zmq_close(g_data);
    if (g_control) zmq_close(g_control);
    if (g_ctx)     zmq_ctx_term(g_ctx);
    g_data = g_control = g_ctx = nullptr;
}

void SendPush(void* sock, const char* data, size_t len)
{
    if (!sock || len == 0) return;
    (void)zmq_send(sock, data, len, ZMQ_DONTWAIT);
}

// ============================================================================
// command formatting
// ============================================================================

struct Args {
    int  t = 2;
    int  g = 2;
    bool armPart = false;
    int  p = 0;          // -pN calibration profile (0 = upload nothing)
    std::string endpoint = kDefaultEndpoint;
};

// Comma-joined parts in slave order. `armPart` prepends the "none" placeholder
// the arm runners expect; hand-run takes the hand parts alone.
std::string Compose(const Args& a, const std::string& left, const std::string& right)
{
    std::string s;
    if (a.armPart) s += "none";
    auto append = [&](const std::string& part) {
        if (!s.empty()) s += ", ";
        s += part;
    };
    if (a.t == 0 || a.t == 2) append(left);
    if (a.t == 1 || a.t == 2) append(right);
    return s;
}

std::string JointPart(const float* q, int n)
{
    char buf[512];
    int  k = std::snprintf(buf, sizeof(buf), "joint");
    for (int i = 0; i < n; ++i) k += std::snprintf(buf + k, sizeof(buf) - k, "%8.3f", q[i]);
    return std::string(buf, k);
}

void SendControl(const Args& a, const char* word)
{
    const std::string s = Compose(a, word, word);
    SendPush(g_control, s.c_str(), s.size());
    Log(std::string("sent: ") + s);
}

// ============================================================================
// CLI
// ============================================================================

void PrintUsage(const char* prog)
{
    std::fprintf(stderr,
        "usage: %s [-tN] [-gN] [-A] [-pN] [-e endpoint]\n"
        "  -tN   side:    0=left  1=right  2=both        (default 2)\n"
        "  -gN   gripper: 0=H9    1=DG5F-M  2=DG5F-S     (default 2)\n"
        "  -A    prepend a \"none\" arm part (drive kida-run / single-run)\n"
        "  -pN   glove calibration profile from calib/pN/ (default 0)\n"
        "        0=upload nothing (keeps what is already in the glove)\n"
        "        1=Lee Donghyuk  2=Choi Taewon  -- see calib/README.md\n"
        "        (this replaces the old -n, which is now the default)\n"
        "  -e    slave endpoint (default %s)\n"
        "\nkeys: a attach  h home  z zero  d deg/rad  p dump pose  q/ESC quit\n",
        prog, kDefaultEndpoint);
}

bool ParseArgs(int argc, char** argv, Args& a)
{
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--help") == 0) { PrintUsage(argv[0]); std::exit(0); }

    opterr = 0;
    int opt;
    while ((opt = getopt(argc, argv, "t:g:e:Ap:h")) != -1) {
        switch (opt) {
            case 't':
            case 'g':
            case 'p': {
                char* end = nullptr;
                const long v = std::strtol(optarg, &end, 10);
                if (end == optarg || *end != '\0') {
                    std::fprintf(stderr, "invalid -%c value: %s\n", opt, optarg);
                    return false;
                }
                (opt == 't' ? a.t : opt == 'g' ? a.g : a.p) = static_cast<int>(v);
                break;
            }
            case 'e': a.endpoint = optarg; break;
            case 'A': a.armPart = true; break;
            case 'h': PrintUsage(argv[0]); std::exit(0);
            default:
                if (optopt) std::fprintf(stderr, "unrecognized or invalid option: -%c\n", optopt);
                PrintUsage(argv[0]);
                return false;
        }
    }
    if (optind < argc) {
        std::fprintf(stderr, "unexpected positional: %s\n", argv[optind]);
        PrintUsage(argv[0]);
        return false;
    }
    if (a.t < 0 || a.t > 2) { std::fprintf(stderr, "-t must be 0, 1 or 2\n"); return false; }
    if (a.g < 0 || a.g > 2) { std::fprintf(stderr, "-g must be 0, 1 or 2\n"); return false; }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    Args arg;
    if (!ParseArgs(argc, argv, arg)) return 64;
    // Validate -pN before anything else touches the terminal or ZMQ: this only
    // stats files, and the upload itself does not happen until a glove connects
    // on the background thread — so without this a typo'd number would sail
    // past and leave the glove on whatever the last run wrote (see manus.h).
    {
        std::string err;
        if (!manus::CheckProfile(arg.p, &err)) {
            std::fprintf(stderr, "[calib] -p%d: %s\n", arg.p, err.c_str());
            return 64;
        }
    }


    std::signal(SIGINT,  HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const int njoint  = (arg.g == 0) ? 9 : 20;
    const int nsides  = (arg.t == 2) ? 2 : 1;

    // UI first: it redirects stdout/stderr into the log pane, and the Manus SDK
    // starts logging within milliseconds of the thread starting. Bring it up
    // after and none of that chatter lands on the raw terminal.
    if (!InitUi(nsides, njoint)) {
        std::fprintf(stderr, "InitUi failed\n");
        ShutdownUi();
        return 1;
    }

    if (!InitZmq(arg.endpoint.c_str())) {
        std::fprintf(stderr, "InitZmq failed\n");
        ShutdownUi();
        ShutdownZmq();
        return 1;
    }

    manus::SetProfile(arg.p);
    std::thread manusThread(manus::ThreadFn, std::cref(g_exit));

    Log("vhand up. 'a' to attach, then move the glove.");
    // p0 inherits whatever the last run wrote into the glove, so retarget gains
    // tuned here may not be against the calibration you think. Say so.
    if (arg.p == 0)
        Log("WARNING: -p0, uploading no profile — the glove keeps its current "
            "calibration (possibly from an earlier run). Raw angles may differ "
            "from vmaster's.");
    else
        Log("[calib] p" + std::to_string(arg.p) + ": calib/p" +
            std::to_string(arg.p) + "/{left,right}.mcal");

    // Retargeted joints, same q[40] layout as vmaster: left 0..19, right 20..39.
    float q[40] = {0};
    float rawL[20] = {0}, rawR[20] = {0};
    bool  haveL = false, haveR = false;

    bool attach   = false;
    bool deg      = false;
    long cnt      = 0;
    int  last_key = -1;

    while (!g_exit.load()) {
        int ch;
        while ((ch = getch()) != ERR) {
            last_key = ch;
            if      (ch == 'q' || ch == 27) g_exit.store(true);
            else if (ch == 'a') { attach = !attach; Log(attach ? "attached" : "detached"); }
            else if (ch == 'h') SendControl(arg, "home");
            else if (ch == 'z') SendControl(arg, "zero");
            else if (ch == 'd') deg = !deg;
            else if (ch == 'p') {
                // Dump in the units handpose.py stores (deg), so a pose you like
                // can be pasted straight into its POSES table.
                for (int s = 0; s < 2; ++s) {
                    if (arg.t != 2 && arg.t != s) continue;
                    std::string line = (s == 0) ? "left : " : "right: ";
                    char b[16];
                    for (int i = 0; i < njoint; ++i) {
                        std::snprintf(b, sizeof(b), "%.0f%s", q[20 * s + i] * 180.0 / M_PI, i + 1 < njoint ? ", " : "");
                        line += b;
                    }
                    Log(line);
                }
            }
        }
        if (g_exit.load()) break;

        manus::Snapshot(rawL, rawR, haveL, haveR);
        if (arg.g == 0) {
            if (haveL) RetargetSide_h9(rawL, q);
            if (haveR) RetargetSide_h9(rawR, q + 20);
        } else if (arg.g == 1) {
            if (haveL) RetargetSide_dg5f('L', rawL, q);
            if (haveR) RetargetSide_dg5f('R', rawR, q + 20);
        } else {
            if (haveL) RetargetSide_dg5s('L', rawL, q);
            if (haveR) RetargetSide_dg5s('R', rawR, q + 20);
        }

        // Only stream a side whose glove has actually reported. A missing glove
        // would otherwise send a constant zero pose and look like a retarget bug.
        const bool sendL = haveL && (arg.t == 0 || arg.t == 2);
        const bool sendR = haveR && (arg.t == 1 || arg.t == 2);
        if (attach && (sendL || sendR)) {
            const std::string cmd = Compose(arg,
                                            sendL ? JointPart(q,      njoint) : "none",
                                            sendR ? JointPart(q + 20, njoint) : "none");
            SendPush(g_data, cmd.c_str(), cmd.size());
        }

        DrawStatus(arg.t, arg.g, attach, deg, arg.armPart, haveL, haveR, cnt, last_key);

        DrawBorder(g_winRaw, deg ? "GLOVE RAW (deg)" : "GLOVE RAW (rad)");
        DrawBorder(g_winOut, deg ? "RETARGETED (deg)" : "RETARGETED (rad)");
        int row = 1;
        const int step = 1 + ValueRows(njoint);
        if (arg.t == 0 || arg.t == 2) {
            DrawValues(g_winRaw, row, haveL ? "L:" : "L: (no glove)", rawL, 20,     deg);
            DrawValues(g_winOut, row, "L:",                           q,    njoint, deg);
            row += step;
        }
        if (arg.t == 1 || arg.t == 2) {
            DrawValues(g_winRaw, row, haveR ? "R:" : "R: (no glove)", rawR,   20,     deg);
            DrawValues(g_winOut, row, "R:",                           q + 20, njoint, deg);
        }
        wnoutrefresh(g_winRaw);
        wnoutrefresh(g_winOut);
        DrawLog();
        doupdate();

        ++cnt;
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMs));
    }

    ShutdownUi();
    g_exit.store(true);
    if (manusThread.joinable()) manusThread.join();
    ShutdownZmq();
    std::printf("vhand: bye\n");
    return 0;
}
