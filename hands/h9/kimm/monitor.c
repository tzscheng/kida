/*
 * monitor.c -- h9 hand motor / encoder check tool with a curses screen
 *
 * Drives one motor at a fixed PWM duty and shows the raw encoders and the
 * joint angles of the whole hand on a curses screen. Used to check the
 * motors / encoders and to read the zero values for conf.txt. It talks to
 * the hand over CAN directly, so do NOT run it together with the runner on
 * the same hand.
 *
 * Build:
 *   gcc -W -Wall -O2 -o monitor monitor.c -lncurses -lm
 *   gcc -W -Wall -O2 -DMONITOR_SIM -o monitor-sim monitor.c -lncurses -lm
 *        (no hardware, software motor model; for trying the screen)
 *
 * Run:
 *   ./monitor <hand name> [-c <can ch>] [-f <conf file>]
 *   e.g.  ./monitor L2      or   ./monitor L2 -c 1
 *
 *   <hand name>  entry name in the configuration file (required; "-n <name>"
 *                is accepted too)
 *   -c  SocketCAN channel number (can<N>)                   (default 0)
 *   -f  configuration file                                  (default conf.txt)
 *
 * Keys:
 *   1..9    select the joint to drive, its level goes back to 0
 *   [ / ]   level -1 / +1 (-8..8, one level = 0x1000 of PWM duty)
 *   space   level 0 (motor free)
 *   q       quit (Ctrl-C too); zero duty is sent before leaving
 *
 * Screen: per joint the PWM duty being sent, the raw 12-bit encoder (MAG),
 * the encoder count from the zero pose (ZPOS) and the joint angle in degree
 * (DEG). The MAG row read at the zero pose is what goes into conf.txt.
 *
 * Joint order: thumb1 thumb2 thumb3 index1 index2 middle1 middle2 ring1 ring2.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <curses.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define N          9
#define LEVEL_MAX  8                 /* level range -8..8                     */
#define LEVEL_STEP 0x1000            /* PWM duty per level                    */
#define LOOP_SLEEP 2000              /* [us] pause per cycle                  */
#define REPLY_ID_OFFSET 1            /* reply CAN ID = request CAN ID + 1     */
#define REPLY_TIMEOUT 0.5            /* [s] wait for a board's reply          */
#define DRAW_PERIOD 0.05             /* screen refresh period [s]             */
#define LOG_MAX    200
#define LOG_LEN    160

static const char *joint_name[N] = {"thumb1", "thumb2", "thumb3", "index1", "index2",
                                    "middle1", "middle2", "ring1", "ring2"};

static volatile sig_atomic_t flag;   /* 0: run, -1: quit                      */
static long cnt;
static int  joint = 0;               /* selected joint (0..8)                 */
static int  L = 0;                   /* level of the selected joint           */

static char   name[64];
static char   type;
static int    can_ch;
static int    send_id[3];
static int    m_dir[N], e_dir[N];    /* motor / encoder sign (motor order)    */
static int    m_idx[N], e_idx[N];    /* joint index -> motor / encoder        */
static double pulley_ratio[2], couple_ratio[2];

static int    dpos[N], dpos_off[N], dpos_old[N], turn[N], zpos[N], duty[N];
static double m[N], q[N], deg[N];

/* CAN health, shown on the screen instead of perror() scribbling over it */
static int  can_ok = 1;
static long can_err;
static long can_stale;               /* late / foreign frames thrown away      */
static char can_msg[96] = "";

static double now_sec(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + 1.0e-9*t.tv_nsec;
}

/* ------------------------------------------------------------------------ */
/* log pane                                                                  */
/* ------------------------------------------------------------------------ */
static char logbuf[LOG_MAX][LOG_LEN];
static int  log_head, log_len;

static void logmsg(const char *fmt, ...)
{
    char *s = logbuf[(log_head + log_len) % LOG_MAX];
    time_t t = time(NULL);
    struct tm tm;
    va_list ap;

    localtime_r(&t, &tm);
    size_t k = strftime(s, LOG_LEN, "%H:%M:%S ", &tm);
    va_start(ap, fmt);
    vsnprintf(s + k, LOG_LEN - k, fmt, ap);
    va_end(ap);

    if (log_len < LOG_MAX) log_len++;
    else log_head = (log_head + 1) % LOG_MAX;
}

/* ------------------------------------------------------------------------ */
/* CAN (classic CAN, 16-bit big-endian payload words)                        */
/* ------------------------------------------------------------------------ */
#ifdef MONITOR_SIM
/* Simulation build: no CAN bus, each motor is a pure integrator whose encoder
 * advances in proportion to the PWM duty. Only for trying the screen. */
static int can_fd = -1;

static int can_open(int ch) { (void)ch; return 0; }

static int can_exchange(int board, int *d)
{
    static double pos[N];
    static int started;
    if (!started) { for (int i = 0; i < N; i++) pos[i] = dpos_off[i] + 100.0*e_dir[i]; started = 1; }
    for (int j = 0; j < N; j++) {                 /* joint j: motor m_idx[j] -> encoder e_idx[j] */
        int mi = m_idx[j], ei = e_idx[j];
        if (mi/3 != board) continue;
        double v = (double)(d[mi%3] - 0x8000)/(double)0x8000;
        pos[ei] += 10.0*m_dir[mi]*e_dir[ei]*v;
    }
    for (int j = 0; j < 3; j++) d[j] = ((int)lround(pos[3*board + j]) % 4096 + 4096) % 4096;
    return 0;
}
#else
static int can_fd = -1;

static int can_open(int ch)
{
    int fd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct timeval tv = {0, 500000};     /* 0.5 s receive timeout */
    int priority = 5;

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("CAN socket"); return -1; }

    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "can%d", ch);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror(ifr.ifr_name); close(fd); return -1; }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("CAN bind"); close(fd); return -1; }
    return fd;
}

/* one board: 3 duty words (+ 0x0001) out, 3 raw encoder words in (into d).
 * 0 on success; on failure d is left untouched and can_msg says why. */
static int can_exchange(int board, int *d)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));
    frame.can_id  = (canid_t)send_id[board];
    frame.can_dlc = 8;
    for (int i = 0; i < 3; i++) {
        frame.data[2*i]   = (unsigned char)((d[i] >> 8) & 0xFF);
        frame.data[2*i+1] = (unsigned char)(d[i] & 0xFF);
    }
    frame.data[6] = 0x00; frame.data[7] = 0x01;

    /* Throw away whatever is already waiting (a reply that arrived after its
     * timeout), so that the answer read below is the one to this request. */
    struct can_frame old;
    while (recv(can_fd, &old, sizeof(old), MSG_DONTWAIT) > 0) can_stale++;

    if (write(can_fd, &frame, sizeof(frame)) != sizeof(frame)) {
        snprintf(can_msg, sizeof(can_msg), "write to board 0x%x failed: %s", send_id[board], strerror(errno));
        return -1;
    }
    /* A board answers with its request ID + 1 (0x10 -> 0x11 ...). Wait for
     * exactly that ID; a frame with another ID is some other board's late
     * reply and must not be taken as this board's encoders. */
    canid_t want = (canid_t)(send_id[board] + REPLY_ID_OFFSET);
    double  t_end = now_sec() + REPLY_TIMEOUT;
    for (;;) {
        int ret = (int)read(can_fd, &frame, sizeof(frame));
        if (ret == (int)sizeof(frame) && frame.can_id == want) break;
        if (ret == (int)sizeof(frame)) can_stale++;
        if (ret <= 0 || now_sec() >= t_end) {
            int hard = (ret < 0 && errno != EAGAIN && errno != EINTR);
            snprintf(can_msg, sizeof(can_msg), "no reply 0x%x from board 0x%x%s%s", (unsigned)want, send_id[board],
                     hard ? ": " : "", hard ? strerror(errno) : "");
            return -1;
        }
    }
    for (int i = 0; i < 3; i++) d[i] = ((int)frame.data[2*i] << 8) | (int)frame.data[2*i+1];
    return 0;
}
#endif

/* ------------------------------------------------------------------------ */
/* hand model                                                                */
/* ------------------------------------------------------------------------ */
/* motor space position -> joint space position (both in motor order) */
static void m2q(const double *m, double *q)
{
    q[0] = pulley_ratio[0]*m[0];
    q[1] = pulley_ratio[0]*m[1] + couple_ratio[0]*q[0];
    q[2] = pulley_ratio[0]*m[2];
    q[3] = pulley_ratio[0]*m[3] + couple_ratio[0]*q[2];
    q[4] = pulley_ratio[0]*m[4];
    q[5] = pulley_ratio[0]*m[5] + couple_ratio[0]*q[4];
    q[6] = pulley_ratio[1]*m[6];                                              /* thumb CMC */
    q[7] = pulley_ratio[0]*m[7] + couple_ratio[1]*q[6];                       /* thumb MCP */
    q[8] = pulley_ratio[0]*m[8] + couple_ratio[1]*q[6] + couple_ratio[0]*q[7];/* thumb IP  */
}

static int load_conf(const char *conf)
{
    char buf[256];
    int  zero[N], found = 0;

    FILE *fp = fopen(conf, "r");
    if (fp == NULL) { fprintf(stderr, "cannot open config file '%s'\n", conf); return -1; }

    while (fscanf(fp, " %255s", buf) == 1) {
        if (strcmp(buf, name) != 0) continue;
        int ret = fscanf(fp, " %c %lf %lf %lf %lf %d %d %d %d %d %d %d %d %d",
                         &type, &pulley_ratio[0], &pulley_ratio[1], &couple_ratio[0], &couple_ratio[1],
                         &zero[0], &zero[1], &zero[2], &zero[3], &zero[4],
                         &zero[5], &zero[6], &zero[7], &zero[8]);
        if (ret != 14) { fprintf(stderr, "config file error at entry '%s'\n", name); fclose(fp); return -1; }
        found = 1;
        break;
    }
    fclose(fp);
    if (!found) { fprintf(stderr, "no entry '%s' in '%s'\n", name, conf); return -1; }

    static const int dir_m[N] = {-1, +1, -1, +1, -1, +1, -1, -1, -1};
    if (type == 'R') {
        static const int dir_e[N] = {+1, -1, +1, -1, +1, -1, -1, -1, -1};
        static const int idx_m[N] = {6, 7, 8, 0, 1, 2, 3, 4, 5};
        static const int idx_e[N] = {6, 7, 8, 0, 1, 2, 3, 4, 5};
        memcpy(m_dir, dir_m, sizeof(dir_m)); memcpy(e_dir, dir_e, sizeof(dir_e));
        memcpy(m_idx, idx_m, sizeof(idx_m)); memcpy(e_idx, idx_e, sizeof(idx_e));
        send_id[0] = 0x10; send_id[1] = 0x20; send_id[2] = 0x30;
    } else if (type == 'L') {
        static const int dir_e[N] = {+1, -1, +1, -1, +1, -1, +1, +1, +1};
        static const int idx_m[N] = {8, 7, 6, 4, 5, 2, 3, 0, 1};
        static const int idx_e[N] = {6, 7, 8, 4, 5, 2, 3, 0, 1};
        memcpy(m_dir, dir_m, sizeof(dir_m)); memcpy(e_dir, dir_e, sizeof(dir_e));
        memcpy(m_idx, idx_m, sizeof(idx_m)); memcpy(e_idx, idx_e, sizeof(idx_e));
        send_id[0] = 0x10; send_id[1] = 0x20; send_id[2] = 0x40;
    } else {
        fprintf(stderr, "type '%c' is not supported (L or R)\n", type);
        return -1;
    }

    /* zero offsets are given in joint order in conf.txt; store in motor order */
    for (int i = 0; i < N; i++) dpos_off[e_idx[i]] = zero[i];
    return 0;
}

/* one check cycle: send the duty of every board, read the encoders back */
static void cycle(void)
{
    int tmp[3], fail = 0;

    for (int i = 0; i < N; i++) duty[i] = 0x8000;
    duty[m_idx[joint]] += m_dir[m_idx[joint]]*L*LEVEL_STEP;
    for (int i = 0; i < N; i++) {
        if (duty[i] > 0xFFFF) duty[i] = 0xFFFF;
        else if (duty[i] < 0x0000) duty[i] = 0x0000;
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) tmp[j] = duty[3*i + j];
        if (can_exchange(i, tmp) < 0) { fail = 1; can_err++; continue; }   /* keep the last reading */
        for (int j = 0; j < 3; j++) dpos[3*i + j] = tmp[j];
    }

    if (fail && can_ok) logmsg("CAN error: %s", can_msg);
    if (!fail && !can_ok) logmsg("CAN recovered");
    can_ok = !fail;

    if (cnt > 0) {
        for (int i = 0; i < N; i++) {
            if (dpos[i] - dpos_old[i] > 2048) turn[i] -= 1;
            else if (dpos[i] - dpos_old[i] < -2048) turn[i] += 1;
        }
    }

    for (int i = 0; i < N; i++) zpos[i] = e_dir[i]*(dpos[i] + turn[i]*4096 - dpos_off[i]);  /* 0 = zero pose */
    for (int i = 0; i < N; i++) m[i] = M_PI*((double)zpos[i]/2048.0);
    m2q(m, q);
    for (int i = 0; i < N; i++) deg[i] = 180.0*q[i]/M_PI;

    for (int i = 0; i < N; i++) dpos_old[i] = dpos[i];
    cnt++;
}

/* release every motor */
static void zero_duty(void)
{
    int tmp[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) tmp[j] = 0x8000;
        can_exchange(i, tmp);
    }
}

/* ------------------------------------------------------------------------ */
/* screen                                                                    */
/* ------------------------------------------------------------------------ */
#define LABEL_W 8
#define COL_W   8

/* text at (y, x), cut at the right edge; pad != 0 fills the rest of the row */
static void put(int y, int x, int attr, int pad, const char *fmt, ...)
{
    char s[512];
    int H, W;
    va_list ap;

    getmaxyx(stdscr, H, W);
    if (y < 0 || y >= H || x >= W - 1) return;
    va_start(ap, fmt);
    vsnprintf(s, sizeof(s), fmt, ap);
    va_end(ap);

    int room = W - 1 - x, len = (int)strlen(s);
    if (len > room) len = room;
    attrset(attr);
    mvaddnstr(y, x, s, len);
    if (pad) for (int i = len; i < room; i++) addch(' ');
    attrset(A_NORMAL);
}

static void draw(double hz)
{
    int H, W, y = 0;
    getmaxyx(stdscr, H, W);
    erase();

#ifdef MONITOR_SIM
    put(y++, 0, A_REVERSE, 1, " monitor   hand %s (%c)   SIMULATION, no CAN bus ", name, type);
#else
    put(y++, 0, A_REVERSE, 1, " monitor   hand %s (%c)   can%d   boards 0x%x 0x%x 0x%x ",
        name, type, can_ch, send_id[0], send_id[1], send_id[2]);
#endif

    /* how to use, always on screen */
    put(y++, 0, 0, 0, "usage   1-9    select the joint to drive (level back to 0)");
    put(y++, 0, 0, 0, "        [ ]    level -1 / +1   (-%d..+%d, one level = 0x%x of PWM duty)", LEVEL_MAX, LEVEL_MAX, LEVEL_STEP);
    put(y++, 0, 0, 0, "        space  level 0 = motor free");
    put(y++, 0, 0, 0, "        q      quit (motors released)");
    put(y++, 0, 0, 0, "        MAG row read at the zero pose = the 9 zero values of conf.txt");
    y++;

    /* joint table: one column per joint, the selected one highlighted */
    put(y, 0, A_BOLD, 0, "%-*s", LABEL_W, "key");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE|A_BOLD : A_BOLD, 0, "%*d", COL_W, i + 1);
    y++;
    put(y, 0, A_BOLD, 0, "%-*s", LABEL_W, "joint");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE|A_BOLD : A_BOLD, 0, "%*s", COL_W, joint_name[i]);
    y++;
    put(y, 0, 0, 0, "%-*s", LABEL_W, "DUTY");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE : 0, 0, "%*x", COL_W, duty[m_idx[i]]);
    y++;
    put(y, 0, 0, 0, "%-*s", LABEL_W, "MAG");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE : 0, 0, "%*d", COL_W, dpos[e_idx[i]]);
    y++;
    put(y, 0, 0, 0, "%-*s", LABEL_W, "ZPOS");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE : 0, 0, "%*d", COL_W, zpos[e_idx[i]]);
    y++;
    put(y, 0, 0, 0, "%-*s", LABEL_W, "DEG");
    for (int i = 0; i < N; i++) put(y, LABEL_W + COL_W*i, i == joint ? A_REVERSE : 0, 0, "%*.1f", COL_W, deg[e_idx[i]]);
    y += 2;

    /* level gauge of the selected joint: -8 ...|... +8 */
    char bar[2*LEVEL_MAX + 2];
    for (int i = 0; i <= 2*LEVEL_MAX; i++) {
        int v = i - LEVEL_MAX;
        bar[i] = (v == 0) ? '|' : ((L > 0 && v > 0 && v <= L) || (L < 0 && v < 0 && v >= L)) ? '#' : '-';
    }
    bar[2*LEVEL_MAX + 1] = '\0';
    put(y++, 0, L ? A_BOLD : 0, 0, "drive   joint %d (%s)   level %+d  [%s]   duty %04x%s",
        joint + 1, joint_name[joint], L, bar, duty[m_idx[joint]], L ? "" : "   (free)");
    put(y, 0, 0, 0, "loop    %7.1f Hz   count %-10ld", hz, cnt);
    if (can_ok) put(y, 40, 0, 0, "CAN OK   (errors %ld, stray frames dropped %ld)", can_err, can_stale);
    else {
        put(y, 40, A_REVERSE|A_BOLD, 0, " CAN ERROR ");
        put(y, 52, A_BOLD, 0, "%s   (errors %ld)", can_msg, can_err);
    }
    y++;

    for (int i = 0; i < W - 1; i++) mvaddch(y, i, '-');
    y++;
    int nlog = H - y;
    if (nlog > log_len) nlog = log_len;
    for (int i = log_len - nlog; i < log_len && nlog > 0; i++) put(y++, 0, 0, 0, "%s", logbuf[(log_head + i) % LOG_MAX]);

    refresh();
}

/* key handling; returns 1 when something changed */
static int poll_keys(void)
{
    int k, changed = 0;

    while ((k = getch()) != ERR) {
        int j0 = joint, L0 = L;
        changed = 1;                                  /* KEY_RESIZE included */
        if (k >= '1' && k <= '9') { joint = k - '1'; L = 0; }
        else if (k == '[') L -= 1;
        else if (k == ']') L += 1;
        else if (k == ' ') L = 0;
        else if (k == 'q') flag = -1;

        if (L > LEVEL_MAX) L = LEVEL_MAX;
        else if (L < -LEVEL_MAX) L = -LEVEL_MAX;

        if (joint != j0) logmsg("joint %d (%s) selected, level 0", joint + 1, joint_name[joint]);
        else if (L != L0) logmsg("joint %d (%s) level %+d", joint + 1, joint_name[joint], L);
    }
    return changed;
}

static void on_signal(int sig) { (void)sig; flag = -1; }

static void usage(const char *prog)
{
    fprintf(stderr, "usage: %s <hand name> [-c <can ch>] [-f <conf file>]\n", prog);
}

int main(int argc, char *argv[])
{
    int opt;
    const char *conf = "conf.txt", *hand = NULL;

    while ((opt = getopt(argc, argv, "c:n:f:h")) != -1) {
        switch (opt) {
        case 'c': can_ch = atoi(optarg); break;
        case 'n': hand = optarg; break;               /* same as the bare argument */
        case 'f': conf = optarg; break;
        default:  usage(argv[0]); return 1;
        }
    }
    if (optind < argc) hand = argv[optind++];
    if (hand == NULL || optind < argc) { usage(argv[0]); return 1; }
    snprintf(name, sizeof(name), "%s", hand);

    if (load_conf(conf) < 0) return 1;

    can_fd = can_open(can_ch);
    if (can_fd < 0) { fprintf(stderr, "CAN initializing error (can%d)\n", can_ch); return 1; }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    idlok(stdscr, FALSE); idcok(stdscr, FALSE);       /* plain overwrite */

    logmsg("start: hand %s (%c)   pulley %.3f %.3f   couple %.2f %.2f",
           name, type, pulley_ratio[0], pulley_ratio[1], couple_ratio[0], couple_ratio[1]);

    double t_draw = 0.0, t_hz = now_sec(), hz = 0.0;
    long   cnt_hz = 0;

    while (flag == 0) {
        int dirty = poll_keys();
        cycle();
        usleep(LOOP_SLEEP);

        double now = now_sec();
        if (now - t_hz >= 0.5) { hz = (double)(cnt - cnt_hz)/(now - t_hz); cnt_hz = cnt; t_hz = now; }
        if (dirty || now - t_draw >= DRAW_PERIOD) { draw(hz); t_draw = now; }
    }

    endwin();

    zero_duty();
    usleep(10000);
#ifndef MONITOR_SIM
    if (close(can_fd) < 0) perror("Close");
#endif

    return 0;
}
