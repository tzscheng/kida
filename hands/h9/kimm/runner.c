/*
 * runner.c -- Hand Runner (UDP command <-> CAN hand bridge)
 *
 * The user program never talks to the hand firmware directly. It sends
 * string commands to this runner over UDP and receives the hand status back
 * over UDP; the runner owns the CAN bus, the PWM duty loop and the
 * configuration file (conf.txt). One runner process drives one hand; for N
 * hands start N runners on different UDP ports.
 *
 * Build:
 *   gcc -W -Wall -O2 -o runner runner.c -lm
 *   gcc -W -Wall -O2 -DRUNNER_SIM -o runner-sim runner.c -lm   (no hardware,
 *        software motor model; for testing the UDP protocol / control loop)
 *
 * Run:
 *   ./runner -p <udp port> -n <hand name> [-c <can ch>] [-f <conf file>]
 *            [-m <model>] [-s <status port>] [-v]
 *   e.g.  ./runner -p 10000 -n L1
 *
 *   -p  UDP port this runner listens on for commands        (required)
 *   -n  hand name = entry name in the configuration file    (required)
 *   -c  SocketCAN channel number (can<N>)                   (default 0)
 *   -f  configuration file                                  (default conf.txt)
 *   -m  hand model                                          (default h9)
 *   -s  UDP port the status is sent to on 127.0.0.1         (default 24000)
 *   -v  verbose: print joint angles 10 times per second
 *
 * UDP command protocol (user program -> runner), one ASCII string per datagram,
 * "head tail" structure:
 *   jointpos d1 ... dN   target joint angles in degree (N = number of joints)
 *                        e.g. "jointpos 10 20 30.0 0.0 -10 0 -25.8 0 30"
 *   home                 move to the home pose
 *   zero                 move to the zero pose
 *   quit                 terminate the runner
 *
 * UDP status protocol (runner -> user program), sent at 100 Hz to
 * 127.0.0.1:24000 independently of the commands:
 *   "<hand name> d1 ... dN"   current joint angles in degree
 *   e.g. "L1 10 20 30 0 10 0 20 0 30"
 *
 * Hand model support: each model is a hand_driver_t (init / exchange / finish
 * plus its joint count, poses and gains). Only the h9 hand (9 DOF: thumb 3,
 * index 2, middle 2, ring 2) is implemented today; add a new driver block and
 * register it in the drivers[] table to support another hand.
 *
 * h9 joint order (both commands and status): thumb1 thumb2 thumb3 index1
 * index2 middle1 middle2 ring1 ring2. Positive = flexion.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAX_JOINT        16          /* upper bound over all supported hands  */
#define DT               0.004       /* control period [s] (250 Hz)           */
#define STATUS_PERIOD    0.01        /* status send period [s] (100 Hz)       */
#define STATUS_ADDR      "127.0.0.1"
#define STATUS_PORT      24000
#define CMD_MAX          1024

/* ------------------------------------------------------------------------ */
/* hand driver interface                                                     */
/* ------------------------------------------------------------------------ */
typedef struct {
    const char   *model;      /* model name used by -m                        */
    int           n_joint;
    const double *home_deg;   /* home pose [deg]                              */
    const double *q_min_deg;  /* command clamp [deg]                          */
    const double *q_max_deg;
    double        kp, kd;     /* joint PD gains: tau = kp*(q_ref-q) - kd*qd   */
                              /* (q in rad, tau normalized to -1..1)          */
    int           pose_ticks; /* duration of home/zero motion [ticks]         */

    /* open the hand, read the initial joint position q0 [rad]; 0 on success  */
    int  (*init)(const char *name, const char *conf, int can_ch, double *q0);
    /* send normalized motor input tau (-1..1) and read joint position q [rad]
     * and velocity qd [rad/s]; 0 on success, -1 on bus failure               */
    int  (*exchange)(const double *tau, double *q, double *qd);
    /* release the motors and close the bus                                   */
    void (*finish)(void);
} hand_driver_t;

/* ------------------------------------------------------------------------ */
/* SocketCAN helpers (classic CAN, 16-bit big-endian payload words)          */
/* ------------------------------------------------------------------------ */
#ifndef RUNNER_SIM
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

static int can_send_u16s(int fd, int id, const int *d, int n)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = (canid_t)id;
    frame.can_dlc = (unsigned char)(2 * n);
    for (int i = 0; i < n; i++) {
        frame.data[2*i]   = (unsigned char)((d[i] >> 8) & 0xFF);
        frame.data[2*i+1] = (unsigned char)(d[i] & 0xFF);
    }
    if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) { perror("CAN write"); return -1; }
    return 0;
}

/* returns number of bytes read (>0), 0/-1 on timeout or error */
static int can_recv_u16s(int fd, int *id, int *d, int n)
{
    struct can_frame frame;
    int ret = (int)read(fd, &frame, sizeof(frame));
    if (ret <= 0) return ret;
    *id = (int)frame.can_id;
    for (int i = 0; i < n; i++) d[i] = ((int)frame.data[2*i] << 8) | (int)frame.data[2*i+1];
    return ret;
}
#endif /* !RUNNER_SIM */

/* ------------------------------------------------------------------------ */
/* h9 hand driver                                                            */
/*                                                                           */
/* 9 DOF tendon-driven hand, 9 DC motors on 3 CAN motor boards (3 motors     */
/* each). Every control tick one frame per board is sent (3 x PWM duty       */
/* 0x0000..0xFFFF, 0x8000 = zero, + 0x0001) and the board answers with the   */
/* 3 raw 12-bit magnetic encoder readings (0..4095, one motor turn).         */
/*                                                                           */
/* conf.txt line:                                                            */
/*   name type pulley_thumb pulley_finger couple0 couple1 zero1 ... zero9    */
/*   e.g. "L2  L  0.422 0.250  0.75 0.75  590 1516 1277 1326 2884 ..."       */
/* The 9 zero values are the raw encoder readings at the zero pose, in joint */
/* order (thumb1..3 index1..2 middle1..2 ring1..2).                          */
/* ------------------------------------------------------------------------ */
#define H9_N 9

static const double h9_home_deg[H9_N] = {40.1, 40.1, 40.1,  17.2, 17.2,  28.6, 28.6,  40.1, 40.1}; /* = h9.py home (rad) */
static const double h9_qmin_deg[H9_N] = {-60, -60, -60,  -60, -60,  -60, -60,  -60, -60};
static const double h9_qmax_deg[H9_N] = {120, 120, 120,  120, 120,  120, 120,  120, 120};

static struct {
    int    fd;
    char   type;              /* 'L' or 'R' */
    int    send_id[3];
    int    m_dir[H9_N], e_dir[H9_N];      /* motor / encoder sign            */
    int    m_idx[H9_N], e_idx[H9_N];      /* joint index -> motor / encoder  */
    double pulley_ratio[2], couple_ratio[2];
    int    dpos[H9_N];        /* raw encoder (0..4095), motor order           */
    int    dpos_off[H9_N];    /* raw encoder at zero pose, motor order        */
    int    dpos_old[H9_N];
    int    turn[H9_N];        /* multi-turn counter                           */
    double q_old[H9_N];       /* joint order, for velocity                    */
    long   cnt;
} h9;

/* motor space position m [rad] -> joint space position q [rad] (motor order) */
static void h9_m2q(const double *m, double *q)
{
    const double *pr = h9.pulley_ratio, *cr = h9.couple_ratio;
    q[0] = pr[0]*m[0];
    q[1] = pr[0]*m[1] + cr[0]*q[0];
    q[2] = pr[0]*m[2];
    q[3] = pr[0]*m[3] + cr[0]*q[2];
    q[4] = pr[0]*m[4];
    q[5] = pr[0]*m[5] + cr[0]*q[4];
    q[6] = pr[1]*m[6];                               /* thumb CMC */
    q[7] = pr[0]*m[7] + cr[1]*q[6];                  /* thumb MCP */
    q[8] = pr[0]*m[8] + cr[1]*q[6] + cr[0]*q[7];     /* thumb IP  */
}

#ifdef RUNNER_SIM
/* Simulation build (gcc -DRUNNER_SIM ...): no CAN bus, each motor is a pure
 * integrator whose encoder advances in proportion to the PWM duty. Only for
 * testing the UDP protocol and the control loop without hardware. */
static int h9_can_exchange(const int *duty)
{
    static double pos[H9_N];
    static int started;
    if (!started) { for (int i = 0; i < H9_N; i++) pos[i] = h9.dpos_off[i] + 100.0*h9.e_dir[i]; started = 1; }
    for (int j = 0; j < H9_N; j++) {                    /* joint j: motor m_idx[j] -> encoder e_idx[j] */
        int mi = h9.m_idx[j], ei = h9.e_idx[j];
        double v = (double)(duty[mi] - 0x8000)/(double)0x8000;        /* -1..1 */
        pos[ei] += 30.0*h9.m_dir[mi]*h9.e_dir[ei]*v;                  /* counts per tick at full duty */
    }
    for (int i = 0; i < H9_N; i++) h9.dpos[i] = ((int)lround(pos[i]) % 4096 + 4096) % 4096;
    return 0;
}
#else
/* one CAN round trip: 3 duty frames out, 3 encoder frames in */
static int h9_can_exchange(const int *duty)
{
    int tmp[4], recv_id;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) tmp[j] = duty[3*i + j];
        tmp[3] = 0x0001;
        if (can_send_u16s(h9.fd, h9.send_id[i], tmp, 4) < 0) return -1;
        int ret = can_recv_u16s(h9.fd, &recv_id, tmp, 3);
        if (ret <= 0) {
            fprintf(stderr, "h9: CAN receive failed (board %d, id 0x%x)\n", i, h9.send_id[i]);
            return -1;
        }
        for (int j = 0; j < 3; j++) h9.dpos[3*i + j] = tmp[j];
    }
    return 0;
}
#endif

/* raw encoder -> joint position [rad] in joint order */
static void h9_observe(double *q, double *qd, int update_turn, int update_vel)
{
    int zpos[H9_N];
    double m[H9_N], qm[H9_N];

    if (update_turn) {
        for (int i = 0; i < H9_N; i++) {
            if      (h9.dpos[i] - h9.dpos_old[i] >  2048) h9.turn[i] -= 1;
            else if (h9.dpos[i] - h9.dpos_old[i] < -2048) h9.turn[i] += 1;
        }
    }
    for (int i = 0; i < H9_N; i++) zpos[i] = h9.e_dir[i]*(h9.dpos[i] + h9.turn[i]*4096 - h9.dpos_off[i]);
    for (int i = 0; i < H9_N; i++) m[i] = M_PI*((double)zpos[i]/2048.0);
    h9_m2q(m, qm);

    for (int i = 0; i < H9_N; i++) {
        q[i]  = qm[h9.e_idx[i]];
        qd[i] = update_vel ? (q[i] - h9.q_old[i])/DT : 0.0;
    }
    for (int i = 0; i < H9_N; i++) { h9.dpos_old[i] = h9.dpos[i]; h9.q_old[i] = q[i]; }
}

static int h9_init(const char *name, const char *conf, int can_ch, double *q0)
{
    char buf[256];
    int  zero[H9_N], found = 0;
    double qd[H9_N];

    FILE *fp = fopen(conf, "r");
    if (fp == NULL) { fprintf(stderr, "h9: cannot open config file '%s'\n", conf); return -1; }

    while (fscanf(fp, " %255s", buf) == 1) {
        if (strcmp(buf, name) != 0) continue;
        int ret = fscanf(fp, " %c %lf %lf %lf %lf %d %d %d %d %d %d %d %d %d",
                         &h9.type, &h9.pulley_ratio[0], &h9.pulley_ratio[1],
                         &h9.couple_ratio[0], &h9.couple_ratio[1],
                         &zero[0], &zero[1], &zero[2], &zero[3], &zero[4],
                         &zero[5], &zero[6], &zero[7], &zero[8]);
        if (ret != 14) { fprintf(stderr, "h9: config file error at entry '%s'\n", name); fclose(fp); return -1; }
        found = 1;
        break;
    }
    fclose(fp);
    if (!found) { fprintf(stderr, "h9: no entry '%s' in '%s'\n", name, conf); return -1; }

    static const int dir_m[H9_N] = {-1, +1, -1, +1, -1, +1, -1, -1, -1};
    if (h9.type == 'R') {
        static const int dir_e[H9_N] = {+1, -1, +1, -1, +1, -1, -1, -1, -1};
        static const int idx_m[H9_N] = {6, 7, 8, 0, 1, 2, 3, 4, 5};
        static const int idx_e[H9_N] = {6, 7, 8, 0, 1, 2, 3, 4, 5};
        memcpy(h9.m_dir, dir_m, sizeof(dir_m)); memcpy(h9.e_dir, dir_e, sizeof(dir_e));
        memcpy(h9.m_idx, idx_m, sizeof(idx_m)); memcpy(h9.e_idx, idx_e, sizeof(idx_e));
        h9.send_id[0] = 0x10; h9.send_id[1] = 0x20; h9.send_id[2] = 0x30;
    } else if (h9.type == 'L') {
        static const int dir_e[H9_N] = {+1, -1, +1, -1, +1, -1, +1, +1, +1};
        static const int idx_m[H9_N] = {8, 7, 6, 4, 5, 2, 3, 0, 1};
        static const int idx_e[H9_N] = {6, 7, 8, 4, 5, 2, 3, 0, 1};
        memcpy(h9.m_dir, dir_m, sizeof(dir_m)); memcpy(h9.e_dir, dir_e, sizeof(dir_e));
        memcpy(h9.m_idx, idx_m, sizeof(idx_m)); memcpy(h9.e_idx, idx_e, sizeof(idx_e));
        h9.send_id[0] = 0x10; h9.send_id[1] = 0x20; h9.send_id[2] = 0x40;
    } else {
        fprintf(stderr, "h9: type '%c' is not supported (L or R)\n", h9.type);
        return -1;
    }

    /* zero offsets are given in joint order in conf.txt; store in motor order */
    for (int i = 0; i < H9_N; i++) h9.dpos_off[h9.e_idx[i]] = zero[i];

#ifdef RUNNER_SIM
    (void)can_ch;
    h9.fd = -1;
    fprintf(stderr, "h9: SIMULATION build, no CAN bus is used\n");
#else
    h9.fd = can_open(can_ch);
    if (h9.fd < 0) { fprintf(stderr, "h9: CAN initializing error (can%d)\n", can_ch); return -1; }
    usleep(10000);
#endif

    /* first transaction: zero duty, read the initial encoder positions */
    int duty[H9_N];
    for (int i = 0; i < H9_N; i++) duty[i] = 0x8000;
    if (h9_can_exchange(duty) < 0) { if (h9.fd >= 0) close(h9.fd); h9.fd = -1; return -1; }

    /* The encoder is single-turn (12 bit). Choose the initial turn count so
     * that the start position is within half a motor turn of the zero pose;
     * the hand is expected to be near its zero pose when the runner starts. */
    for (int i = 0; i < H9_N; i++) {
        int d = h9.dpos[i] - h9.dpos_off[i];
        h9.turn[i] = (d > 2048) ? -1 : (d < -2048) ? +1 : 0;
    }
    h9_observe(q0, qd, 0, 0);
    h9.cnt = 0;
    return 0;
}

static int h9_exchange(const double *tau, double *q, double *qd)
{
    int duty[H9_N];
    double v[H9_N];

    for (int i = 0; i < H9_N; i++) v[h9.m_idx[i]] = fmax(-1.0, fmin(1.0, tau[i]));
    for (int i = 0; i < H9_N; i++) {
        duty[i] = (int)((double)0x8000*h9.m_dir[i]*v[i]) + 0x8000;
        if (duty[i] > 0xFFFF) duty[i] = 0xFFFF;
        else if (duty[i] < 0) duty[i] = 0;
    }
    if (h9_can_exchange(duty) < 0) return -1;
    h9_observe(q, qd, 1, 1);
    h9.cnt++;
    return 0;
}

static void h9_finish(void)
{
    int duty[H9_N];
    for (int i = 0; i < H9_N; i++) duty[i] = 0x8000;
    h9_can_exchange(duty);                 /* release motors */
    usleep(100000);
    if (h9.fd >= 0) close(h9.fd);
    h9.fd = -1;
}

static const hand_driver_t h9_driver = {
    .model      = "h9",
    .n_joint    = H9_N,
    .home_deg   = h9_home_deg,
    .q_min_deg  = h9_qmin_deg,
    .q_max_deg  = h9_qmax_deg,
    .kp         = 2.0,
    .kd         = 0.0,
    .pose_ticks = 500,                     /* 2 s at 250 Hz */
    .init       = h9_init,
    .exchange   = h9_exchange,
    .finish     = h9_finish,
};

/* ------------------------------------------------------------------------ */
/* driver registry                                                           */
/* ------------------------------------------------------------------------ */
static const hand_driver_t *drivers[] = { &h9_driver };

static const hand_driver_t *find_driver(const char *model)
{
    for (size_t i = 0; i < sizeof(drivers)/sizeof(drivers[0]); i++)
        if (strcmp(drivers[i]->model, model) == 0) return drivers[i];
    return NULL;
}

/* ------------------------------------------------------------------------ */
/* moving-average waypoint smoother (joint trajectory generator)             */
/*                                                                           */
/* A linear path of T steps from p0 to p1 is pushed into a queue; every tick */
/* the mean of the first `ws` queued points is emitted and the head popped,  */
/* with the window growing from 1 up to TRJ_WINDOW. The output reaches p1    */
/* after about T ticks and then holds it.                                    */
/* ------------------------------------------------------------------------ */
#define TRJ_MAXLEN 4096
#define TRJ_WINDOW 1000

typedef struct {
    double buf[TRJ_MAXLEN][MAX_JOINT];
    int head, len, ws, n;
} trj_t;

static trj_t trj;

static void trj_target(trj_t *t, int n, const double *p0, const double *p1, int T)
{
    if (T < 1) T = 1;
    if (T > TRJ_MAXLEN - 1) T = TRJ_MAXLEN - 1;
    t->n = n; t->head = 0; t->len = 0; t->ws = 1;

    memcpy(t->buf[t->len++], p0, sizeof(double)*n);
    for (int j = 1; j <= T; j++) {
        for (int k = 0; k < n; k++) t->buf[t->len][k] = p0[k] + (p1[k] - p0[k])*((double)j/T);
        t->len++;
    }
}

static void trj_generate(trj_t *t, double *out)
{
    for (int k = 0; k < t->n; k++) out[k] = 0.0;
    for (int i = 0; i < t->ws; i++) {
        int idx = (i < t->len - 1) ? i : t->len - 1;
        const double *p = t->buf[(t->head + idx) % TRJ_MAXLEN];
        for (int k = 0; k < t->n; k++) out[k] += p[k];
    }
    for (int k = 0; k < t->n; k++) out[k] /= t->ws;

    if (t->len > 1) { t->head = (t->head + 1) % TRJ_MAXLEN; t->len--; }
    if (t->ws < TRJ_WINDOW) t->ws++;
    if (t->ws > t->len) t->ws = t->len;
}

/* ------------------------------------------------------------------------ */
/* runner                                                                    */
/* ------------------------------------------------------------------------ */
enum { MODE_IDLE = 0, MODE_TRACK = 1 };

static volatile sig_atomic_t exiting;
static void on_signal(int sig) { (void)sig; exiting = 1; }

static double tdiff(struct timespec a, struct timespec b)
{
    return (double)(a.tv_sec - b.tv_sec) + 1e-9*(double)(a.tv_nsec - b.tv_nsec);
}

static void tadd(struct timespec *t, double sec)
{
    long ns = (long)(sec*1e9);
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) { t->tv_sec++; t->tv_nsec -= 1000000000L; }
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "usage: %s -p <udp port> -n <hand name> [-c <can ch>] [-f <conf file>]\n"
        "          [-m <model>] [-s <status port>] [-v]\n"
        "  -p  UDP command port (required)      -n  hand name in conf file (required)\n"
        "  -c  CAN channel (default 0)          -f  config file (default conf.txt)\n"
        "  -m  hand model (default h9)          -s  status UDP port (default %d)\n"
        "  -v  verbose\n"
        "example: %s -p 10000 -n L1\n", prog, STATUS_PORT, prog);
}

static int parse_int(const char *s, int lo, int hi, int *out)
{
    char *end; errno = 0;
    long v = strtol(s, &end, 10);
    if (errno || end == s || *end || v < lo || v > hi) return -1;
    *out = (int)v;
    return 0;
}

/* command result */
enum { CMD_NONE = 0, CMD_JOINTPOS, CMD_HOME, CMD_ZERO, CMD_QUIT, CMD_ERROR };

/* parse one datagram; on CMD_JOINTPOS the target is written to tgt_deg[n] */
static int parse_command(char *buf, int n, double *tgt_deg)
{
    char *save;
    char *w = strtok_r(buf, " \t\r\n", &save);
    if (!w) return CMD_NONE;

    int cmd;
    if      (strcmp(w, "jointpos") == 0) cmd = CMD_JOINTPOS;
    else if (strcmp(w, "home")     == 0) cmd = CMD_HOME;
    else if (strcmp(w, "zero")     == 0) cmd = CMD_ZERO;
    else if (strcmp(w, "quit")     == 0) cmd = CMD_QUIT;
    else { fprintf(stderr, "runner: unknown command '%s'\n", w); return CMD_ERROR; }

    if (cmd == CMD_JOINTPOS) {
        for (int i = 0; i < n; i++) {
            char *v = strtok_r(NULL, " \t\r\n", &save), *end;
            if (!v) { fprintf(stderr, "runner: jointpos needs %d values\n", n); return CMD_ERROR; }
            errno = 0;
            tgt_deg[i] = strtod(v, &end);
            if (end == v || *end || errno || !isfinite(tgt_deg[i])) {
                fprintf(stderr, "runner: jointpos: bad value '%s'\n", v); return CMD_ERROR;
            }
        }
    }
    if (strtok_r(NULL, " \t\r\n", &save)) { fprintf(stderr, "runner: too many arguments for '%s'\n", w); return CMD_ERROR; }
    return cmd;
}

int main(int argc, char *argv[])
{
    int  port = -1, status_port = STATUS_PORT, can_ch = 0, verbose = 0, opt;
    const char *name = NULL, *conf = "conf.txt", *model = "h9";

    while ((opt = getopt(argc, argv, "p:n:c:f:m:s:vh")) != -1) {
        switch (opt) {
        case 'p': if (parse_int(optarg, 1, 65535, &port) < 0) { usage(argv[0]); return 64; } break;
        case 'n': name = optarg; break;
        case 'c': if (parse_int(optarg, 0, 255, &can_ch) < 0) { usage(argv[0]); return 64; } break;
        case 'f': conf = optarg; break;
        case 'm': model = optarg; break;
        case 's': if (parse_int(optarg, 1, 65535, &status_port) < 0) { usage(argv[0]); return 64; } break;
        case 'v': verbose = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 64;
        }
    }
    if (port < 0 || name == NULL || optind != argc) { usage(argv[0]); return 64; }

    const hand_driver_t *hand = find_driver(model);
    if (hand == NULL) {
        fprintf(stderr, "runner: unknown hand model '%s' (supported:", model);
        for (size_t i = 0; i < sizeof(drivers)/sizeof(drivers[0]); i++) fprintf(stderr, " %s", drivers[i]->model);
        fprintf(stderr, ")\n");
        return 64;
    }
    const int n = hand->n_joint;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* command socket */
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) { perror("UDP socket"); return 1; }
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("UDP bind"); close(sd); return 1; }

    /* status socket */
    int ss = socket(AF_INET, SOCK_DGRAM, 0);
    if (ss < 0) { perror("UDP status socket"); close(sd); return 1; }
    struct sockaddr_in saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port   = htons((uint16_t)status_port);
    inet_pton(AF_INET, STATUS_ADDR, &saddr.sin_addr);

    /* hand */
    double q[MAX_JOINT] = {0}, qd[MAX_JOINT] = {0}, q_ref[MAX_JOINT] = {0}, tau[MAX_JOINT] = {0};
    if (hand->init(name, conf, can_ch, q) < 0) { close(sd); close(ss); return 1; }

    printf("runner: hand '%s' (%s, %d joints) on can%d, command UDP :%d, status -> %s:%d @ %.0f Hz\n",
           name, hand->model, n, can_ch, port, STATUS_ADDR, status_port, 1.0/STATUS_PERIOD);
    printf("runner: initial joint position [deg]:");
    for (int i = 0; i < n; i++) printf(" %.1f", q[i]*180.0/M_PI);
    printf("\n");
    fflush(stdout);

    int  mode = MODE_IDLE, rc = 0, can_fail = 0;
    long tick = 0;
    struct timespec next, next_status, now;
    clock_gettime(CLOCK_MONOTONIC, &next);
    next_status = next;

    while (!exiting) {
        /* ---- commands (bounded work per tick) ---- */
        for (int k = 0; k < 32 && !exiting; k++) {
            char buf[CMD_MAX];
            ssize_t len = recv(sd, buf, sizeof(buf) - 1, MSG_DONTWAIT | MSG_TRUNC);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                perror("UDP recv"); rc = 1; break;
            }
            if ((size_t)len >= sizeof(buf) || memchr(buf, '\0', (size_t)len)) {
                fprintf(stderr, "runner: invalid command datagram\n");
                continue;
            }
            buf[len] = '\0';

            double tgt_deg[MAX_JOINT], tgt[MAX_JOINT];
            int cmd = parse_command(buf, n, tgt_deg);
            if (cmd == CMD_QUIT) { printf("runner: quit\n"); exiting = 1; break; }
            if (cmd == CMD_HOME) for (int i = 0; i < n; i++) tgt_deg[i] = hand->home_deg[i];
            if (cmd == CMD_ZERO) for (int i = 0; i < n; i++) tgt_deg[i] = 0.0;
            if (cmd != CMD_JOINTPOS && cmd != CMD_HOME && cmd != CMD_ZERO) continue;

            for (int i = 0; i < n; i++) {
                double d = tgt_deg[i];
                if (d < hand->q_min_deg[i] || d > hand->q_max_deg[i]) {
                    d = fmax(hand->q_min_deg[i], fmin(hand->q_max_deg[i], d));
                    fprintf(stderr, "runner: joint %d target %.1f clamped to %.1f deg\n", i + 1, tgt_deg[i], d);
                }
                tgt[i] = d*M_PI/180.0;
            }

            /* duration: fixed for home/zero, proportional to the error for jointpos */
            int T;
            if (cmd == CMD_JOINTPOS) {
                double e2 = 0.0;
                for (int i = 0; i < n; i++) e2 += (tgt[i] - q[i])*(tgt[i] - q[i]);
                T = (int)(0.06*(1.0/DT)*sqrt(e2)) + 1;
            } else T = hand->pose_ticks;

            trj_target(&trj, n, q, tgt, T);
            mode = MODE_TRACK;
            if (verbose) {
                printf("runner: %s ->", cmd == CMD_HOME ? "home" : cmd == CMD_ZERO ? "zero" : "jointpos");
                for (int i = 0; i < n; i++) printf(" %.1f", tgt[i]*180.0/M_PI);
                printf("  (%d ticks)\n", T);
            }
        }
        if (exiting || rc) break;

        /* ---- control ---- */
        if (mode == MODE_TRACK) {
            trj_generate(&trj, q_ref);
            for (int i = 0; i < n; i++) tau[i] = hand->kp*(q_ref[i] - q[i]) - hand->kd*qd[i];
        } else {
            for (int i = 0; i < n; i++) tau[i] = 0.0;
        }

        if (hand->exchange(tau, q, qd) < 0) {
            /* lost the hand: release the motors until a new command arrives */
            if (mode != MODE_IDLE || can_fail == 0) fprintf(stderr, "runner: CAN exchange failed, motors released\n");
            mode = MODE_IDLE;
            can_fail++;
        } else {
            if (can_fail) { fprintf(stderr, "runner: CAN exchange recovered after %d failures\n", can_fail); can_fail = 0; }
        }

        /* ---- status @ 100 Hz ---- */
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (tdiff(now, next_status) >= 0.0) {
            char msg[64 + 16*MAX_JOINT];
            int  pos = snprintf(msg, sizeof(msg), "%s", name);
            for (int i = 0; i < n; i++) pos += snprintf(msg + pos, sizeof(msg) - pos, " %.2f", q[i]*180.0/M_PI);
            sendto(ss, msg, (size_t)pos, 0, (struct sockaddr *)&saddr, sizeof(saddr));
            tadd(&next_status, STATUS_PERIOD);
            if (tdiff(now, next_status) > 0.0) next_status = now;   /* skip missed slots */
        }

        if (verbose && tick % 25 == 0) {
            printf("[%8ld %s]", tick, mode == MODE_TRACK ? "track" : "idle ");
            for (int i = 0; i < n; i++) printf(" %6.1f", q[i]*180.0/M_PI);
            printf("\n");
        }
        tick++;

        /* ---- fixed period ---- */
        tadd(&next, DT);
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (tdiff(now, next) > 0.0) next = now;     /* the CAN exchange was late: skip */
        int err;
        do { err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL); }
        while (err == EINTR && !exiting);
    }

    hand->finish();
    close(sd);
    close(ss);
    printf("runner: shutdown\n");
    return rc;
}
