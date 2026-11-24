/*
 * canid.c -- find the CAN IDs the h9 motor boards answer with
 *
 * Sends the normal request frame (3 x zero duty 0x8000 + 0x0001, the same
 * frame the runner / monitor send when the motors are free) to each board
 * request ID a few times and lists every frame that comes back: its CAN ID,
 * length and data. No motor is driven. Do NOT run it together with the runner
 * or the monitor on the same bus.
 *
 * Build:
 *   gcc -W -Wall -O2 -o canid canid.c
 *
 * Run:
 *   ./canid [-c <can ch>] [-n <count>] [<request id> ...]
 *   e.g.  ./canid            (can0, 10 rounds, request IDs 0x10 0x20 0x30 0x40)
 *         ./canid -c 1 -n 20
 *         ./canid 0x10 0x20 0x40
 *
 *   -c  SocketCAN channel number (can<N>)                   (default 0)
 *   -n  number of requests per ID                           (default 10)
 *   request IDs default to 0x10 0x20 0x30 0x40. The board is the same for
 *   both hands and answers all four (3 encoders per ID, reply ID = request
 *   ID + 1). An R hand uses 0x10 0x20 0x30, an L hand 0x10 0x20 0x40; the
 *   unused group has encoder chips but no magnets, so its values are garbage.
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define MAX_REQ    16
#define MAX_REPLY  8                 /* distinct reply IDs kept per request   */
#define WAIT_FIRST 300               /* [ms] wait for the first reply         */
#define WAIT_MORE  30                /* [ms] keep listening for more frames   */

typedef struct {
    canid_t id;                      /* raw can_id, flags included            */
    int     count;
    int     dlc;
    unsigned char data[8];           /* last frame seen                       */
} reply_t;

typedef struct {
    int     req_id;
    int     sent, answered;          /* requests sent / with at least 1 reply */
    int     n_reply;
    reply_t reply[MAX_REPLY];
    int     overflow;                /* more distinct IDs than MAX_REPLY      */
} probe_t;

static int can_open(int ch)
{
    int fd;
    struct ifreq ifr;
    struct sockaddr_can addr;

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("CAN socket"); return -1; }

    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, IFNAMSIZ, "can%d", ch);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { perror(ifr.ifr_name); close(fd); return -1; }

    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("CAN bind"); close(fd); return -1; }
    return fd;
}

static void note(probe_t *p, const struct can_frame *f)
{
    reply_t *r = NULL;
    for (int i = 0; i < p->n_reply; i++) if (p->reply[i].id == f->can_id) r = &p->reply[i];
    if (r == NULL) {
        if (p->n_reply == MAX_REPLY) { p->overflow = 1; return; }
        r = &p->reply[p->n_reply++];
        memset(r, 0, sizeof(*r));
        r->id = f->can_id;
    }
    r->count++;
    r->dlc = f->can_dlc > 8 ? 8 : f->can_dlc;
    memcpy(r->data, f->data, 8);
}

/* one request to p->req_id, then collect everything that comes back.
 * 0 on success (with or without a reply), -1 when the request cannot be sent */
static int probe_once(int fd, probe_t *p)
{
    struct can_frame f;
    struct pollfd pfd = {fd, POLLIN, 0};
    int got = 0;

    while (recv(fd, &f, sizeof(f), MSG_DONTWAIT) > 0) ;         /* start from an empty queue */

    memset(&f, 0, sizeof(f));
    f.can_id  = (canid_t)p->req_id;
    f.can_dlc = 8;
    for (int i = 0; i < 3; i++) { f.data[2*i] = 0x80; f.data[2*i+1] = 0x00; }   /* zero duty */
    f.data[6] = 0x00; f.data[7] = 0x01;
    if (write(fd, &f, sizeof(f)) != sizeof(f)) {
        fprintf(stderr, "write to 0x%x failed: %s\n", p->req_id, strerror(errno));
        return -1;
    }
    p->sent++;

    while (poll(&pfd, 1, got ? WAIT_MORE : WAIT_FIRST) > 0) {
        if (recv(fd, &f, sizeof(f), MSG_DONTWAIT) != sizeof(f)) break;
        note(p, &f);
        got = 1;
    }
    if (got) p->answered++;
    return 0;
}

static void report(const probe_t *p)
{
    printf("request 0x%02x : %d/%d answered", p->req_id, p->answered, p->sent);
    if (p->n_reply == 0) { printf("   (no reply)\n"); return; }
    printf("\n");
    for (int i = 0; i < p->n_reply; i++) {
        const reply_t *r = &p->reply[i];
        if (r->id & CAN_ERR_FLAG) {
            printf("    error frame 0x%08x  x%d  (bus error, not a board reply)\n", r->id & CAN_ERR_MASK, r->count);
            continue;
        }
        printf("    reply id 0x%02x%s%s  x%d  dlc %d  last data:",
               r->id & ((r->id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK),
               (r->id & CAN_EFF_FLAG) ? " (extended)" : "", (r->id & CAN_RTR_FLAG) ? " (RTR)" : "",
               r->count, r->dlc);
        for (int j = 0; j < r->dlc; j++) printf(" %02x", r->data[j]);
        if (r->dlc >= 6) {
            printf("   = encoder");
            for (int j = 0; j < 3; j++) printf(" %d", (r->data[2*j] << 8) | r->data[2*j+1]);
        }
        printf("\n");
    }
    if (p->overflow) printf("    (more than %d different reply IDs, the rest is not listed)\n", MAX_REPLY);
}

int main(int argc, char *argv[])
{
    int opt, ch = 0, count = 10, n_req = 0;
    probe_t probe[MAX_REQ];

    memset(probe, 0, sizeof(probe));
    while ((opt = getopt(argc, argv, "c:n:h")) != -1) {
        switch (opt) {
        case 'c': ch = atoi(optarg); break;
        case 'n': count = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-c <can ch>] [-n <count>] [<request id> ...]\n", argv[0]);
            return 1;
        }
    }
    for (; optind < argc && n_req < MAX_REQ; optind++) probe[n_req++].req_id = (int)strtol(argv[optind], NULL, 0);
    if (n_req == 0) {
        static const int def[] = {0x10, 0x20, 0x30, 0x40};
        for (n_req = 0; n_req < 4; n_req++) probe[n_req].req_id = def[n_req];
    }
    if (count < 1) count = 1;

    int fd = can_open(ch);
    if (fd < 0) { fprintf(stderr, "CAN initializing error (can%d)\n", ch); return 1; }

    printf("can%d: %d requests (zero duty) to each of %d IDs ...\n", ch, count, n_req);
    for (int k = 0; k < count; k++) {
        for (int i = 0; i < n_req; i++) {
            if (probe_once(fd, &probe[i]) < 0) { close(fd); return 1; }
            usleep(2000);
        }
    }
    close(fd);

    printf("\n");
    for (int i = 0; i < n_req; i++) report(&probe[i]);

    printf("\nsummary:");
    for (int i = 0; i < n_req; i++) {
        const probe_t *p = &probe[i];
        printf("  0x%02x ->", p->req_id);
        int shown = 0;
        for (int j = 0; j < p->n_reply; j++) {
            if (p->reply[j].id & CAN_ERR_FLAG) continue;
            printf(" 0x%02x", p->reply[j].id & ((p->reply[j].id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK));
            shown++;
        }
        if (!shown) printf(" none");
    }
    printf("\n");
    return 0;
}
