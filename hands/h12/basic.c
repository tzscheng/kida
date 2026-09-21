// H12 standalone 12-DOF joint position controller, UDP port 1234.
// Hardware calibration, I/O and observations come directly from eio.c.
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <time.h>
#include <netinet/in.h>
#include "eio.c"

#define N_JOINT 12

// Joint order: thumb1..4, index1..2, middle1..2, ring1..2, little1..2.
// The first two joints are X330; the remaining ten are CAN joints.
static double kp[N_JOINT] = {1.0, 1.0, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
static double kd[N_JOINT] = {0.02, 0.02, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005};

static int sd = -1;
static volatile sig_atomic_t exiting;
static int mode;
static double target[N_JOINT];

static void on_signal(int sig){ (void)sig; exiting = 1; }

static double seconds_between(struct timespec a, struct timespec b){
    return (a.tv_sec-b.tv_sec) + (a.tv_nsec-b.tv_nsec)*1e-9;
}

static void usage(const char *prog){
    printf("usage: %s -t 0|1 [-c CAN-index] [-u ttyUSB-index] [-v]\n"
           "  -t 0=left, 1=right (required); -c/-u default to 0\n"
           "  UDP :1234: joint q0 ... q11 (radians), stop, quit\n"
           "  q0/q1=X330 ID1/ID2, q2..q11=CAN flexion joints (eio.c order)\n"
           "  Starts stopped; stop sends zero current/PWM, quit disables X330 torque.\n", prog);
}

// Validate the entire command before changing the active target or mode.
static int handle_command(char *buf){
    char *next_word;
    char *word = strtok_r(buf, " \t\r\n", &next_word);
    if (!word) return 1;

    double candidate[N_JOINT] = {0};
    int new_mode = 1;
    if (!strcmp(word, "joint")) {
        for (int i = 0; i < N_JOINT; i++) {
            char *value = strtok_r(NULL, " \t\r\n", &next_word), *end;
            if (!value) return 0;
            errno = 0;
            candidate[i] = strtod(value, &end);
            if (end == value || *end || errno || !isfinite(candidate[i])) return 0;
        }
    } else if (!strcmp(word, "stop")) new_mode = 0;
    else if (!strcmp(word, "quit")) new_mode = -1;
    else return 0;

    if (strtok_r(NULL, " \t\r\n", &next_word)) return 0;
    if (new_mode < 0) exiting = 1;
    else {
        memcpy(target, candidate, sizeof(target));
        mode = new_mode;
    }
    return 1;
}

int main(int argc, char **argv){
    int ch = 0, tty = 0, type = -1, verbose = 0, c, rc = 0;
    
    while ((c = getopt(argc, argv, "c:u:t:vh")) != -1) {
        if (c == 'h') { usage(argv[0]); return 0; }
        if (c == 'v') { verbose = 1; continue; }
        if (c != 'c' && c != 'u' && c != 't') { usage(argv[0]); return 64; }

	char *end;
        errno = 0;
        long value = strtol(optarg, &end, 10);
	
        if (errno || end == optarg || *end || value < 0 || value > INT_MAX) {
            usage(argv[0]); return 64;
        }
	
        if (c == 'c') ch = (int)value;
        else if (c == 'u') tty = (int)value;
        else type = (int)value;
    }
    
    if (optind != argc || (type != 0 && type != 1)) { usage(argv[0]); return 64; }
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) { perror("UDP socket"); return 1; }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1234);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("UDP bind");
        close(sd);
        return 1;
    }
    
    char args[64];
    snprintf(args, sizeof(args), "%d %d %d", ch, tty, type);
    init(args);
    printf("H12 %s: can%d, /dev/ttyUSB%d, UDP :1234; stopped\n", type == 0 ? "left" : "right", ch, tty);

    double y[2*N_JOINT] = {0}, tau[N_JOINT] = {0};
    double *q = y, *qd = y + N_JOINT;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    long count = 0;
    
    while (!exiting) {
        // Bounded work per control tick; all targets update on this thread.
        for (int i = 0; i < 32 && !exiting; i++) {
            char buf[4096];
            ssize_t n = recv(sd, buf, sizeof(buf)-1, MSG_DONTWAIT | MSG_TRUNC);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
                perror("UDP recv");
                rc = 1;
                break;
            }
	    
            if ((size_t)n >= sizeof(buf) || memchr(buf, '\0', (size_t)n)) {
                fprintf(stderr, "Invalid UDP command length/content\n");
                continue;
            }
	    
            buf[n] = '\0';
            if (!handle_command(buf)) {
                fprintf(stderr, "Invalid command: joint needs exactly 12 finite radians; stop/quit take no arguments\n");
            }
            // Apply stop before processing any subsequently queued movement.
            if (mode == 0) break;
        }
	
        if (exiting || rc) break;
        // The first transaction is zero drive to obtain initial joint positions.
        if (count) {
            for (int i = 0; i < N_JOINT; i++) {
                tau[i] = mode ? kp[i]*(target[i] - q[i]) - kd[i]*qd[i] : 0.0;
            }
        }
	
        // Bound X330 input before eio.c converts it to an integer current.
        for (int i = 0; i < 2; i++) tau[i] = fmax(-0.6, fmin(0.6, tau[i]));
	
        // eio.c ignores q_ref, qd_ref, kp and kd in this current/PWM backend.
        if (step(tau, NULL, NULL, NULL, NULL, y) < 0) {
            rc = 1;
            break;
        }
	
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (verbose && count % 50 == 0) {
            printf("[%ld mode:%d]", count, mode);
            for (int i = 0; i < N_JOINT; i++) printf(" %6.3f", q[i]);
            putchar('\n');
        }
	
        count++;
        next.tv_nsec += (long)(DT*1e9);
        if (next.tv_nsec >= 1000000000L) { next.tv_sec++; next.tv_nsec -= 1000000000L; }
        // Skip missed deadlines; eio.c computes velocity using its fixed DT.
        if (seconds_between(now, next) > 0) next = now;
	
        int error;
        do { error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL); }
        while (error == EINTR && !exiting);
        if (error && error != EINTR) { rc = 1; break; }
    }
    
    finish();
    if (sd >= 0) close(sd);
    return rc;
}
