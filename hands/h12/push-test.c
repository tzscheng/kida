#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <termios.h>
#include "sockcan.h"

static volatile sig_atomic_t stopped;

static void stop(int sig) {
    (void)sig;
    stopped = 1;
}

int main(int argc, char **argv) {
    int ch = 0, opt, status = 0;
    while ((opt = getopt(argc, argv, "c:h")) != -1) {
        if (opt == 'c') {
            char *end;
            long value = strtol(optarg, &end, 10);
            if (!*optarg || *end || value < 0 || value > 999999) {
                fprintf(stderr, "Invalid CAN channel: %s\n", optarg);
                return 1;
            }
            ch = (int)value;
        } else {
            printf("Usage: %s [-c CAN_CHANNEL]\n"
                   "[ decrease both duties, ] increase both duties (motor 1: 0x1000, motor 2: 0x0800)\n"
                   "Space: neutral (0x8000), q/Ctrl+C: neutral and quit\n", argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }

    struct termios saved, raw;
    if (tcgetattr(STDIN_FILENO, &saved) < 0) {
        perror("tcgetattr");
        return 1;
    }
    /* can_init does not check SIOCGIFINDEX; reject missing interfaces here. */
    char name[32];
    snprintf(name, sizeof(name), "can%d", ch);
    if (!if_nametoindex(name)) {
        fprintf(stderr, "CAN interface not found: %s\n", name);
        return 1;
    }
    int fd = can_init(ch);
    if (fd < 0) return 1;

    struct sigaction action = {0};
    action.sa_handler = stop;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
    raw = saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
        perror("tcsetattr");
        close(fd);
        return 1;
    }

    int duty[2] = {0x8000, 0x8000};
    printf("%s, CAN ID 0x10, motors 1 and 2\n"
           "[: decrease  ]: increase  Steps: motor 1=0x1000, motor 2=0x0800\n"
           "Space: neutral  q/Ctrl+C: quit\n"
           "duty: 0x8000 0x8000\n", name);
    /* Refresh the command every 10 ms, while servicing keys and CAN replies. */
    while (!stopped) {
        if (can_send_u16s(fd, 0x10, duty, 2) < 0) {
            status = 1;
            break;
        }
        struct pollfd input = {.fd = STDIN_FILENO, .events = POLLIN};
        int ready = poll(&input, 1, 10);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            status = 1;
            break;
        }
        if (input.revents & POLLIN) {
            char key;
            if (read(STDIN_FILENO, &key, 1) != 1) break;
            if (key == 'q') break;
            if (key == '[' || key == ']' || key == ' ') {
                const int step[2] = {0x1000, 0x0800};
                for (int i = 0; i < 2; ++i) {
                    int value = key == ' ' ? 0x8000 :
                                duty[i] + (key == ']' ? step[i] : -step[i]);
                    if (value < 0) value = 0;
                    if (value > 0xffff) value = 0xffff;
                    duty[i] = value;
                }
                printf("duty: 0x%04X 0x%04X\n", duty[0], duty[1]);
            }
        }
        if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
        struct canfd_frame reply;
        /* Feedback is unused; drain without waiting for a connected motor. */
        for (int i = 0; i < 64; ++i) {
            if (recv(fd, &reply, sizeof(reply), MSG_DONTWAIT) <= 0) break;
        }
    }

    duty[0] = duty[1] = 0x8000;
    if (can_send_u16s(fd, 0x10, duty, 2) < 0) status = 1;
    usleep(10000);
    close(fd);
    if (tcsetattr(STDIN_FILENO, TCSANOW, &saved) < 0) {
        perror("tcsetattr");
        status = 1;
    }
    return status;
}
