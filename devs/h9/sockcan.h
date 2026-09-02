#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>


int can_init(int ch){
    int fd;
    char buf[32];
    struct sockaddr_can addr;

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
	perror("Socket");
	return -1;
    }

    sprintf(buf, "can%d", ch);
    struct ifreq ifr= {0};
    strncpy(ifr.ifr_name, buf, IFNAMSIZ-1);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int priority=5;
    setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority));

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
	perror("Bind");
	return -1;
    }

    return fd;
}


int can_send_u16s(int fd, int id, int *d, int n){
    struct can_frame frame;
    frame.can_dlc = (unsigned char)2*n;

    frame.can_id = (unsigned int)id;
    int i;

    for(i = 0; i < n; i++){
        frame.data[2*i] = (unsigned char)(d[i] / 256);
	frame.data[2*i+1] = (unsigned char)(d[i] % 256);
    }

    if(write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
	perror("Write.....");
	return -1;
    }

    return 0;
}

int can_recv_u16s(int fd, int *id, int *d, int n){
    struct can_frame frame;

    int ret = read(fd, &frame, sizeof(frame));
    if (ret <= 0) return ret;

    *id = (int)frame.can_id;
    int i;

    for(i = 0; i < n; i++) d[i] = (int)frame.data[2*i]*256 + (int)frame.data[2*i+1];
    return ret;
}
