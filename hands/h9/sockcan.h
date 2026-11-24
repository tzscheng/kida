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


/* One request/reply round trip with a motor board.
 *
 * The board answers a request on CAN ID `id` with a frame on `id + 1`
 * (0x10 -> 0x11, 0x20 -> 0x21, 0x30 -> 0x31, 0x40 -> 0x41). Matching the
 * reply by order alone is not safe: a reply that arrives after the receive
 * timeout would be read as the answer to the NEXT request, and from then on
 * every encoder group would land on the wrong joints. So frames already
 * waiting are thrown away before sending, and only a frame with the expected
 * ID is accepted.
 *
 * d[0..n_send-1] is sent; on success d[0..n_recv-1] holds the reply and the
 * return value is > 0. On failure (<= 0) d is left as it was sent. */
int can_xfer_u16s(int fd, int id, int *d, int n_send, int n_recv){
    struct can_frame frame;
    int i, k, ret;

    while (recv(fd, &frame, sizeof(frame), MSG_DONTWAIT) > 0) ;   //drop late replies

    if (can_send_u16s(fd, id, d, n_send) < 0) return -1;

    for(k = 0; k < 16; k++){   //bounded: skip at most a few foreign frames
	ret = read(fd, &frame, sizeof(frame));
	if (ret <= 0) return ret;
	if ((int)frame.can_id != id + 1) continue;

	for(i = 0; i < n_recv; i++) d[i] = (int)frame.data[2*i]*256 + (int)frame.data[2*i+1];
	return ret;
    }

    return 0;
}
