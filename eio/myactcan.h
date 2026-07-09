#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>


int myact_can_init(int ch){
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


int myact_can_release(int fd) {
    if (close(fd) < 0) {
	perror("Close");
	return -1;
    }

    return 0;
}


int myact_can_cur(int fd, int id, double target, double* pos, double* vel, double *cur){
    struct can_frame frame;
    int16_t iqcont = (int16_t)(100.0*target); //A -> 0.01A

    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;
    
    frame.data[0] = 0xA1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = (uint8_t)iqcont;
    frame.data[5] = (uint8_t)(iqcont >> 8);
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    //printf("tau: %lf  iqcont: %d   iqcont(hex): %x   data[4]: %x   data[5]: %x\n", tau, iqcont, iqcont, frame.data[4], frame.data[5]);
    if (write(fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	perror("Write1");
	return -1;
    }

    int ret = read(fd, &frame, sizeof(struct can_frame));
    if (ret <= 0) {
	perror("Read1");
	return ret;
    }
    //printf("[%x] %x %x %x %x %x %x %x %x\n", frame.can_id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

    int16_t dvel = (frame.data[5] << 8) + frame.data[4];    
    *vel = (double)dvel;

    int16_t dcur = (frame.data[3] << 8) + frame.data[2];
    *cur = 0.01*(double)dcur;
    
    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;
    
    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	perror("Write2");
	return -1;
    }

    ret = read(fd, &frame, sizeof(struct can_frame));
    if (ret <= 0) {
	perror("Read2");
	return ret;
    }

    //int pos = frame.data[7]*256*256*256 + frame.data[6]*256*256 + frame.data[5]*256 + frame.data[4];
    int32_t dpos = (frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4];
    *pos = 0.01*(double)dpos;
    
    //printf("[%x] %x %x %x %x %x %x %x %x  pos=%5.2lf\n", frame.can_id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7], *pos);
    //printf("[%x] %x %x %x %x %x %x %x %x  dpos=%d\n", frame.can_id, frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7], dpos);
    return ret;
}

int myact_can_pos(int fd, int id, int16_t max_speed, double target, double* pos, double* vel, double* cur){
    int32_t target_pos = (int32_t)(100.0*target);
    struct can_frame frame;
    
    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;
    
    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = (uint8_t)(max_speed);
    frame.data[3] = (uint8_t)(max_speed >> 8);
    frame.data[4] = (uint8_t)(target_pos);
    frame.data[5] = (uint8_t)(target_pos >> 8);
    frame.data[6] = (uint8_t)(target_pos >> 16);
    frame.data[7] = (uint8_t)(target_pos >> 24);

    if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
	perror("Write");
	return -1;
    }

    int ret = read(fd, &frame, sizeof(frame));
    if (ret <= 0) return ret;
    
    //int16_t dpos = (frame.data[7] << 8) + frame.data[6];
    //*pos = (double)dpos;
    
    int16_t dvel = (frame.data[5] << 8) + frame.data[4];
    *vel = (double)dvel;

    int16_t dcur = (frame.data[3] << 8) + frame.data[2];
    *cur = 0.01*(double)dcur;
    
    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;

    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	perror("Write");
	return -1;
    }

    ret = read(fd, &frame, sizeof(struct can_frame));
    if (ret <= 0) {
	printf("AAA\n");
	return ret;
    }

    //int pos = frame.data[7]*256*256*256 + frame.data[6]*256*256 + frame.data[5]*256 + frame.data[4];
    int32_t dpos = (frame.data[7] << 24) + (frame.data[6] << 16) + (frame.data[5] << 8) + frame.data[4];
    *pos = 0.01*(double)dpos;

    return ret;
}


// Motion mode (0x400+id). PDF p.97/89: 5 packed fields in 8 bytes, big-endian.
// Units: q/qd in rad/(rad/s), tau in Nm, dt in s.
// kp/kd units are not stated in the manual but inferred from the PD formula
// IqRef = [kp*Δp + kd*Δv + t_ff]*KT, since t_ff is Nm: kp in Nm/rad, kd in Nm·s/rad.
// Numeric ranges in the wire encoding: kp 0..500, kd 0..5.
// 0x400 reply only echoes setpoints, so q is read back via 0x92 (multi-turn pos)
// and qd is finite-differenced from the previous call's q (0 on first call).
int myact_can_mit(int fd, int id, double tau, double q_ref, double qd_ref, double kp, double kd, double dt, double *q, double *qd){
    static double q_prev[256];
    static int    q_prev_ok[256] = {0};
    const double DEG2RAD = 3.14159265358979323846 / 180.0;
    struct can_frame frame;

    if (q_ref  < -12.5) q_ref  = -12.5; else if (q_ref  >  12.5) q_ref  =  12.5;
    if (qd_ref < -45.0) qd_ref = -45.0; else if (qd_ref >  45.0) qd_ref =  45.0;
    if (kp     <   0.0) kp     =   0.0; else if (kp     > 500.0) kp     = 500.0;
    if (kd     <   0.0) kd     =   0.0; else if (kd     >   5.0) kd     =   5.0;
    if (tau    < -24.0) tau    = -24.0; else if (tau    >  24.0) tau    =  24.0;

    uint16_t p_des = (uint16_t)((q_ref  + 12.5) / 25.0 * 65535.0);  // 16-bit, -12.5..12.5 rad
    uint16_t v_des = (uint16_t)((qd_ref + 45.0) / 90.0 *  4095.0);  // 12-bit, -45..45 rad/s
    uint16_t kp_i  = (uint16_t)(( kp           ) / 500.0 *  4095.0); // 12-bit, 0..500
    uint16_t kd_i  = (uint16_t)(( kd           ) /   5.0 *  4095.0); // 12-bit, 0..5
    uint16_t t_ff  = (uint16_t)((tau    + 24.0) / 48.0  *  4095.0); // 12-bit, -24..24 Nm

    frame.can_id  = 0x400 + id;
    frame.can_dlc = 8;
    frame.data[0] = (uint8_t)(p_des >> 8);
    frame.data[1] = (uint8_t)(p_des & 0xFF);
    frame.data[2] = (uint8_t)(v_des >> 4);
    frame.data[3] = (uint8_t)(((v_des & 0x0F) << 4) | ((kp_i >> 8) & 0x0F));
    frame.data[4] = (uint8_t)(kp_i & 0xFF);
    frame.data[5] = (uint8_t)(kd_i >> 4);
    frame.data[6] = (uint8_t)(((kd_i & 0x0F) << 4) | ((t_ff >> 8) & 0x0F));
    frame.data[7] = (uint8_t)(t_ff & 0xFF);

    if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
	perror("Write mot");
	return -1;
    }

    int ret = read(fd, &frame, sizeof(frame));  // drain motion-mode reply
    if (ret <= 0) {
	perror("Read mot reply");
	return ret;
    }

    frame.can_id  = 0x140 + id;
    frame.can_dlc = 8;
    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(fd, &frame, sizeof(frame)) != sizeof(frame)) {
	perror("Write 0x92");
	return -1;
    }

    ret = read(fd, &frame, sizeof(frame));
    if (ret <= 0) {
	perror("Read 0x92");
	return ret;
    }

    int32_t dpos = (frame.data[7] << 24) | (frame.data[6] << 16) | (frame.data[5] << 8) | frame.data[4];
    *q = (double)dpos * 0.01 * DEG2RAD;  // 0.01 deg/LSB -> rad

    int idx = id & 0xFF;
    if (q_prev_ok[idx]) {
	*qd = (*q - q_prev[idx]) / dt;
    } else {
	*qd = 0.0;
	q_prev_ok[idx] = 1;
    }
    q_prev[idx] = *q;

    return ret;
}


int myact_can_stop(int fd, int id){
    struct can_frame frame;

    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;

    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	perror("Write");
	return -1;
    }

    int ret = read(fd, &frame, sizeof(struct can_frame));
    return ret;
}

// Brake release (0x77). PDF 2.28: the holding brake is engaged after power-on,
// locking the rotor. Servo commands (0xA1/0xA4) release it implicitly, but
// motion mode (0x400) does not, so call this once before myact_can_mit().
int myact_can_brake_release(int fd, int id){
    struct can_frame frame;

    frame.can_id = 0x140 + id;
    frame.can_dlc = 8;

    frame.data[0] = 0x77;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(fd, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
	perror("Write");
	return -1;
    }

    int ret = read(fd, &frame, sizeof(struct can_frame));
    return ret;
}
