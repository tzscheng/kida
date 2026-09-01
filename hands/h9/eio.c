#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "sockcan.h"

#define DT 0.004

static int  fd;
static long cnt;
static char type = '\0';

static int send_id[3];
static int recv_id;

static int m_dir[9];
static int e_dir[9];
static int m_idx[9];
static int e_idx[9];

static double pulley_ratio[9];
static double couple_ratio[9];

static int dpos[9];      //magnetic encoder position in 16bit (0 ~ 65535)
static int dpos_off[9];  //magnetic encoder offset
static int dpos_old[9];  //magnetic encoder position one step before
static int turn[9];      //magnetic encoder N-turn counter
static int zpos[9];      //magnetic encoder position with N-turn counting (0: neutral position) 
static int duty[9];      //joint motor input duty (0x0000 ~ 0xFFFF, zero duty: 0x8000)

static double m[9];   //joint (decoupled) position in radian
static double q[9];   //joint position in radian
static double q_old[9];
static double q_dot[9];
static double deg[9]; //joint position in degree
static double v[9];   //normalized motor input (-1.0 ~ 1.0)

static int fail_step(const char *msg){
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    return -1;
}

//motor space pos. -> joint space pos.
static void m2q(double *m, double *q){
    q[0] = pulley_ratio[0] * m[0];
    q[1] = pulley_ratio[0] * m[1] + couple_ratio[0]*q[0];
    q[2] = pulley_ratio[0] * m[2];
    q[3] = pulley_ratio[0] * m[3] + couple_ratio[0]*q[2];
    q[4] = pulley_ratio[0] * m[4];
    q[5] = pulley_ratio[0] * m[5] + couple_ratio[0]*q[4];
    q[6] = pulley_ratio[1] * m[6];
    q[7] = pulley_ratio[0] * m[7] + couple_ratio[1]*q[6];
    q[8] = pulley_ratio[0] * m[8] + couple_ratio[1]*q[6] + couple_ratio[0]*q[7];
}

static int exchange_can_duty(int *duty_cmd){
    int tmp[4];

    for(int i = 0; i < 3; i++){
	for(int j = 0; j < 3; j++) tmp[j] = duty_cmd[3*i+j];
	tmp[3] = 0x0001;

	if (can_send_u16s(fd, send_id[i], tmp, 4) < 0) return -1;
	int ret = can_recv_u16s(fd, &recv_id, tmp, 3);
	if (ret <= 0) {
	    fprintf(stderr, "CAN receive failed at frame %d send_id=0x%x ret=%d\n", i, send_id[i], ret);
	    fflush(stderr);
	    return -1;
	}
	for(int j = 0; j < 3; j++) dpos[3*i+j] = tmp[j];
    }

    return 0;
}

static void update_observation(double *y, int update_turn, int update_velocity){
    //check turn count
    if(update_turn){
	for(int i = 0; i < 9; i++){
	    if (dpos[i] - dpos_old[i] > 2048) turn[i] -= 1;
	    else if (dpos[i] - dpos_old[i] < -2048) turn[i] += 1;
	}
    }

    //zpos:0 => neutral position
    for(int i = 0; i < 9; i++) zpos[i] = e_dir[i]*(dpos[i] + turn[i]*4096 - dpos_off[i]);

    //zpos => m
    for(int i = 0; i < 9; i++) m[i] = M_PI*((double)zpos[i]/2048.0);

    //m -> q
    m2q(m, q);

    for (int i = 0; i < 9; i++) q_dot[i] = update_velocity ? (q[i] - q_old[i])/DT : 0.0;

    //radian => deg
    for(int i = 0; i < 9; i++) deg[i] = 180.0*q[i]/M_PI;

    for(int i = 0; i < 9; i++) {
	y[i] = q[e_idx[i]];
	y[i+9] = q_dot[e_idx[i]];
    }

    for(int i = 0; i < 9; i++) {
	dpos_old[i] = dpos[i];
	q_old[i] = q[i];
    }
}

void init(const char* args) {
    char buf[256];
    char name[64];
    int tmp[9];
    int ch, ret;

    ret = sscanf(args, "%d %s", &ch, name);
    if (ret != 2){
	fprintf(stderr, "argument error\n");
	exit(0);
    }
    
    FILE *fp = fopen("conf.txt", "r");
    if (fp == NULL) {
	fprintf(stderr, "no config file\n");
	exit(0);
    }

    while(EOF != fscanf(fp, " %s ", buf)){
	if (strcmp(buf, name) == 0) {
	    ret = fscanf(fp, " %c %lf %lf %lf %lf %d %d %d %d %d %d %d %d %d ", &type, &pulley_ratio[0], &pulley_ratio[1], &couple_ratio[0], &couple_ratio[1], &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5], &tmp[6], &tmp[7], &tmp[8]);
	    if(ret != 14) { fprintf(stderr, "config file error\n"); exit(0); }
	}		 	    
    }

    if(type == '\0'){
	fprintf(stderr, "No [%s] in config file\n", name);
	exit(0);
    }

    else if(type == 'R'){
	m_dir[0] = -1; m_dir[1] = +1; m_dir[2] = -1; m_dir[3] = +1; m_dir[4] = -1; m_dir[5] = +1; m_dir[6] = -1, m_dir[7] = -1; m_dir[8] = -1;
	e_dir[0] = +1; e_dir[1] = -1; e_dir[2] = +1; e_dir[3] = -1; e_dir[4] = +1; e_dir[5] = -1; e_dir[6] = -1, e_dir[7] = -1; e_dir[8] = -1;
	m_idx[0] = 6; m_idx[1] = 7; m_idx[2] = 8; m_idx[3] = 0; m_idx[4] = 1; m_idx[5] = 2; m_idx[6] = 3; m_idx[7] = 4; m_idx[8] = 5;
	e_idx[0] = 6; e_idx[1] = 7; e_idx[2] = 8; e_idx[3] = 0; e_idx[4] = 1; e_idx[5] = 2; e_idx[6] = 3; e_idx[7] = 4; e_idx[8] = 5;
	send_id[0] = 0x10; send_id[1] = 0x20; send_id[2] = 0x30;
    }

    else if(type == 'L'){
	m_dir[0] = -1; m_dir[1] = +1; m_dir[2] = -1; m_dir[3] = +1; m_dir[4] = -1; m_dir[5] = +1; m_dir[6] = -1, m_dir[7] = -1; m_dir[8] = -1;
	e_dir[0] = +1; e_dir[1] = -1; e_dir[2] = +1; e_dir[3] = -1; e_dir[4] = +1; e_dir[5] = -1; e_dir[6] = +1, e_dir[7] = +1; e_dir[8] = +1;
	m_idx[0] = 8; m_idx[1] = 7; m_idx[2] = 6; m_idx[3] = 4; m_idx[4] = 5; m_idx[5] = 2; m_idx[6] = 3; m_idx[7] = 0; m_idx[8] = 1;
	e_idx[0] = 6; e_idx[1] = 7; e_idx[2] = 8; e_idx[3] = 4; e_idx[4] = 5; e_idx[5] = 2; e_idx[6] = 3; e_idx[7] = 0; e_idx[8] = 1;
	send_id[0] = 0x10; send_id[1] = 0x20; send_id[2] = 0x40;
    }

    else {
	fprintf(stderr, "Type %c is not supported\n", type);
	exit(0);
    }

    //set joint position offset
    for (int i = 0; i < 9; i++) dpos_off[e_idx[i]] = tmp[i];
    fclose(fp);    

    fd = can_init(ch);
    if (fd < 0) {
	fprintf(stderr, "CAN initializing error\n");
	exit(0);
    }

    usleep(10000);
    printf("start...\n");
}


int step(double* tau, double* q_ref, double* qd_ref, double* kp, double* kd, double* y){
    (void)q_ref; (void)qd_ref; (void)kp; (void)kd;   // real HW: motor controller has its own PD, ignore targets
    
    for(int i = 0; i < 9; i++){
	v[m_idx[i]] = 1.0*tau[i];
    }
    

    for(int i = 0; i < 9; i++){
        //normalized voltage -> PWM duty (0-65536)
	duty[i] = (int)((double)0x8000*m_dir[i]*v[i]) + 0x8000;

	//overflow check
	if(duty[i] > 65535) duty[i] = 0xFFFF;  //65535
	else if(duty[i] < 0) duty[i] = 0x0000; //0
    }

    if (exchange_can_duty(duty) < 0) return fail_step("CAN exchange failed");
    update_observation(y, cnt > 0, cnt > 0);

    //printf("[%8ld]  %x %x %x    %x %x    %x %x    %x %x |  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d\n", cnt, duty[m_idx[0]], duty[m_idx[1]], duty[m_idx[2]], duty[m_idx[3]], duty[m_idx[4]], duty[m_idx[5]], duty[m_idx[6]], duty[m_idx[7]], duty[m_idx[8]], dpos[e_idx[0]], dpos[e_idx[1]], dpos[e_idx[2]], dpos[e_idx[3]], dpos[e_idx[4]], dpos[e_idx[5]], dpos[e_idx[6]], dpos[e_idx[7]], dpos[e_idx[8]]);
    //printf("[%8ld]  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d |  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d\n", cnt, dpos[e_idx[0]], dpos[e_idx[1]], dpos[e_idx[2]], dpos[e_idx[3]], dpos[e_idx[4]], dpos[e_idx[5]], dpos[e_idx[6]], dpos[e_idx[7]], dpos[e_idx[8]], zpos[e_idx[0]], zpos[e_idx[1]], zpos[e_idx[2]], zpos[e_idx[3]], zpos[e_idx[4]], zpos[e_idx[5]], zpos[e_idx[6]], zpos[e_idx[7]], zpos[e_idx[8]]);
    //printf("[%8ld]  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d |  %5.3f %5.3f %5.3f    %5.3f %5.3f    %5.3f %5.3f    %5.3f %5.3f\n", cnt, zpos[e_idx[0]], zpos[e_idx[1]], zpos[e_idx[2]], zpos[e_idx[3]], zpos[e_idx[4]], zpos[e_idx[5]], zpos[e_idx[6]], zpos[e_idx[7]], zpos[e_idx[8]], q[e_idx[0]], q[e_idx[1]], q[e_idx[2]], q[e_idx[3]], q[e_idx[4]], q[e_idx[5]], q[e_idx[6]], q[e_idx[7]], q[e_idx[8]]);
    //printf("[%8ld]  %5.1f %5.1f %5.1f    %5.1f %5.1f    %5.1f %5.1f    %5.1f %5.1f\n", cnt, deg[e_idx[0]], deg[e_idx[1]], deg[e_idx[2]], deg[e_idx[3]], deg[e_idx[4]], deg[e_idx[5]], deg[e_idx[6]], deg[e_idx[7]], deg[e_idx[8]]);
    
    //usleep(2000);
    cnt++;
    return 0;
}


void reset(double* y){
    // Keep reset as a dedicated zero-command readback path; do not re-enter
    // the exported CEnv step() ABI symbol from inside reset().
    for (int i = 0; i < 9; i++) duty[i] = 0x8000;

    if (exchange_can_duty(duty) < 0) {
	fprintf(stderr, "reset step failed\n");
	fflush(stderr);
	exit(1);
    }

    update_observation(y, cnt > 0, 0);
    cnt = 0;
}


void finish(){
    int tmp[4];
    
    //set duty zero
    for(int i = 0; i < 3; i++){
	for(int j = 0; j < 3; j++) tmp[j] = 0x8000;
	tmp[3] = 0x0001;
	can_send_u16s(fd, send_id[i], tmp, 4);
	can_recv_u16s(fd, &recv_id, tmp, 3);
    }
    
    usleep(100000);
    if (close(fd) < 0) perror("Close");
    printf("shutdown\n");
}
