#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "sockcan.h"

#define N_DRIVER 5
#define N_PULSES_PER_REV 120000 //25600  //256*100
#define DT 0.002

static int flx_motor_map[10] = {1, 0, 3, 2, 5, 4, 7, 6, 9, 8}; //flexion motor map
static int send_id[N_DRIVER] = {0x10, 0x20, 0x30, 0x40, 0x50};

int mode;
int sd;
long cnt;

//integer states
int dpos[10];      //magnetic encoder position in 16bit (0 ~ 65535)
int duty[10]; //={0x8000, 0x8000};      //joint motor input duty (0x0000 ~ 0xFFFF, zero duty: 0x8000)
int duty_can[10];

//mechanical config.
int encoder_dir[10] = {-1, +1, -1, +1, -1, +1, -1, +1, -1, +1};
int motor_dir[10]   = {-1, +1, -1, +1, -1, +1, -1, +1, -1, +1};

//floating point states
double m[10];   //motor position in radian
double m_old[10];
double m_dot[10];
double tau[10];

//mode1 controller variables: joint p2p
double target[10];
double esum[10];

static int exchange_can_duty(int fd, int *duty_cmd, int *dpos_out){
    int recv_id;
    int tmp[10];

    for(int i = 0; i < 10; i++) duty_can[i] = duty_cmd[flx_motor_map[i]];

    for(int i = 0; i < N_DRIVER; i++){
	if (can_send_u16s(fd, send_id[i], duty_can+2*i, 2) < 0) return -1;
	int ret = can_recv_u32s(fd, &recv_id, tmp+2*i, 2);
	if (ret <= 0) {
	    fprintf(stderr, "CAN-FD receive failed at frame %d send_id=0x%x ret=%d\n", i, send_id[i], ret);
	    return -1;
	}
    }

    for(int i = 0; i < 10; i++) dpos_out[i] = tmp[flx_motor_map[i]];
    return 0;
}

static void send_zero_duty(int fd){
    int recv_id;
    int tmp[10];

    for(int i = 0; i < 10; i++) duty_can[i] = 0x8000;

    for(int i = 0; i < N_DRIVER; i++){
	can_send_u16s(fd, send_id[i], duty_can+2*i, 2);
	can_recv_u32s(fd, &recv_id, tmp+2*i, 2);
    }
}


void joint_pid(){
    int i;
    double e;
    double kp = 0.3;
    double kd = 0.005;
    double ki = 0; //0.005;
    
    for(i = 0; i < 10; i++){
	e = target[i] - m[i];
	esum[i] += e;
	tau[i] = kp*e - kd*m_dot[i] + ki*esum[i];
    }
}


void* pull_thread(void *args){    
    char buf[4096];
    char *token;
    char *next;
    int i, ret;

    struct sockaddr_in clnt_addr;
    int len = sizeof(struct sockaddr_in);
    
    while(1) { 
	ret = recvfrom(sd, buf, sizeof(buf), 0,(struct sockaddr *)&clnt_addr, (socklen_t*)&len);
	if(ret < 0) { perror("err:"); exit(0); }
	buf[ret] = '\0';	

	if(strcmp("quit", buf) == 0) { mode = -1; break; }
	token = strtok_r(buf, " ", &next);

	if(strcmp(token, "stop") == 0) mode = 0;

	else if(strcmp(token, "pid") == 0){
	    mode = 1;
	    int ret = sscanf(next, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &target[0], &target[1], &target[2], &target[3], &target[4], &target[5], &target[6], &target[7], &target[8], &target[9]);
	    if(ret != 10) { printf("wrong command... ret=%d\n", ret); continue; }
	    for (i = 0; i < 10; i++) esum[i] = 0;
	}

	else if(strcmp(token, "test") == 0){
	    mode = 1;
	    target[0] = 0.0;
	    target[1] = 0.0;
	    target[2] = 1.0;
	    target[3] = 1.0;
	    target[4] = 0.0;
	    target[5] = 0.0;
	    target[6] = 0.0;
	    target[7] = 0.0;
	    target[8] = 0.0;
	    target[9] = 0.0;
	    for (i = 0; i < 10; i++) esum[i] = 0;
	}

	else if(strcmp(token, "zero") == 0){
	    mode = 1;
	    for (i = 0; i < 10; i++) target[i] = 0.0;
	    for (i = 0; i < 10; i++) esum[i] = 0;
	}

    }

    //printf("pull-thread exit\n");
    pthread_exit(args);
}



int main(int argc, char* argv[]) {
    int i, c, ch=0;
    int rc = 0;

    while((c = getopt(argc, (char* const*)argv, "c:")) != -1) {
	switch(c) {
	case 'c': ch = atoi(optarg); break; //can channel
	case '?': printf("Unknown flag : %c", optopt); break;
	}
    }

    struct sockaddr_in my_addr;
    sd = socket(AF_INET, SOCK_DGRAM, 0);
    
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(1234);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if(bind(sd, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0) 
    {
	printf("Can't bind\n");
	exit(1);
    }                 
    
    pthread_t pull_th;
    pthread_create(&pull_th, NULL, &pull_thread, (void*)NULL);

    int fd = can_init(ch);
    if (fd < 0) exit(0);
    printf("%d\n", fd);

    double v[10]; //normalized supply voltage (-1 ~ 1)
    
    while(mode >= 0){
	for(i = 0; i < 10; i++){
	    //torque to normalized voltage
	    v[i] = (double)motor_dir[i] * 1.0 * tau[i];
	    if (v[i] > 1.0) v[i] = 1.0;
	    else if (v[i] < -1.0) v[i] = -1.0;
	    
	    //normalized voltage -> PWM duty (0-65536)
	    duty[i] = (int)((double)0x8000*v[i]) + 0x8000;

	    //overflow check (double check)
	    if(duty[i] > 65535) duty[i] = 0xFFFF;  //65535
	    else if(duty[i] < 0) duty[i] = 0x0000; //0
	}
	
	if (exchange_can_duty(fd, duty, dpos) < 0) {
	    rc = 1;
	    goto shutdown;
	}

	//dpos => m
	for(i = 0; i < 10; i++) {
	    m[i] = (double)encoder_dir[i] * 2.0*M_PI * (double)dpos[i]/(double)N_PULSES_PER_REV;
	    if(cnt > 0) m_dot[i] = (m[i] - m_old[i])/DT;
	}
	
        //controller selection
	switch(mode){
	case 1: joint_pid(); break;
	default: for(i = 0; i < 10; i++){ tau[i] = 0; } break;
	}
	
	//printf("[cnt: %8ld mode: %d] %5d %5d %5d %5d | %5d %5d %5d %5d\n", cnt, mode, m100[0], m100[1], m100[2], m100[3],  dpos[0], dpos[1], dpos[2], dpos[3]);
	printf("[cnt: %8ld mode: %d] %5.2lf %5.2lf | %5.2lf %5.2lf | %5.2lf %5.2lf | %5.2lf %5.2lf | %5.2lf %5.2lf\n", cnt, mode, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9]);  //<----printing floating point results in CAN-FD trouble
	
	//prepare for next loop
	for(i = 0; i < 10; i++) m_old[i] = m[i];
	cnt++;
    }
    
shutdown:
    send_zero_duty(fd);

    usleep(10000);
    if (close(fd) < 0) perror("Close");
    usleep(10000);

    if (mode != -1) pthread_cancel(pull_th);
    pthread_join(pull_th, NULL);
    close(sd);

    printf("shutdown\n");
    return rc;
}
