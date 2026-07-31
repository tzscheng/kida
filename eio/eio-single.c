#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "myactcan.h"

//#define DT 0.00435 //230Hz

char buf[4096];
int fd; //CAN file descriptor
int ch; //0:left 1:right
int cmode; //control mode 0:torque control 1:built-in position control 2:motion mode (PD+ff)
int htype; //hand type [0]: h9 (driven inside the Python loop) [1]: DG-5F-M [2]: DG-5F-S
           //1 and 2 share this file's UDP relay but fork different helper binaries,
           //because the -M and -S are distinct DGSDK models.
int sockfd;
pthread_t cam_th;
struct sockaddr_in hand_addr;

double dir0[7] = {+1.0, +1.0, +1.0, -1.0, +1.0, +1.0, +1.0}; //motor direction
double dir1[7] = {-1.0, -1.0, -1.0, +1.0, -1.0, -1.0, -1.0}; //motor direction
double dir[7];

//double kt[7] = {2.4, 2.4, 2.1, 2.1, 2.1, 1.9, 1.9}; //torque constant (Nm/A) //original
double kt[7] = {1.4, 1.4, 1.3, 1.3, 1.3, 1.9, 1.9}; //torque constant (Nm/A)  //1~5 current I-gain set zero
double q[7];
double q_dot1[7]; //direct velocity feedback
//double q_dot2[7]; //(q - q_old)/DT
double amp[7]; //
//double q_old[7];
long cnt;

void init(const char* args) {
    int ch, ret;
    ret = sscanf(args, "%d %d %d", &ch, &cmode, &htype);
    if (ret != 3){
	fprintf(stderr, "argment error args=%s\n", args);
	exit(0);
    }
    
    if(!(ch == 0 || ch == 1)) {
	printf("wrong ch. ch=%d\n", ch);
	exit(0);
    }

    //set motor direction
    if(ch == 0) memcpy(dir, dir0, sizeof(double)*7);
    else memcpy(dir, dir1, sizeof(double)*7);
    
    fd = myact_can_init(ch);
    if (fd < 0) {
	fprintf(stderr, "CAN initializing error\n");
	exit(0);
    }
    
    //htype 1/2: start with dg-5 (1 = DG-5F-M -> eio-dg5f, 2 = DG-5F-S -> eio-dg5s)
    if(htype != 0){
	const char* hbin = (htype == 2) ? "eio/eio-dg5s" : "eio/eio-dg5f";
	const char* hnam = (htype == 2) ? "eio-dg5s"     : "eio-dg5f";

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&hand_addr, 0, sizeof(hand_addr));
	hand_addr.sin_family = AF_INET;
 	hand_addr.sin_port = htons(6660 + ch); //6660: left hand, 6661: right hand
	inet_pton(AF_INET, "127.0.0.1", &hand_addr.sin_addr);

	pid_t pid = fork();
	if(pid == 0){
	    if (ch == 0) execl(hbin, hnam, "-t0", "-r", NULL);
	    else execl(hbin, hnam, "-t1", "-r", NULL);
	    perror("exec"); //<-- should never reach here
	    exit(0);
	}
	//wait until the hand helper is fully ready
	usleep(2000000);
    }

    usleep(100000);
    printf("init done\n");
}

// 6-arg signature matching CEnv's unified step call (the 2026-06-07 5-tuple
// command; sim.py passes tau, q_ref, qd_ref, kp, kd, y). kp/kd are ACCEPTED
// BUT IGNORED — the PD gains live in the drive firmware (capability ledger).
// Keeping the parameters is ABI-critical: with the old 4-arg form, CEnv's 4th
// argument (kp) landed in this function's y — silent corruption/segfault.
// See eio-kida.c for the full rationale.
void step(double* tau, double* q_ref, double* qd_ref, double* kp, double* kd, double* y){
    (void)kp; (void)kd;   // firmware owns the gains
    double zero_tau[7]    = {0};
    double zero_q_ref[27] = {0};
    double zero_qd_ref[7] = {0};
    double target[7];
    double pos; //[deg]
    double vel; //[deg/s]
    double cur; //[A]
    int ret;

    // Channel semantics:
    //   tau    = torque (arm reads it when cmode==0; feedforward when cmode==2)
    //   q_ref  = position rad (arm reads it when cmode==1 or 2, hand always reads it)
    //   qd_ref = velocity rad/s (arm reads it when cmode==2)
    // The dg5f hand is intrinsically position-controlled, so its position target
    // comes from q_ref regardless of arm cmode. (The UDP+fork relay is a dg5f-specific
    // quirk kept local to this file.) NULL inputs fall back to zero arrays so the
    // controllers can leave a channel "inactive" (e.g. before the first 'init'/'home'
    // command) without us crashing the firmware path that needs a real pointer.
    if (!tau)    tau    = zero_tau;
    if (!q_ref)  q_ref  = zero_q_ref;
    if (!qd_ref) qd_ref = zero_qd_ref;

    for(int i = 0; i < 7; i++){
	if (cmode == 0){
	    target[i] = dir[i] * 1.0/kt[i] * tau[i];  //torque to current (Nm -> A)
	    ret = myact_can_cur(fd, i+1, target[i], &pos, &vel, &cur);
	}
	else if (cmode == 1){
	    target[i] = dir[i] * (180.0 * q_ref[i]/M_PI);  //rad -> deg
	    ret = myact_can_pos(fd, i+1, 500, target[i], &pos, &vel, &cur);
	}
	else if (cmode == 2){
	    // motion mode: motor-side PD on q_ref/qd_ref + tau feedforward (Nm direct, no kt).
	    // kp/kd/dt hardcoded uniformly across the 7 arm joints; tune here (per-joint
	    // arrays if needed). dt matches the ~230 Hz arm loop (legacy //#define DT line).
	    const double kp_mot = 20.0;    // Nm/rad   (manual range 0..500, unit inferred from PD formula)
	    const double kd_mot =  0.5;    // Nm·s/rad (manual range 0..5)
	    const double dt     =  0.00435; // s, step period for qd finite-difference (~230 Hz)
	    double q_act, qd_act;
	    ret = myact_can_mit(fd, i+1,
	                        dir[i] * tau[i], dir[i] * q_ref[i], dir[i] * qd_ref[i],
	                        kp_mot, kd_mot, dt,
	                        &q_act, &qd_act);
	    pos = q_act  * 180.0 / M_PI;   // rad -> deg, so the shared rad-conversion below stays uniform
	    vel = qd_act * 180.0 / M_PI;
	    cur = 0.0;                      // motion-mode reply doesn't carry current
	}
	else{
	    printf("wrong control mode... cmode=%d\n", cmode);
	    exit(0);
	}

	if(ret < 0) {
	    fprintf(stderr, "error\n");
	    exit(0);
	}

	q[i] = dir[i] * M_PI*pos/180.0;  //deg -> rad
	q_dot1[i] = dir[i] * M_PI*vel/180.0;  // deg/s -> rad/s
	amp[i] = dir[i] * cur;

	y[i] = q[i];
	y[i+7] = q_dot1[i];
	y[i+14] = amp[i];
    }

    //update hand target — via q_ref (hand is position-only by hardware nature)
    if(htype != 0){
	float data[60];
	socklen_t len = sizeof(hand_addr);
	buf[0] = 'S';

	for(int i = 0; i < 20; i++) data[i] = (float)q_ref[i+7];
	memcpy(buf+1, data, 20*sizeof(float));
	sendto(sockfd, buf, 20*sizeof(float)+1, 0,(struct sockaddr *)&hand_addr, sizeof(hand_addr));

	recvfrom(sockfd, data, 60*sizeof(double), 0, (struct sockaddr*)&hand_addr, &len);
	for(int i = 0; i < 20; i++) {
	    y[i+21] = (double)data[i]; //pos
	    y[i+41] = (double)data[i+20]; //vel
	    y[i+61] = (double)data[i+40]; //cur
	}
     }

    //usleep(10000);
    //printf("[%ld] %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf | %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf\n", cnt, y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11], y[12], y[13]);
    cnt++;
}

void reset(double* y){
    // 7 arm DoF + up to 20 hand DoF when htype != 0. step() handles NULL inputs
    // (treats them as zero), so reset can just pass NULL — the arm sees a zero
    // current/position command for one cycle and the hand gets zero q_ref
    // (dg5f/dg5s firmware rests at zero on power-on, so this is benign).
    step(NULL, NULL, NULL, NULL, NULL, y);   // kp/kd ignored (firmware gains)
}


void finish(){
    int i;
    int ret;
    
    for(i = 0; i < 7; i++){
	ret = myact_can_stop(fd, i+1);
    }

    /*if(ctype == 1){
	stream_on = 0;
	pthread_join(cam_th, (void**)NULL);
	printf("cam thread finished..\n");
	}*/
    
    if(htype != 0){
	buf[0] = 'Q';
	sendto(sockfd, buf, 1, 0, (struct sockaddr *)&hand_addr, sizeof(hand_addr));
	usleep(1000000);
    }

    usleep(10000);
    myact_can_release(fd);
    usleep(10000);
    printf("ret=%d shutdown\n", ret);    
}


/*
void step2(double* u, double* y){
    int i;
    double target[7];
    double pos, vel;
    int ret;
    
    for(i = 0; i < 7; i++){
	target[i] = dir[i] * (180.0 * u[i]/M_PI);  //rad -> deg
	ret = myact_can_pos(fd, i+1, 500, target[i], &pos, &vel);
	
	if(ret < 0) {
	    fprintf(stderr, "error\n");
	    exit(0);
	}
	
	y[i] = dir[i] * M_PI*pos/180.0;
	y[i+7] = dir[i] * M_PI*vel/180.0;
    }

    printf("[POS: %ld] %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf | %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf %8.3lf\n", cnt, y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11], y[12], y[13]);
    cnt++;
}
*/
