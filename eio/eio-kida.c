#define _GNU_SOURCE
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
//#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "myactcan.h"

char buf[4096];
pthread_t th[4];
int th_arg[4] = {0, 1, 2, 3}; //left-arm, right-arm, left-hand, right-hand
long cnt;
int flag;
int cmode; //control mode [0]:torque control [1]: built-in position controller [2]: motion mode (PD+ff)
int htype; //hand type [0]: h9 (driven inside the Python loop) [1]: DG-5F-M [2]: DG-5F-S
           //1 and 2 share this file's UDP relay but fork different helper binaries,
           //because the -M and -S are distinct DGSDK models.
int sockfd;
struct sockaddr_in hand1_addr, hand2_addr;

//arm torque constatnt (Nm/A)
//double kt[2][7] = {{2.4, 2.4, 2.1, 2.1, 2.1, 1.9, 1.9}, {2.4, 2.4, 2.1, 2.1, 2.1, 1.9, 1.9}}; // <== from specification table
double kt[2][7] = {{1.4, 1.4, 1.3, 1.3, 1.3, 1.9, 1.9}, {1.4, 1.4, 1.3, 1.3, 1.3, 1.9, 1.9}}; // <==1~5 current I-gain set zero

double dir[2][7] = {{+1.0, +1.0, +1.0, -1.0, +1.0, +1.0, +1.0}, {-1.0, -1.0, -1.0, +1.0, -1.0, -1.0, -1.0}}; //direction
double u_tau[2][7]; //arm torque input  (Nm, used by cmode 0 and as feedforward in cmode 2)
double u_q  [2][7]; //arm position ref  (rad, used by cmode 1 and 2)
double u_qd [2][7]; //arm velocity ref  (rad/s, used by cmode 2)
double q[2][7]; //arm joint position (rad)
double q_dot[2][7];
double amp[2][7];

static void* arm_task(void *arg){
    int fd, ret;
    double target[7];
    double pos; //[deg]
    double vel; //[deg/s]
    double cur; //[A]

    int tid = *(int*)arg;
    int core_id = tid + 1; //expect 1 & 2
    
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(core_id, &mask);

    ret = pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    if (ret != 0) { perror("pthread_setaffinity_np"); exit(0); }
    
    fd = myact_can_init(tid);
    if (fd < 0) { fprintf(stderr, "CAN initializing error\n"); exit(0); }

    printf("thread[%d] start\n", tid);
    usleep(10000);

    while(flag == 0){
	for(int i = 0; i < 7; i++){
	    //torque control mode
	    if(cmode == 0){
		target[i] = dir[tid][i] * 1.0/kt[tid][i] * u_tau[tid][i];  //torque to current (Nm -> A)
		ret = myact_can_cur(fd, i+1, target[i], &pos, &vel, &cur);
	    }

	    //built-in position control mode
	    else if (cmode == 1){
		target[i] = dir[tid][i] * (180.0 * u_q[tid][i]/M_PI);  //rad -> deg
		ret = myact_can_pos(fd, i+1, 500, target[i], &pos, &vel, &cur);
	    }

	    //motion mode: motor-side PD on q_ref/qd_ref + tau feedforward (Nm direct, no kt).
	    //kp/kd/dt hardcoded uniformly across the 7 joints; tune here (per-joint arrays
	    //if needed). dt matches the ~230 Hz arm loop.
	    else if (cmode == 2){
		const double kp_mot = 20.0;    // Nm/rad   (manual range 0..500, unit inferred from PD formula)
		const double kd_mot =  0.5;    // Nm·s/rad (manual range 0..5)
		const double dt     =  0.00435; // s, step period for qd finite-difference (~230 Hz)
		double q_act, qd_act;
		ret = myact_can_mit(fd, i+1,
		                    dir[tid][i] * u_tau[tid][i],
		                    dir[tid][i] * u_q  [tid][i],
		                    dir[tid][i] * u_qd [tid][i],
		                    kp_mot, kd_mot, dt,
		                    &q_act, &qd_act);
		pos = q_act  * 180.0 / M_PI;   // rad -> deg so the shared rad-conversion below stays uniform
		vel = qd_act * 180.0 / M_PI;
		cur = 0.0;                      // motion-mode reply doesn't carry current
	    }

	    else{
		printf("wrong control mode... cmode=%d\n", cmode);
		exit(0);
	    }

	    if(ret < 0) {
		fprintf(stderr, "error1\n");
		exit(0);
	    }

	    q[tid][i] = dir[tid][i] * M_PI*pos/180.0; //deg -> rad
	    q_dot[tid][i] = dir[tid][i] * M_PI*vel/180.0; //deg/s -> rad/s
	    amp[tid][i] = dir[tid][i] * cur;
	}
    }

    for(int i = 0; i < 7; i++){
	ret = myact_can_stop(fd, i+1);
	if(ret < 0) {
	    fprintf(stderr, "error2\n");
	}
    }
    
    usleep(10000);
    myact_can_release(fd);
    printf("can-thread[%d] finished\n", tid);
    pthread_exit(0);
}

void init(const char* args) {
    int ret = sscanf(args, "%d %d", &cmode, &htype);
    if (ret != 2){
	fprintf(stderr, "argment error\n");
	exit(0);
    }
    
    pthread_create(&th[0], NULL, &arm_task, (void*)&th_arg[0]);
    pthread_create(&th[1], NULL, &arm_task, (void*)&th_arg[1]);
    
    //htype 1/2: start with dg-5 (1 = DG-5F-M -> eio-dg5f, 2 = DG-5F-S -> eio-dg5s)
    if(htype != 0){
	const char* hbin = (htype == 2) ? "eio/eio-dg5s" : "eio/eio-dg5f";
	const char* hnam = (htype == 2) ? "eio-dg5s"     : "eio-dg5f";

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&hand1_addr, 0, sizeof(hand1_addr));
	hand1_addr.sin_family = AF_INET;
	hand1_addr.sin_port = htons(6660);
	inet_pton(AF_INET, "127.0.0.1", &hand1_addr.sin_addr);

	memset(&hand2_addr, 0, sizeof(hand2_addr));
	hand2_addr.sin_family = AF_INET;
	hand2_addr.sin_port = htons(6661);
	inet_pton(AF_INET, "127.0.0.1", &hand2_addr.sin_addr);

	pid_t pid = fork();
	if(pid == 0){
	    execl(hbin, hnam, "-t0", "-r", NULL);
	    perror("exec1"); //<-- should never reach here
	    exit(0);
	}

	pid = fork();
	if(pid == 0){
	    execl(hbin, hnam, "-t1", "-r", NULL);
	    perror("exec2"); //<-- should never reach here
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
// BUT IGNORED: on this hardware the PD gains live in the drive firmware
// (capability ledger, docs/backend-interface.md — per-step gain writes would
// need a CAN register protocol that doesn't exist). Keeping the parameters in
// the signature is ABI-critical: with the old 4-arg form, CEnv's 4th argument
// (kp) landed in this function's y, so sensor feedback was written into the
// caller's gain buffer (or NULL) — silent corruption/segfault.
void step(double* tau, double* q_ref, double* qd_ref, double* kp, double* kd, double* y){
    (void)kp; (void)kd;   // firmware owns the gains — see comment above
    double zero_tau   [14] = {0};
    double zero_q_ref [54] = {0};
    double zero_qd_ref[14] = {0};
    // Channel semantics:
    //   tau    = torque (arm_task reads when cmode==0; feedforward when cmode==2)
    //   q_ref  = position rad (arm_task reads when cmode==1 or 2; hands always read it)
    //   qd_ref = velocity rad/s (arm_task reads when cmode==2)
    // The dg5f hands are intrinsically position-controlled, so their position targets
    // come from q_ref regardless of arm cmode. (The UDP+fork relay is a dg5f-specific
    // quirk kept local to this file.) NULL inputs fall back to zero arrays so the
    // controllers can leave a channel "inactive" (e.g. before the first 'init'/'home'
    // command) without us crashing the firmware path that needs a real pointer.
    if (!tau)    tau    = zero_tau;
    if (!q_ref)  q_ref  = zero_q_ref;
    if (!qd_ref) qd_ref = zero_qd_ref;

    // Always publish all three reference channels into the shared u_* arrays; arm_task
    // picks whichever ones its cmode actually consumes. Cheaper than the old per-cmode
    // branch and avoids stale-data ambiguity when cmode changes between calls.
    for(int i = 0; i < 2; i++){
	for(int j = 0; j < 7; j++){
	    u_tau[i][j] = tau   [7*i+j];
	    u_q  [i][j] = q_ref [7*i+j];
	    u_qd [i][j] = qd_ref[7*i+j];
	}
    }

    //update hand targets — via q_ref (hands are position-only by hardware nature)
    if(htype != 0){
	float data[60];
	socklen_t len = sizeof(hand1_addr);
	buf[0] = 'S';

	//send command to left-hand
	for(int i = 0; i < 20; i++) data[i] = (float)q_ref[i+14];
	memcpy(buf+1, data, 20*sizeof(float));
	sendto(sockfd, buf, 20*sizeof(float)+1, 0,(struct sockaddr *)&hand1_addr, sizeof(hand1_addr));

	//receive feedback from left-hand
	recvfrom(sockfd, data, 60*sizeof(double), 0, (struct sockaddr*)&hand1_addr, &len);
	for(int i = 0; i < 20; i++) {
	    y[i+42] = (double)data[i]; //pos
	    y[i+62] = (double)data[i+20]; //vel
	    y[i+82] = (double)data[i+40]; //cur
	}

	//send command to right-hand
	for(int i = 0; i < 20; i++) data[i] = (float)q_ref[i+34];
	memcpy(buf+1, data, 20*sizeof(float));
	sendto(sockfd, buf, 20*sizeof(float)+1, 0,(struct sockaddr *)&hand2_addr, sizeof(hand2_addr));

	//receive feedback from right-hand
	recvfrom(sockfd, data, 60*sizeof(double), 0, (struct sockaddr*)&hand2_addr, &len);
	for(int i = 0; i < 20; i++) {
	    y[i+102] = (double)data[i]; //pos
	    y[i+122] = (double)data[i+20]; //vel
	    y[i+142] = (double)data[i+40]; //cur
	}
    }
    
    //wait for a while(target frequency = 240Hz)
    usleep(3000);

    //update arm status
    for(int i = 0; i < 2; i++){
	for(int j = 0; j < 7; j++){    
	    y[7*i+j] = q[i][j];
	    y[7*i+j+14] = q_dot[i][j];
	    y[7*i+j+28] = amp[i][j];
	}
    }
    
    //printf("[%ld] %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf | %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf\n", cnt, y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11], y[12], y[13]);
    //printf("[%ld] %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf | %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf\n", cnt, y[14], y[15], y[16], y[17], y[18], y[19], y[20], y[21], y[22], y[23], y[24], y[25], y[26], y[27]);
    //printf("[%ld] %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf | %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf %8.4lf\n", cnt, q_dot1[0], q_dot1[1], q_dot1[2], q_dot1[3], q_dot1[4], q_dot1[5], q_dot1[6], q_dot2[0], q_dot2[1], q_dot2[2], q_dot2[3], q_dot2[4], q_dot2[5], q_dot2[6]);
    //printf("%d %d %d %d %d %d %d  |  %d %d %d %d %d %d %d\n", mode[0][0], mode[0][1], mode[0][2], mode[0][3], mode[0][4], mode[0][5], mode[0][6],  mode[1][0], mode[1][1], mode[1][2], mode[1][3], mode[1][4], mode[1][5], mode[1][6]);
    cnt++;
}

void reset(double* y){
    // 14 arm DoF + up to 40 hand DoF when htype != 0.
    // cmode==1 needs q_ref; we temporarily swap to torque mode for the arm encoder
    // read (motors receive one cycle of zero current). Hands still get a zero q_ref
    // (htype != 0 path) — dg5f/dg5s firmware naturally rests at position 0 on
    // power-on so commanding zero is benign.
    double zero_tau[54]   = {0};
    double zero_q_ref[54] = {0};
    int saved = cmode;
    if (cmode == 1 || cmode == 2) cmode = 0;
    step(zero_tau, zero_q_ref, NULL, NULL, NULL, y);   // kp/kd ignored (firmware gains)
    cmode = saved;
}


void finish(){
    //SET finish flag
    flag = -1;
    
    //wait until arm thread finished
    pthread_join(th[0], (void**)NULL);
    pthread_join(th[1], (void**)NULL);

    //wait until cam thread finished
    /*if(ctype == 1){
	stream_on = 0;
	pthread_join(th[4], (void**)NULL);
	//pthread_join(th[5], (void**)NULL);
	printf("cam thread finished..\n");
	}*/

    //finish hand proc.
    if(htype != 0){
	buf[0] = 'Q';
	sendto(sockfd, buf, 1, 0,(struct sockaddr *)&hand1_addr, sizeof(hand1_addr));
	sendto(sockfd, buf, 1, 0,(struct sockaddr *)&hand2_addr, sizeof(hand2_addr));
	usleep(1000000);
    }
    
    usleep(100000);
    printf("shutdown\n");
}
