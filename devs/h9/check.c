#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>
#include <pthread.h>
#include "sockcan.h"

int flag;
long cnt;
int joint = 0;
int L = 0;

int m_dir[9]; //motor direction
int e_dir[9]; //encoder direction
int m_idx[9];
int e_idx[9];

double pulley_ratio[2]; // = {0.422, 0.422, 0.422,   0.422, 0.422, 0.422,   0.365, 0.422, 0.422,}; // 0.365 =  2.7/7.4   0.422 = 2.7/6.4  
double couple_ratio[2];

int dpos[9];
int dpos_off[9];
int turn[9];
int zpos[9];
int duty[9];
int dpos_old[9];
int zpos_old[9];
double m[9];
double q[9];
double deg[9];


int getch(void){
    int ch;
    struct termios old;
    struct termios new;

    tcgetattr(0, &old);

    new = old;
    new.c_lflag &= ~(ICANON|ECHO);
    new.c_cc[VMIN] = 1;
    new.c_cc[VTIME] = 0;

    tcsetattr(0, TCSAFLUSH, &new);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &old);

    return ch;
}


void* key_thread(void *args) {
    char ch;
    
    while(flag == 0) { 
	ch = getch();

	switch(ch){
	case '1': joint = 0; L = 0; break;
	case '2': joint = 1; L = 0; break;
	case '3': joint = 2; L = 0; break;
	case '4': joint = 3; L = 0; break;
	case '5': joint = 4; L = 0; break;
	case '6': joint = 5; L = 0; break;
	case '7': joint = 6; L = 0; break;
	case '8': joint = 7; L = 0; break;
	case '9': joint = 8; L = 0; break;
	    
	case '[': L -= 1; break;
	case ']': L += 1; break;
	case ' ': L =  0; break;
	case 'q': flag = -1;
	default: break;
	}

	if(L > 8) L = 8;
	else if(L < -8) L = -8;
    }

    pthread_exit(args);
}


//motor space pos. -> joint space pos.
void m2q(double *m, double *q){
    q[0] = pulley_ratio[0] * m[0];
    q[1] = pulley_ratio[0] * m[1] + couple_ratio[0]*q[0];
    q[2] = pulley_ratio[0] * m[2];
    q[3] = pulley_ratio[0] * m[3] + couple_ratio[0]*q[2];
    q[4] = pulley_ratio[0] * m[4];
    q[5] = pulley_ratio[0] * m[5] + couple_ratio[0]*q[4];
    q[6] = pulley_ratio[1] * m[6]; //thumb CMC
    q[7] = pulley_ratio[0] * m[7] + couple_ratio[1]*q[6];  //thumb MCP
    q[8] = pulley_ratio[0] * m[8] + couple_ratio[1]*q[6] + couple_ratio[0]*q[7];  //thumb IP
}


int main(int argc, char* argv[]) {
    int i, j, ret, opt, ch=0;
    char type = '\0';
    char* name = NULL;
    char buf[256];
    int send_id[3];
    int recv_id;
    int tmp[9];
    
    while((opt = getopt(argc, (char* const*)argv, "c:n:")) != -1) {
	switch(opt) {
	case 'c': ch = atoi(optarg); break; //can channel
	case 'n': name = strdup(optarg); break;
	case '?': printf("Unknown flag : %c", optopt); break;
	}
    }

    if(name == NULL) {
	fprintf(stderr, "you must specify name e.g., ./check -n hand_name\n");
	return 0;
    }

    FILE *fp = fopen("conf.txt", "r");
    if (fp == NULL) {
	fprintf(stderr, "no config file\n");
	return 0;
    }
       
    while(EOF != fscanf(fp, " %s ", buf)){
	if (strcmp(buf, name) == 0) {
	    ret = fscanf(fp, " %c %lf %lf %lf %lf %d %d %d %d %d %d %d %d %d ", &type, &pulley_ratio[0], &pulley_ratio[1], &couple_ratio[0], &couple_ratio[1], &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5], &tmp[6], &tmp[7], &tmp[8]);
	    if(ret != 14) { fprintf(stderr, "config file error\n"); return -1; }
	}		 	    
    }

    if(type == '\0'){
	fprintf(stderr, "No [%s] in config file\n", name);
	return 0;
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
	return 0;
    }

    //set joint position offset
    for (i = 0; i < 9; i++) dpos_off[e_idx[i]] = tmp[i];
    fclose(fp);

    pthread_t th;
    pthread_create(&th, NULL, &key_thread, (void*)NULL);
    
    int fd = can_init(ch);
    printf("start\n"); 

    struct timespec t1={0, 0}, t2={0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t1);

    
    while(flag == 0) {
	for(i = 0; i < 9; i++) duty[i] = 0x8000;
	duty[m_idx[joint]] += m_dir[m_idx[joint]]*L*0x1000;
	
	for(i = 0; i < 9; i++){
	    if (duty[i] > 0xFFFF) duty[i] = 0xFFFF;
	    else if (duty[i] < 0x0000) duty[i] = 0x0000;
	}	
	
	for(i = 0; i < 3; i++){
	    for(j = 0; j < 3; j++) tmp[j] = duty[3*i+j];
	    tmp[3] = 0x0001;

	    can_send_u16s(fd, send_id[i], tmp, 4);
	    can_recv_u16s(fd, &recv_id, tmp, 3);
	    for(j = 0; j < 3; j++) dpos[3*i+j] = tmp[j];
	}	

	if(cnt > 0){
	    for(i = 0; i < 9; i++){
		if (dpos[i] - dpos_old[i] > 2048) turn[i] -= 1;
		else if (dpos[i] - dpos_old[i] < -2048) turn[i] += 1;
	    }
	}

	//zpos:0 => neutral position
	for(i = 0; i < 9; i++) zpos[i] = e_dir[i]*(dpos[i] + turn[i]*4096 - dpos_off[i]);

	//zpos => m
	for(i = 0; i < 9; i++) m[i] = M_PI*((double)zpos[i]/2048.0);

	//m -> q
	m2q(m, q);
	
	//radian => deg
	for(i = 0; i < 9; i++) deg[i] = 180.0*q[i]/M_PI;

	printf("[%8ld]  %x %x %x    %x %x    %x %x    %x %x |  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d\n", cnt, duty[m_idx[0]], duty[m_idx[1]], duty[m_idx[2]], duty[m_idx[3]], duty[m_idx[4]], duty[m_idx[5]], duty[m_idx[6]], duty[m_idx[7]], duty[m_idx[8]], dpos[e_idx[0]], dpos[e_idx[1]], dpos[e_idx[2]], dpos[e_idx[3]], dpos[e_idx[4]], dpos[e_idx[5]], dpos[e_idx[6]], dpos[e_idx[7]], dpos[e_idx[8]]);
	//printf("[%8ld]  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d |  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d\n", cnt, dpos[e_idx[0]], dpos[e_idx[1]], dpos[e_idx[2]], dpos[e_idx[3]], dpos[e_idx[4]], dpos[e_idx[5]], dpos[e_idx[6]], dpos[e_idx[7]], dpos[e_idx[8]], zpos[e_idx[0]], zpos[e_idx[1]], zpos[e_idx[2]], zpos[e_idx[3]], zpos[e_idx[4]], zpos[e_idx[5]], zpos[e_idx[6]], zpos[e_idx[7]], zpos[e_idx[8]]);
        //printf("[%8ld]  %5d %5d %5d    %5d %5d    %5d %5d    %5d %5d |  %5.3f %5.3f %5.3f    %5.3f %5.3f    %5.3f %5.3f    %5.3f %5.3f\n", cnt, zpos[e_idx[0]], zpos[e_idx[1]], zpos[e_idx[2]], zpos[e_idx[3]], zpos[e_idx[4]], zpos[e_idx[5]], zpos[e_idx[6]], zpos[e_idx[7]], zpos[e_idx[8]], q[e_idx[0]], q[e_idx[1]], q[e_idx[2]], q[e_idx[3]], q[e_idx[4]], q[e_idx[5]], q[e_idx[6]], q[e_idx[7]], q[e_idx[8]]);
	//printf("[%8ld]  %5.1f %5.1f %5.1f    %5.1f %5.1f    %5.1f %5.1f    %5.1f %5.1f\n", cnt, deg[e_idx[0]], deg[e_idx[1]], deg[e_idx[2]], deg[e_idx[3]], deg[e_idx[4]], deg[e_idx[5]], deg[e_idx[6]], deg[e_idx[7]], deg[e_idx[8]]);
	
	usleep(2000);
	for(i = 0; i < 9; i++) dpos_old[i] = dpos[i];
	cnt++;
    }

    clock_gettime(CLOCK_MONOTONIC, &t2);
    double sec = ((double)t2.tv_sec+1.0e-9*t2.tv_nsec) - ((double)t1.tv_sec+1.0e-9*t1.tv_nsec);
    double hz = (double)cnt/sec;
    printf("time: %.5f sec  %.5f hz\n", sec, hz);

    
    //set zero duty
    for(i = 0; i < 3; i++){
	for(j = 0; j < 3; j++) tmp[j] = 0x8000;
	tmp[3] = 0x0001;

	can_send_u16s(fd, send_id[i], tmp, 4);
	can_recv_u16s(fd, &recv_id, tmp, 3);
    }
    
    usleep(10000);
    if (close(fd) < 0) perror("Close");

    pthread_join(th, NULL);
    fprintf(stderr, "%s: shutdown\n", argv[0]);
    return 0;
}
