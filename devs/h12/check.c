#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termio.h>
#include <pthread.h>
#include "sockcan.h"

#define N_DRIVER 1
//#define N_DRIVER 5

int flag;
int opcode;
long cnt;

int motor_dir[10]   = {+1, -1, +1, -1, +1, -1, +1, -1, +1, -1};
int motor;
int L;

int duty[10];
int dpos[10];
int dvel[10];
int dpos_old[10];
	       

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
	case '1': motor = 0; L = 0; break;
	case '2': motor = 1; L = 0; break;	    
	case '3': motor = 2; L = 0; break;
	case '4': motor = 3; L = 0; break;
	case '5': motor = 4; L = 0; break;
	case '6': motor = 5; L = 0; break;
	case '7': motor = 6; L = 0; break;
	case '8': motor = 7; L = 0; break;    
	case '9': motor = 8; L = 0; break;
	case '0': motor = 9; L = 0; break;    
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


int main(int argc, char* argv[]) {
    int i, opt, ch=0;
    //int send_id[5] = {0x10, 0x20, 0x30, 0x40, 0x50};
    int send_id[5] = {0x40, 0x20, 0x30, 0x40, 0x50};
    int recv_id;
    
    while((opt = getopt(argc, (char* const*)argv, "c:")) != -1) {
	switch(opt) {
	case 'c': ch = atoi(optarg); break; //can channel
	case '?': printf("Unknown flag : %c", optopt); break;
	}
    }
    
    pthread_t th;
    pthread_create(&th, NULL, &key_thread, (void*)NULL);
    
    int fd = can_init(ch);
    if (fd < 0) exit(0);
    printf("start\n"); 

    struct timespec t1={0, 0}, t2={0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t1);

    while(flag == 0) {
	for(i = 0; i < 10; i++) duty[i] = 0x8000;
	duty[motor] += motor_dir[motor] * L * 0x1000;
	
	for(i = 0; i < 10; i++){
	    if (duty[i] > 0xFFFF) duty[i] = 0xFFFF;
	    else if (duty[i] < 0x0000) duty[i] = 0x0000;
	}
	
 	//can_send_u16s(fd, send_id, duty, 10);
 	//can_send_u16s(fd, 0x11, duty, 2); 
	
	for(i = 0; i < N_DRIVER; i++){
	    //can_send_u16s(fd, send_id[i], duty+2*i, 2);
	    can_send_u16s(fd, send_id[i], duty+2*i, 2);
	    can_recv_u32s(fd, &recv_id, dpos+2*i, 2);
	}

	//int dd[10];
	//for(i = 0; i < 10; i++) dd[i] = dpos[i] - dpos_old[i];
	
	//printf("[%8ld:%d:%d]  %5x %5x %5x %5x| %5d %5d %5d %5d | %5d %5d %5d %5d\n", cnt, motor, L, duty[0], duty[1], duty[2], duty[3], dpos[0], dpos[1], dpos[2], dpos[3], dd[0], dd[1], dd[2], dd[3]);
	//printf("[%8ld:%d:%d]  %5x %5x %5x %5x %5x %5x| %5d %5d %5d %5d %5d %5d\n", cnt, motor, L, duty[0], duty[1], duty[2], duty[3], duty[4], duty[5], dpos[0], dpos[1], dpos[2], dpos[3], dpos[4], dpos[5]);
	//printf("[%8ld:%d:%d]  %5x %5x %5x %5x %5x %5x %5x %5x| %5d %5d %5d %5d %5d %5d %5d %5d\n", cnt, motor, L, duty[0], duty[1], duty[2], duty[3], duty[4], duty[5], duty[6], duty[7], dpos[0], dpos[1], dpos[2], dpos[3], dpos[4], dpos[5], dpos[6], dpos[7]);
	printf("[%8ld:%d:%d]  %5x %5x %5x %5x %5x %5x %5x %5x %5x %5x| %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n", cnt, motor, L, duty[0], duty[1], duty[2], duty[3], duty[4], duty[5], duty[6], duty[7], duty[8], duty[9], dpos[0], dpos[1], dpos[2], dpos[3], dpos[4], dpos[5], dpos[6], dpos[7], dpos[8], dpos[9]);
	
	for(i = 0; i < 10; i++) dpos_old[i] = dpos[i];
	cnt++;
    }

    //set zero duty
    for(i = 0; i < 10; i++) duty[i] = 0x8000;
    //can_send_u16s(fd, send_id, duty, 10);
    
    for(i = 0; i < N_DRIVER; i++){
	can_send_u16s(fd, send_id[i], duty+2*i, 2);
	can_recv_u32s(fd, &recv_id, dpos+2*i, 2);
    }


    //calculate communication frequency
    clock_gettime(CLOCK_MONOTONIC, &t2);
    double sec = ((double)t2.tv_sec+1.0e-9*t2.tv_nsec) - ((double)t1.tv_sec+1.0e-9*t1.tv_nsec);
    double hz = (double)cnt/sec;
    printf("time: %.5f sec  %.5f hz\n", sec, hz);

    usleep(10000);
    if (close(fd) < 0) perror("Close");
    usleep(10000);
    
    pthread_join(th, NULL);
    fprintf(stderr, "%s: shutdown\n", argv[0]);
    return 0;
}
