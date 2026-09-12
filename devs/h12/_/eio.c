#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "sockcan.h"
#include "dynamixel_sdk.h"

#define N_PULSES_PER_REV 120000 //25600  //256*100
#define N_DRIVER 5
#define DT 0.002

//X330 address
#define ADDR_OPERATING_MODE         11
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_CURRENT           102
#define ADDR_PRESENT_POSITION       132 

static int flx_motor_map[10] = {1, 0, 3, 2, 5, 4, 7, 6, 9, 8}; //flexion motor map
static int send_id[N_DRIVER] = {0x10, 0x20, 0x30, 0x40, 0x50};

//right hand case
static int encoder_dir[12] = {+1, -1,   -1, +1, -1, +1, -1, +1, -1, +1, -1, +1};
static int motor_dir[12]   = {+1, -1,   -1, +1, -1, +1, -1, +1, -1, +1, -1, +1};
//static int dxl_off[2] = {1486, 517};
static int dxl_off[2] = {1486, 1517};

static int dpos[12]; 
static int duty[10];
static double q_old[12];

static int groupwrite_num;
static int groupread_num;

static int fd1; //can-fd file descripter
static int fd2; //dynamixel file descripter
static long cnt;
static int recv_id;

static int fail_step(const char *msg){
    fprintf(stderr, "%s\n", msg);
    fflush(stderr);
    return -1;
}

static void m2q(double *m, double *q){
    double ratio = 0.25;
    q[0] = m[0];
    q[1] = m[1];
    q[2] = m[2];
    q[3] = m[3] + ratio*q[2];
    q[4] = m[4];
    q[5] = m[5] + ratio*q[4];
    q[6] = m[6];
    q[7] = m[7] + ratio*q[6];
    q[8] = m[8];
    q[9] = m[9] + ratio*q[8];
    q[10] = m[10];
    q[11] = m[11] + ratio*q[10];
}

static int write_dynamixel_current(int *mA){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_addparam_result = False;

    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 1, mA[0], 2);
    if (dxl_addparam_result != True) return fail_step("[ID1] groupSyncWrite addparam failed");
    dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 2, mA[1], 2);
    if (dxl_addparam_result != True) return fail_step("[ID2] groupSyncWrite addparam failed");

    groupSyncWriteTxPacket(groupwrite_num);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) {
	fprintf(stderr, "Dynamixel write failed: %s\n", getTxRxResult(2.0, dxl_comm_result));
	return -1;
    }
    groupSyncWriteClearParam(groupwrite_num);
    return 0;
}

static int read_dynamixel_position(void){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_getdata_result = False;

    groupSyncReadTxRxPacket(groupread_num);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) {
	fprintf(stderr, "Dynamixel read failed: %s\n", getTxRxResult(2.0, dxl_comm_result));
	return -1;
    }
    if ((dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, 1, ADDR_PRESENT_POSITION, 4)) != True) return fail_step("[ID1] groupSyncRead getdata failed");
    if ((dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, 2, ADDR_PRESENT_POSITION, 4)) != True) return fail_step("[ID2] groupSyncRead getdata failed");

    dpos[0] = groupSyncReadGetData(groupread_num, 1, ADDR_PRESENT_POSITION, 4);
    dpos[1] = groupSyncReadGetData(groupread_num, 2, ADDR_PRESENT_POSITION, 4);
    return 0;
}

static int exchange_can_duty(int *duty_cmd){
    int tmp[10];

    for(int i = 0; i < N_DRIVER; i++) {
	if (can_send_u16s(fd1, send_id[i], duty_cmd+2*i, 2) < 0) return -1;
	int ret = can_recv_u32s(fd1, &recv_id, tmp+2*i, 2);
	if (ret <= 0) {
	    fprintf(stderr, "CAN-FD receive failed at frame %d send_id=0x%x ret=%d\n", i, send_id[i], ret);
	    fflush(stderr);
	    return -1;
	}
    }
    for(int i = 0; i < 10; i++) dpos[i+2] = tmp[flx_motor_map[i]];
    return 0;
}

static void update_observation(double *y, int update_velocity){
    double q[12];
    double m[12];

    for(int i = 0; i < 2; i++) m[i] = (double)encoder_dir[i] * M_PI * (double)(dpos[i] - dxl_off[i])/2048.0;
    for(int i = 2; i < 12; i++) m[i] = (double)encoder_dir[i] * 2.0*M_PI * (double)dpos[i]/(double)N_PULSES_PER_REV;

    m2q(m, q);

    for(int i = 0; i < 12; i++) {
	y[i] = q[i];
	y[i+12] = update_velocity ? (q[i] - q_old[i])/DT : 0.0;
	q_old[i] = q[i];
    }

    printf("[%ld] %6.2f %6.2f %6.2f %6.2f   %6.2f %6.2f   %6.2f %6.2f   %6.2f %6.2f   %6.2f %6.2f\n", cnt, q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11]);
}

void init(const char* args) {
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    int ch, tty, type, ret;
    ret = sscanf(args, "%d %d %d", &ch, &tty, &type);
    if (ret != 3){
	fprintf(stderr, "argument error\n");
	exit(1);
    }
    
    //left hand case
    if (type == 0){
	encoder_dir[0] = -1;
	encoder_dir[1] = +1;
	motor_dir[0] = -1;
	motor_dir[1] = +1;
	dxl_off[0] = 1486;
	dxl_off[1] = 517;
    }

    else if (type != 1){
	printf("Hand type=%d should be 0(left) or 1(right)\n", type);
	exit(1);
    }
    
    fd1 = can_init(ch);
    if(fd1 < 0) exit(1);

    char devname[32];
    sprintf(devname, "/dev/ttyUSB%d", tty);
    fd2 = portHandler(devname);
    if(fd2 < 0) exit(1);
		    
    // Initialize PacketHandler Structs
    packetHandler();
    groupwrite_num = groupSyncWrite(fd2, 2.0, ADDR_GOAL_CURRENT, 2);
    groupread_num = groupSyncRead(fd2, 2.0, ADDR_PRESENT_POSITION, 4);
    
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_addparam_result = False;              // AddParam result
    //uint8_t dxl_getdata_result = False;               // GetParam result
    uint8_t dxl_error = 0;                          // Dynamixel error

    // Open port
    if (openPort(fd2)) printf("Succeeded to open the port!\n");
    else {printf("Failed to open the port!\n"); exit(1);}

    // Set port baudrate
    if (setBaudRate(fd2, 1000000)) printf("Succeeded to change the baudrate!\n");
    else {printf("Failed to change the baudrate!\n"); exit(1);}
    
    // Enable Dynamixel1 Torque
    write1ByteTxRx(fd2, 2.0, 1, ADDR_TORQUE_ENABLE, 1);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    else if ((dxl_error = getLastRxPacketError(fd2, 2.0)) != 0) printf("%s\n", getRxPacketError(2.0, dxl_error));
    else printf("Dynamixel1 has been successfully connected \n");

    // Enable Dynamixel2 Torque
    write1ByteTxRx(fd2, 2.0, 2, ADDR_TORQUE_ENABLE, 1);
    if ((dxl_comm_result = getLastTxRxResult(fd2, 2.0)) != COMM_SUCCESS) printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    else if ((dxl_error = getLastRxPacketError(fd2, 2.0)) != 0) printf("%s\n", getRxPacketError(2.0, dxl_error));
    else printf("Dynamixel2 has been successfully connected \n");

    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, 1);
    if (dxl_addparam_result != True) { fprintf(stderr, "[ID1] groupSyncRead addparam failed\n"); exit(1); }

    // Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, 2);
    if (dxl_addparam_result != True) { fprintf(stderr, "[ID2] groupSyncRead addparam failed\n"); exit(1); }
}

int step(double* tau, double* q_ref, double* qd_ref, double* kp, double* kd, double* y){
    (void)q_ref; (void)qd_ref; (void)kp; (void)kd;   // real HW: motor controller has its own PD, ignore targets
    int mA[2];

    double v[10]; //normalized voltage (-1 ~ 1)
    int tmp[10];

    //torque to mA 
    for(int i = 0; i < 2; i++) {
	mA[i] = motor_dir[i]*(int)(1000.0*tau[i]);
	if(mA[i] > 600) mA[i] = 600;
	else if (mA[i] < -600) mA[i] = -600;
    }
			       
    //torque to duty (motor 3-12)
    for(int i = 0; i < 10; i++) {
	//torque to normalized voltage
	v[i] = (double)motor_dir[i+2] * 1.0 * tau[i+2];
	if (v[i] > 1.0) v[i] = 1.0;
	else if (v[i] < -1.0) v[i] = -1.0;

	//normalized voltage -> PWM duty (0-65536)
	tmp[i] = (int)((double)0x8000*v[i]) + 0x8000;
	//if(i >= 6) tmp[i] = 0x8000;
        //tmp[i] = 0x8000;
	
        //overflow check (double check)
	if(tmp[i] > 65535) tmp[i] = 0xFFFF;  //65535
	else if(tmp[i] < 0) tmp[i] = 0x0000; //0
    }

    if (write_dynamixel_current(mA) < 0) return -1;
    if (read_dynamixel_position() < 0) return -1;

    //cAN-FD send-receive
    for(int i = 0; i < 10; i++) duty[i] = tmp[flx_motor_map[i]];
    if (exchange_can_duty(duty) < 0) return -1;
    update_observation(y, cnt > 0);
    cnt++;
    return 0;
}

void reset(double* y){
    // Do not call step() here: it is an exported CEnv ABI symbol, and h12 has
    // hit loader/symbol-preemption issues when reset re-entered it internally.
    // Keep reset as a dedicated zero-command readback path.
    int zero_mA[2] = {0};

    for(int i = 0; i < 10; i++) duty[i] = 0x8000;

    if (write_dynamixel_current(zero_mA) < 0 ||
	read_dynamixel_position() < 0 ||
	exchange_can_duty(duty) < 0) {
	fprintf(stderr, "reset step failed\n");
	fflush(stderr);
	exit(1);
    }

    cnt = 0;
    update_observation(y, 0);
}


void finish(){

    //CAN-FD
    for(int i = 0; i < 10; i++) duty[i] = 0x8000;
    for(int i = 0; i < N_DRIVER; i++) {
	can_send_u16s(fd1, send_id[i], duty+2*i, 2);
	can_recv_u32s(fd1, &recv_id, dpos+2*i, 2);
    }
    
    // Disable Dynamixel torque
    write1ByteTxRx(fd2, 2.0, 1, ADDR_TORQUE_ENABLE, 0);
    write1ByteTxRx(fd2, 2.0, 2, ADDR_TORQUE_ENABLE, 0);
    closePort(fd2);
    
    usleep(10000);
    if (close(fd1) < 0) perror("Close");
}
