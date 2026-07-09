#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "DGSDK.h"

// Global State
static int connected = 0;
static ReceivedGripperData rgd;

static void connected_callback() {
    connected = 1;
    printf("[CALLBACK] Connected to gripper server.\n");
}

static void disconnected_callback() {
    connected = 0;
    printf("[CALLBACK] Disconnected from gripper server.\n");
}

static void receive_callback(const ReceivedGripperData data) {
    rgd = data; 
}

void print_usage(char* cmd){
  printf("Usage : %s [option]\n", cmd);
  printf("   -t [0 or 1]        Set hand type (0: Left, 1: Right)\n"
	 "   -r                 Reply joint position\n"
  	 "   -v                 Set verbose\n");
}

int main(int argc, char* argv[]) {
    struct sockaddr_in srv_addr, clnt_addr;
    int opt, htype = -1, reply = 0, verbose = 0, ret;
    char buf[4096];
    long cnt = 0;

    while((opt = getopt(argc, argv, "t:rv")) != -1){
	switch(opt){
	case 't': htype = atoi(optarg); break;
	case 'r': reply = 1; break;
	case 'v': verbose = 1; break;
	default: print_usage(argv[0]); return 0;
	}
    }
    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    int len = sizeof(struct sockaddr_in);

    memset(&srv_addr, 0, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(htype == 0) srv_addr.sin_port = htons(6660);
    else if(htype == 1) srv_addr.sin_port = htons(6661);
    else { print_usage(argv[0]); return 0; }
    
    if (bind(sockfd, (struct sockaddr *)&srv_addr, sizeof(srv_addr)) < 0){
	printf("Can't bind\n");
	return 0;
    }

    //gripper init
    GripperSystemSetting gss;
    memset(&gss, 0, sizeof(GripperSystemSetting));
    gss.communicationMode = COMMUNICATION_MODE_ETHERNET;
    gss.controlMode = CONTROL_MODE_DEVELOPER;

    char ip[MAX_GRIPPER_IP_ADDRESS_SIZE];
    if(htype == 0) sprintf(ip, "192.168.0.73"); //left
    else sprintf(ip, "192.168.0.72"); //right
    
    memcpy(gss.ip, ip, MAX_GRIPPER_IP_ADDRESS_SIZE);
    gss.port = 502;
    gss.slaveID = 1;
    gss.readTimeout = 1000; 
    gss.baudrate = 115200;
    DG_RESULT result = SetGripperSystem(gss);

    //register callbacks
    CallbackForOnConnected(connected_callback);
    CallbackForOnDisconnected(disconnected_callback);
    CallbackForOnReceivedGripperData(receive_callback);

    //connect to gripper
    result = ConnectToGripper();
    if (result != DG_RESULT_NONE) {
	printf("ConnectToGripper() error\n");
        return 0;
    }

    const int timeout_ms = 10000;
    int elapsed = 0;
    while (connected == 0 && elapsed < timeout_ms) {
	usleep(100000);
        elapsed += 100;
    }

    if (connected == 0){
	printf("Connect error\n");
	return 0;
    }

    //set gripper option
    GripperSetting gs;
    memset(&gs, 0, sizeof(GripperSetting));
    float zero_array[MAX_JOINT_COUNT] = {0, };
    int dataTypes[MAX_RECEIVED_DATA_TYPE_COUNT] = {1, 2, 3, 4, 0, 0};

    if (htype == 0) gs.model = DG_MODEL_DG_5F_LEFT;
    else gs.model = DG_MODEL_DG_5F_RIGHT;
    
    gs.fingerCount = 5;
    gs.jointCount = 20;
    gs.movingInpose = 1;
    
    memcpy(gs.jointOffset, zero_array, sizeof(zero_array));
    memcpy(gs.jointInpose, zero_array, sizeof(zero_array));
    memcpy(gs.receivedDataType, dataTypes, sizeof(dataTypes));
    result = SetGripperOption(gs);
    usleep(200000);

    //gripper start
    result = SystemStart();
    if (result == DG_RESULT_NONE) { printf("[INFO] System Started (Motor Torque ON).\n"); }
    else { printf("[ERROR] System Start Failed! Error Code: %d\n", result);  return 0;}

    usleep(200000);
    SetMotionTimeAllEqual(500);
    
    float u[20]; //desired joint pos in rad
    float u_deg[20]; //desired joint pos in deg
    float out[60]; //joint pos + vel + cur
    
    while (1) {
	ret = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&clnt_addr, (socklen_t*)&len);
        if (ret < 0) { printf("recvfrom() error"); continue; }
	
 	if (buf[0] == 'S'){
	    memcpy(u, buf+1, sizeof(u));
	    for(int i = 0; i < 20; i++) u_deg[i] = (180.0/PI)*u[i];
	    result = MoveServoJoint(u_deg);
	    if (result != DG_RESULT_NONE) {
		printf("[ERROR] MoveServoJoint failed: %d\n", result);
		return 0;
	    }
	}
	else if(buf[0] == 'A'){
	    //memcpy(u, buf+1, sizeof(u));
	    int n = sscanf(buf+1, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", u, u+1, u+2, u+3, u+4, u+5, u+6, u+7, u+8, u+9, u+10, u+11, u+12, u+13, u+14, u+15, u+16, u+17, u+18, u+19);	    
	    if (n != 20){
		printf("wrong cmd... n=%d\n", n);
		continue;
	    }
	    
	    for(int i = 0; i < 20; i++) u_deg[i] = (180.0/PI)*u[i];
	    result = MoveJointAll(u_deg);
	    if (result != DG_RESULT_NONE) {
		printf("[ERROR] MoveJointAll failed: %d\n", result);
		return 0;
	    }
	    
	}
	else if (buf[0] == 'Q') break; 
	else { printf("wrong cmd...\n"); continue; }

	for(int i = 0; i < 20; i++) {
	    out[i] = rgd.joint[i]/(180.0/PI);
	    out[i+20] = rgd.velocity[i];
	    out[i+40] = rgd.current[i];
	}
	
	if(reply == 1){
	    sendto(sockfd, out, sizeof(out), 0, (struct sockaddr*)&clnt_addr, sizeof(clnt_addr));
	}
	
	if(verbose == 1) {
	    printf("[q]  %7.3f%7.3f%7.3f%7.3f  %7.3f%7.3f%7.3f%7.3f  %7.3f%7.3f%7.3f%7.3f  %7.3f%7.3f%7.3f%7.3f  %7.3f%7.3f%7.3f%7.3f\n", out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10], out[11], out[12], out[13], out[14], out[15], out[16], out[17], out[18], out[19]);
	    //printf("[vel] %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f\n", vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6], vel[7], vel[8], vel[9], vel[10], vel[11], vel[12], vel[13], vel[14], vel[15], vel[16], vel[17], vel[18], vel[19]);
	    //printf("[cur] %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f  %7.1f%7.1f%7.1f%7.1f\n", cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], cur[6], cur[7], cur[8], cur[9], cur[10], cur[11], cur[12], cur[13], cur[14], cur[15], cur[16], cur[17], cur[18], cur[19]);
	    //printf("[%d] %s\n", htype, buf);
	}
	cnt++;
    }

    //stop gripper system
    result = SystemStop();
    if (result == DG_RESULT_NONE) {
        printf("[INFO] System Stopped (Motor Torque OFF).\n");
    }
    
    //disconnect to gripper
    result = DisconnectToGripper();    
    if (result == DG_RESULT_NONE) {
        printf("[INFO] Disconnected Successfully.\n");
    }

    close(sockfd);
    usleep(1000000);
    return 0;
}
