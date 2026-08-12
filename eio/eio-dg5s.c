// eio-dg5s — UDP<->DGSDK relay for the Tesollo DG-5F-S (20-DoF) hand.
//
// Same wire protocol and role as eio-dg5f (the DG-5F-M relay): bind a UDP port
// (6660 left / 6661 right), take 'S'/'A' joint commands from eio-kida.so or
// eio-single.so, forward them to the gripper over Ethernet/Modbus via DGSDK,
// and reply with the 60-float (pos/vel/current) feedback block.
//
// It is a SEPARATE binary rather than a flag on eio-dg5f because the S is a
// distinct SDK model — DG_MODEL_DG_5F_S_LEFT/RIGHT (0x5F14/0x5F24) vs the M's
// DG_MODEL_DG_5F_LEFT/RIGHT (0x5F12/0x5F22). SetGripperOption() rejects or
// mis-scales the duty payload if the wrong one is set, so the two must not
// share a code path that can be selected wrongly at runtime.
//
// Everything else (joint count, deg<->rad conversion, UDP framing) is identical
// to the M, so the Python side keeps using dg5f.py as the hand controller.
//
// Model differences that matter and are NOT handled here (they belong to the
// controller / arm model, see AGENTS.md):
//   - joint limits differ per model (dg5f.py clips to the S ranges)
//   - the S weighs 880 g vs the M's 1570 g (arm gravity comp payload)
//   - rated joint torque 0.11 Nm vs 0.4 Nm — do not port M grasp forces over

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

// Latest fingertip tactile frame (5 fingers x 18 taxels, uint16 ADC counts).
// static => reads before the first frame return zeros, so no "seen" flag is
// needed. Only the -S tactile sensor writes here; see receive_fingertip_callback.
static ReceivedFingertipSensorData rfd;

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

// The same callback fires for every fingertip sensor family the SDK knows
// (force-torque included), and they share one struct. Guard BEFORE the assign:
// a stray FT frame must not overwrite rfd.tactile with its unrelated bytes.
static void receive_fingertip_callback(const ReceivedFingertipSensorData data) {
    if (data.sensorType == DG_SENSOR_TYPE_TACTILE_S) rfd = data;
}

void print_usage(char* cmd){
  printf("Usage : %s [option]\n", cmd);
  printf("   -t [0 or 1]        Set hand type (0: Left, 1: Right)\n"
	 "   -r                 Reply joint position\n"
  	 "   -v                 Set verbose\n"
	 "   -P/-D/-I/-L <f>    Joint PID gains / I-limit (default 1 / 5 / 0.05 / 0.1)\n");
}

int main(int argc, char* argv[]) {
    struct sockaddr_in srv_addr, clnt_addr;
    int opt, htype = -1, reply = 0, verbose = 0, ret;
    char buf[4096];
    long cnt = 0;

    // Joint PID gains pushed at startup. Defaults are the values the DGSDK
    // v2.0.1 sample program uses for its standard init (main.cpp:196).
    // These are NOT optional: the gripper's position loop runs in the joint
    // firmware off these gains, and a hand whose flash has never been written
    // (a new unit, or one reset) comes up with them at zero — SetGripperOption,
    // SystemStart and MoveJointAll all return DG_RESULT_NONE, feedback streams
    // normally, and nothing moves, with the current feedback pinned at 0.
    // eio-dg5f.c gets away with omitting this only because the -M hands here
    // had gains written into flash by DGManager at some point.
    float gP = 1.0f, gD = 5.0f, gI = 0.05f, gL = 0.1f;

    while((opt = getopt(argc, argv, "t:rvP:D:I:L:")) != -1){
	switch(opt){
	case 't': htype = atoi(optarg); break;
	case 'r': reply = 1; break;
	case 'v': verbose = 1; break;
	case 'P': gP = atof(optarg); break;
	case 'D': gD = atof(optarg); break;
	case 'I': gI = atof(optarg); break;
	case 'L': gL = atof(optarg); break;
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

    // Tesollo ships the DG-5F-S on 169.254.186.73 (left) / .72 (right); these
    // are the addresses after re-IPing them onto the robot LAN with DGManager
    // (Control Manual 1.3.4 "Set IP & PORT", 0x07). Same subnet as the M hands.
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
    CallbackForOnReceivedFingertipSensorData(receive_fingertip_callback);

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
    // 5 = DEVELOPER_MODE_RECEIVED_DATA_TYPE_FINGER_FT_SENSOR — this array is the
    // ONLY switch that makes the hand stream fingertip sensor frames at all
    // (GripperSetting has no sensor-type field). Drop the 5 and the tactile
    // callback simply never fires, with no error anywhere.
    int dataTypes[MAX_RECEIVED_DATA_TYPE_COUNT] = {1, 2, 3, 4, 5, 0};

    // <-- the one real difference from eio-dg5f.c: the S has its own model IDs.
    if (htype == 0) gs.model = DG_MODEL_DG_5F_S_LEFT;   //0x5F14
    else gs.model = DG_MODEL_DG_5F_S_RIGHT;             //0x5F24

    gs.fingerCount = 5;
    gs.jointCount = 20;
    gs.movingInpose = 1;

    memcpy(gs.jointOffset, zero_array, sizeof(zero_array));
    memcpy(gs.jointInpose, zero_array, sizeof(zero_array));
    memcpy(gs.receivedDataType, dataTypes, sizeof(dataTypes));
    result = SetGripperOption(gs);
    if (result != DG_RESULT_NONE) {
	printf("[ERROR] SetGripperOption failed: %d (wrong model for the attached hand?)\n", result);
	return 0;
    }
    usleep(200000);

    //joint PID gains — must be set BEFORE SystemStart(), same order as the SDK
    //sample. Without this the hand accepts every command and never moves.
    result = SetJointGainPIDAllEqual(gP, gD, gI, gL);
    if (result != DG_RESULT_NONE) {
	printf("[ERROR] SetJointGainPIDAllEqual failed: %d\n", result);
	return 0;
    }
    printf("[INFO] Joint PID gains set: P=%g D=%g I=%g iLimit=%g\n", gP, gD, gI, gL);

    //gripper start
    result = SystemStart();
    if (result == DG_RESULT_NONE) { printf("[INFO] System Started (Motor Torque ON). [DG-5F-S %s]\n", htype == 0 ? "left" : "right"); }
    else { printf("[ERROR] System Start Failed! Error Code: %d\n", result);  return 0;}

    usleep(200000);
    result = SetMotionTimeAllEqual(500);
    if (result != DG_RESULT_NONE) printf("[WARN] SetMotionTimeAllEqual failed: %d\n", result);

    float u[20]; //desired joint pos in rad
    float u_deg[20]; //desired joint pos in deg
    float out[60]; //joint pos + vel + cur
    uint16_t tout[18 * MAX_FINGER_COUNT]; //tactile reply, 90 taxels = 180 bytes

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
	// 'R' = read-only: fall through to the feedback block without commanding a
	// motion. Bring-up needs this — 'S' and 'A' both move the hand, so without
	// it there is no way to read the current pose before deciding where to send
	// it. Not part of the eio-kida.so/eio-single.so hot path; it exists for
	// manual probing (utils/dg5probe).
	else if (buf[0] == 'R') { /* no motion */ }
	// 'T' = tactile read: reply with the raw 90 x uint16 block, finger-major
	// ([f0t0..f0t17, f1t0..f1t17, ...]), then `continue` — it deliberately does
	// NOT fall through to the 60-float joint block, so the protocol dg5.py and
	// eio-kida.so speak is untouched. Replies regardless of -r, and replies all
	// zeros until the first tactile frame lands. The reply goes to the recvfrom()
	// peer, so a probe on its own socket never steals the arm loop's 'S' answer.
	else if (buf[0] == 'T') {
	    memcpy(tout, rfd.tactile, sizeof(tout));
	    sendto(sockfd, tout, sizeof(tout), 0, (struct sockaddr*)&clnt_addr, sizeof(clnt_addr));
	    if (verbose == 1) {
		printf("[tac] f0:%5u f1:%5u f2:%5u f3:%5u f4:%5u (taxel 0 of each finger)\n",
		       tout[0], tout[18], tout[36], tout[54], tout[72]);
	    }
	    cnt++;
	    continue;
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
