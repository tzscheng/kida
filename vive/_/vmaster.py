#!/usr/bin/env -S uv run python
# -*- mode: python -*-
import sys, tty, termios, select, argparse, time, numpy as np
import threading, socket, subprocess, zmq
import triad_openvr, tact

def parse_args():
    par = argparse.ArgumentParser()
    par.add_argument('-t', default=-1, type=int, help='operation type: [0]kida-left  [1]kida-right  [2]kida  [5]gos10')
    par.add_argument('-g', default=-1, type=int, help='gripper type: [0]H9  [1]DG5F')
    par.add_argument('-r', action='store_true', default=False, help='start receiver')
    par.add_argument('-u', action='store_true', default=False, help='use UDP instead of zeromq push')
    return par.parse_args()

def getch(timeout=0.01):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout) #timeout=0 => non-blocking
        if r: return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def glove_retarget_task():
    global q, n_joint
    rcv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rcv_sock.bind(('127.0.0.1', 6633))

    while True:
        data, clnt_addr = rcv_sock.recvfrom(1024)
        msg = data.decode('utf-8', errors='ignore')
        _q = np.fromstring(msg[2:], sep=' ')

        if arg.g == 0 and msg[0] == 'L':
            q[0] = 1.7*_q[0] + 0.85;   q[1] = 1.2*_q[2] + 0.48;  q[2] = 1.2*_q[3] + 0.48
            q[3] = 1.2*_q[5] + 0.0;    q[4] = 1.5*_q[6] + 0.0 ;  q[5] = 1.2*_q[9] + 0.0
            q[6] = 1.5*_q[10] + 0.0;   q[7] = 1.2*_q[13] + 0.0;  q[8] = 1.5*_q[14] + 0.0

        elif arg.g == 0 and msg[0] == 'R':
            q[20] = 1.7*_q[0] + 0.85;  q[21] = 1.2*_q[2] + 0.48;  q[22] = 1.2*_q[3] + 0.48
            q[23] = 1.2*_q[5] + 0.0;   q[24] = 1.5*_q[6] + 0.0;   q[25] = 1.2*_q[9] + 0.0
            q[26] = 1.5*_q[10] + 0.0;  q[27] = 1.2*_q[13] + 0.0;  q[28] = 1.5*_q[14] + 0.0
            
        elif arg.g == 1 and msg[0] == 'L':
            q[0]  =  1.6*_q[1] - 1.5;   q[1]  = 1.4*_q[0] + 0.5;   q[2]  = -1.2*_q[2] + 0.0;   q[3]  = -1.2*_q[3] + 0.0
            q[4]  =  1.0*_q[4] + 0.2;   q[5]  = 1.3*_q[5] + 0.0;   q[6]  =  1.2*_q[6] + 0.0;   q[7]  =  1.2*_q[7] + 0.0
            q[8]  =  1.0*_q[8] + 0.2;   q[9]  = 1.2*_q[9] + 0.0;   q[10] =  1.2*_q[10] + 0.0;  q[11] =  1.2*_q[11] + 0.0
            q[12] =  1.0*_q[12] + 0.2;  q[13] = 1.2*_q[13] - 0.4;  q[14] =  1.2*_q[14] + 0.0;  q[15] =  1.2*_q[15] + 0.0
            q[16] = -0.1*_q[17] + 0.0;  q[17] = 1.0*_q[16] + 0.2;  q[18] =  1.0*_q[17] - 0.4;  q[19] =  1.2*_q[18] + 0.0

        elif arg.g == 1 and msg[0] == 'R':
            q[20] = -1.6*_q[1] + 1.5;   q[21] = -1.4*_q[0] - 0.5;  q[22] =  1.2*_q[2] + 0.0;   q[23] =  1.2*_q[3] + 0.0
            q[24] = -1.0*_q[4] + 0.2;   q[25] =  1.3*_q[5] + 0.0;  q[26] =  1.2*_q[6] + 0.0;   q[27] =  1.2*_q[7] + 0.0
            q[28] = -1.0*_q[8] + 0.2;   q[29] =  1.2*_q[9] + 0.0;  q[30] =  1.2*_q[10] + 0.0;  q[31] =  1.2*_q[11] + 0.0
            q[32] = -1.0*_q[12] + 0.0;  q[33] =  1.2*_q[13] - 0.4; q[34] =  1.2*_q[14] + 0.0;  q[35] =  1.2*_q[15] + 0.0
            q[36] =  0.1*_q[17] + 0.0;  q[37] = -1.0*_q[16] + 0.0; q[38] =  1.0*_q[17] - 0.4;  q[39] =  1.2*_q[18] + 0.0

        elif arg.g == 2 and msg[0] == 'L':
            q[0]  =  1.0*_q[0] + 0.0;   q[1]  = 1.0*_q[1] + 0.0;   q[2]  =  1.0*_q[2] + 0.0;   q[3]  =  1.0*_q[3] + 0.0
            q[4]  =  1.0*_q[5] + 0.0;   q[5]  = 1.0*_q[6] + 0.0;   q[6]  =  1.0*_q[9] + 0.0;   q[7]  =  1.0*_q[10] + 0.0
            q[8]  =  1.0*_q[13] + 0.0;  q[9]  = 1.0*_q[14] + 0.0;  q[10] =  1.0*_q[17] + 0.0;  q[11] =  1.0*_q[18] + 0.0
            
arg = parse_args()
if arg.t < 0: print('choose operation type...'); exit(0)

q = np.zeros(40, dtype=np.float32) #hand target joint
feed_cnt = 0
    
if arg.g >= 0:
    if   arg.g == 0: n_joint = 9
    elif arg.g == 1: n_joint = 20
    threading.Thread(target=glove_retarget_task, daemon=True).start()

if   arg.t in [0, 1]: endpoint = 'headcam wristcam'
elif arg.t == 2:      endpoint = 'headcam rightcam leftcam'
elif arg.t == 5:      endpoint = 'topcam sidecam'
    
#subprocess.Popen(['./logger', '-t%d' %arg.t, '-q'])
#if arg.r: subprocess.Popen(['./receiver', '-z', endpoint])
#time.sleep(0.2)

CURSOR_HOME = "\033[H" # \033[H: 커서를 홈 위치(0, 0)로 이동
CLEAR_REST_OF_SCREEN = "\033[J" #\033[J: 커서 위치부터 화면 끝까지 지우기 (잔여 프롬프트 제거용)

vr = triad_openvr.triad_openvr()
print(vr.print_discovered_objects()) # 1. 초기 객체 발견 목록

#make sorted tracker key 
tracker_devices = {k: vr.devices[k] for k in vr.devices.keys() if k.startswith("tracker_")}
if not tracker_devices: print('no tracker found'); exit()
serial_key_pairs = [(tracker_devices[k].get_serial(), k) for k in tracker_devices.keys()]
serial_key_pairs.sort(key=lambda x: x[0])
tracker_key = [k for _,k in serial_key_pairs]
print('sorted trackers by serial:')
for i, (s, k) in enumerate(serial_key_pairs): print('[%d] %s  serial=%s' %(i, k, s))

#check tracker number
if arg.t in [0, 1] and len(tracker_key) != 1: print('more than one tracker exist'); exit()
elif arg.t == 2 and len(tracker_key) != 2: print('no. of tracker is not two'); exit()
elif arg.t == 5 and len(tracker_key) != 1: print('more than one tracker exist'); exit()
    
ctx = zmq.Context()
# slave로 명령 전송: PUSH. slave쪽이 bind하므로 master는 connect.
slave_sock = ctx.socket(zmq.PUSH)
slave_sock.setsockopt(zmq.IMMEDIATE, 1) # connected peer가 없으면 큐잉하지 않고 즉시 EAGAIN
slave_sock.setsockopt(zmq.CONFLATE, 1)
slave_sock.setsockopt(zmq.LINGER, 0)
slave_sock.connect('ipc:///dev/shm/default')
#slave_sock.connect('tcp://127.0.0.1:6600')

# logger로 명령 전송: PUSH. logger쪽이 bind하므로 master는 connect.
logger_sock = ctx.socket(zmq.PUSH)
logger_sock.setsockopt(zmq.IMMEDIATE, 1)
logger_sock.setsockopt(zmq.CONFLATE, 1)
logger_sock.setsockopt(zmq.LINGER, 0)
logger_sock.connect('ipc:///dev/shm/logger')
#logger_sock.connect('tcp://127.0.0.1:6611')

def send_slave(data):
    try: slave_sock.send(data, flags=zmq.DONTWAIT)
    except zmq.Again: pass

def send_logger(data):
    try: logger_sock.send(data, flags=zmq.DONTWAIT)
    except zmq.Again: pass
    
sys.stdout.write("\033[2J\033[H")
sys.stdout.flush()

offset = [np.zeros(3), np.zeros(3)]
_xyz = [np.zeros(3), np.zeros(3)] #xyz position before offset operation
xyz  = [np.zeros(3), np.zeros(3)] #xyz position after  offset operation
rpy  = [np.zeros(3), np.zeros(3)]
bias  = [np.zeros(3), np.zeros(3)]
scale = [1.0, 1.0]
if   arg.t == 0: bias[0] = np.array([0.30,  0.23, -0.40])
elif arg.t == 1: bias[1] = np.array([0.30, -0.23, -0.40])
elif arg.t == 2:
    bias[0] = np.array([0.30,  0.23, -0.40])
    bias[1] = np.array([0.30, -0.23, -0.40])
elif arg.t == 5:
    bias[0] = np.array([0.50, -0.15,  0.30]); scale[0] = 2.0

ee_attach = False
hand_attach = False
log_on = False
jawpos = 0.1
cnt = 0

#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#slave_addr  = ('127.0.0.1', 6600)
#logger_addr = ('127.0.0.1', 6611)

while(True):
    key = getch(timeout=0.033) #30 Hz Teleoperation
    if key != None:
        if   key == '/':
            send_slave(b'randomize')
        elif key == 'r':
            if   arg.t in [0, 1]: send_slave(b'rest, home')
            elif arg.t == 2:      send_slave(b'rest, home, home')
            ee_attach = False; hand_attach = False
        elif key == '\x1b':
            send_logger(b'quit'); time.sleep(0.2); break #ESC key
        elif key == 'z':
            jawpos +=  0.02;
            if jawpos > 0.10: jawpos = 0.10
        elif key == 'x':
            jawpos += -0.02;
            if jawpos < 0.0: jawpos = 0.0
        elif key == 'q':
            send_slave(b'quit')
            send_logger(b'quit')
        elif key == 'c':
            if log_on: log_on = False; send_logger(b'log-off')
            else:      log_on = True;  send_logger(b'log-on')
        elif key == 'b':
            hand_attach = not hand_attach
        elif key == 'a':
            was_attached = ee_attach
            ee_attach = not ee_attach
            if was_attached:
                # detach: save current output xyz so re-attach resumes from here;
                # also reset offset so the displayed xyz stays at the saved bias.
                if arg.t in [0, 2, 5]:
                    bias[0] = xyz[0].copy(); offset[0] = -_xyz[0]
                if arg.t in [1, 2]:
                    bias[1] = xyz[1].copy(); offset[1] = -_xyz[1]
            else:
                # attach: capture current tracker pose as origin
                if arg.t in [0, 2, 5]: offset[0] = -_xyz[0]
                if arg.t in [1, 2]:    offset[1] = -_xyz[1]
        elif key == 'h': #go to home pose
            send_slave(b'home')
            jawpos = 0.1
            # robot returned to home: restore bias to per-mode home values,
            # and reset offset so the displayed xyz stays at the new bias.
            if   arg.t == 0: bias[0] = np.array([0.30,  0.23, -0.40]); offset[0] = -_xyz[0]
            elif arg.t == 1: bias[1] = np.array([0.30, -0.23, -0.40]); offset[1] = -_xyz[1]
            elif arg.t == 2:
                bias[0] = np.array([0.30,  0.23, -0.40]); offset[0] = -_xyz[0]
                bias[1] = np.array([0.30, -0.23, -0.40]); offset[1] = -_xyz[1]
            elif arg.t == 5: bias[0] = np.array([0.50, -0.15,  0.30]); offset[0] = -_xyz[0]
        elif key == 'i':
            if arg.t in [0, 1]: cmd = 'init, home'
            elif arg.t == 2: cmd = 'init, home, home'
            send_slave(cmd.encode())
            ee_attach = False; hand_attach = False
        continue
        
    sys.stdout.write(CURSOR_HOME)
    print('type: %d   ee: %d   hand: %d   log: %d' %(arg.t, ee_attach, hand_attach, log_on))
    print('cnt:%10d' %cnt)

    #for tracker_key in sorted_trackers:
    for i in range(len(tracker_key)):
        #pose = tracker_devices[tracker_key[i]].get_pose_euler()
        pose = tracker_devices[tracker_key[i]].get_pose_matrix()
        if pose == None: continue

        T01 = tact.T_trans([1.2, 0, 0]) @ tact.T_rot_z(np.pi/2) @ tact.T_rot_x(np.pi/2)
        T12 = np.array([[pose[0][0], pose[0][1], pose[0][2], pose[0][3]], [pose[1][0], pose[1][1], pose[1][2], pose[1][3]], [pose[2][0], pose[2][1], pose[2][2], pose[2][3]], [0, 0, 0, 1]])

        if   arg.t == 0 or (arg.t == 2 and i == 0): T32 = tact.T_rot_x( np.pi/2) @ tact.T_rot_z(np.pi/2)
        elif arg.t == 1 or (arg.t == 2 and i == 1): T32 = tact.T_rot_x(-np.pi/2) @ tact.T_rot_z(np.pi/2)
        elif arg.t == 5: T32 = tact.T_rot_x(np.pi) @ tact.T_rot_z(np.pi/2)
        
        #T02 = T01 @ T12
        T03 = T01 @ T12 @ np.linalg.inv(T32)

        #kida left arm
        if arg.t == 0 or (arg.t == 2 and i == 0):
            _xyz[0] = T03[0:3, 3]
            xyz[0] = scale[0]*(_xyz[0] + offset[0]) + bias[0]
            rpy[0] = tact.rotation_to_euler(T03[:3, :3])

        #kida right arm
        elif arg.t == 1 or (arg.t == 2 and i == 1):
            _xyz[1] = T03[0:3, 3]
            xyz[1] = scale[1]*(_xyz[1] + offset[1]) + bias[1]
            rpy[1] = tact.rotation_to_euler(T03[:3, :3])

        #gos10
        elif arg.t == 5:
            _xyz[0] = T03[0:3, 3]
            xyz[0] = scale[0]*(_xyz[0] + offset[0]) + bias[0]
            rpy[0] = tact.rotation_to_euler(T03[:3, :3])
        
    if arg.t in [0, 1]:
        arm_cmd = 'task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[arg.t][0], xyz[arg.t][1], xyz[arg.t][2], rpy[arg.t][0], rpy[arg.t][1], rpy[arg.t][2])
        if arg.g >= 0:
            hand_cmd = 'joint'
            for i in range(n_joint): hand_cmd += '%8.3f' %q[20*arg.t+i]
            
    elif arg.t == 2:
        arm_cmd = 'task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2])
        if arg.g >= 0:
            hand_cmd = 'joint'
            for i in range(n_joint): hand_cmd += '%8.3f' %q[i]
            hand_cmd += ', joint'
            for i in range(n_joint): hand_cmd += '%8.3f' %q[i+20]                

    elif arg.t == 5:
        arm_cmd = 'task %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], jawpos)

    # Send when at least one of ee_attach/hand_attach is on. Substitute
    # 'none' for whichever side is detached so the slave keeps a consistent
    # two-part frame. -t5 has no hand part, so hand_attach is moot there.
    have_arm  = arg.t in [0, 1, 2, 5]
    have_hand = arg.g >= 0 and arg.t in [0, 1, 2]
    send_arm  = have_arm  and ee_attach
    send_hand = have_hand and hand_attach
    if send_arm or send_hand:
        cmd = arm_cmd if send_arm else 'none'
        if have_hand:
            cmd = cmd + ', ' + (hand_cmd if send_hand else 'none')
        send_slave(cmd.encode())
        send_logger(cmd.encode())
                    
    if arg.t == 0:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2]))
        if   arg.g == 0: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]))
        elif arg.g == 1: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], q[14], q[15], q[16], q[17], q[18], q[19])) 
        
    elif arg.t == 1:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2]))
        if   arg.g == 0: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28]))
        elif arg.g == 1: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28], q[29], q[30], q[31], q[32], q[33], q[34], q[35], q[36], q[37], q[38], q[39])) 
        
    elif arg.t == 2:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2]))
        if   arg.g == 0:
            print('\nL-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]))
            print('\nR-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28]))
        elif arg.g == 1:
            print('\nL-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], q[14], q[15], q[16], q[17], q[18], q[19]))
            print('\nR-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28], q[29], q[30], q[31], q[32], q[33], q[34], q[35], q[36], q[37], q[38], q[39]))

    elif arg.t == 5:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2]))
            
    sys.stdout.write(CLEAR_REST_OF_SCREEN)
    sys.stdout.flush()
    cnt += 1
