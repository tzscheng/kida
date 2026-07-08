#!/usr/bin/env -S uv run python
# -*- mode: python -*-
import sys, os, tty, termios, select, argparse, time, atexit, numpy as np
import subprocess, zmq
import triad_openvr, tact

# pymanus.so is built into ../../fgx/manus by fg's dev/manus build (next to the
# Manus SDK it links against); make it importable regardless of cwd.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'fgx', 'manus'))
import pymanus

def parse_args():
    par = argparse.ArgumentParser()
    par.add_argument('-t', default=-1, type=int, help='operation type: [0]kida-left  [1]kida-right  [2]kida  [5]gos10')
    par.add_argument('-g', default=-1, type=int, help='gripper type: [0]H9  [1]DG5F')
    par.add_argument('-r', action='store_true', default=False, help='start receiver')
    par.add_argument('-u', action='store_true', default=False, help='use UDP instead of zeromq push')
    par.add_argument('-n', action='store_true', default=False, help='skip Manus glove calibration (use SDK defaults)')
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

arg = parse_args()
if arg.t < 0: print('choose operation type...'); exit(0)

q = np.zeros(40, dtype=np.float32) #hand target joint
feed_cnt = 0

# Glove ingest: pymanus runs the Manus SDK in-process (was a separate
# manus-udp daemon + UDP 6633 in the original). Calib paths/filenames here
# match the C++ vmaster (../manus/calib/*.mcal).
if arg.g >= 0:
    if   arg.g == 0: n_joint = 9
    elif arg.g == 1: n_joint = 20

    # The Manus SDK (spdlog) emits a stream of [info]/[warning] lines that
    # would clobber our cursor-home redraw. Stash the real terminal as
    # Python-level sys.stdout/sys.stderr so print() still reaches the user,
    # then redirect fd 1 / fd 2 (where C-level printf/fprintf goes) into a
    # log file. Override the path via PYMANUS_LOG=...; `tail -f` it to see
    # SDK diagnostics.
    sys.stdout.flush(); sys.stderr.flush()
    _log_path = os.environ.get('PYMANUS_LOG',
                               os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'fg', 'dev', 'manus', 'pymanus.log'))
    sys.stdout = os.fdopen(os.dup(1), 'w', buffering=1)
    sys.stderr = os.fdopen(os.dup(2), 'w', buffering=1)
    _log_fd = os.open(_log_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    os.dup2(_log_fd, 1)
    os.dup2(_log_fd, 2)
    os.close(_log_fd)
    print('[pymanus] SDK log -> %s' % _log_path)

    pymanus.start(skip_calib=arg.n)
    atexit.register(pymanus.stop)

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
# 데이터 소켓 (매-프레임 task 프레임 송신용): CONFLATE on — 최신 자세만 살리고
# 중간 프레임은 버려도 OK.
# 컨트롤 소켓 (1회성 명령 송신용): CONFLATE off — 컨트롤 메시지가 뒤이은 task
# 프레임에 silently 덮어써져 사라지는 일을 막는다. 데이터/컨트롤 두 PUSH 가
# 같은 endpoint 에 connect 하면 peer 의 PULL 이 두 connection 을 fair-queue 로
# 받아주므로 logger/slave 코드 변경은 불필요.
def make_push(endpoint, conflate):
    s = ctx.socket(zmq.PUSH)
    if conflate:
        # 데이터 소켓: 최신 자세만 살리고, 종료 시 잔여 task 프레임은 버려도 무방.
        s.setsockopt(zmq.IMMEDIATE, 1) # connected peer가 없으면 큐잉하지 않고 즉시 EAGAIN
        s.setsockopt(zmq.CONFLATE, 1)
        s.setsockopt(zmq.LINGER, 0)
    else:
        # 컨트롤 소켓은 ZMQ 기본값 (no conflate, no immediate, SNDHWM=1000). 보낼
        # 때 여전히 ZMQ_DONTWAIT 라 wedged peer 때문에 UI 가 막힐 일은 없음.
        # LINGER=200ms — close 시 마지막 "quit" 가 실제로 peer 의 kernel pipe
        # 로 flush 될 때까지 대기 (peer 가 사라졌으면 200ms 후 포기).
        s.setsockopt(zmq.LINGER, 200)
    s.connect(endpoint)
    return s

# slave/logger 쪽이 bind 하므로 master 는 connect.
slave_sock      = make_push('ipc:///dev/shm/default', conflate=True)
logger_sock     = make_push('ipc:///dev/shm/logger',  conflate=True)
slave_ctl_sock  = make_push('ipc:///dev/shm/default', conflate=False)
logger_ctl_sock = make_push('ipc:///dev/shm/logger',  conflate=False)
#slave_sock.connect('tcp://127.0.0.1:6600')
#logger_sock.connect('tcp://127.0.0.1:6611')

def _send(sock, data):
    try: sock.send(data, flags=zmq.DONTWAIT)
    except zmq.Again: pass

def send_slave(data):      _send(slave_sock,      data)
def send_logger(data):     _send(logger_sock,     data)
def send_slave_ctl(data):  _send(slave_ctl_sock,  data)
def send_logger_ctl(data): _send(logger_ctl_sock, data)
    
sys.stdout.write("\033[2J\033[H")
sys.stdout.flush()

offset = [np.zeros(3), np.zeros(3)]
_xyz = [np.zeros(3), np.zeros(3)] #xyz position before offset operation
xyz  = [np.zeros(3), np.zeros(3)] #xyz position after  offset operation
rpy  = [np.zeros(3), np.zeros(3)]
rpy_raw = [np.zeros(3), np.zeros(3)]   # before R_off applied; UI only
bias  = [np.zeros(3), np.zeros(3)]
scale = [1.0, 1.0]
# Rotation analogue of (offset, bias). R_out = R_m · R_off, with
# R_off = R_m_at_attach^T · R_bias captured on attach. R_bias holds the
# slave's last commanded orientation (identity when unknown — first attach
# after start may still jump; subsequent attach/detach cycles will be smooth).
R_m_cur = [np.eye(3), np.eye(3)]
R_off   = [np.eye(3), np.eye(3)]
R_bias  = [np.eye(3), np.eye(3)]
R_out   = [np.eye(3), np.eye(3)]
if   arg.t == 0: bias[0] = np.array([0.30,  0.23, -0.40]); scale[0] = 1.2
elif arg.t == 1: bias[1] = np.array([0.30, -0.23, -0.40]); scale[1] = 1.2
elif arg.t == 2:
    bias[0] = np.array([0.30,  0.23, -0.40]); scale[0] = 1.2
    bias[1] = np.array([0.30, -0.23, -0.40]); scale[1] = 1.2
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
            send_slave_ctl(b'randomize')
        elif key == 'r':
            if   arg.t in [0, 1]: send_slave_ctl(b'rest, home')
            elif arg.t == 2:      send_slave_ctl(b'rest, home, home')
            ee_attach = False; hand_attach = False
        elif key == '\x1b':
            # 컨트롤 소켓 LINGER 가 close 시 quit 전달을 보장 — explicit sleep 불필요.
            send_logger_ctl(b'quit'); break #ESC key
        elif key == 'z':
            #jawpos +=  0.02;
            #if jawpos > 0.10: jawpos = 0.10
            jawpos = 0.10
        elif key == 'x':
            #jawpos += -0.02;
            #if jawpos < 0.0: jawpos = 0.0
            jawpos = 0.02
        elif key == 'q':
            send_slave_ctl(b'quit')
            send_logger_ctl(b'quit')
        elif key == 'c':
            if log_on: log_on = False; send_logger_ctl(b'log-off')
            else:      log_on = True;  send_logger_ctl(b'log-on')
        elif key == 'b':
            hand_attach = not hand_attach
        elif key == 'a':
            was_attached = ee_attach
            ee_attach = not ee_attach
            if was_attached:
                # detach: save current output xyz/rpy so re-attach resumes from here;
                # also reset offset/R_off so the displayed pose stays at the saved bias.
                if arg.t in [0, 2, 5]:
                    bias[0] = xyz[0].copy(); offset[0] = -_xyz[0]
                    R_bias[0] = R_out[0].copy(); R_off[0] = R_m_cur[0].T @ R_bias[0]
                if arg.t in [1, 2]:
                    bias[1] = xyz[1].copy(); offset[1] = -_xyz[1]
                    R_bias[1] = R_out[1].copy(); R_off[1] = R_m_cur[1].T @ R_bias[1]
            else:
                # attach: capture current tracker pose as origin so output starts at bias / R_bias.
                if arg.t in [0, 2, 5]:
                    offset[0] = -_xyz[0]
                    R_off[0] = R_m_cur[0].T @ R_bias[0]
                if arg.t in [1, 2]:
                    offset[1] = -_xyz[1]
                    R_off[1] = R_m_cur[1].T @ R_bias[1]
        elif key == 'h': #go to home pose
            send_slave_ctl(b'home')
            jawpos = 0.1
            # robot returned to home: restore bias to per-mode home values,
            # and reset offset so the displayed xyz stays at the new bias.
            # R_bias defaults to identity — replace with the slave's actual
            # EE orientation at home if/when those values are known.
            if   arg.t == 0:
                bias[0] = np.array([0.30,  0.23, -0.40]); offset[0] = -_xyz[0]
                R_bias[0] = np.eye(3); R_off[0] = R_m_cur[0].T @ R_bias[0]
            elif arg.t == 1:
                bias[1] = np.array([0.30, -0.23, -0.40]); offset[1] = -_xyz[1]
                R_bias[1] = np.eye(3); R_off[1] = R_m_cur[1].T @ R_bias[1]
            elif arg.t == 2:
                bias[0] = np.array([0.30,  0.23, -0.40]); offset[0] = -_xyz[0]
                bias[1] = np.array([0.30, -0.23, -0.40]); offset[1] = -_xyz[1]
                R_bias[0] = np.eye(3); R_off[0] = R_m_cur[0].T @ R_bias[0]
                R_bias[1] = np.eye(3); R_off[1] = R_m_cur[1].T @ R_bias[1]
            elif arg.t == 5:
                bias[0] = np.array([0.50, -0.15,  0.30]); offset[0] = -_xyz[0]
                R_bias[0] = np.eye(3); R_off[0] = R_m_cur[0].T @ R_bias[0]
        elif key == 'i':
            if arg.t in [0, 1]: cmd = 'init, home'
            elif arg.t == 2: cmd = 'init, home, home'
            send_slave_ctl(cmd.encode())
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
            R_m_cur[0] = T03[:3, :3]
            R_out[0]   = R_m_cur[0] @ R_off[0]
            rpy[0]     = tact.rotation_to_euler(R_out[0])
            rpy_raw[0] = tact.rotation_to_euler(R_m_cur[0])

        #kida right arm
        elif arg.t == 1 or (arg.t == 2 and i == 1):
            _xyz[1] = T03[0:3, 3]
            xyz[1] = scale[1]*(_xyz[1] + offset[1]) + bias[1]
            R_m_cur[1] = T03[:3, :3]
            R_out[1]   = R_m_cur[1] @ R_off[1]
            rpy[1]     = tact.rotation_to_euler(R_out[1])
            rpy_raw[1] = tact.rotation_to_euler(R_m_cur[1])

        #gos10
        elif arg.t == 5:
            _xyz[0] = T03[0:3, 3]
            xyz[0] = scale[0]*(_xyz[0] + offset[0]) + bias[0]
            R_m_cur[0] = T03[:3, :3]
            R_out[0]   = R_m_cur[0] @ R_off[0]
            rpy[0]     = tact.rotation_to_euler(R_out[0])
            rpy_raw[0] = tact.rotation_to_euler(R_m_cur[0])

    # Snapshot retargeted glove joints (was a UDP-fed background thread).
    if arg.g >= 0: q = pymanus.get_q(arg.g)

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
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f   [ %7.3f %7.3f %7.3f ]' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], rpy_raw[0][0], rpy_raw[0][1], rpy_raw[0][2]))
        if   arg.g == 0: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]))
        elif arg.g == 1: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], q[14], q[15], q[16], q[17], q[18], q[19])) 
        
    elif arg.t == 1:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f   [ %7.3f %7.3f %7.3f ]' %(xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2], rpy_raw[1][0], rpy_raw[1][1], rpy_raw[1][2]))
        if   arg.g == 0: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28]))
        elif arg.g == 1: print('\nHAND-TARGET\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28], q[29], q[30], q[31], q[32], q[33], q[34], q[35], q[36], q[37], q[38], q[39])) 
        
    elif arg.t == 2:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f   [ %7.3f %7.3f %7.3f ]\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f   [ %7.3f %7.3f %7.3f ]' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], rpy_raw[0][0], rpy_raw[0][1], rpy_raw[0][2], xyz[1][0], xyz[1][1], xyz[1][2], rpy[1][0], rpy[1][1], rpy[1][2], rpy_raw[1][0], rpy_raw[1][1], rpy_raw[1][2]))
        if   arg.g == 0:
            print('\nL-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]))
            print('\nR-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28]))
        elif arg.g == 1:
            print('\nL-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13], q[14], q[15], q[16], q[17], q[18], q[19]))
            print('\nR-HAND-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f' %(q[20], q[21], q[22], q[23], q[24], q[25], q[26], q[27], q[28], q[29], q[30], q[31], q[32], q[33], q[34], q[35], q[36], q[37], q[38], q[39]))

    elif arg.t == 5:
        print('\nARM-TARGET\n%7.3f %7.3f %7.3f %7.3f %7.3f %7.3f   [ %7.3f %7.3f %7.3f ]' %(xyz[0][0], xyz[0][1], xyz[0][2], rpy[0][0], rpy[0][1], rpy[0][2], rpy_raw[0][0], rpy_raw[0][1], rpy_raw[0][2]))
            
    sys.stdout.write(CLEAR_REST_OF_SCREEN)
    sys.stdout.flush()
    cnt += 1
