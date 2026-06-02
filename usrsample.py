import numpy as np, zmq, cv2

SERVER = None  # None: local IPC. Remote: set to kida host IP, e.g. '192.168.0.10'
#SERVER = '192.168.0.2'

cmd_ep     = 'tcp://%s:5555' %SERVER if SERVER else 'ipc:///dev/shm/default'
proprio_ep = 'tcp://%s:5556' %SERVER if SERVER else 'ipc:///dev/shm/proprio'
headcam_ep = 'tcp://%s:5557' %SERVER if SERVER else 'ipc:///dev/shm/headcam'

ctx = zmq.Context()
push = ctx.socket(zmq.PUSH) #push socket for slave command
push.connect(cmd_ep)

sub1 = ctx.socket(zmq.SUB) #subscribe socket for proprioception
sub1.connect(proprio_ep)
sub1.setsockopt(zmq.SUBSCRIBE, b'')

sub2 = ctx.socket(zmq.SUB) #subscribe socket for RGB image
sub2.connect(headcam_ep) #'headcam'(5557), 'leftcam'(5558), 'rightcam'(5559) available
sub2.setsockopt(zmq.SUBSCRIBE, b'')

poller = zmq.Poller() #receiveing sockets event poller
poller.register(sub1, zmq.POLLIN)
poller.register(sub2, zmq.POLLIN)
cnt = 0

while True:
    try: events = dict(poller.poll(timeout=100))  # ms
    except zmq.ZMQError: break

    #read proprioception(4byte x 162 @ 120Hz in real robot (30Hz in simulation))
    if sub1 in events:
        payload = sub1.recv()
        proprio = np.frombuffer(payload, dtype=np.float32)

        #'proprio' contains
        # - dual-arm joint-pos(14)
        # - dual-arm joint-vel(14)
        # - dual-arm joint-current(14)
        # - left-hand joint pos(20)
        # - left-hand joint vel(20)
        # - left-hand joint current(20)
        # - right-hand joint pos(20)
        # - right-hand joint vel(20)
        # - right-hand joint current(20)
        # => total 162 * np.float32

        #print first five per one second
        if cnt%30 == 0: print('proprioception: %6.2f %6.2f %6.2f %6.2f %6.2f ...' %(proprio[0], proprio[1], proprio[2], proprio[3], proprio[4]))
        continue

    #read headcam image (640x480@30Hz jpeg)
    elif sub2 in events:
        payload = sub2.recv()

        #display image
        img = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            cv2.imshow('headcam', img)
            cv2.waitKey(1)

        cnt += 1


    #do robot task example....
    if cnt == 100:
        #initialization motion (raising arms and make home pose)
        print('1. set initial position...')
        cmd = 'init, home, home'
        push.send(cmd.encode())

    elif cnt == 300:
        #test joint motion
        print('2. set test joint position...')
        dual_arm_cmd   = 'joint -0.5  0.1 -0.1  2.0  0.1  0.1  0.1     -0.5  0.1 -0.1  2.0  0.1  0.1  0.1'
        #left_hand_cmd  = 'joint -0.5  1.0 -0.5 -0.5   0.2 0.5 0.5 0.5   0.1 0.5 0.5 0.5   0.0 0.5 0.5 0.5  -0.2 -0.2 0.5 0.5'
        #right_hand_cmd = 'joint  0.5 -1.0  0.5  0.5  -0.2 0.5 0.5 0.5  -0.1 0.5 0.5 0.5  -0.0 0.5 0.5 0.5   0.2  0.2 0.5 0.5'
        left_hand_cmd  = 'joint  0 0 0 0   0 0 0 0   0 0 0 0   0 0 0 0   0 0 0 0'
        right_hand_cmd = 'joint  0 0 0 0   0 0 0 0   0 0 0 0   0 0 0 0   0 0 0 0' 
        cmd = dual_arm_cmd + ', ' + left_hand_cmd + ', ' + right_hand_cmd 
        push.send(cmd.encode())

    elif cnt == 500:
        #test task motion
        print('3. set test task-space position...')
        cmd = 'task  0.30 0.25 -0.40 0.0 0.0 0.0   0.30 -0.25 -0.40 0.0 0.0 0.0, home, home'
        push.send(cmd.encode())

    elif cnt == 700:
        #finishing motion (arms down)
        print('4. set rest position...')
        cmd = 'rest'
        push.send(cmd.encode())

    elif cnt == 800:
        #shutdown the slave
        print('5. fisnish...')
        print('Print ESC on window to quit..')
        #cmd = 'quit'
        #push.send(cmd.encode())
        break;
