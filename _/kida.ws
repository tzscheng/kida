#!/usr/bin/env -S uv run python
# -*- mode: python -*-
import sys, os, argparse, ctypes, numpy as np, zmq
import tact, kida, dg5 as gmod

def parse_args():
    par = argparse.ArgumentParser()
    par.add_argument('-g', default=1, type=int, help='gripper type [0: h9, 1: dg5]')
    par.add_argument('-x', action='store_true', default=False, help='real')
    par.add_argument('-m', action='store_true', default=False, help='mujoco')
    par.add_argument('-v', action='store_true', default=False, help='verbose')
    par.add_argument('-b', action='store_true', default=False, help='use built-in pd controller (real h/w only)')
    return par.parse_args()

def msgproc(msg):
    global cnt
    w = [s.split() for s in msg.decode().split(',')]
    if w[0][0] == 'quit': env.finish(); sys.exit(0)
    elif w[0][0] == 'reset': cnt = 0; return
    if len(w) in (1, 2, 3):
        arms.msgproc(w[0])
        if len(w) >= 2: hand1.msgproc(w[1])
        if len(w) == 3: hand2.msgproc(w[2])
    else: print('wrong command')
    
try: os.sched_setaffinity(0, {0})
except Exception as e: print('set affinity failed'); sys.exit()

arg = parse_args()
if   arg.g == 0: gname = 'h9';  import h9 as gmod
elif arg.g == 1: gname = 'dg5'; import dg5 as gmod
else: print('wrong gripper type'); sys.exit()
cnt = 0

if arg.x: #start with real H/W
    cdll = ctypes.CDLL('eio/eio-kida.so')
    cmode = 1 if arg.b else 0
    cdll.init(b'%d 1' %cmode) #control mode, hand-type
    env = tact.CEnv(cdll, n_y=kida.Controller.n_y + 2*gmod.Controller.n_y, n_u=kida.Controller.n_u + 2*gmod.Controller.n_u, backend='real', has_pd=arg.b)
#elif arg.m:
#    cdll = ctypes.CDLL(f'{tact.pkg_dir}/bin/mjenv.so')
#    cdll.init('/home/ubuntu/uv1/fgx/mujoco/models/kida/mjmodel.xml'.encode(), 16)
#    env = tact.CEnv(cdll, n_y=..., backend='mujoco')
else: #start with tact simulator
    env = tact.Env('yml/kida', offset=[0, 0, 1.2, 0, 0, 0], render=True, redraw=16)
    env.add('yml/%s-left'  %gname, prefix='hand1.', base='tcp1', offset=[0.05, 0, 0, 0, 90, -90])
    env.add('yml/%s-right' %gname, prefix='hand2.', base='tcp2', offset=[0.05, 0, 0, 0, 90,  90])
    env.add('yml/desk1')
    env.set(dt=0.001, g=[0, 0, -9.81], view=[0, 0, -0.34+1.2, 1.3, 180, 20])
    env.edit('tcp1', m=0.6, c=[0.05, 0, 0])
    env.edit('tcp2', m=0.6, c=[0.05, 0, 0])

ctx = zmq.Context()
pull = ctx.socket(zmq.PULL)
pull.bind('ipc:///dev/shm/default')

ppub = ctx.socket(zmq.PUB)
ppub.setsockopt(zmq.CONFLATE, 1)
ppub.bind('ipc:///dev/shm/proprio')

if arg.x:
    proprio_update_cycle = 2  # about 120Hz
    rgb_update_cycle = 0      # no publish (use rs2/msender)
else:
    proprio_update_cycle = 33  # about 30Hz
    rgb_update_cycle = 33      # about 30Hz
    cam_name = ['headcam', 'leftcam', 'rightcam']
    cpub = []
    for i in range(len(cam_name)):
        cpub.append(ctx.socket(zmq.PUB))
        cpub[i].setsockopt(zmq.CONFLATE, 1)
        cpub[i].bind('ipc:///dev/shm/%s' %cam_name[i])

        
while True:
    try: msg = pull.recv(flags=zmq.NOBLOCK)
    except zmq.ZMQError: pass
    else: msgproc(msg); continue
    
    if cnt == 0:
        arms  = kida.Controller(env, 'yml/kida', verbose=arg.v)
        hand1 = gmod.Controller(env, 'yml/%s-left'  %gname, prefix='hand1.', verbose=False)
        hand2 = gmod.Controller(env, 'yml/%s-right' %gname, prefix='hand2.', verbose=False)
        y = env.reset()

    # Compose multi-controller output. Each returns (tau, q_ref, qd_ref) with each
    # channel either an array or None. combine: all-None → None, else concat with
    # zero-fill for None members so length matches n_u.
    def combine(parts, sizes):
        if all(p is None for p in parts): return None
        return np.concatenate([p if p is not None else np.zeros(s) for p, s in zip(parts, sizes)])

    arm_tau,  arm_qr,  arm_qdr  = arms.update(y[:arms.n_y])
    h1_tau,   h1_qr,   h1_qdr   = hand1.update(y[arms.n_y:arms.n_y+hand1.n_y])
    h2_tau,   h2_qr,   h2_qdr   = hand2.update(y[arms.n_y+hand1.n_y:])

    sizes = [arms.n_u, hand1.n_u, hand2.n_u]
    tau    = combine([arm_tau, h1_tau, h2_tau], sizes)
    q_ref  = combine([arm_qr,  h1_qr,  h2_qr],  sizes)
    qd_ref = combine([arm_qdr, h1_qdr, h2_qdr], sizes)
    y = env.step(tau=tau, q_ref=q_ref, qd_ref=qd_ref)
    
    #update priprio feedback
    if cnt % proprio_update_cycle == 0:
        proprio = y.astype(np.float32).tobytes()
        ppub.send(proprio)

    #update camera feedback (simulation only)
    if rgb_update_cycle > 0 and cnt % rgb_update_cycle == 0:
        for i in range(len(cam_name)):
            img = env.get_rgb_image(cam_name[i])
            if img != None: cpub[i].send(img)

    cnt += 1
