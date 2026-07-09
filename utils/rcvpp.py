import zmq, numpy as np #sys, time, argparse

#par = argparse.ArgumentParser()
#par.add_argument('-n', default=4, type=int, help='number of item')
#arg, src = par.parse_known_args()

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('ipc:///dev/shm/proprio')
sub.setsockopt(zmq.SUBSCRIBE, b'')

while True:
    payload = sub.recv()
    proprio_data = np.frombuffer(payload, dtype=np.float32)
    print(proprio_data)
