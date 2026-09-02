import numpy as np, tact

class Controller:
    n_y = 24 #number of outputs
    n_u = 12 #number of control input

    def __init__(self, env, ymlname, prefix='', rate=None, verbose=False):
        self.verbose = verbose
        self.rate = rate    # control loop ticks/sec; rate-aware logic TBD

        #self.m = tact.Model(ymlname)
        self.env = env
        self.prefix = prefix
        
        self.shift(0)
        self.T = 0
        
        kp = np.array([1.0, 1.0,   1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        kd = np.array([0.02, 0.02,   0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

        self.frame = {'tip1': '3d', 'tip2': '3d', 'tip3': '3d', 'tip4': '3d', 'tip5': '3d'}
        #self.jtc = tact.JacobianTransposeController(self.m, self.frame, 100, 0.5)
        self.pid = tact.PIDController(kp, kd, 0.0, 0.001)       
        #self.pid = tact.PIDController(0.2, 0.01, 0.0, 0.001)

        self.trj1 = tact.MovingAverageWaypointSmoother(100)
        self.trj2 = tact.MovingAverageWaypointSmoother(100)
        
    def shift(self, s):
        self.s = self.next_s = s
        self.t = 0

    def one_step_forward(self):
        if self.s != self.next_s: self.shift(self.next_s)
        else: self.t += 1
        self.T += 1
        
    def msgproc(self, w):
        if w[0] in ['zero', 'home', 'test0']: self.shift(w[0])
        elif w[0] in ['joint', 'xmanus']: self.v = np.array(w[1:], dtype=float); self.shift(w[0])
        elif w[0] == 'mcheck': self.v = int(w[1]); self.shift(w[0])
        
    def update(self, y):
        q, qd = y[:12], y[12:]
        tau = np.zeros(12)

        if self.s == 'zero':
            if self.t == 0: self.trj1.target(np.zeros((1, 12)), [700], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)

        elif self.s == 'home':
            if self.t == 0: self.trj1.target(np.array([[0.5, 1.0, 0.7, 0.7,   0.5, 0.5,  1.0, 1.0,  1.0, 1.0,  1.0, 1.0]]), [700], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)
            
        elif self.s == 'test0':
            if self.t == 0: self.trj2.target(np.array([[0.05, 0.0, 0.08,   0.03, 0.02, 0.13,   0.03, 0, 0.13,   0.03, -0.02, 0.13,   0.03, -0.04, 0.13]]), [1000], self.m.fk(self.frame, q), self.T)
            x_d = self.trj2.generate()
            tau = self.jtc.update(x_d, q, qd) #+ self.m.gravity(q)

        elif self.s == 'joint':
            if self.t == 0: self.trj1.target(self.v.reshape((1, 12)), [1000], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)
            
        #for data glove streaming
        elif self.s == 'xmanus':
            tau = self.pid.update(self.v, q, qd)
            
        elif self.s == 'mcheck':
            if self.t < 30: tau[self.v] = 0.1

        #print(self.m.fk(self.frame, q))   
        #time.sleep(0.1)
        
        self.one_step_forward()
        return tau, None, None, None, None
