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
        if w[0] in ['zero', 'home', 'test']: self.shift(w[0])
        elif w[0] in ['joint', 'xmanus']: self.v = np.array(w[1:], dtype=float); self.shift(w[0])
        elif w[0] == 'mcheck': self.v = int(w[1]); self.shift(w[0])
        
    def update(self, y):
        q, qd = y[:12], y[12:]
        tau = np.zeros(12)

        if self.s == 'zero':
            if self.t == 0: self.trj1.target(np.array([[0.0, 0.5, 0.0, 0.0,   0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0]]), [700], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)

        elif self.s == 'home':
            if self.t == 0: self.trj1.target(np.array([[0.4, 1.0, 0.6, 0.6,   0.5, 0.5,  0.6, 0.6,  0.6, 0.6,  0.6, 0.6]]), [700], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)
            
        elif self.s == 'test':
            if self.t == 0: self.trj1.target(np.array([[-1.7, 2.5, 0.0, 0.0,   0.5, 0.5,   0.0, 0.0,   0.0, 0.0,   0.0, 0.0]]), [1000], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)

        elif self.s == 'joint':
            if self.t == 0: self.trj1.target(self.v.reshape((1, 12)), [1000], q, self.T)
            q_d = self.trj1.generate()
            tau = self.pid.update(q_d, q, qd)
            
        #for data glove streaming
        elif self.s == 'xmanus':
            tau = self.pid.update(self.v, q, qd)
            
        elif self.s == 'mcheck':
            if self.t < 100: tau[self.v] = 0.4

        #print(self.m.fk(self.frame, q))   
        #time.sleep(0.1)

        if self.verbose:
            deg = q*180/np.pi
            print('[%8d] %6.2f %6.2f %6.2f %6.2f  %6.2f %6.2f  %6.2f %6.2f  %6.2f %6.2f  %6.2f %6.2f' %(self.T, deg[0], deg[1], deg[2], deg[3], deg[4], deg[5], deg[6], deg[7], deg[8], deg[9], deg[10], deg[11]))
        
        self.one_step_forward()
        return tau, None, None, None, None
