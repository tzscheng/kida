import numpy as np, tact

class Controller:
    n_y = 18 #number of outputs
    n_u = 9 #number of control input

    def __init__(self, env, ymlname, prefix='', rate=None, verbose=False):
        self.has_pd = env.has_pd
        self.verbose = verbose

        self.m = tact.Model(ymlname)
        self.env = env
        self.prefix = prefix

        # rate = control loop ticks/sec; runner passes this. Fall back to 240
        # (HW pacing) when None — covers direct construction and CEnv-real.
        self.rate = rate if rate is not None else 240

        self.shift(0)
        self.T = 0

        #self.sk = [0.20, 0.10, 0.00,   0.10, 0.05,   0.10, 0.05,   0.10, 0.05]

        if env.backend == 'real': kp = [2.0, 2.0, 2.0,   2.0, 2.0,   2.0, 2.0,   2.0, 2.0]; kd = [0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 0.0,   0.0, 0.0]
        else: kp = [0.5, 0.5, 0.5,   0.5, 0.5,   0.5, 0.5,   0.5, 0.5]; kd = [0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 0.0,   0.0, 0.0]
        
        self.pid = tact.PIDController(kp, kd, 0.0, 0.004)
        # Implicit joint-PD gains for the has_pd path (tact backend) — per-decision
        # control outputs, returned from update() as part of the 5-tuple command
        # (tau, q_ref, qd_ref, kp, kd); these attrs are just the gain table.
        # Same values the former YAML `k:` entries carried (gains are control
        # policy, not plant — they moved out of the YAML).
        self.kp = np.full(9, 0.5)   # former h9 yml k: [0.5, 0] (tact sim only;
        self.kd = np.zeros(9)       # real/mujoco ignore the kwargs)
        self.trj = tact.MovingAverageWaypointSmoother(1000)

    def shift(self, s):
        self.s = self.next_s = s
        self.t = 0

    def one_step_forward(self):
        if self.s != self.next_s: self.shift(self.next_s)
        else: self.t += 1
        self.T += 1
        
    def msgproc(self, w):
        if w[0] in ['zero', 'home', 'ready']:  self.shift(w[0])
        elif w[0] == 'joint' and len(w) == 10: self.v = np.array(w[1:], dtype=float); self.shift(w[0])
        #elif w[0] == 'jointdeg': self.v = (np.pi/180)*np.array(w[1:], dtype=float); self.shift(w[0])
        #elif w[0] == 'jointalldeg' and 0 <= float(w[1]) <= 50: self.v = (np.pi/180)*float(w[1]); self.shift(w[0])
        
    def update(self, y):
        q, qd = y[:9], y[9:]
        # Each state branch sets exactly one of (tau, q_ref). Unused channel stays None.
        tau = None
        q_ref = None

        if self.s == 'joint':
            if self.t == 0:
                e_eff = np.linalg.norm(self.v - q) #effective joint error
                duration = int(0.06*self.rate*e_eff) + 1
                self.trj.target(self.v.reshape((1, 9)), [duration], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        elif self.s == 'zero':
            if self.t == 0: self.trj.target(np.zeros((1, 9)), [1000], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        elif self.s == 'home':
            if self.t == 0: self.trj.target(np.array([[0.7, 0.7, 0.7,   0.3, 0.3,   0.5, 0.5,   0.7, 0.7]]), [1000], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        elif self.s == 'ready':
            if self.t == 0: self.trj.target(np.array([[1.2, 0.3, 0.3,   0.8, 0.8,   0.8, 0.8,   0.3, 0.3]]), [1000], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        if self.verbose:
            deg = q*180/np.pi
            print('[%8d] %6.2f %6.2f %6.2f   %6.2f %6.2f   %6.2f %6.2f   %6.2f %6.2f' %(self.T, deg[0], deg[1], deg[2], deg[3], deg[4], deg[5], deg[6], deg[7], deg[8]))

        self.one_step_forward()
        return tau, q_ref, None, self.kp, self.kd
