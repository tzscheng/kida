import numpy as np, tact

class Controller:
    n_y = 60 #number of outputs #pos, vel, current (activation torque)
    n_u = 20 #number of control input

    def __init__(self, env, ymlname, prefix='', rate=None, verbose=False):
        self.has_pd = env.has_pd
        # Real-HW override: dg5 (dg5f hand) is intrinsically position-controlled — its
        # firmware always runs onboard PD on the q_ref it receives. The hand always
        # emits q_ref regardless of the arm's mode.
        if env.backend == 'real': self.has_pd = True
        self.verbose = verbose

        self.m = tact.Model(ymlname)
        self.env = env
        self.prefix = prefix

        # rate = control loop ticks/sec; runner passes this. Fall back to 240
        # (HW pacing) when None — covers direct construction and CEnv-real.
        self.rate = rate if rate is not None else 240

        if   '-left'  in ymlname: self.home = np.array([-0.4,  0.4, -0.4, -0.4,   0.4, 0.4, 0.4, 0.4,   0.2, 0.4, 0.4, 0.4,  0, 0.4, 0.4, 0.4,  -0.2, -0.2, 0.4, 0.4])
        elif '-right' in ymlname: self.home = np.array([ 0.4, -0.4,  0.4,  0.4,  -0.4, 0.4, 0.4, 0.4,  -0.2, 0.4, 0.4, 0.4,  0, 0.4, 0.4, 0.4,   0.2,  0.2, 0.4, 0.4])

        self.shift(0)
        self.T = 0

        kp = np.array([1.0, 1.0, 1.0, 1.0]*5, dtype=float)
        kd = np.array([0.01, 0.01, 0.01, 0.01]*5, dtype=float)
        self.pid = tact.PIDController(kp, kd, 0, 0.005)
        # Implicit joint-PD gains for the has_pd path (tact backend) — per-decision
        # control outputs, returned from update() as part of the 5-tuple command
        # (tau, q_ref, qd_ref, kp, kd); these attrs are just the gain table.
        # Same values the former YAML `k:` entries carried (gains are control
        # policy, not plant — they moved out of the YAML).
        self.kp = kp
        self.kd = kd
        self.trj = tact.MovingAverageWaypointSmoother(20)
        self.joint_err_w = np.array([1.0, 1.0, 0.7, 0.5,  1.0, 1.0, 0.7, 0.5,  1.0, 1.0, 0.7, 0.5,  1.0, 1.0, 0.7, 0.5,  1.0, 1.0, 0.7, 0.5]) #joint error weight

    def shift(self, s):
        self.s = self.next_s = s
        self.t = 0

    def one_step_forward(self):
        if self.s != self.next_s: self.shift(self.next_s)
        else: self.t += 1
        self.T += 1

    def msgproc(self, w):
        if w[0] in ['zero', 'home']:  self.shift(w[0])
        elif w[0] == 'joint' and len(w) == 21: self.v = np.array(w[1:], dtype=float); self.shift(w[0])

    def update(self, y):
        q, qd, act = y[0:20], y[20:40], y[40:60]
        # Each state branch sets exactly one of (tau, q_ref). Unused channel stays None.
        tau = None
        q_ref = None

        if self.s == 'joint':
            if self.t == 0:
                e_eff = np.linalg.norm(self.v - q) #effective joint error
                duration = int(0.06*self.rate*e_eff) + 1
                self.trj.target(self.v.reshape((1, 20)), [duration], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        elif self.s == 'zero':
            if self.t == 0: self.trj.target(np.zeros((1, 20)), [600], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        elif self.s == 'home':
            if self.t == 0: self.trj.target(self.home.reshape((1, 20)), [600], q, self.T)
            if self.has_pd: q_ref = self.trj.generate()
            else:                tau   = self.pid.update(self.trj.generate(), q, qd)

        self.one_step_forward()
        return tau, q_ref, None, self.kp, self.kd
