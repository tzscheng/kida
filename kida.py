import numpy as np, tact

from hconf import attach_hand_payload

class Controller:
    n_y = 42 #number of outputs
    n_u = 14 #number of control input

    def __init__(self, env, ymlname, prefix='', rate=None, verbose=False, gripper=None):
        # TODO when has_pd=True is used (kida-run -b): wrap the self.m.ik(...) calls
        # in update()'s task/home-task/task-loop branches with try/except RuntimeError
        # → fall back to self.q_ref_old. IK can fail at workspace boundary / near
        # singularities and currently crashes kida-run. Skipped while we stay on
        # has_pd=False (cmode=0, JTC path) where IK is only called at __init__.
        #self.has_pd = False
        self.has_pd = env.has_pd
        self.verbose = verbose

        self.m = tact.Model(ymlname)
        # The arm-only model has no articulated hand. When one is selected,
        # represent it as an equivalent rigid payload at each TCP.
        if gripper is not None:
            attach_hand_payload(self.m, 'tcp1', gripper)
            attach_hand_payload(self.m, 'tcp2', gripper)
        self.env = env
        self.prefix = prefix

        # rate = control loop ticks/sec; runner passes this (kida-run → 240).
        # When None (e.g. start with CEnv-real where dt isn't exposed), fall
        # back to the kida HW pacing (240 Hz, eio usleep(3000)).
        self.rate = rate if rate is not None else 240

        self.shift(0)
        self.T = 0

        #kp = np.array([200, 200, 40, 40, 40, 15, 15]*2, dtype=float)
        #kd = np.array([4.0, 4.0, 1.0, 1.0, 1.0, 0.4, 0.4]*2, dtype=float)
        kp = np.array([150, 150, 35, 35, 35, 15, 15]*2, dtype=float)
        kd = np.array([4.0, 4.0, 1.0, 1.0, 1.0, 0.4, 0.4]*2, dtype=float)
        self.pid = tact.PIDController(kp, kd, 0, 0.005)
        # Implicit joint-PD gains for the has_pd path (tact backend) — per-decision
        # control outputs, returned from update() as part of the 5-tuple command
        # (tau, q_ref, qd_ref, kp, kd); these attrs are just the gain table.
        # Same values the former YAML `k:` entries carried (gains are control
        # policy, not plant — they moved out of the YAML).
        self.kp = kp
        self.kd = kd
        self.trj1 = tact.MovingAverageWaypointSmoother(10) #joint space trajectory

        Kp = np.array([600, 600, 600, 10, 10, 10]*2, dtype=float)
        Kd = np.array([10, 10, 10, 0.10, 0.10, 0.10]*2, dtype=float)
        #Kp = np.array([1500, 1500, 1500, 60, 60, 60]*2, dtype=float) # claude guide
        #Kd = np.array([80, 80, 80, 2.0, 2.0, 2.0]*2, dtype=float)
        self.jtc = tact.JacobianTransposeController(self.m, {'tcp1':'6d', 'tcp2':'6d'}, Kp, Kd)
        self.task_Kp = Kp
        self.task_Kd = Kd
        self.trj2 = tact.MovingAverageWaypointSmoother(5) #task space trajectory

        #self.sk = np.array([0, 0, 2.5, 0, 0, 0, 0]*2) #spring stiffness
        #self.rq = np.array([0, 0, -0.1, 0, 0, 0, 0]*2) #reference - q
        self.sk = np.array([0, 2.5, 0.0, 0, 0, 0, 0]*2) #spring stiffness
        self.rq = np.array([0, 0.1, 0.0, 0, 0, 0, 0]*2) #reference - q
        
        self.joint_err_w = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.3, 0.3]*2) #joint error weight
        self.task_err_w = np.array([1.0, 1.0, 1.0, 0.2, 0.2, 0.2]*2) #task error weight
        self.ik_tolerance = 0.002

        #self.init1 = [-1.0, 0.2, 0, 1.8, 0.2, 0.3, 0]*2
        #self.init2 = [-0.6, 0.2, 0, 1.7, 0.2, 0.3, 0]*2
        self.init1 = [-1.0, 0.2, 0, 2.1, 0.2, 0.3, 0]*2
        self.init2 = [-0.4, 0.2, 0, 1.7, 0.2, 0.3, 0]*2

        self.home_task = [0.30, 0.23, -0.40, 0, 0, 0, 0.30, -0.23, -0.40, 0, 0, 0]
        self.home = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, self.init2, self.home_task, tolerance=self.ik_tolerance)

        self.x_d1 = np.array([0.3, 0.15, -0.30, 0, 0, 0,   0.3, -0.35, -0.30, 0, 0, 0])
        self.x_d2 = np.array([0.3, 0.35, -0.30, 0, 0, 0,   0.3, -0.15, -0.30, 0, 0, 0])
        self.x_d3 = np.array([0.3, 0.35, -0.50, 0, 0, 0,   0.3, -0.15, -0.50, 0, 0, 0])
        self.x_d4 = np.array([0.3, 0.15, -0.50, 0, 0, 0,   0.3, -0.35, -0.50, 0, 0, 0])

        self.q_d1 = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, self.home, self.x_d1, tolerance=self.ik_tolerance)
        self.q_d2 = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, self.home, self.x_d2, tolerance=self.ik_tolerance)
        self.q_d3 = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, self.home, self.x_d3, tolerance=self.ik_tolerance)
        self.q_d4 = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, self.home, self.x_d4, tolerance=self.ik_tolerance)

    def shift(self, s):
        self.s = self.next_s = s
        self.t = 0

    def scan(self):
        #depth = self.env.raymap('%sheadcam' %self.prefix, 30, 20, 1.5)
        rgb = self.env.get_image('%sheadcam' %self.prefix)

    #Postural spring sk*(rq-q) projected into null(J) so it can't push the TCP.
    #N = I - J^+ J with damped pinv (J full row-rank in regular configs; damping
    #keeps it well-behaved near kinematic singularities). For 7-DOF redundant
    #arm, null(J) is 1-D per arm, so only the component of the elbow bias that
    #lies in that 1-D space survives — small but task-orthogonal.
    def _null_space_postural(self, q, J=None, damping=1e-4):
        if J is None: J = self.m.jacob({'tcp1':'6d', 'tcp2':'6d'}, q)
        tau = self.sk * (self.rq - q)
        JJt = J @ J.T + (damping*damping) * np.eye(J.shape[0])
        return tau - J.T @ np.linalg.solve(JJt, J @ tau)

    def one_step_forward(self):
        if self.s != self.next_s: self.shift(self.next_s)
        else: self.t += 1
        self.T += 1

    def msgproc(self, w):
        if w[0] == 'joint':
            if len(w[1:]) == 14:
                self.v = np.array(w[1:], dtype=float)
                lo = [-1.5, -1.5, -1.5, -1.0, -1.5, -1.5, -1.5]*2
                hi = [ 1.5,  1.5,  1.5,  2.5,  1.5,  1.5,  1.5]*2
                self.v = np.clip(self.v, lo, hi)
                self.shift(w[0])

        elif w[0] == 'task':
            if len(w[1:]) == 12:
                self.v = np.array(w[1:], dtype=float)
                lo = [ 0.0, -0.5, -0.6, -1.57, -1.30, -1.57]*2
                hi = [ 0.6,  0.5,  0.0,  1.57,  1.30,  1.57]*2
                self.v = np.clip(self.v, lo, hi)
                self.shift(w[0])

        elif w[0] in ['init', 'rest', 'home', 'home-task', 'joint-loop', 'task-loop']: self.shift(w[0])
        elif w[0] in ['free', 'gcomp'] and not self.has_pd: self.shift(w[0])
        elif w[0] == 'scan': self.scan()

    def update(self, y):
        q, qd = y[0:14], y[14:28]
        x = self.m.fk({'tcp1':'6d', 'tcp2':'6d'}, q)

        # Each state branch sets exactly one of (tau, q_ref). For 'free' (no branch fires)
        # has_pd holds last commanded position; external path stays at None (zero tau).
        tau = None
        q_ref = self.q_ref_old if (self.T > 0 and self.has_pd) else None

        if self.s == 'joint':
            if self.t == 0:
                e_eff = np.linalg.norm(self.joint_err_w*(self.v - q)) #effective joint error
                duration = int(4.0*self.rate*e_eff) + 1
                self.trj1.target(self.v.reshape((1, 14)), [duration], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'task':
            if self.t == 0:
                #e_eff = np.linalg.norm(self.task_err_w*self._task_error(q, self.v)) #effective task error
                e_eff = np.linalg.norm(self.task_err_w*self.m.error({'tcp1':'6d', 'tcp2':'6d'}, q, self.v)) #effective task error
                duration = int(4.0*self.rate*e_eff) + 1
                self.trj2.target(self.v.reshape((1, 12)), [duration], x, self.T)
            if self.has_pd:
                tau = self.m.gravity(q) + self._null_space_postural(q)
                q_ref = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp1':'6d', 'tcp2':'6d'}, q)
                #tau = self._task_tau(self.trj2.generate(), q, qd, J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                tau = self.jtc.update(self.trj2.generate(), q, qd, J=J) + self.m.gravity(q) + self._null_space_postural(q, J=J)

        elif self.s == 'init':
            if self.t == 0: self.trj1.target(np.array([self.init1, self.init2, self.home]), [2*self.rate, self.rate, self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'rest':
            if self.t == 0: self.trj1.target([self.init2, self.init1, [0]*14], [self.rate, self.rate, 2*self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'home':
            if self.t == 0: self.trj1.target([self.home], [2*self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'home-task':
            if self.t == 0: self.trj2.target(([self.home_task]), [self.rate], x, self.T)
            if self.has_pd:
                tau = self.m.gravity(q) + self._null_space_postural(q)
                q_ref = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp1':'6d', 'tcp2':'6d'}, q)
                #tau = self._task_tau(self.trj2.generate(), q, qd, J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                tau = self.jtc.update(self.trj2.generate(), q, qd, J=J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                
        elif self.s == 'joint-loop':
            if self.t % (self.rate*8) == 0: self.trj1.target(np.array([self.q_d1, self.q_d1, self.q_d2, self.q_d2, self.q_d3, self.q_d3, self.q_d4, self.q_d4]), [self.rate, self.rate, self.rate, self.rate, self.rate, self.rate, self.rate, self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'task-loop':
            if self.t % (self.rate*8) == 0: self.trj2.target(np.array([self.x_d1, self.x_d1, self.x_d2, self.x_d2, self.x_d3, self.x_d3, self.x_d4, self.x_d4]), [self.rate, self.rate, self.rate, self.rate, self.rate, self.rate, self.rate, self.rate], x, self.T)
            if self.has_pd:
                tau = self.m.gravity(q) + self._null_space_postural(q)
                q_ref = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp1':'6d', 'tcp2':'6d'}, q)
                #tau = self._task_tau(self.trj2.generate(), q, qd, J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                tau = self.jtc.update(self.trj2.generate(), q, qd, J=J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                
        elif self.s == 'gcomp':
            # gcomp only reachable when has_pd=False (gated in msgproc)
            tau = self.m.gravity(q)

        if self.verbose:
            #print('[%d] %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f' %(self.T, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11]))
            print('[%d] %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f | %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f' %(self.T, q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13]))

        self.one_step_forward()
        self.q_ref_old = q_ref
        return tau, q_ref, None, self.kp, self.kd


        #elif self.s == 'task-ik':
        #    if self.t == 0:
        #        q_d = self.m.ik({'tcp1':'6d', 'tcp2':'6d'}, q, self.v, tolerance=self.ik_tolerance)
        #        e_eff = np.linalg.norm(self.joint_err_w*(q_d - q)) #effective joint error
        #        duration = int(1.0*self.rate*e_eff) + 1 #print(duration)
        #        self.trj1.target(q_d.reshape((1, 14)), [duration], q, self.T)
        #    u = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)
