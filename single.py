import numpy as np, tact

from hconf import attach_hand_payload

class Controller:
    n_y = 21 #number of outputs
    n_u = 7 #number of control input

    def __init__(self, env, ymlname, prefix='', rate=None, verbose=False, gripper=None):
        # TODO when has_pd=True is used (single-run -b): wrap the self.m.ik(...) calls
        # in update()'s task/home-task/task-loop branches with try/except RuntimeError
        # → fall back to self.q_ref_old. IK can fail at workspace boundary / near
        # singularities and currently crashes single-run. Skipped while we stay on
        # has_pd=False (cmode=0, JTC path) where IK is only called at __init__.
        #self.has_pd = False
        self.has_pd = env.has_pd
        self.verbose = verbose

        self.m = tact.Model(ymlname)
        # When a hand is selected, add its equivalent payload to the bare TCP.
        if gripper is not None:
            attach_hand_payload(self.m, 'tcp', gripper)
        self.env = env
        self.prefix = prefix

        # rate = control loop ticks/sec; runner passes this (single-run → 240).
        # When None (e.g. start with CEnv-real where dt isn't exposed), fall
        # back to the kida HW pacing (240 Hz, eio usleep(3000)).
        self.rate = rate if rate is not None else 240

        self.shift(0)
        self.T = 0
        
        kp = np.array([150, 150, 35, 35, 35, 15, 15], dtype=float)
        kd = np.array([4.0, 4.0, 1.0, 1.0, 1.0, 0.4, 0.4], dtype=float)
        self.pid = tact.PIDController(kp, kd, 0, 0.005)
        # Implicit joint-PD gains for the has_pd path (tact backend) — per-decision
        # control outputs, returned from update() as part of the 5-tuple command
        # (tau, q_ref, qd_ref, kp, kd); these attrs are just the gain table.
        # Same values the former YAML `k:` entries carried (gains are control
        # policy, not plant — they moved out of the YAML).
        self.kp = kp
        self.kd = kd
        self.trj1 = tact.MovingAverageWaypointSmoother(10) #joint space traj generator

        Kp = np.array([600, 600, 600, 10, 10, 10], dtype=float)
        Kd = np.array([10, 10, 10, 0.10, 0.10, 0.10], dtype=float)
        self.jtc = tact.JacobianTransposeController(self.m, {'tcp':'6d'}, Kp, Kd)
        self.task_Kp = Kp
        self.task_Kd = Kd
        self.trj2 = tact.MovingAverageWaypointSmoother(5) #task space traj generator

        #self.sk = np.array([0, 0, 2.5, 0, 0, 0, 0]) #spring stiffness
        #self.rq = np.array([0, 0, -0.1, 0, 0, 0, 0]) #reference - q
        self.sk = np.array([0, 2.5, 2.5, 0, 0, 0, 0]) #spring stiffness
        self.rq = np.array([0, 0.1, 0.0, 0, 0, 0, 0]) #reference - q
        
        self.joint_err_w = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.3, 0.3]) #joint error weight
        self.task_err_w = np.array([1.0, 1.0, 1.0, 0.2, 0.2, 0.2]) #task error weight
        self.ik_tolerance = 0.002
        
        if   '-left'  in ymlname: self.y_sign =  1.0
        elif '-right' in ymlname: self.y_sign = -1.0
        else: print('wrong ymlname=%s' %ymlname); exit(0)

        #self.init1 = [-1.0, 0.2, 0, 1.8, 0.2, 0.3, 0]
        #self.init2 = [-0.6, 0.2, 0, 1.7, 0.2, 0.3, 0]
        self.init1 = [-1.0, 0.2, 0, 2.1, 0.2, 0.3, 0]
        self.init2 = [-0.4, 0.2, 0, 1.7, 0.2, 0.3, 0]
        
        self.home_task = [0.30, self.y_sign*0.23, -0.40, 0, 0, 0]
        self.home = self.m.ik({'tcp':'6d'}, self.init2, self.home_task, tolerance=self.ik_tolerance)

        #for task-space loop test
        self.x_d1 = np.array([0.3, self.y_sign*0.15, -0.30, 0, 0, 0])
        self.x_d2 = np.array([0.3, self.y_sign*0.35, -0.30, 0, 0, 0])
        self.x_d3 = np.array([0.3, self.y_sign*0.35, -0.50, 0, 0, 0])
        self.x_d4 = np.array([0.3, self.y_sign*0.15, -0.50, 0, 0, 0])

        #for joint space loop test
        self.q_d1 = self.m.ik({'tcp':'6d'}, self.home, self.x_d1, tolerance=self.ik_tolerance)
        self.q_d2 = self.m.ik({'tcp':'6d'}, self.home, self.x_d2, tolerance=self.ik_tolerance)
        self.q_d3 = self.m.ik({'tcp':'6d'}, self.home, self.x_d3, tolerance=self.ik_tolerance)
        self.q_d4 = self.m.ik({'tcp':'6d'}, self.home, self.x_d4, tolerance=self.ik_tolerance)
        
    def shift(self, s):
        self.s = self.next_s = s
        self.t = 0

    #Project postural spring into null(J) so it doesn't disturb task-space JTC.
    #N = I - J^+ J via damped pinv (see kida.py:_null_space_postural for rationale).
    def _null_space_postural(self, q, J=None, damping=1e-4):
        if J is None: J = self.m.jacob({'tcp':'6d'}, q)
        tau = self.sk * (self.rq - q)
        JJt = J @ J.T + (damping*damping) * np.eye(J.shape[0])
        return tau - J.T @ np.linalg.solve(JJt, J @ tau)

    def one_step_forward(self):
        if self.s != self.next_s: self.shift(self.next_s)
        else: self.t += 1
        self.T += 1
        
    def msgproc(self, w):
        if w[0] == 'joint':
            if len(w[1:]) == 7:
                self.v = np.array(w[1:], dtype=float)
                lo = [-1.5, -1.5, -1.5, -1.0, -1.5, -1.5, -1.5]*2
                hi = [ 1.5,  1.5,  1.5,  2.5,  1.5,  1.5,  1.5]*2
                self.v = np.clip(self.v, lo, hi)
                self.shift(w[0])
            
        elif w[0] == 'task':
            if len(w[1:]) == 6:
                self.v = np.array(w[1:], dtype=float)
                lo = [ 0.0, -0.5, -0.6, -1.57, -1.30, -1.57]
                hi = [ 0.6,  0.5,  0.0,  1.57,  1.30,  1.57]
                self.v = np.clip(self.v, lo, hi)
                self.shift(w[0])

        elif w[0] in ['init', 'rest', 'home', 'home-task', 'joint-loop', 'task-loop']: self.shift(w[0])
        elif w[0] in ['free', 'gcomp'] and not self.has_pd: self.shift(w[0])
        elif w[0] == 'scan': self.scan()
                
    def update(self, y):
        q, qd, act = y[0:7], y[7:14], y[14:21]
        x = self.m.fk({'tcp':'6d'}, q)

        # Each state branch sets exactly one of (tau, q_ref). For 'free' (no branch fires)
        # has_pd holds last commanded position; external path stays at None (zero tau).
        tau = None
        q_ref = self.q_ref_old if (self.T > 0 and self.has_pd) else None

        if self.s == 'joint':
            if self.t == 0:
                e_eff = np.linalg.norm(self.joint_err_w*(self.v - q)) #effective joint error
                duration = int(4.0*self.rate*e_eff) + 1
                self.trj1.target(self.v.reshape((1, 7)), [duration], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'task':
            if self.t == 0:
                #e_eff = np.linalg.norm(self.task_err_w*self._task_error(q, self.v)) #effective task error
                e_eff = np.linalg.norm(self.task_err_w*self.m.error({'tcp':'6d'}, q, self.v)) #effective task error
                duration = int(4.0*self.rate*e_eff) + 1
                self.trj2.target(self.v.reshape((1, 6)), [duration], x, self.T)
            if self.has_pd:
                tau = self.m.gravity(q) + self._null_space_postural(q)
                q_ref = self.m.ik({'tcp':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp':'6d'}, q)
                #tau = self._task_tau(self.trj2.generate(), q, qd, J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                tau = self.jtc.update(self.trj2.generate(), q, qd, J=J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                
        elif self.s == 'init':
            if self.t == 0: self.trj1.target(np.array([self.init1, self.init2, self.home]), [2*self.rate, self.rate, self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'rest':
            if self.t == 0: self.trj1.target(np.array([self.init2, self.init1, [0]*7]), [self.rate, self.rate, 2*self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'home':
            if self.t == 0: self.trj1.target(np.array([self.home]), [2*self.rate], q, self.T)
            if self.has_pd:
                tau = self.m.gravity(q)
                q_ref = self.trj1.generate()
            else: tau = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)

        elif self.s == 'home-task':
            if self.t == 0: self.trj2.target(np.array([self.home_task]), [self.rate], x, self.T)
            if self.has_pd:
                tau = self.m.gravity(q) + self._null_space_postural(q)
                q_ref = self.m.ik({'tcp':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp':'6d'}, q)
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
                q_ref = self.m.ik({'tcp':'6d'}, q, self.trj2.generate(), tolerance=self.ik_tolerance)
            else:
                J = self.m.jacob({'tcp':'6d'}, q)
                #tau = self._task_tau(self.trj2.generate(), q, qd, J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                tau = self.jtc.update(self.trj2.generate(), q, qd, J=J) + self.m.gravity(q) + self._null_space_postural(q, J=J)
                
        elif self.s == 'gcomp':
            # gcomp only reachable when has_pd=False (gated in msgproc)
            tau = self.m.gravity(q)

        if self.verbose:
            #print('[%d] %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f' %(self.T, x[0], x[1], x[2], x[3], x[4], x[5]))
            print('[%d] %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f' %(self.T, q[0], q[1], q[2], q[3], q[4], q[5], q[6]))
            
        self.one_step_forward()
        self.q_ref_old = q_ref
        return tau, q_ref, None, self.kp, self.kd



        #elif self.s == 'task-ik': #IK based taskspace position control
        #    if self.t == 0:
        #        q_d = self.m.ik({'tcp':'6d'}, q, self.v, tolerance=self.ik_tolerance)
        #        e_eff = np.linalg.norm(self.joint_err_w*(q_d - q)) #effective joint error
        #        duration = int(1.0*self.rate*e_eff) + 1 #print(duration)
        #        self.trj1.target(q_d.reshape((1, 7)), [duration], q, self.T)
        #    u = self.pid.update(self.trj1.generate(), q, qd) + self.m.gravity(q)
