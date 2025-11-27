        # basic trajectory
        self.robot = init_kinova_robot()
        self.robot.ctrl.set_joint_gains(Kp=2.0, Kv=2.0, Ki=1.0)

        self.q0 = np.zeros(self.robot.kin.n, dtype=npu.dtype)
        self.qd0 = np.zeros_like(self.q0)
        self.qf = np.array([np.pi/4, -np.pi/2, np.pi/3, -np.pi/3, 0.0, np.pi/6, 0.0], dtype=npu.dtype)

        self.robot.setup_quintic_traj(freq=1000.0, ti=0.0, tf=5.0, q_des=self.qf)
        self.Q_EF_np = self.robot.get_ee_traj()

        dt_sim = self.env.m.opt.timestep          # MuJoCo integration step
        dt_ctrl = 1.0 / self.robot.freq           # 0.01 s for 100 Hz
        nsub = max(1, int(round(dt_ctrl / dt_sim)))
        self.env.nsub = nsub

        self.t_prev = 0.0
        self.control_mode = ControlMode.RNEA
        self.tracking_mode = TrackingMode.TRAJ
        self.track_current = False
        self.pt_set = False
