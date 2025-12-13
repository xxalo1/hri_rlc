import numpy as np

from .dynamics import Dynamics
from .kin import Kinematics
from common_utils import numpy_util as npu
from common_utils import FloatArray

class Controller:
    """Robot controller base class."""
    def __init__(self, 
        dynamics: Dynamics, 
        Kv: float | None = None, 
        Kp: float | None = None,
        Ki: float | None = None,
        Kx: float | None = None,
        Dx: float | None = None,
        Kix: float | None = None,
    ) -> None:
        
        self.dyn = dynamics

        # joint-space PID gains
        if Kv is None: Kv = 2.0
        if Kp is None: Kp = 5.0
        if Ki is None: Ki = 0.0
        self.set_joint_gains(Kp, Kv, Ki)

        # task-space gains [pos; ori] (6x6)
        if Kx is None: Kx = 1.0
        if Dx is None: Dx = 2.0
        if Kix is None: Kix = 0.0
        self.set_task_gains(Kx, Dx, Kix)

    @property
    def kin(self) -> Kinematics[FloatArray]:
        return self.dyn.kin

    @staticmethod
    def _orientation_error(R: FloatArray, R_des: FloatArray) -> FloatArray:
        """
        Compute orientation error between current and desired rotation.

        The error is defined in so(3) as::

            e_R = 0.5 * vee( R_des^T R - R^T R_des )

        where vee(.) maps a 3x3 skew-symmetric matrix to R^3.

        Parameters
        ----------
        R : ndarray, shape (3, 3)
            Current end effector rotation matrix in the world frame.
        R_des : ndarray, shape (3, 3)
            Desired end effector rotation matrix in the world frame.

        Returns
        -------
        e_R : ndarray, shape (3,)
            Orientation error vector in so(3).
        """
        R_err = R_des.T @ R

        return 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ], dtype=npu.dtype)


    def set_joint_gains(self,
        Kp: float | FloatArray,
        Kv: float | FloatArray,
        Ki: float | FloatArray = 0.0,
    ) -> None:
        """
        Set proportional, derivative, and integral gains.
        
        Parameters
        -----------
        Kp: float | Tensor, (n, n)
            Proportional gain.
        Kv: float | Tensor, (n, n)
            Derivative gain.
        Ki: float | Tensor, (n, n)
            Integral gain.
        
        --------
        Notes:
        Stores gains as Tensors in self.Kp, self.Kv, self.Ki.
        1- If a float is provided, it is multiplied by identity.
        2- otherwise, the provided Tensor is used directly.
        """
        n = self.kin.n
        I = np.eye(n, dtype=npu.dtype)

        if isinstance(Kp, int): Kp = float(Kp)
        if isinstance(Kv, int): Kv = float(Kv)
        if isinstance(Ki, int): Ki = float(Ki)
        if isinstance(Kp, float): Kp = Kp * I
        if isinstance(Kv, float): Kv = Kv * I
        if isinstance(Ki, float): Ki = Ki * I
        
        self.Kp = Kp
        self.Kv = Kv
        self.Ki = Ki
        self.e_int = np.zeros((n,), dtype=npu.dtype)


    def set_task_gains(self,
        Kx: float | FloatArray,
        Dx: float | FloatArray,
        Kix: float | FloatArray = 0.0
    ) -> None:
        """Set 6x6 task-space impedance gains for [position; orientation]."""
        I6 = np.eye(6, dtype=npu.dtype)

        if isinstance(Kx, float): Kx = Kx * I6
        if isinstance(Dx, float): Dx = Dx * I6
        if isinstance(Kix, float): Kix = Kix * I6

        self.Kx = Kx
        self.Dx = Dx
        self.Kix = Kix
        self.e_task_int = np.zeros(6, dtype=npu.dtype)


    def pid(self,
        q: FloatArray,
        qd: FloatArray,
        q_des: FloatArray,
        qd_des: FloatArray,
        dt: float,
    ) -> FloatArray:
        """
        Joint-space PID controller.

        Parameters
        ----------
        q  : ndarray, shape (n,)  
            current joint positions
        qd : ndarray, shape (n,)  
            current joint velocities
        q_des  : ndarray, shape (n,) 
            desired joint positions
        qd_des : ndarray, shape (n,) 
            desired joint velocities
        dt : float 
            time step since last call (s)

        Returns
        -------
        tau : ndarray, shape (n,) 
            torque command
        """

        e  = q_des  - q
        de = qd_des - qd

        self.e_int = self.e_int + e * dt

        tau_p = self.Kp @ e
        tau_d = self.Kv @ de
        tau_i = self.Ki @ self.e_int

        tau_g = self.dyn.gravity_vector()
        tau = tau_p + tau_d + tau_i + tau_g

        return tau


    def computed_torque(self, 
        q: FloatArray, 
        qd: FloatArray, 
        q_des: FloatArray, 
        qd_des: FloatArray,
        qdd_des: FloatArray,
    ) -> FloatArray:
        """Compute torque using computed torque control."""
        e = q_des - q
        de = qd_des - qd
        v = qdd_des + self.Kv @ de + self.Kp @ e

        tau = self.dyn.rnea(q, qd, v)

        return tau


    def impedance_ee(
        self,
        q: FloatArray,
        qd: FloatArray,
        T_des: FloatArray,
        xd_des: FloatArray | None,
        xdd_des: FloatArray | None,
        dt: float,
        xdd_sec: FloatArray | None = None,
        J_sec: FloatArray | None = None,
    ) -> FloatArray:
        """
        6D end effector impedance controller in task space.

        This controller regulates the end effector pose [position; orientation]
        in the world frame using an acceleration level impedance law in task
        space. The desired task acceleration is mapped to joint accelerations
        via a dynamic pseudo inverse, and joint torques are obtained from
        inverse dynamics.

        Parameters
        ----------
        q : ndarray, shape (n,)
            Current joint position.
        qd : ndarray, shape (n,)
            Current joint velocity.
        T_des : ndarray, shape (4, 4)
            Desired end effector pose in the world frame as a homogeneous
            transform.
        xd_des : ndarray, shape (6,), optional
            Desired end effector spatial velocity [v; omega] in the world
            frame. Default is zeros if None.
        xdd_des : ndarray, shape (6,), optional
            Desired end effector spatial acceleration in the world frame.
            Default is zeros if None.
        dt : float
            Controller sampling time in seconds, used to integrate the task
            space error for the integral term.
        xdd_sec : ndarray, shape (m2,), optional
            Desired secondary task acceleration in its own task space. This
            secondary task is injected in the dynamic nullspace of the main
            end effector task. Default is None.
        J_sec : ndarray, shape (m2, n), optional
            Jacobian of the secondary task. Default is None.

        Returns
        -------
        tau : ndarray, shape (n,)
            Joint torques computed by inverse dynamics to realize the desired
            main task and the secondary behavior.
        """

        if xd_des is None: xd_des = np.zeros(6, dtype=npu.dtype)
        if xdd_des is None: xdd_des = np.zeros(6, dtype=npu.dtype)

        self.kin.step(q=q, qd=qd)
        T_wf = self.kin.forward_kinematics()   # (n+1, 4, 4)
        T_ee = T_wf[-1]                        # last frame is EE

        R = T_ee[:3, :3]
        p = T_ee[:3, 3]

        R_des = T_des[:3, :3]
        p_des = T_des[:3, 3]

        # position and orientation errors
        e_pos = p_des - p
        e_ori = self._orientation_error(R, R_des)
        e = np.concatenate((e_pos, e_ori))    # (6,)

        # EE spatial velocity
        J = self.kin.full_jac()              # (n, 6, n), world-frame geometric Jacobian
        J_ee = J[-1, :, :]                   # (6, n), EE Jacobian
        xd = J_ee @ qd                     # (6,)

        de = xd_des - xd
        self.e_task_int += e * dt

        # desired task acceleration
        xdd_main = (
            xdd_des
            + self.Dx @ de
            + self.Kx @ e
            + self.Kix @ self.e_task_int
        )  # (6,)

        qdd_star = self.dyn.task_to_joint(
            xdd_main,
            J_ee,
            xdd_sec=xdd_sec,
            J_sec=J_sec,
        )

        tau = self.dyn.rnea(q, qd, qdd_star)

        return tau
