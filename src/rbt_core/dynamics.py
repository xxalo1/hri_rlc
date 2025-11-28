from __future__ import annotations

import numpy as np

from .kin import Kinematics
from common_utils import numpy_util as npu
from common_utils import FloatArray


class Dynamics():

    def __init__(self, kin: Kinematics[FloatArray],
                 g_vec: FloatArray | None = None
                 ) -> None:
        self.kin = kin
        if g_vec is None:
            g_vec = np.array([0.0, 0.0, -9.81], dtype=npu.dtype)
        self.g_vec = g_vec


    @staticmethod
    def _cross(a: FloatArray, b: FloatArray) -> FloatArray:
        """Small wrapper: cross product for (3,) vectors."""
        return np.cross(a, b, axis=-1)


    def Dynamics_matrices(self) -> tuple[FloatArray, FloatArray]:
        """Jacobian/energy-form M(q)."""
        M = self.inertia_matrix()
        tau_g = self.gravity_vector()
        return M, tau_g


    def inertia_matrix(self) -> FloatArray:
        """
        Compute the joint-space inertia matrix M(q).

        This method uses the link masses, center-of-mass Jacobians, and world-frame
        inertia tensors provided by the associated Kinematics object at its current
        configuration self.kin.q.

        Internally, M(q) is assembled from the translational and rotational
        kinetic-energy contributions of each link, expressed via the COM Jacobian:

            M(q) = Σ_i [ m_i Jv_i(q)^T Jv_i(q) + Jw_i(q)^T I_i(q) Jw_i(q) ]

        To evaluate M at a different configuration, call self.kin.step(q=...) before
        calling this method.

        Returns
        -------
        M : ndarray, shape (n, n)
            Symmetric joint-space inertia matrix at the current configuration.
        """
        m  = self.kin.mass
        J_com = self.kin.jac_com()
        Ic_wl = self.kin.Ic_wl
        Jv_com = J_com[:, :3, :]         # (n, 3, n)
        Jw     = J_com[:, 3:, :]         # (n, 3, n)

        Mv = np.einsum('ian, iam, i -> nm', Jv_com, Jv_com, m)
        Mw = np.einsum('ian, iab, ibm -> nm', Jw, Ic_wl, Jw)
        
        M = Mv + Mw
        M = 0.5 * (M + M.T)  # tidy symmetry
        return M


    def gravity_vector(self) -> FloatArray:
        """Gravity vector tau_g(q)."""
        J_com = self.kin.jac_com()
        m = self.kin.mass
        g = self.g_vec
        Jv_com = J_com[:, :3, :]         # (n, 3, n)
        tau_g = np.einsum('i, ikn , k -> n', m, Jv_com, g)  # (n,)
        return -tau_g


    def rnea(self,
        q: FloatArray,
        qd: FloatArray,
        qdd: FloatArray,
    ) -> FloatArray:
        """
        Recursive Newton-Euler inverse dynamics.

        Computes joint torques tau(q, qd, qdd) for a fixed-base, all-revolute
        manipulator using link quantities expressed in the WORLD frame.

        Assumes:
            - joint i rotates about z_{i-1} (z-axis of frame i-1)
            - forward_kinematics() returns T_wf[k] for frames k = 0..n
                (0: base, k: joint k origin)
            - kin.mass, kin.com_wl, kin.Ic_wl correspond to links 1..n.

        Parameters
        ----------
        q   : (n,) joint positions
        qd  : (n,) joint velocities
        qdd : (n,) joint accelerations

        Returns
        -------
        tau : (n,) joint torques
        """
        dtype = q.dtype
        n = self.kin.n

        T_wf = self.kin.forward_kinematics()            # (n+1, 4, 4)
        p = T_wf[:, :3, 3]                              # (n+1, 3) origins
        z = T_wf[:, :3, 2]                              # (n+1, 3) z-axes

        m = self.kin.mass                               # (n,)
        com_wl = self.kin.com_wl                        # (n, 3)
        Ic_wl = self.kin.Ic_wl                          # (n, 3, 3)

        w   = np.zeros((n, 3), dtype=dtype)        # angular velocity
        dw  = np.zeros((n, 3), dtype=dtype)        # angular accel
        dvJ = np.zeros((n, 3), dtype=dtype)        # linear accel at joint origin
        a_c = np.zeros((n, 3), dtype=dtype)        # linear accel at COM

        # base (frame 0) state
        w_prev  = np.zeros(3, dtype=dtype)
        dw_prev = np.zeros(3, dtype=dtype)
        dv_prev = -self.g_vec                # a_0 = -g

        for i in range(n):
            # joint i+1: parent frame index = i, joint frame index = i+1
            idx_parent = i
            idx_joint = i + 1

            z_im1 = z[idx_parent]                       # axis of joint i+1 in world
            qdi   = qd[i]
            qddi  = qdd[i]

            # angular velocity & acceleration
            w_i = w_prev + qdi * z_im1
            dw_i = dw_prev + qddi * z_im1 + self._cross(w_prev, qdi * z_im1)

            # linear accel of joint origin i+1
            r_prev_to_joint = p[idx_joint] - p[idx_parent]
            dv_i = (
                dv_prev
                + self._cross(dw_prev, r_prev_to_joint)
                + self._cross(w_prev, self._cross(w_prev, r_prev_to_joint))
            )

            # COM accel for link i+1
            r_joint_to_com = com_wl[i] - p[idx_joint]
            a_c_i = (
                dv_i
                + self._cross(dw_i, r_joint_to_com)
                + self._cross(w_i, self._cross(w_i, r_joint_to_com))
            )

            w[i]   = w_i
            dw[i]  = dw_i
            dvJ[i] = dv_i
            a_c[i] = a_c_i

            # propagate for next link
            w_prev, dw_prev, dv_prev = w_i, dw_i, dv_i

        # --- link wrenches in world frame ---

        F = np.zeros((n, 3), dtype=dtype)          # linear force at COM
        N_c = np.zeros((n, 3), dtype=dtype)        # moment about COM

        for i in range(n):
            I_i = Ic_wl[i]                              # (3, 3)
            w_i = w[i]
            dw_i = dw[i]
            a_c_i = a_c[i]

            F_i = m[i] * a_c_i
            N_ci = I_i @ dw_i + self._cross(w_i, I_i @ w_i)

            F[i] = F_i
            N_c[i] = N_ci

        # --- backward recursion: joint torques ---

        f = np.zeros((n, 3), dtype=dtype)          # net force at joint origin
        n_m = np.zeros((n, 3), dtype=dtype)        # net moment at joint origin
        tau = np.zeros(n, dtype=dtype)

        for i in reversed(range(n)):
            idx_joint = i + 1    # origin of joint i+1
            # vector from joint origin to COM
            r_joint_to_com = com_wl[i] - p[idx_joint]

            # moment about joint origin from this link alone
            N0_i = N_c[i] + self._cross(r_joint_to_com, F[i])

            if i == n - 1:
                # last link: no child
                f_i = F[i]
                n_i = N0_i
            else:
                # include child wrench transported to this joint origin
                f_child = f[i + 1]
                n_child = n_m[i + 1]

                r_joint_to_next = p[idx_joint + 1] - p[idx_joint]

                f_i = F[i] + f_child
                n_i = N0_i + n_child + self._cross(r_joint_to_next, f_child)

            f[i] = f_i
            n_m[i] = n_i

            # joint axis for joint i+1 is z_{i} (parent frame)
            axis = z[i]
            tau[i] = np.dot(axis, n_i)

        return tau


    def dyn_pinv(self, J_task: FloatArray) -> tuple[FloatArray, FloatArray]:
        """
        Dynamically consistent pseudo-inverse of a task Jacobian.

        J_dyn = M^{-1} J^{T} (J M^{-1} J^{T})^{-1}

        Parameters
        ----------
        J_task : ndarray, shape (m, n) 
            task Jacobian

        Returns
        -------
        J_dyn : ndarray, shape (n, m)
            Dynamic pseudo-inverse of J_task.
        Lambda : ndarray, shape (m, m)
            Task-space inertia matrix Λ = (J M^{-1} J^{T})^{-1}.
        """
        M = self.inertia_matrix()

        # Solve M X = J^T  → X = M^{-1} J^T   (n, m)
        X = np.linalg.solve(M, J_task.T)

        # Λ^{-1} = J M^{-1} J^{T} = J X       (m, m)
        Lambda_inv = J_task @ X
        Lambda = np.linalg.inv(Lambda_inv)

        # J_dyn = M^{-1} J^{T} Λ = X Λ        (n, m)
        J_dyn = X @ Lambda

        return J_dyn, Lambda


    def task_to_joint(self,
        xdd: FloatArray,
        J_task: FloatArray,
        xdd_sec: FloatArray | None = None,
        J_sec: FloatArray | None = None,
    ) -> FloatArray:
        """
        Map main + secondary task accelerations to a joint acceleration:

            qdd* = J1_dyn xdd  +  N1 (J2_pinv xdd_sec)

        where
            J1_dyn  = dynamic pseudo-inverse for main task
            J2_pinv = (possibly) kinematic pseudo-inverse for secondary
            N1      = I - J1_dyn J1

        Parameters
        ----------
        xdd : ndarray, shape (m1,)  
            desired accel in main task space
        J_task : ndarray, shape (m1, n) 
            main task Jacobian
        xdd_sec : ndarray, shape (m2,), optional
            secondary task accel
        J_sec : ndarray, shape (m2, n), optional 
            secondary task Jacobian

        Returns
        -------
        qdd_star : ndarray, shape (n,)
        """
        n = J_task.shape[1]
        I = np.eye(n, dtype=xdd.dtype)

        J1_dyn, Lambda1 = self.dyn_pinv(J_task)   # (n, m1), (m1, m1)
        qdd_main = J1_dyn @ xdd                     # (n,)
        
        qdd_sec = np.zeros(n, dtype=xdd.dtype)
        
        if xdd_sec is not None and J_sec is not None:
            # simplest: use kinematic pseudoinverse for secondary
            # J2_pinv = J2^T (J2 J2^T)^-1
            J2 = J_sec
            JJt = J2 @ J2.T                    # (m2, m2)
            JJt_inv = np.linalg.inv(JJt)
            J2_pinv = J2.T @ JJt_inv           # (n, m2)

            qdd_sec_candidate = J2_pinv @ xdd_sec  # (n,)

            # nullspace projector of main task
            N1 = I - J1_dyn @ J_task

            qdd_sec = qdd_sec + N1 @ qdd_sec_candidate
        
        qdd_star = qdd_main + qdd_sec

        return qdd_star
