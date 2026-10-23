"""
The calibrated kinematic model: ImprovedFwdKins / ImprovedInvKins.
"""

from typing import Dict, Optional

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame
from uncertainty_networks.se3 import log_so3

from ..robot import Robot
from .workflow import CalibrationData


class ImprovedKinematics(Robot):
    """
    TODO: implement -- the core deliverable.

    A Robot whose fwd_kins / inv_kins account for the calibrated error
    model. You choose the parameterization; the handout's error model is
        theta*_k = theta_k + delta_theta_k,sys(theta_k)   (all joints; ||.|| <= delta_sys, slope <= omega_sys)
        d*_k = R(phi_k) d_k                                (k = 2..6, ||phi_k|| <= phi_max; d*_1 = d_1)
        F*_we = F_we dF_ee                                 (||alpha_ee|| <= 0.01, ||epsilon_ee|| <= 0.5)
    plus the constant base frame F_X relating the robot base to the
    tracker (see workflow.estimate_base_frame). HomeRobot() gives you
    delta_theta_sys at home directly (workflow.measure_home_offsets); pass
    it as joint_offsets. Every parameter you include must be identifiable
    from your data -- look at the singular values of the Jacobian.

    A parameter vector that covers this model, for example:
        params[0:6]     xi_X          twist of the base frame about F_X_init, F_X = F_X_init * Exp(xi_X)
        params[6:16]    (t1, t2) x 5  tilt of d_2, d_3, n_4, n_5, n_6, two components each in the
                                      plane perpendicular to the nominal axis
        params[16:22]   xi_we         twist of the tool frame, F_we = F_WE * Exp(xi_we)
        params[22:]     c_k,j         per-joint distortion polynomial coefficients
                                      delta_theta_k,sys = offset_k + sum_j c_k,j (theta_k / scale_k)^j
    with every entry zero meaning "the nominal robot at F_X_init".

    The nominal Robot's attributes (AXES, F_CW, F_WE, limits, timing,
    wrist_solutions) are inherited and may be reused.
    """

    def __init__(self, robot: Robot, F_X_init: Frame, joint_offsets=None, n_poly: int = 2):
        super().__init__(robot.name + "_improved", robot.F_rob, robot.theta_min, robot.theta_max,
                         robot.t_settle, robot.theta_dot_max, robot.theta_ddot_max)
        self.F_X_init = F_X_init  # starting estimate of the base frame (from the nominal fit)
        self.joint_offsets = np.zeros(6) if joint_offsets is None else np.asarray(joint_offsets, dtype=np.float64).reshape(6)
        self.n_poly = n_poly      # e.g. the degree of a per-joint distortion polynomial
        self.params = self.initial_params()

    # ------------------------------------------------------------------ parameterization

    def initial_params(self) -> np.ndarray:
        """
        TODO: implement.

        output: the parameter vector that makes this model identical to the
                nominal kinematics with base frame F_X_init -- with the
                layout in the class docstring that is simply
                np.zeros(6 + 10 + 6 + 6 * self.n_poly)
        """
        raise NotImplementedError("TODO: implement initial_params")

    def base_frame(self, params: Optional[np.ndarray] = None) -> Frame:
        """
        TODO: implement.

        output: F_X (robot base relative to the tracker) for params (default:
                self.params) -- e.g. F_X_init * Exp(params[0:6]), see
                uFrame.perturb / uncertainty_networks.se3.exp_se3
        """
        raise NotImplementedError("TODO: implement base_frame")

    # ------------------------------------------------------------------ kinematics

    def fwd_kins(self, theta, params: Optional[np.ndarray] = None) -> Frame:
        """
        ImprovedFwdKins(theta).

        TODO: implement.

        input: theta - 6 encoder joint values; params - defaults to self.params
        output: F_ee relative to the robot base frame, with the calibrated
                error model (joint offsets + distortion, tilted axes, tool
                frame offset) applied
        """
        raise NotImplementedError("TODO: implement ImprovedKinematics.fwd_kins")

    def predict_in_tracker(self, theta, params: Optional[np.ndarray] = None) -> Frame:
        """The end-effector pose in the tracker frame: base_frame * fwd_kins."""
        return self.base_frame(params) * self.fwd_kins(theta, params)

    def inv_kins(self, F: Frame, seed=None, tol: float = 1e-10, max_iter: int = 50) -> np.ndarray:
        """
        ImprovedInvKins(F) -- provided.

        There is no closed form once the axes are tilted, so this iterates
        on your fwd_kins: both nominal wrist branches (Robot.inv_kins'
        closed form) are refined with damped Gauss-Newton / Levenberg-
        Marquardt steps on the pose error [alpha; epsilon] of
        fwd_kins(theta)^-1 F, and the in-limit solution nearest `seed`
        (default: the current command) is returned. Works as soon as your
        fwd_kins does.

        input: F - desired F_ee relative to the robot base frame
        output: theta with fwd_kins(theta) ~= F; raises ValueError if no
                branch satisfies the joint limits
        """
        seed = self.theta_cmd if seed is None else np.asarray(seed, dtype=np.float64).reshape(6)
        p_w = F.p - F.R * self.F_WE.p - self.F_CW.p   # nominal wrist center -> seeds for the linear joints
        step_max = np.array([50.0] * 3 + [0.3] * 3)
        h = np.array([1e-3] * 3 + [1e-6] * 3)

        def pose_delta(theta):
            dF = self.fwd_kins(theta).inv() * F
            return np.concatenate([log_so3(dF.R.matrix), dF.p.vec.ravel()])

        candidates = []
        for wrist in self.wrist_solutions(F.R):
            theta = np.concatenate([[p_w.x, p_w.y, p_w.z], wrist])
            lam, delta = 1e-4, pose_delta(theta)
            for _ in range(max_iter):
                cost = float(delta @ delta)
                if np.sqrt(cost) < tol:
                    break
                J = np.zeros((6, 6))
                for k in range(6):
                    th = theta.copy()
                    th[k] += h[k]
                    J[:, k] = (pose_delta(th) - delta) / h[k]
                while True:  # Levenberg-Marquardt: shrink the step until the cost decreases
                    JtJ = J.T @ J
                    step = -np.linalg.solve(JtJ + lam * np.diag(np.diag(JtJ) + 1e-12), J.T @ delta)
                    trial = theta + np.clip(step, -step_max, step_max)
                    delta_trial = pose_delta(trial)
                    if float(delta_trial @ delta_trial) < cost or lam > 1e6:
                        theta, delta, lam = trial, delta_trial, max(lam / 10.0, 1e-12)
                        break
                    lam *= 10.0
            theta[3] = (theta[3] + np.pi) % (2 * np.pi) - np.pi
            theta[5] = (theta[5] + np.pi) % (2 * np.pi) - np.pi
            if self.within_limits(theta):
                candidates.append(theta)
        if not candidates:
            raise ValueError(f"inv_kins: no joint solution within limits for {F!r}")
        return min(candidates, key=lambda th: float(np.linalg.norm(th - seed)))

    # ------------------------------------------------------------------ fitting

    def residuals(self, params: np.ndarray, data: CalibrationData) -> np.ndarray:
        """
        TODO: implement.

        input: params - candidate parameter vector; data - CalibrationData
        output: flat residual vector comparing predict_in_tracker(theta_j,
                params) with data.frames[j] for every sample (choose how to
                weigh rotation vs translation)
        """
        raise NotImplementedError("TODO: implement residuals")

    def fit(self, data: CalibrationData, **kwargs):
        """
        TODO: implement.

        Non-linear least squares over the parameters (scipy.optimize is
        allowed). Must set self.params. Return whatever diagnostics you want
        to report (final cost, iterations, ...).
        """
        raise NotImplementedError("TODO: implement fit")

    def jacobian_rank_report(self, data: CalibrationData, params: Optional[np.ndarray] = None) -> dict:
        """
        TODO: implement.

        Numerically differentiate residuals w.r.t. params and report the
        singular values (and which parameter directions are poorly
        determined). Use it to justify your parameterization and your
        choice of calibration poses in the report.
        """
        raise NotImplementedError("TODO: implement jacobian_rank_report")

    # ------------------------------------------------------------------ reporting

    def parameters_report(self) -> Dict[str, np.ndarray]:
        """
        TODO: implement.

        The handout (method 1 (e)) asks your program to report the updated
        kinematic parameters in physical terms, and your debugging runs to
        compare them with ground truth (rob.ground_truth_parameters()).
        Report at least: the axis tilt vectors / unit axes, the angles
        between the axes, the per-joint offsets and scale factors, and the
        tool-frame offset. Note which of these are only defined up to the
        base-frame gauge (and so are better compared as angles between axes).
        """
        raise NotImplementedError("TODO: implement parameters_report")
