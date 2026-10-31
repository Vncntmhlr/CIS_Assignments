"""
The calibrated kinematic model: ImprovedFwdKins / ImprovedInvKins.
"""

from typing import Dict, Optional

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame
from uncertainty_networks.se3 import exp_se3, exp_so3, log_so3

from ..robot import Robot
from ..simulation import constants
from .workflow import CalibrationData
from scipy.optimize import least_squares


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
        Return zero initial corrections for the base, axes, tool, and joint
        distortion. Measured joint_offsets are stored separately and retained.
        """
        # 6 base corrections + 10 axis tilts + 6 tool corrections
        # + n_poly distortion coefficients for each of the 6 joints.
        return np.zeros(6 + 10 + 6 + 6 * self.n_poly, dtype=float)

    def base_frame(self, params: Optional[np.ndarray] = None) -> Frame:
        """
        Return F_X_init * Exp(params[0:6]), the corrected robot-base pose
        relative to the tracker. Default to self.params when params is None.
        The correction uses twist ordering [rotation; translation-like].
        """
        params = self.params if params is None else np.asarray(params, dtype=float)
        correction = Frame(exp_se3(params[0:6]))
        F_X_estimated = self.F_X_init * correction
        return F_X_estimated

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

        theta = np.asarray(theta, dtype=float).reshape(6)
        params = self.params if params is None else np.asarray(params, dtype=float)

        # 1. Evaluate the joint-distortion polynomials.
        coefficients = params[22:].reshape(6, self.n_poly)

        # Fixed scales for normalizing the commanded coordinates.
        scale = np.maximum(
            np.maximum(np.abs(self.theta_min), np.abs(self.theta_max)),
            1e-12,
        )
        theta_normalized = theta / scale

        distortion = np.zeros(6)
        for power in range(1, self.n_poly + 1):
            distortion += coefficients[:, power - 1] * theta_normalized**power

        # Corrected joint coordinates: command + home offset + distortion.
        theta_cal = theta + self.joint_offsets + distortion

        # 2. Correct the axis directions.
        # Rows: d1, d2, d3, n4, n5, n6.
        axes_cal = np.array([
            axis.vec.ravel() for axis in self.AXES
        ], dtype=float)

        tilts = params[6:16].reshape(5, 2)

        for joint in range(1, 6):  # Joint 1's axis remains unchanged.
            nominal_axis = axes_cal[joint].copy()

            # The nominal axes are Cartesian unit vectors.
            # Select the two Cartesian directions perpendicular to this axis.
            perpendicular_basis = np.eye(3)[np.abs(nominal_axis) < 0.5]

            tilt_vector = tilts[joint - 1] @ perpendicular_basis
            axes_cal[joint] = exp_so3(tilt_vector) @ nominal_axis

        # 3. Build the calibrated carriage transform.
        pc_cal = vct3(*(theta_cal[:3] @ axes_cal[:3]))
        Fc_cal = Frame(Rot(), pc_cal)

        # 4. Build the calibrated wrist transform.
        Rw_cal = (
            Rot(matrix=exp_so3(axes_cal[3] * theta_cal[3]))
            * Rot(matrix=exp_so3(axes_cal[4] * theta_cal[4]))
            * Rot(matrix=exp_so3(axes_cal[5] * theta_cal[5]))
        )
        Fw_cal = Frame(Rw_cal, vct3())

        #5. Build the calibrated tool transform.
        F_we_cal = self.F_WE * Frame(exp_se3(params[16:22]))

        return Fc_cal * self.F_CW * Fw_cal * F_we_cal


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
        input: params - candidate parameter vector; data - CalibrationData
        output: flat vector of rotation then position errors for every sample,
                scaled by the assignment's angular and positional targets.
        """
        if len(data.thetas) != len(data.frames):
            raise ValueError("Each joint vector must have a corresponding observed frame")

        residuals = []
        for theta, F_observed in zip(data.thetas, data.frames):
            F_predicted = self.predict_in_tracker(theta, params)
            F_error = F_predicted.inv() * F_observed

            rotation_error = log_so3(F_error.R.matrix)
            position_error = F_error.p.vec.ravel()
            residuals.extend(rotation_error / constants.TARGET_ANG_ERR_RAD)
            residuals.extend(position_error / constants.TARGET_POS_ERR_MM)

        return np.asarray(residuals, dtype=float)

    def fit(self, data: CalibrationData, **kwargs):
        """
        TODO: implement.

        Non-linear least squares over the parameters (scipy.optimize is
        allowed). Must set self.params. Return whatever diagnostics you want
        to report (final cost, iterations, ...).
        """
        if len(data.frames) == 0:
            raise ValueError("At least one observation is required for fitting")

        iterations = 0

        def count_iteration(params):
            nonlocal iterations
            iterations += 1

        kwargs.setdefault("x_scale", "jac")
        result = least_squares(
            self.residuals,
            self.params,
            args=(data,),
            method="trf",
            callback=count_iteration,
            **kwargs,
        )
        self.params = result.x.copy()

        return {
            "iterations": iterations,
            "final_cost": float(result.cost),
            "nfev": result.nfev,
            "success": result.success,
            "message": result.message,
        }

    def jacobian_rank_report(self, data: CalibrationData, params: Optional[np.ndarray] = None) -> dict:
        """
        Estimate the residual Jacobian using central differences, then its SVD.
        Rank uses a relative tolerance of 1e-6 to allow for differentiation
        noise. Results use the current parameter units and residual scaling.
        Each poorly determined direction is a row in parameter-vector order.
        """
        params = np.asarray(self.params if params is None else params, dtype=float).copy()
        if params.shape != self.params.shape or not np.isfinite(params).all():
            raise ValueError("params must be a finite vector matching self.params")

        residuals = self.residuals(params, data)
        if residuals.size == 0:
            raise ValueError("At least one observation is required for a rank report")

        n_params = params.size
        J = np.zeros((residuals.size, n_params))
        for j in range(n_params):
            step = 1e-6 * max(1.0, abs(params[j]))
            params_plus, params_minus = params.copy(), params.copy()
            params_plus[j] += step
            params_minus[j] -= step
            J[:, j] = (
                self.residuals(params_plus, data)
                - self.residuals(params_minus, data)
            ) / (2.0 * step)

        # Keep all right singular vectors if there are fewer residuals than parameters.
        _, singular_values, Vt = np.linalg.svd(J, full_matrices=J.shape[0] < n_params)
        tolerance = 1e-6 * singular_values[0]
        rank = int(np.count_nonzero(singular_values > tolerance))

        return {
            "rank": rank,
            "n_params": n_params,
            "singular_values": singular_values,
            "tolerance": float(tolerance),
            "nullity": n_params - rank,
            "poorly_determined_directions": Vt[rank:].copy(),
        }

    # ------------------------------------------------------------------ reporting

    def parameters_report(self) -> Dict[str, np.ndarray]:
        """
        Report the fitted model without reading simulator ground truth.

        Axis rows are d1, d2, d3, n4, n5, n6; axis_angles_deg[i, j] is the
        angle between rows i and j. Tilt vectors are in radians. Absolute
        axes/tilts and tool parameters depend on the chosen frame convention;
        mutual axis angles are invariant under a common rotation of the axes.

        joint_offset is the measured home-offset estimate (mm/rad).
        joint_scale is the dimensionless distortion slope at theta=0;
        joint_scale_factor is 1 + joint_scale. These describe the fitted
        polynomial, not a unique separation of the simulator's ripple and
        linear terms. joint_poly_coefficients use joint_normalization.

        F_we_twist has ordering [rotation; translation-like]. F_we_offset
        is its exponential, the right-multiplied tool correction, and F_we
        is the corrected wrist-to-end-effector transform (both 4x4 matrices).
        """
        axes_cal = np.array([axis.vec.ravel() for axis in self.AXES], dtype=float)
        axis_tilts = np.zeros((6, 3))
        tilts = self.params[6:16].reshape(5, 2)

        for joint in range(1, 6):
            nominal_axis = axes_cal[joint].copy()
            perpendicular_basis = np.eye(3)[np.abs(nominal_axis) < 0.5]
            axis_tilts[joint] = tilts[joint - 1] @ perpendicular_basis
            axes_cal[joint] = exp_so3(axis_tilts[joint]) @ nominal_axis

        axis_angles_deg = np.degrees(np.arccos(np.clip(axes_cal @ axes_cal.T, -1.0, 1.0)))
        np.fill_diagonal(axis_angles_deg, 0.0)

        scale = np.maximum(
            np.maximum(np.abs(self.theta_min), np.abs(self.theta_max)), 1e-12
        )
        coefficients = self.params[22:].reshape(6, self.n_poly).copy()
        joint_scale = coefficients[:, 0] / scale if self.n_poly > 0 else np.zeros(6)

        F_we_twist = self.params[16:22].copy()
        F_we_offset = exp_se3(F_we_twist)
        F_we_cal = self.F_WE * Frame(F_we_offset)
        F_we = np.eye(4)
        F_we[:3, :3] = F_we_cal.R.matrix
        F_we[:3, 3] = F_we_cal.p.vec.ravel()

        return {
            "axis_tilts": axis_tilts,
            "axes": axes_cal,
            "axis_angles_deg": axis_angles_deg,
            "joint_offset": self.joint_offsets.copy(),
            "joint_scale": joint_scale,
            "joint_scale_factor": 1.0 + joint_scale,
            "joint_poly_coefficients": coefficients,
            "joint_normalization": scale,
            "F_we_twist": F_we_twist,
            "F_we_offset": F_we_offset,
            "F_we": F_we,
        }
