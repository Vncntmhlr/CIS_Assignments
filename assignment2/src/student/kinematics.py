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
        if (isinstance(self.n_poly, (bool, np.bool_))
                or not isinstance(self.n_poly, (int, np.integer)) or self.n_poly < 0):
            raise ValueError("n_poly must be a nonnegative integer")
        if self.joint_offsets.shape != (6,) or not np.all(np.isfinite(self.joint_offsets)):
            raise ValueError("joint_offsets must contain 6 finite home corrections")
        # Home offsets are supplied measurements, not fitted intercepts.
        return np.zeros(22 + 6 * self.n_poly, dtype=np.float64)

    def base_frame(self, params: Optional[np.ndarray] = None) -> Frame:
        """
        TODO: implement.

        output: F_X (robot base relative to the tracker) for params (default:
                self.params) -- e.g. F_X_init * Exp(params[0:6]), see
                uFrame.perturb / uncertainty_networks.se3.exp_se3
        """
        from uncertainty_networks.se3 import exp_se3

        values = np.asarray(self.params if params is None else params, dtype=np.float64)
        if values.shape != (22 + 6 * self.n_poly,) or not np.all(np.isfinite(values)):
            raise ValueError("params must be a finite vector of length 22 + 6*n_poly")
        # xi_X = [alpha; epsilon] is expressed in the initial base frame:
        # T_tracker_base = T_tracker_base_init * Exp(xi_X), not the reverse.
        return self.F_X_init * Frame(exp_se3(values[:6]))

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
        from uncertainty_networks.se3 import exp_se3, exp_so3

        values = np.asarray(self.params if params is None else params, dtype=np.float64)
        if values.shape != (22 + 6 * self.n_poly,) or not np.all(np.isfinite(values)):
            raise ValueError("params must be a finite vector of length 22 + 6*n_poly")
        theta = np.asarray(theta, dtype=np.float64)
        if theta.shape != (6,) or not np.all(np.isfinite(theta)):
            raise ValueError("theta must contain 6 finite encoder values")

        # Anchor the variable correction at measured physical home theta=-o.
        # Coefficients have joint units; the polynomial arguments are unitless.
        corrected = theta + self.joint_offsets
        if self.n_poly:
            scales = np.maximum(np.abs(self.theta_min), np.abs(self.theta_max))
            if not np.all(np.isfinite(scales)) or np.any(scales <= 0):
                raise ValueError("joint normalization scales must be positive and finite")
            powers = (corrected / scales)[:, None] ** np.arange(1, self.n_poly + 1)
            corrected = corrected + np.sum(values[22:].reshape(6, self.n_poly) * powers, axis=1)

        # Two rotation-vector components perpendicular to each nominal axis
        # avoid the unobservable tilt component along that axis. d_1 is exact.
        axes = [vct3(axis) for axis in self.AXES]
        perpendicular = [(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)]
        for k, components in enumerate(perpendicular, start=1):
            phi = np.zeros(3)
            phi[list(components)] = values[6 + 2 * (k - 1):8 + 2 * (k - 1)]
            axes[k] = Rot(matrix=exp_so3(phi)) * axes[k]

        position = sum((corrected[k] * axes[k].vec.ravel() for k in range(3)), np.zeros(3))
        wrist = Rot()
        for k in range(3, 6):
            wrist = wrist * Rot(axes[k], angle=corrected[k])
        tool = self.F_WE * Frame(exp_se3(values[16:22]))
        # Preserve F_c * F_cw * R_4 * R_5 * R_6 * F_we. The base-to-tracker
        # transform is composed only by predict_in_tracker, never twice here.
        return Frame(Rot(), vct3(position)) * self.F_CW * Frame(wrist, vct3()) * tool

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
        from scipy.spatial.transform import Rotation
        from ..simulation import constants

        if len(data.thetas) == 0 or len(data.thetas) != len(data.frames):
            raise ValueError("residuals require nonempty paired encoder/Frame samples")
        base = self.base_frame(params)
        residual = np.empty((len(data), 6), dtype=np.float64)
        for j, (theta, observed) in enumerate(zip(data.thetas, data.frames)):
            if not isinstance(observed, Frame):
                raise ValueError("each observation must be a Frame")
            rotation, position = observed.R.matrix, observed.p.vec
            if (rotation.shape != (3, 3) or position.shape != (3, 1)
                    or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(position))
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
                    or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8, rtol=0)):
                raise ValueError("observed Frames must be finite with proper rotations")
            delta = observed.inv() * (base * self.fwd_kins(theta, params))
            # Signed components retain direction for optimization. The SO(3)
            # log is stable near zero; translation is dF.p, not Log_SE3(dF).
            residual[j, :3] = Rotation.from_matrix(delta.R.matrix).as_rotvec()
            residual[j, 3:] = delta.p.vec.ravel()
        # Target-based dimensionless weights; not measured noise covariances.
        residual[:, :3] /= constants.TARGET_ANG_ERR_RAD
        residual[:, 3:] /= constants.TARGET_POS_ERR_MM
        return residual.ravel()

    def fit(self, data: CalibrationData, **kwargs):
        """
        TODO: implement.

        Non-linear least squares over the parameters (scipy.optimize is
        allowed). Must set self.params. Return whatever diagnostics you want
        to report (final cost, iterations, ...).
        """
        from time import perf_counter
        from scipy.optimize import least_squares
        from ..simulation import constants

        started = perf_counter()
        physical_constraints = kwargs.pop("physical_constraints", True)
        if not isinstance(physical_constraints, (bool, np.bool_)):
            raise ValueError("physical_constraints must be boolean")
        initial = np.asarray(self.params, dtype=np.float64).copy()
        initial_residual = self.residuals(initial, data)
        # Keep the full public layout. Holding xi_X's x component at its
        # starting value selects the known local base/axis/tool gauge.
        fixed_index = 0
        active = np.arange(1, len(initial))
        if initial_residual.size < len(active):
            raise ValueError("too few scalar observations for the free parameters")
        default_scales = np.r_[[0.01] * 3, [1.0] * 3, [0.01] * 10,
                               [0.01] * 3, [0.5] * 3,
                               np.repeat([0.2] * 3 + [0.01] * 3, self.n_poly)]
        scales = np.asarray(kwargs.pop("parameter_scales", default_scales), dtype=np.float64)
        if scales.shape != initial.shape or not np.all(np.isfinite(scales)) or np.any(scales <= 0):
            raise ValueError("parameter_scales must be positive, finite, and match params")
        lower, upper = kwargs.pop("bounds", (-np.inf, np.inf))
        lower = np.broadcast_to(np.asarray(lower, dtype=np.float64), initial.shape)
        upper = np.broadcast_to(np.asarray(upper, dtype=np.float64), initial.shape)
        if (np.any(np.isnan(lower)) or np.any(np.isnan(upper))
                or np.any(initial < lower) or np.any(initial > upper)
                or np.any(lower[active] >= upper[active])):
            raise ValueError("bounds must contain the initial parameters and have positive free widths")
        options = {"max_nfev": 100, "ftol": 1e-10, "xtol": 1e-10,
                   "gtol": 1e-10, "verbose": 0}
        for name in tuple(options):
            if name in kwargs:
                options[name] = kwargs.pop(name)
        if kwargs:
            raise TypeError(f"unknown fitting options: {', '.join(kwargs)}")

        if physical_constraints:
            from vendor.functions._fit_constraints import solve_constrained, distortion_bounds

            candidate, info = solve_constrained(self, data, initial, initial_residual,
                                                scales, lower, upper, options)
            initial_cost = float(.5 * initial_residual @ initial_residual)
            initially_feasible = min(distortion_bounds(self, initial)[1]) >= -1e-7
            # Restoring feasibility can increase cost from an infeasible start.
            # A feasible start retains the historical non-increasing-cost rule.
            accepted = bool(info['solver_success'] and info['constraint_feasible']
                            and np.isfinite(candidate).all() and np.isfinite(info['final_cost'])
                            and (not initially_feasible or info['final_cost'] <= initial_cost
                                 + 1e-12 * max(1., initial_cost)))
            if accepted:
                self.params = candidate.copy()
            return dict(info, success=accepted, params_updated=accepted, x=candidate.copy(),
                        n_params=len(initial), n_free_params=len(active),
                        fixed_parameter_indices=[0], fixed_parameter_values=[float(initial[0])],
                        gauge="base twist x held at its initial value; not a physical measurement",
                        parameter_scales=scales.copy(), rotation_scale_rad=constants.TARGET_ANG_ERR_RAD,
                        translation_scale_mm=constants.TARGET_POS_ERR_MM, initial_cost=initial_cost,
                        cost_ratio=info['final_cost']/initial_cost if initial_cost > 0 else 0.,
                        wall_time_s=float(perf_counter()-started))

        counts = {"residual_calls": 1, "iterations": 0}  # Include the initial validation call.

        def unpack(scaled):
            values = initial.copy()
            values[active] = scaled * scales[active]
            return values

        def objective(scaled):
            counts["residual_calls"] += 1
            return self.residuals(unpack(scaled), data)

        def callback(x):
            counts["iterations"] += 1

        result = least_squares(
            objective, initial[active] / scales[active], method="trf",
            jac="3-point", tr_solver="exact", x_scale=1.0, callback=callback,
            bounds=(lower[active] / scales[active], upper[active] / scales[active]),
            **options)
        candidate = unpack(result.x)
        initial_cost = float(0.5 * initial_residual @ initial_residual)
        final_cost = float(result.cost)
        accepted = bool(result.success and np.all(np.isfinite(candidate))
                        and np.all(np.isfinite(result.fun))
                        and final_cost <= initial_cost + 1e-12 * max(1.0, initial_cost))
        # A failed/nonconverged attempt leaves the previous model intact.
        if accepted:
            self.params = candidate.copy()
        return {
            "solver": "TRF", "physical_constraints": False,
            "success": accepted, "solver_success": bool(result.success),
            "status": int(result.status), "message": str(result.message),
            "params_updated": accepted, "x": candidate.copy(),
            "n_params": len(initial), "n_free_params": len(active),
            "fixed_parameter_indices": [fixed_index],
            "fixed_parameter_values": [float(initial[fixed_index])],
            "gauge": "base twist x held at its initial value; not a physical measurement",
            "parameter_scales": scales.copy(),
            "rotation_scale_rad": constants.TARGET_ANG_ERR_RAD,
            "translation_scale_mm": constants.TARGET_POS_ERR_MM,
            "initial_cost": initial_cost, "final_cost": final_cost,
            "cost_ratio": final_cost / initial_cost if initial_cost > 0 else 0.0,
            "optimality": float(result.optimality), "nfev": int(result.nfev),
            "njev": int(result.njev), **counts,
            "wall_time_s": float(perf_counter() - started),
        }

    def jacobian_rank_report(self, data: CalibrationData, params: Optional[np.ndarray] = None) -> dict:
        """
        TODO: implement.

        Numerically differentiate residuals w.r.t. params and report the
        singular values (and which parameter directions are poorly
        determined). Use it to justify your parameterization and your
        choice of calibration poses in the report.
        """
        from time import perf_counter
        from scipy.spatial.transform import Rotation
        from uncertainty_networks.se3 import exp_se3, log_se3
        from ..simulation import constants

        started = perf_counter()
        values = np.asarray(self.params if params is None else params, dtype=np.float64).copy()
        baseline = self.residuals(values, data)  # Validate the actual Phase B inputs.
        angles = np.linalg.norm(baseline.reshape(-1, 6)[:, :3] * constants.TARGET_ANG_ERR_RAD, axis=1)
        if np.any(angles > np.pi - 1e-4):
            raise ValueError("rotation residual too close to pi for a smooth local Jacobian")
        scales = np.r_[[0.01] * 3, [1.0] * 3, [0.01] * 10,
                        [0.01] * 3, [0.5] * 3,
                        np.repeat([0.2] * 3 + [0.01] * 3, self.n_poly)]
        names = ([f"base_{part}_{axis}" for part in ["rot", "trans"] for axis in "xyz"]
                 + [f"axis{k}_{axis}" for k, axes in zip(range(2, 7),
                                                        ["xz", "xy", "yz", "xz", "xy"])
                    for axis in axes]
                 + [f"tool_{part}_{axis}" for part in ["rot", "trans"] for axis in "xyz"]
                 + [f"joint{k}_c{j}" for k in range(1, 7) for j in range(1, self.n_poly + 1)])
        # Columns are derivatives w.r.t. z=p/scale. Scaling the physical
        # perturbations avoids mixing mm/rad and tiny coefficient steps.
        step = 1e-4 * np.maximum(1.0, np.abs(values / scales))

        def differentiate(multiplier):
            jacobian = np.empty((baseline.size, values.size))
            for k in range(values.size):
                delta = np.zeros_like(values)
                delta[k] = multiplier * step[k] * scales[k]
                jacobian[:, k] = (self.residuals(values + delta, data)
                                  - self.residuals(values - delta, data)) / (2 * multiplier * step[k])
            return jacobian

        coarse, jacobian = differentiate(2.0), differentiate(1.0)
        relative_tolerance = 1e-7

        def analyze(columns):
            matrix = jacobian[:, columns]
            difference = matrix - coarse[:, columns]
            fd_error = float(np.linalg.norm(difference, ord=2))
            # Full V is needed only for underdetermined small data sets;
            # avoid a huge square U when there are many measurements.
            _, singular, vectors = np.linalg.svd(matrix, full_matrices=matrix.shape[0] < matrix.shape[1])
            singular = np.pad(singular, (0, matrix.shape[1] - len(singular)))
            maximum = float(singular[0])
            machine_threshold = np.finfo(float).eps * max(matrix.shape) * maximum
            # Engineering tolerance plus a two-step numerical-error estimate;
            # this is a rank decision, not a measurement-noise threshold.
            threshold = max(relative_tolerance * maximum, 5 * fd_error, machine_threshold)
            rank = int(np.count_nonzero(singular > threshold))

            def direction(index):
                vector = vectors[index].copy()
                dominant = np.argsort(np.abs(vector))[-6:][::-1]
                return {
                    "singular_value": float(singular[index]),
                    "relative_singular_value": float(singular[index] / maximum) if maximum else 0.0,
                    "scaled_direction": vector,
                    "physical_direction": scales[columns] * vector,
                    "dominant_parameters": [{"index": int(columns[k]), "name": names[columns[k]],
                                              "scaled_component": float(vector[k])} for k in dominant],
                    "unit_weighted_noise_gain": float(1 / singular[index]) if index < rank else float("inf"),
                }

            coarse_singular = np.linalg.svd(coarse[:, columns], compute_uv=False)
            weak = np.flatnonzero((singular > threshold) & (singular < maximum * 0.01))
            if len(weak) < min(3, rank):
                weak = np.arange(max(0, rank - 3), rank)
            return {
                "rank": rank, "n_params": len(columns), "nullity": len(columns) - rank,
                "column_indices": columns.copy(), "singular_values": singular,
                "singular_values_per_sqrt_sample": singular / np.sqrt(len(data)),
                "condition_number": float(maximum / singular[-1]) if rank == len(columns) else float("inf"),
                "observable_condition_number": float(maximum / singular[rank - 1]) if rank else float("inf"),
                "rank_threshold": threshold, "machine_threshold": machine_threshold,
                "machine_rank": int(np.count_nonzero(singular > machine_threshold)),
                "fd_difference_norm": fd_error,
                "fd_relative_difference": fd_error / max(maximum, np.finfo(float).tiny),
                "coarse_step_rank": int(np.count_nonzero(coarse_singular > threshold)),
                "null_directions": [direction(k) for k in range(rank, len(columns))],
                "weak_directions": [direction(k) for k in weak],
                "weak_relative_count": int(np.count_nonzero((singular > threshold)
                                                              & (singular < maximum * 0.01))),
                "nullspace_scaled": vectors[rank:].copy(),
            }

        full = analyze(np.arange(len(values)))
        reduced = analyze(np.arange(1, len(values)))

        def gauge_parameters(angle):
            # Q rotates about the exact d_1=x axis. X->XQ, d->Q^-1 d,
            # W->Q^-1 W preserves predictions since F_CW commutes with Q.
            q = Frame(Rot('x', angle), vct3())
            base = Frame(exp_se3(values[:6])) * q
            tool = self.F_WE.inv() * q.inv() * self.F_WE * Frame(exp_se3(values[16:22]))

            def homogeneous(frame):
                result = np.eye(4)
                result[:3, :3], result[:3, 3] = frame.R.matrix, frame.p.vec.ravel()
                return result

            changed = values.copy()
            changed[:6], changed[16:22] = log_se3(homogeneous(base)), log_se3(homogeneous(tool))
            for k, components in enumerate([(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)], 1):
                phi = np.zeros(3)
                phi[list(components)] = values[6 + 2 * (k - 1):8 + 2 * (k - 1)]
                axis = self.AXES[k].vec.ravel()
                changed_axis = q.R.matrix.T @ Rotation.from_rotvec(phi).apply(axis)
                cross = np.cross(axis, changed_axis)
                sine, cosine = np.linalg.norm(cross), float(axis @ changed_axis)
                if cosine < -1 + 1e-10:
                    raise ValueError("axis tilt is too close to the antipodal chart singularity")
                changed_phi = cross * (np.arctan2(sine, cosine) / sine) if sine > 1e-15 else np.zeros(3)
                changed[6 + 2 * (k - 1):8 + 2 * (k - 1)] = changed_phi[list(components)]
            return changed

        gauge_step = 1e-5
        gauge = (gauge_parameters(gauge_step) - gauge_parameters(-gauge_step)) / (2 * gauge_step * scales)
        gauge /= np.linalg.norm(gauge)
        alignment = float(np.clip(np.linalg.norm(full["nullspace_scaled"] @ gauge), 0.0, 1.0))
        relative_response = float(np.linalg.norm(jacobian @ gauge)
                                  / max(full["singular_values"][0], np.finfo(float).tiny))
        verified = bool(full["nullity"] == 1 and reduced["nullity"] == 0
                        and alignment > 0.999 and abs(gauge[0]) > 1e-4
                        and relative_response < 5 * relative_tolerance)
        return {
            "rank": full["rank"], "n_params": len(values),
            "singular_values": full["singular_values"],
            "condition_number": full["condition_number"],
            "full": full, "reduced": reduced, "fixed_parameter_indices": [0],
            "jacobian": jacobian / scales, "scaled_jacobian": jacobian,
            "parameter_scales": scales, "parameter_names": names,
            "physical_fd_steps": step * scales, "rank_relative_tolerance": relative_tolerance,
            "gauge_direction_scaled": gauge, "gauge_nullspace_alignment": alignment,
            "gauge_relative_response": relative_response, "gauge_verified": verified,
            "n_samples": len(data), "n_distinct_poses": len({tuple(theta) for theta in data.thetas}),
            "residual_calls": 1 + 4 * len(values),
            "interpretation": "Target-weighted scaled SVD; unit-noise gains are not covariance estimates. "
                              "Repeats can reduce independent variance but cannot supply missing excitation.",
            "wall_time_s": float(perf_counter() - started),
        }

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
        from uncertainty_networks.se3 import exp_so3, exp_se3
        from numpy.polynomial import Polynomial
        from vendor.functions._fit_constraints import distortion_bounds

        values = self.params.copy()
        base = self.base_frame(values)  # Validate the current parameter vector.
        tilts = np.zeros((6, 3))
        perpendicular = [(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)]
        axes = np.array([axis.vec.ravel() for axis in self.AXES])
        for k, components in enumerate(perpendicular, start=1):
            tilts[k, list(components)] = values[6 + 2*(k-1):8 + 2*(k-1)]
            axes[k] = exp_so3(tilts[k]) @ axes[k]
        angles = np.degrees(np.arccos(np.clip(axes @ axes.T, -1., 1.)))
        np.fill_diagonal(angles, 0.)
        scales = np.maximum(abs(self.theta_min), abs(self.theta_max))
        coefficients = values[22:].reshape(6, self.n_poly).copy()
        offset, derivative_zero, derivative_home = [], [], []
        for k in range(6):
            polynomial = Polynomial(np.r_[self.joint_offsets[k], coefficients[k]])
            offset.append(polynomial(self.joint_offsets[k]/scales[k]))
            derivative_zero.append(polynomial.deriv()(self.joint_offsets[k]/scales[k])/scales[k])
            derivative_home.append(polynomial.deriv()(0.)/scales[k])
        base_correction = Frame(exp_se3(values[:6]))
        tool_correction = Frame(exp_se3(values[16:22]))
        bounds, margins, _ = distortion_bounds(self, values)
        return {
            "axis_tilts": tilts, "axes": axes, "axis_angles_deg": angles,
            "linear_axis_angles_deg": angles[:3, :3].copy(),
            "wrist_axis_angles_deg": angles[3:, 3:].copy(),
            "axes_in_tracker_at_joint_zero": axes @ base.R.matrix.T,
            "joint_home_correction": self.joint_offsets.copy(),
            "joint_offset": np.array(offset),
            "joint_scale": np.array(derivative_zero),
            "joint_scale_factor": 1 + np.array(derivative_zero),
            "joint_home_scale_factor": 1 + np.array(derivative_home),
            "joint_polynomial_coefficients": coefficients,
            "joint_normalization_scales": scales.copy(),
            "joint_distortion_range": np.array([[b['minimum'], b['maximum']] for b in bounds]),
            "joint_derivative_range": np.array([[b['minimum_derivative'], b['maximum_derivative']] for b in bounds]),
            "joint_distortion_max_abs": np.array([b['max_absolute'] for b in bounds]),
            "joint_derivative_max_abs": np.array([b['max_absolute_derivative'] for b in bounds]),
            "joint_magnitude_limits": np.array([b['magnitude_limit'] for b in bounds]),
            "joint_derivative_limits": np.array([b['derivative_limit'] for b in bounds]),
            "physical_joint_bounds_satisfied": bool(min(margins) >= -1e-7),
            "base_twist": values[:6].copy(), "F_X": base,
            "F_X_correction": base_correction,
            "base_correction_translation_mm": base_correction.p.vec.ravel().copy(),
            "F_we_twist": values[16:22].copy(), "F_we_correction": tool_correction,
            "F_we": self.F_WE * tool_correction,
            "tool_correction_translation_mm": tool_correction.p.vec.ravel().copy(),
            "gauge": {"fixed_parameter_index": 0, "fixed_value": float(values[0]),
                      "meaning": "Chosen base-roll coordinate, not independently measured physical roll"},
            "gauge_dependent": ["base twist/frame", "base-coordinate axes and tilts", "tool correction"],
            "gauge_invariant": ["inter-axis angles", "joint maps", "tracker pose predictions",
                                "axes_in_tracker_at_joint_zero"],
            "conventions": {
                "axes": "Unit axes in the selected base frame before wrist joint rotations; angles are unsigned",
                "joint_offset": "Fitted total systematic correction at encoder theta=0; includes the measured home anchor",
                "joint_scale": "Derivative of fitted systematic correction at encoder zero; total local gear factor is 1+joint_scale",
                "home": "Polynomial is anchored at theta=-joint_home_correction, not encoder zero",
                "twists": "Rotation-first SE(3) twists; actual correction translation is J(alpha)*epsilon",
                "units": "mm for translations/joints 1-3; rad for rotations/joints 4-6; axis angles in degrees"},
            "limitations": ["Reported offsets/scales are effective polynomial quantities, not separately identifiable sinusoidal constant/linear terms",
                            "No hidden truth is consulted; estimates include noise, homing uncertainty and possible tracker/model bias",
                            "Hard joint-map bounds do not certify physical FK/IK accuracy or unique base/axis/tool recovery"],
        }
