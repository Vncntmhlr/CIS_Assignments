"""
Simulation for the robot: CreateRobot, MoveJoints, MoveRobot, HomeRobot,
State, CommandedPose, plus the ground-truth oracles used for reporting.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame
from data_types.uncertain_types import uFrame
from uncertainty_networks.se3 import exp_so3

from assignments.assignment1.src.utils import sample_in_ball, sample_perpendicular
from ..robot import Robot, RobotState
from .clock import clock
from . import constants


@dataclass
class _RobotTruth:
    """Hidden actual kinematics. Never read this in calibration code."""
    F_rob_actual: Frame            # F*_rob = dF_rob F_rob
    axes_actual: List[vct3]        # d*_1..3, n*_4..6 (d*_1 == d_1 exactly)
    phi: np.ndarray                # (6, 3) axis tilt vectors (row 0 is zero)
    distort: np.ndarray            # (6, 5) rows of [c, s, sigma, A, B] for delta_theta_k,sys
    F_we_actual: Frame             # F*_we = F_we dF_ee
    xi_we: np.ndarray              # the twist behind F_we_actual
    sigma_rnd: np.ndarray          # (6,) std of delta_theta_k,rnd


class RobotSim(Robot):
    """The handout's robot, with the nominal interface of Robot plus the
    hidden actual kinematics behind MoveJoints. The controller's theta
    (what State() reports) is an encoder value: the physical joint value
    is theta* = theta + delta_theta_sys(theta) + delta_theta_rnd, resampled
    on every move; the robot's calibration body (if attached) is moved to
    the actual end-effector pose."""

    def __init__(self, name, F_rob, theta_min, theta_max, t_settle, theta_dot_max, theta_ddot_max,
                 truth: _RobotTruth, rng: np.random.Generator):
        super().__init__(name, F_rob, theta_min, theta_max, t_settle, theta_dot_max, theta_ddot_max)
        self._truth = truth
        self._rng = rng
        self._theta_star = np.zeros(6)
        self.theta_cmd = self._encoders_at_home()  # the robot starts physically at home

    # ------------------------------------------------------------------ hidden kinematics

    def _delta_theta_sys(self, theta) -> np.ndarray:
        """delta_theta_k,sys(theta_k) = c + s theta + sigma sin(A theta + B)."""
        c, s, sigma, A, B = self._truth.distort.T
        theta = np.asarray(theta, dtype=np.float64)
        return c + s * theta + sigma * np.sin(A * theta + B)

    def _encoders_at_home(self) -> np.ndarray:
        """Encoder values theta with theta + delta_theta_sys(theta) + rnd = 0 (theta* = 0)."""
        rnd = self._rng.normal(0.0, self._truth.sigma_rnd)
        theta = -self._delta_theta_sys(np.zeros(6)) - rnd
        for _ in range(5):  # fixed point; the sys term is tiny and smooth
            theta = -self._delta_theta_sys(theta) - rnd
        return theta

    def _actual_fwd_kins(self, theta_star) -> Frame:
        """F*_ee(theta*) with the actual axis directions and actual F_we, relative to the actual base."""
        d = self._truth.axes_actual
        p_c = d[0] * theta_star[0] + d[1] * theta_star[1] + d[2] * theta_star[2]
        R_w = Rot(matrix=exp_so3(d[3].vec.ravel() * theta_star[3])) \
            * Rot(matrix=exp_so3(d[4].vec.ravel() * theta_star[4])) \
            * Rot(matrix=exp_so3(d[5].vec.ravel() * theta_star[5]))
        return Frame(Rot(), p_c) * self.F_CW * Frame(R_w, vct3()) * self._truth.F_we_actual

    def _sync_calibration_body(self):
        if self.calibration_body is not None and self.calibration_body_attached:
            self.calibration_body.actual_placement = self._truth.F_rob_actual * self._actual_fwd_kins(self._theta_star)

    # ------------------------------------------------------------------ controller interface

    def move_joints(self, theta) -> float:
        """delta_t = MoveJoints(theta)

        Moves so that the commanded F_ee = FwdKins(theta); the physical joint
        values become theta + delta_theta_sys(theta) + delta_theta_rnd.
        Returns the simulated motion time, which is also added to the clock.
        A move to the current command still costs t_settle (and re-rolls the
        repeatability noise).
        """
        theta = self.check_limits(theta)
        dt = self.motion_time(self.theta_cmd, theta)
        clock.advance(dt)
        self.theta_cmd = theta.copy()
        self._theta_star = theta + self._delta_theta_sys(theta) + self._rng.normal(0.0, self._truth.sigma_rnd)
        self._sync_calibration_body()
        return dt

    def move_robot(self, F: Frame) -> float:
        """delta_t = MoveRobot(F) = MoveJoints(InvKins(F))"""
        return self.move_joints(self.inv_kins(F))

    def home_robot(self) -> np.ndarray:
        """theta_encoders = HomeRobot()

        Drives the robot to its physical home position, theta* = 0, and
        returns the encoder values there (the same values State() reports
        from now on): theta = -delta_theta_sys(theta) - delta_theta_rnd. The
        move costs the usual motion time and re-rolls the repeatability noise.
        """
        theta = self._encoders_at_home()
        clock.advance(self.motion_time(self.theta_cmd, theta))
        self.theta_cmd = theta
        self._theta_star = np.zeros(6)
        self._sync_calibration_body()
        return theta.copy()

    def state(self) -> RobotState:
        """(theta, theta_dot, ...) = State(): the encoder values, not theta*."""
        return super().state(t_elapsed=clock.elapsed)

    # ------------------------------------------------------------------ ground truth (reporting only)

    def actual_ee_pose(self, theta) -> Frame:
        """GROUND TRUTH, for validation/reporting only (the handout: these
        values "will not be available during evaluation").

        The deterministic actual end-effector pose in the WORKSPACE frame,
        F*_rob F*_ee(theta + delta_theta_sys), i.e. where the robot really
        goes for an encoder command theta, ignoring the per-move
        repeatability noise. Does not move the robot or advance the clock.
        """
        theta = np.asarray(theta, dtype=np.float64).reshape(6)
        return self._truth.F_rob_actual * self._actual_fwd_kins(theta + self._delta_theta_sys(theta))

    def ground_truth_parameters(self) -> Dict[str, np.ndarray]:
        """GROUND TRUTH, for debugging/reporting only: the hidden kinematic
        parameters, so a calibration can report how its estimates compare
        (handout, method 1 (e)). Keys: axis_tilts (6x3, phi_k; row 0 zero),
        axes (6x3 unit vectors d*_k), joint_offset / joint_scale /
        ripple_std / ripple_freq / ripple_phase (6,), F_we_twist (6,),
        F_rob_actual (Frame)."""
        t = self._truth
        return {
            "axis_tilts": t.phi.copy(),
            "axes": np.array([d.vec.ravel() for d in t.axes_actual]),
            "joint_offset": t.distort[:, 0].copy(), "joint_scale": t.distort[:, 1].copy(),
            "ripple_std": t.distort[:, 2].copy(), "ripple_freq": t.distort[:, 3].copy(), "ripple_phase": t.distort[:, 4].copy(),
            "F_we_twist": t.xi_we.copy(), "F_rob_actual": t.F_rob_actual,
        }


def create_robot(name: str, F_rob: Frame,
                 alpha_max_rob: float = constants.ALPHA_MAX_ROB,
                 eps_max_rob: float = constants.EPS_MAX_ROB,
                 theta_min=constants.THETA_MIN, theta_max=constants.THETA_MAX,
                 t_settle: float = constants.T_SETTLE,
                 phi_max: float = constants.PHI_MAX,
                 sigma_rnd=constants.SIGMA_RND,
                 theta_dot_max=constants.THETA_DOT_MAX, theta_ddot_max=constants.THETA_DDOT_MAX,
                 sigma_sys=constants.SIGMA_SYS, delta_sys=constants.DELTA_SYS, omega_sys=constants.OMEGA_SYS,
                 seed: Optional[int] = None) -> RobotSim:
    """Rob = CreateRobot(F_rob, alpha_max_rob, eps_max_rob, theta_min, theta_max,
                         t_settle, phi_max, sigma_rnd, theta_dot_max, theta_ddot_max)

    Creates the robot approximately at F_rob and draws its hidden actual
    kinematics once: F*_rob = dF_rob F_rob; d*_k = R(phi_k) d_k for
    k = 2..6 (d*_1 = d_1) with phi_k perpendicular to d_k (a component
    along d_k would not change the axis) and ||phi_k|| <= phi_max; the
    per-joint distortion c_k + s_k theta + sigma_sys,k sin(A_k theta + B_k)
    within the handout's magnitude (delta_sys) and slope (omega_sys)
    bounds; and F*_we = F_we dF_ee with ||alpha_ee|| <= 0.01,
    ||epsilon_ee|| <= 0.5mm. Pass seed for a reproducible robot.
    """
    if not isinstance(F_rob, Frame):
        raise TypeError(f"create_robot: F_rob must be a Frame, got {F_rob!r} (did you fill in the Step 1 TODOs?)")
    rng = np.random.default_rng(seed)
    frac = constants.BOUND_MIN_FRAC

    xi_rob = np.concatenate([sample_in_ball(rng, alpha_max_rob, 3, frac), sample_in_ball(rng, eps_max_rob, 3, frac)])
    F_rob_actual = uFrame(F_rob).as_left_convention().perturb(xi_rob)   # F*_rob = dF_rob F_rob

    phi = np.zeros((6, 3))
    axes_actual = [Robot.AXES[0]]
    for k, d in enumerate(Robot.AXES[1:], start=1):
        phi[k] = sample_perpendicular(rng, d, phi_max, frac)
        axes_actual.append(vct3(exp_so3(phi[k]) @ d.vec.ravel()))

    sigma_sys = np.asarray(sigma_sys, dtype=np.float64).reshape(6)
    delta_sys = np.asarray(delta_sys, dtype=np.float64).reshape(6)
    omega_sys = np.asarray(omega_sys, dtype=np.float64).reshape(6)
    theta_range = np.maximum(np.abs(np.asarray(theta_min, dtype=np.float64)), np.abs(np.asarray(theta_max, dtype=np.float64)))
    distort = np.zeros((6, 5))
    for k in range(6):
        share = rng.uniform(*constants.RIPPLE_SLOPE_SHARE)       # slope budget: ripple gets share, scale the rest
        A = share * omega_sys[k] / sigma_sys[k]
        s = rng.choice([-1.0, 1.0]) * rng.uniform(frac, 1.0) * (1.0 - share) * omega_sys[k]
        c_max = delta_sys[k] - sigma_sys[k] - abs(s) * theta_range[k]   # magnitude budget left for the offset
        c = rng.choice([-1.0, 1.0]) * rng.uniform(frac, 1.0) * max(c_max, 0.0)
        distort[k] = [c, s, sigma_sys[k], A, rng.uniform(0.0, constants.DISTORT_PHASE_MAX)]

    xi_we = np.concatenate([sample_in_ball(rng, constants.WE_ALPHA_MAX, 3, frac),
                            sample_in_ball(rng, constants.WE_EPS_MAX, 3, frac)])
    F_we_actual = uFrame(Robot.F_WE).perturb(xi_we)                      # F*_we = F_we dF_ee

    truth = _RobotTruth(F_rob_actual=F_rob_actual, axes_actual=axes_actual, phi=phi, distort=distort,
                        F_we_actual=F_we_actual, xi_we=xi_we,
                        sigma_rnd=np.asarray(sigma_rnd, dtype=np.float64).reshape(6))
    return RobotSim(name, F_rob, theta_min, theta_max, t_settle, theta_dot_max, theta_ddot_max, truth, rng)
