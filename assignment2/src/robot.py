from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame


@dataclass
class RobotState:
    """State() -- the robot controller's internal state."""
    theta: np.ndarray            # commanded joint values (mm for joints 1-3, radians for 4-6)
    theta_dot: np.ndarray        # joint velocities; always zero (the robot has settled)
    t_elapsed: float             # simulated elapsed time when the state was read, sec
    calibration_body_attached: bool


def _wrap_angle(a: float) -> float:
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


class Robot:
    """
    Nominal cartesian-base + spherical-wrist robot from the handout:

        F_ee(theta) = F_c(theta_1..3) F_cw F_w(theta_4..6) F_we
        F_c = translation by theta_1 d_1 + theta_2 d_2 + theta_3 d_3
        F_w = R(n_4, theta_4) R(n_5, theta_5) R(n_6, theta_6)

    Units are mm and radians. Only the nominal kinematics live here; the
    simulated robot (simulation.robot.RobotSim) subclasses this and keeps the
    hidden actual kinematics to itself.
    """

    AXES = [vct3(1, 0, 0), vct3(0, 1, 0), vct3(0, 0, 1)] * 2  # d_1..3 then n_4..6 (d_{k+3} = n_k)
    F_CW = Frame(Rot(), vct3(500, 0, 0))
    F_WE = Frame(Rot(), vct3(100, 0, 0))

    def __init__(self, name, F_rob: Frame, theta_min, theta_max, t_settle: float,
                 theta_dot_max, theta_ddot_max):
        self.name = name
        self.F_rob = F_rob  # approximate pose of the robot base in the workspace; set by students
        self.theta_min = np.asarray(theta_min, dtype=np.float64).reshape(6)
        self.theta_max = np.asarray(theta_max, dtype=np.float64).reshape(6)
        self.t_settle = float(t_settle)
        self.theta_dot_max = np.asarray(theta_dot_max, dtype=np.float64).reshape(6)
        self.theta_ddot_max = np.asarray(theta_ddot_max, dtype=np.float64).reshape(6)
        self.theta_cmd = np.zeros(6)  # robot starts at home
        self.calibration_body = None  # PlacedBody of the one special mBee, once created
        self.calibration_body_attached = False

    def __repr__(self):
        return f"Robot(name={self.name!r}, theta_cmd={np.round(self.theta_cmd, 4).tolist()})"

    # ------------------------------------------------------------------ kinematics

    def fwd_kins(self, theta) -> Frame:
        '''
        input: theta - 6 joint values (mm, mm, mm, rad, rad, rad)
        output: F_ee, the end-effector pose relative to the robot base frame
        '''
        theta = np.asarray(theta, dtype=np.float64).reshape(6)
        p_c = vct3(theta[0], theta[1], theta[2])
        R_w = Rot('x', theta[3]) * Rot('y', theta[4]) * Rot('z', theta[5])
        return Frame(Rot(), p_c) * self.F_CW * Frame(R_w, vct3()) * self.F_WE

    def wrist_solutions(self, R: Rot) -> List[np.ndarray]:
        '''
        input: R - wrist rotation, R = Rx(theta_4) Ry(theta_5) Rz(theta_6)
        output: the (up to two) Euler-angle triples reproducing R; near the
                theta_5 = +-pi/2 singularity theta_4 is set to 0
        '''
        M = R.matrix
        s5 = float(np.clip(M[0, 2], -1.0, 1.0))
        theta_5 = float(np.arcsin(s5))
        if abs(np.cos(theta_5)) < 1e-9:
            theta_6 = float(np.arctan2(M[1, 0], M[1, 1]))
            return [np.array([0.0, theta_5, theta_6])]
        theta_4 = float(np.arctan2(-M[1, 2], M[2, 2]))
        theta_6 = float(np.arctan2(-M[0, 1], M[0, 0]))
        first = np.array([theta_4, theta_5, theta_6])
        second = np.array([_wrap_angle(theta_4 + np.pi), _wrap_angle(np.pi - theta_5), _wrap_angle(theta_6 + np.pi)])
        return [first, second]

    def inv_kins(self, F: Frame, seed=None) -> np.ndarray:
        '''
        input: F - desired F_ee relative to the robot base frame
               seed - joint vector used to pick between the two wrist
                   branches (nearest wins); defaults to the current command
        output: theta such that fwd_kins(theta) == F. Raises ValueError if no
                branch satisfies the joint limits.
        '''
        seed = self.theta_cmd if seed is None else np.asarray(seed, dtype=np.float64).reshape(6)
        p_w = F.p - F.R * self.F_WE.p - self.F_CW.p  # position of the wrist center = F_c translation
        candidates = []
        for wrist in self.wrist_solutions(F.R):
            theta = np.concatenate([[p_w.x, p_w.y, p_w.z], wrist])
            if self.within_limits(theta):
                candidates.append(theta)
        if not candidates:
            raise ValueError(f"inv_kins: no joint solution within limits for {F!r}")
        return min(candidates, key=lambda th: float(np.linalg.norm(th - seed)))

    # ------------------------------------------------------------------ limits and timing

    def within_limits(self, theta) -> bool:
        theta = np.asarray(theta, dtype=np.float64).reshape(6)
        return bool(np.all(theta >= self.theta_min - 1e-9) and np.all(theta <= self.theta_max + 1e-9))

    def check_limits(self, theta):
        theta = np.asarray(theta, dtype=np.float64).reshape(6)
        if not self.within_limits(theta):
            raise ValueError(f"joint command {theta.tolist()} violates limits "
                             f"[{self.theta_min.tolist()}, {self.theta_max.tolist()}]")
        return theta

    def is_reachable(self, F: Frame) -> bool:
        try:
            self.inv_kins(F)
            return True
        except ValueError:
            return False

    def motion_time(self, theta_from, theta_to) -> float:
        '''
        input: theta_from, theta_to - joint vectors
        output: t_settle plus the trapezoidal (or triangular, for short moves)
                velocity-profile time of the slowest joint
        '''
        d = np.abs(np.asarray(theta_to, dtype=np.float64) - np.asarray(theta_from, dtype=np.float64)).reshape(6)
        v, a = self.theta_dot_max, self.theta_ddot_max
        t = np.where(d < v ** 2 / a, 2.0 * np.sqrt(d / a), d / v + v / a)
        return self.t_settle + float(np.max(t))

    # ------------------------------------------------------------------ controller interface

    def state(self, t_elapsed: float = 0.0) -> RobotState:
        return RobotState(theta=self.theta_cmd.copy(), theta_dot=np.zeros(6), t_elapsed=t_elapsed,
                          calibration_body_attached=self.calibration_body_attached)

    def commanded_pose(self) -> Frame:
        """CommandedPose() = FwdKins(theta_cmd)"""
        return self.fwd_kins(self.theta_cmd)
