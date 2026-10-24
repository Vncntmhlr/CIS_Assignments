"""
Data-collection workflow: which robot poses to visit, how to read them, and
how to register the robot base to the tracker.
"""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from data_types.nominal_types import vct3, Frame

from ..robot import Robot
from assignments.assignment1.src.tracker import Tracker
from ..simulation import PlacedBody


@dataclass
class CalibrationData:
    """One calibration data set.

    thetas[j] is the commanded (encoder) joint vector and frames[j] the
    calibration body's observed pose relative to the tracker, F_ee,j (this
    handout's tracker has no jiggle, so the tracker frame is a fixed
    reference). t_collect is the simulated time the collection took.
    """
    thetas: List[np.ndarray] = field(default_factory=list)
    frames: List[Frame] = field(default_factory=list)
    t_collect: float = 0.0

    def __len__(self):
        return len(self.thetas)


def measure_home_offsets(rob: Robot, n_homings: int = 5) -> np.ndarray:
    """
    TODO: implement.

    HomeRobot() drives the robot to theta* = 0 and returns the encoder
    values there, i.e. -delta_theta_sys(home) - delta_theta_rnd. Repeat it
    and average to estimate the systematic joint offsets at home (after the
    first homing the move is tiny, so each repeat costs about t_settle).

    input:
        rob - the robot (rob.home_robot())
        n_homings - how many homings to average
    output: estimated delta_theta_sys at home, np.ndarray of 6
    """
    if (isinstance(n_homings, (bool, np.bool_))
            or not isinstance(n_homings, (int, np.integer)) or n_homings < 1):
        raise ValueError("n_homings must be a positive integer")
    readings = []
    for _ in range(n_homings):
        encoders = np.asarray(rob.home_robot(), dtype=np.float64)
        if encoders.shape != (6,) or not np.all(np.isfinite(encoders)):
            raise ValueError("home_robot must return 6 finite encoder values")
        readings.append(encoders.copy())
    # At physical home: 0 = theta_enc + delta_sys(home) + delta_rnd.
    # This estimates the additive physical-joint correction, not theta_enc.
    return -np.mean(readings, axis=0)


# this should contain a desired accuracy which will affect the number of calibration poses
def plan_calibration_poses(rob: Robot, trk: Tracker, mbee_local_positions: List[vct3], desired_accuracy: float, **kwargs) -> List[np.ndarray]:
    """
    TODO: implement -- this is a design task.

    Choose the joint vectors the robot will visit during calibration to reach the desired accuracy. Every
    parameter of your kinematic model must be excited by the data (axis
    tilts need travel along each axis; the wrist axes and F_we need wrist
    rotations; the joint distortion needs several stops along each joint),
    every pose must keep all calibration-body markers inside the tracker's
    viewing volume (Tracker.is_visible, using the nominal rob.F_rob and
    rob.fwd_kins -- they are only tens of mm off), and the order of the
    poses determines the motion time. The wrist moves at 0.1 rad/s: a pi/2
    turn costs ~16 s.

    The handout's method 1: decide how closely spaced the samples along an
    axis must be from the error bounds (delta_sys^trk, omega_sys^trk,
    delta_sys^rob, omega_sys^rob), then sweep each axis in turn from the
    home position. Joints 1-3 are decoupled from 4-6.

    input:
        rob - the Robot (nominal kinematics + limits)
        trk - the Tracker (for visibility checks)
        mbee_local_positions - the calibration body's marker layout a*_k
        desired_accuracy - target RMS orientation error in rad (the limiting
            metric); use it to size the plan -- how many distinct poses and
            how often each is read (a pose listed twice in a row is
            re-commanded and re-read, costing only t_settle + delta_t_samp)
        kwargs - whatever your design needs (grid sizes, margins, ...)
    output: ordered List of joint vectors (np.ndarray of 6)
    """
    from ..simulation import constants

    desired_accuracy = float(desired_accuracy)
    position_accuracy = float(kwargs.pop("position_accuracy", constants.TARGET_POS_ERR_MM))
    noise_fraction = float(kwargs.pop("noise_fraction", 0.5))
    angular_step = float(kwargs.pop("max_angular_step", np.pi / 4))
    if kwargs:
        raise TypeError(f"unknown planning options: {', '.join(kwargs)}")
    if (not np.isfinite(desired_accuracy) or desired_accuracy <= 0
            or not np.isfinite(position_accuracy) or position_accuracy <= 0
            or not np.isfinite(noise_fraction) or not 0 < noise_fraction < 1
            or not np.isfinite(angular_step) or angular_step <= 0):
        raise ValueError("accuracies/step must be positive and 0 < noise_fraction < 1")
    markers = np.asarray([p.vec.ravel() for p in mbee_local_positions], dtype=np.float64)
    if (markers.ndim != 2 or markers.shape[1] != 3 or len(markers) < 3
            or not np.all(np.isfinite(markers))):
        raise ValueError("need at least 3 finite calibration markers")
    if np.any(np.linalg.norm(markers, axis=1) > constants.CAL_BODY_MAX_RADIUS_MM + 1e-9):
        raise ValueError("calibration markers must lie within the 75 mm radius")
    centered = markers - markers.mean(axis=0)
    information = (np.sum(centered ** 2) * np.eye(3) - centered.T @ centered)
    eigenvalues = np.linalg.eigvalsh(information)
    if eigenvalues[0] <= 3 * np.finfo(float).eps * eigenvalues[-1]:
        raise ValueError("calibration marker geometry must be noncollinear")

    # First-order independent-noise estimate: Cov(alpha) = sigma_b^2 G^-1.
    # Consecutive commands deliberately re-roll robot repeatability too.
    marker_angle_var = constants.SENSOR_NOISE_STD ** 2 * np.trace(np.linalg.inv(information))
    angle_var = marker_angle_var + np.sum(constants.SIGMA_RND[3:] ** 2)
    tool_lever = rob.F_WE.p.norm() + constants.WE_EPS_MAX
    position_var = (3 * constants.SENSOR_NOISE_STD ** 2 / len(markers)
                    + np.sum(markers.mean(axis=0) ** 2) * marker_angle_var
                    + np.sum(constants.SIGMA_RND[:3] ** 2)
                    + tool_lever ** 2 * np.sum(constants.SIGMA_RND[3:] ** 2))
    # Tightening the angle request also tightens the spatial design budget.
    position_accuracy *= min(1.0, desired_accuracy / constants.TARGET_ANG_ERR_RAD)
    repetitions = int(np.ceil(max(angle_var / (noise_fraction * desired_accuracy) ** 2,
                                  position_var / (noise_fraction * position_accuracy) ** 2)))

    # Classroom first-derivative interpolation bound: error <= L*h/2.
    # Linear residual profiles include tracker bias along a fixed-wrist sweep.
    # This bounds individual profiles with exact endpoints, not final FK error.
    profile_slope = (constants.OMEGA_SYS[:3] + constants.TRACKER_SYS_GRAD_MAX
                     * (1 + constants.OMEGA_SYS[:3]))
    linear_steps = 2 * (position_accuracy / 8) / profile_slope
    wrist_steps = np.minimum(min(angular_step, np.pi / 4),
                              2 * (desired_accuracy / 24) / constants.OMEGA_SYS[3:])
    steps = np.r_[linear_steps, wrist_steps]
    lower, upper = np.asarray(rob.theta_min), np.asarray(rob.theta_max)
    if (lower.shape != (6,) or upper.shape != (6,)
            or not np.all(np.isfinite(np.r_[lower, upper])) or np.any(upper <= lower)):
        raise ValueError("need finite nonzero travel for all six joints")
    # This list interface expresses encoder commands, not HomeRobot actions.
    # Step 6 performs physical homing; zero is the common nominal reference.
    home = np.clip(np.zeros(6), lower, upper)
    intervals = np.ceil((upper - lower) / steps)
    if (not np.all(np.isfinite(intervals))
            or repetitions * (np.sum(intervals) + 13) > 100000):
        raise ValueError("requested accuracy exceeds the 100000-reading planning limit")
    visits = [home.copy()]
    for k in range(6):
        # Monotone lower-to-upper traversal visits every knot and returns to
        # the same reference; total single-axis travel is the minimum 2*range.
        values = np.linspace(lower[k], upper[k], int(intervals[k]) + 1)
        for value in values:
            if np.isclose(value, home[k], atol=1e-12, rtol=0):
                continue
            theta = home.copy()
            theta[k] = value
            visits.append(theta)
        visits.append(home.copy())

    poses = []
    seen_home = False
    for theta in visits:
        if not rob.within_limits(theta):
            raise ValueError("calibration plan violates joint limits")
        frame = rob.F_rob * rob.fwd_kins(theta)
        if not trk.is_visible([frame * p for p in mbee_local_positions]):
            raise ValueError("full single-axis plan is not visible; revise scene placement")
        at_home = np.array_equal(theta, home)
        # Pool repeated reference readings; each return still gets one check.
        count = 1 if at_home and seen_home else repetitions
        poses.extend(theta.copy() for _ in range(count))
        seen_home = seen_home or at_home
    return poses


def collect_calibration_data(rob: Robot, trk: Tracker, mbee_placed: PlacedBody,
                             thetas: List[np.ndarray], repeats: int = 1) -> CalibrationData:
    """
    TODO: implement.

    Visit each joint vector with rob.move_joints, read the calibration body
    with sample_marker_bodies and store its frame relative to the tracker.
    Decide what to do when the body is not visible (None), and whether/how
    to use repeats (re-sampling costs 0.03 s; re-commanding the same pose
    costs t_settle and re-rolls the joint repeatability noise).

    input:
        rob, trk - robot and tracker
        mbee_placed - the robot calibration body (attached)
        thetas - joint vectors to visit, in order
        repeats - readings per pose
    output: CalibrationData
    """
    from copy import deepcopy
    from .. import simulation

    if (isinstance(repeats, (bool, np.bool_))
            or not isinstance(repeats, (int, np.integer)) or repeats < 1):
        raise ValueError("repeats must be a positive integer")
    poses = [np.asarray(theta, dtype=np.float64).copy() for theta in thetas]
    if any(theta.shape != (6,) or not np.all(np.isfinite(theta)) for theta in poses):
        raise ValueError("each pose must contain 6 finite encoder commands")

    data = CalibrationData()
    started = simulation.clock.elapsed
    for theta in poses:
        rob.move_joints(theta)
        for _ in range(repeats):
            # The robot is settled; pair this snapshot with its reported
            # encoders, not a requested command or hidden physical joints.
            encoders = np.asarray(rob.state().theta, dtype=np.float64).copy()
            if encoders.shape != (6,) or not np.all(np.isfinite(encoders)):
                raise ValueError("robot state must contain 6 finite encoder values")
            observation = simulation.sample_marker_bodies(trk).get(mbee_placed.body.name)
            if observation is None:
                # Never reuse a cached frame or add an unmatched theta.
                # The failed snapshot still costs time; do not retry forever.
                continue
            data.thetas.append(encoders)
            data.frames.append(deepcopy(observation.frame))
    data.t_collect = float(simulation.clock.elapsed - started)
    return data


def estimate_base_frame(data: CalibrationData, kins: Robot) -> Frame:
    """
    TODO: implement.

    With the calibration body's frame being exactly F_ee, every sample
    satisfies  F_ee,j = F_X * kins.fwd_kins(theta_j)  for one constant
    unknown F_X (the robot base expressed in the tracker's frame). Estimate
    F_X from all samples (your average_frames may help).

    input:
        data - CalibrationData
        kins - the kinematics to use (the nominal Robot at first; later your
            ImprovedKinematics)
    output: F_X
    """
    from . import registration

    if len(data.thetas) == 0 or len(data.thetas) != len(data.frames):
        raise ValueError("base estimation requires nonempty, paired theta/frame samples")
    estimates = []
    for theta, observed in zip(data.thetas, data.frames):
        theta = np.asarray(theta, dtype=np.float64)
        if theta.shape != (6,) or not np.all(np.isfinite(theta)):
            raise ValueError("each sample must contain 6 finite encoder values")
        if not isinstance(observed, Frame):
            raise ValueError("each sample must contain an observed Frame")
        # T_tracker_ee = T_tracker_base * T_base_ee(theta).
        # Right-cancel the nominal EE pose; reversing these factors is wrong.
        estimates.append(observed * kins.fwd_kins(theta).inv())
    return registration.average_frames(estimates)
