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
from ..simulation import PlacedBody, sample_marker_bodies, clock
from .registration import average_frames
from ..simulation import constants

@dataclass
class CalibrationData:
    """One calibration data set.

    thetas[j] is the commanded (encoder) joint vector and frames[j] the
    calibration body's observed pose relative to the tracker, F_ee,j (this
    handout's tracker has no jiggle, so the tracker frame is a fixed
    reference). The lists include valid observations from all repeats in
    collection order. t_collect is the simulated time the collection took.
    repeat_ids[j] and pose_ids[j] identify the zero-based repeat and input
    pose for each collected observation, including gaps from skipped poses.
    """
    thetas: List[np.ndarray] = field(default_factory=list)
    frames: List[Frame] = field(default_factory=list)
    t_collect: float = 0.0
    repeat_ids: List[int] = field(default_factory=list)
    pose_ids: List[int] = field(default_factory=list)

    def __len__(self):
        return len(self.thetas)


def measure_home_offsets(rob: Robot, n_homings: int = 5) -> np.ndarray:
    """
    Estimate systematic joint offsets from repeated home encoder readings.

    HomeRobot() drives the robot to theta* = 0 and returns the encoder
    values there, i.e. -delta_theta_sys(home) - delta_theta_rnd. Repeat it
    and average to estimate the systematic joint offsets at home (after the
    first homing the move is tiny, so each repeat costs about t_settle).

    input:
        rob - the robot (rob.home_robot())
        n_homings - how many homings to average
    output: estimated delta_theta_sys at home, np.ndarray of 6
    """
    if n_homings < 1:
        raise ValueError("n_homings must be at least 1")

    encoder_readings = []
    for _ in range(n_homings):
        rob.home_robot()
        theta = rob.state().theta
        encoder_readings.append(theta.copy())
     
    # At physical home, encoder theta is the negative joint offset plus noise.
    return -np.mean(encoder_readings, axis=0)

def plan_calibration_poses(rob: Robot, trk: Tracker, mbee_local_positions: List[vct3], desired_accuracy: float, **kwargs) -> List[np.ndarray]:
    """
    Initial axis-sweep planner using predicted visibility; does not move or
    sample the robot. TODO: derive spacing and repetitions from desired_accuracy.
# Minimum sampling coverage for every joint.
samples_per_axis = np.full(6, samples_per_joint, dtype=int)

# Allocate half the orientation-error target to wrist interpolation.
# Divide by 3 so the three wrist error bounds add to this budget.
allowed_error_per_wrist_joint = 0.5 * desired_accuracy / 3.0

for joint in range(3, 6):  # Python indices 3, 4, 5 = joints 4, 5, 6
    slope_bound = constants.OMEGA_SYS[joint]
    joint_range = rob.theta_max[joint] - rob.theta_min[joint]

    if slope_bound > 0:
        max_spacing = (
            2.0 * allowed_error_per_wrist_joint / slope_bound
        )
        required_samples = int(np.ceil(joint_range / max_spacing)) + 1

        samples_per_axis[joint] = max(
            samples_per_joint, required_samples
        )
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
        kwargs - samples_per_joint (default 5, at least 2), and reference_theta
            (default six zeros, the nominal home command). The actual home
            encoder readings may differ from zero because of joint offsets.
    output: ordered List of joint vectors (np.ndarray of 6)
    """
    if not np.isfinite(desired_accuracy) or desired_accuracy <= 0:
        raise ValueError("desired_accuracy must be positive and finite")
    samples_per_joint = kwargs.get("samples_per_joint", 5)
    if (isinstance(samples_per_joint, (bool, np.bool_))
            or not isinstance(samples_per_joint, (int, np.integer))
            or samples_per_joint < 2):
        raise ValueError("samples_per_joint must be an integer of at least 2")
    if len(mbee_local_positions) == 0:
        raise ValueError("At least one marker position is required for visibility checks")

    reference_theta = np.asarray(
        kwargs.get("reference_theta", np.zeros(6)), dtype=float
    ).reshape(6).copy()

    def valid_pose(theta):
        if not np.isfinite(theta).all() or not rob.within_limits(theta):
            return False
        F_ee_world = rob.F_rob * rob.fwd_kins(theta)
        markers_world = [F_ee_world * p for p in mbee_local_positions]
        return trk.is_visible(markers_world)

    if not valid_pose(reference_theta):
        raise ValueError("Choose a reference_theta within joint limits with all markers visible")

    # Minimum sampling coverage for every joint.
    samples_per_axis = np.full(6, samples_per_joint, dtype=int)

    # Allocate half the orientation-error target to wrist interpolation.
    # Divide by 3 so the three wrist error bounds add to this budget.
    allowed_error_per_wrist_joint = 0.05 * desired_accuracy / 3.0

    for joint in range(3, 6):  # Python indices 3, 4, 5 = joints 4, 5, 6
        slope_bound = constants.OMEGA_SYS[joint]
        joint_range = rob.theta_max[joint] - rob.theta_min[joint]

        if slope_bound > 0:
            max_spacing = (
                2.0 * allowed_error_per_wrist_joint / slope_bound
            )
            required_samples = int(np.ceil(joint_range / max_spacing)) + 1

            samples_per_axis[joint] = max(
                samples_per_joint, required_samples
            )

    poses = [reference_theta.copy()]
    for joint in range(6):
        #define the joint values to sweep through for this joint. We define a linear space between the minimum and maximum joint limits for the current joint, with the spacing between each sample defined by the samples_per_joint parameter. 
        joint_values = np.linspace(
            rob.theta_min[joint], rob.theta_max[joint], samples_per_axis[joint]
        )
        # Sweep through the joint values, keeping the other joints at the reference_theta
        for value in joint_values:
            theta = reference_theta.copy()
            theta[joint] = value
        #ensure the current pose is valid (within joint limits and with all markers visible) before adding it to the list of poses. If the pose is valid and not already in the list, it is appended to the poses list
            if not valid_pose(theta):
                continue
            if not np.array_equal(theta, poses[-1]):
                poses.append(theta)

        # Return to the reference before starting the next joint's sweep.
        if not np.array_equal(poses[-1], reference_theta):
            poses.append(reference_theta.copy())
    print(
    f"Joint {joint + 1}: "
    f"allowed error={allowed_error_per_wrist_joint:.3g}, "
    f"max spacing={max_spacing:.3g}, "
    f"required={required_samples}, "
    f"used={samples_per_axis[joint]}"
)

    return poses


def collect_calibration_data(rob: Robot, trk: Tracker, mbee_placed: PlacedBody,
                             thetas: List[np.ndarray], repeats: int = 1) -> CalibrationData:
    """
    Repeat the pose sequence, collecting one tracker reading after each move.

    Each valid observation is stored as matching entries in thetas, frames,
    repeat_ids, and pose_ids. Repeated poses keep separate frame measurements.
    Invisible readings are skipped, but their movement and sampling time is
    included in t_collect.

    input:
        rob, trk - robot and tracker
        mbee_placed - the robot calibration body (attached)
        thetas - joint vectors to visit, in order
        repeats - number of complete passes through the pose sequence
    output: CalibrationData with paired thetas/frames, repeat_ids, pose_ids,
        and t_collect. Observations are ordered by repeat, then input pose.
    """
    if isinstance(repeats, (bool, np.bool_)) or not isinstance(repeats, (int, np.integer)) or repeats < 1:
        raise ValueError("repeats must be a positive integer")

    data = CalibrationData()
    t_start = clock.elapsed

    for repeat in range(repeats):
        for n, theta in enumerate(thetas):
            rob.move_joints(theta)
            observations = sample_marker_bodies(trk)
            observed = observations[mbee_placed.body.name]

            if observed is None:
                continue

            theta_recorded = np.asarray(theta, dtype=float).reshape(6).copy()
            data.thetas.append(theta_recorded)
            data.frames.append(observed.frame)
            data.repeat_ids.append(repeat)
            data.pose_ids.append(n)

    data.t_collect = clock.elapsed - t_start
    return data


def estimate_base_frame(data: CalibrationData, kins: Robot) -> Frame:
    """
    With the calibration body's frame being exactly F_ee, every sample
    satisfies  F_ee,j = F_X * kins.fwd_kins(theta_j)  for one constant
    unknown F_X (the robot base expressed in the tracker's frame). Estimate
    F_X from all samples using average_frames. Cache the inverse predicted
    frame for each unique joint vector within this call.

    input:
        data - CalibrationData containing paired observations across all repeats
        kins - the kinematics to use (the nominal Robot at first; later your
            ImprovedKinematics)
    output: F_X
    """
    inverse_frames = {}
    base_frames = []

    if len(data.thetas) != len(data.frames):
        raise ValueError("Each joint vector must have a corresponding observed frame")

    for theta, F_observed in zip(data.thetas, data.frames):
        # Cache by actual joint values.
        theta_key = tuple(np.asarray(theta, dtype=float).reshape(6))

        if theta_key not in inverse_frames:
            inverse_frames[theta_key] = kins.fwd_kins(theta).inv()

        F_X = F_observed * inverse_frames[theta_key]
        base_frames.append(F_X)

    if not base_frames:
        raise ValueError("No valid observations to estimate the base frame")

    return average_frames(base_frames)
