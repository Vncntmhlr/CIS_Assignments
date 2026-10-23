"""
Simulation for Tracker: CreateTracker and
SampleMarkerBodies from the assignment PDF.
"""

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from data_types.nominal_types import Frame, vct3
from data_types.uncertain_types import uvct3, uFrame
from data_types.covariance_types import Covariance
from uncertainty_networks.se3 import exp_se3

from ..tracker import Tracker
from ..utils import sample_normal
from .marker_body import PlacedBody
from . import constants


@dataclass
class ObservedBody:
    """One marker body's result from a single SampleMarkerBodies call."""
    frame: Frame              # this body's pose relative to the tracker (Fptr,j / Fcal,j)
    markers: List[uvct3]      # the individual noisy marker readings behind that pose estimate


def create_tracker(name: str, F_approx: Frame) -> Tracker:
    """CreateTracker(F, {mB1, ..., mBN}) -> trk

    Only the tracker's approximate pose is handled here; which marker
    bodies to read is decided per-call by sample_marker_bodies.
    """
    if not isinstance(F_approx, Frame):
        raise TypeError(f"create_tracker: F_approx must be a Frame, got {F_approx!r} (did you fill in the design TODOs?)")
    return Tracker(name, F_approx)


def sample_marker_bodies(tracker: Tracker, placed_bodies: List[PlacedBody],
                         sensor_noise_std: float = None,
                         jiggle_angle_std: float = None, jiggle_pos_std: float = None) -> Dict[str, ObservedBody]:
    """{F1, ..., FN} = SampleMarkerBodies(trk)

    One simulated camera snapshot of every body in placed_bodies: a single
    shared tracker-pose jiggle is drawn for this call (the whole point of
    "one call reads every tracked body at once"), then each body's markers
    are read (independent per-marker sensor noise, plus the tracker's
    systematic error field if it has one) and registered via
    Tracker.procrustes_solver. The result is also saved on each body
    (observed_frame / observed_marker_positions).

    sensor_noise_std and the jiggle stds default to this assignment's
    constants (SENSOR_NOISE_STD, TRACKER_JIGGLE_ANGLE_STD, TRACKER_JIGGLE_POS_STD).
    """
    F_nom = tracker.F.F if isinstance(tracker.F, uFrame) else tracker.F

    if jiggle_angle_std is None:
        jiggle_angle_std = constants.TRACKER_JIGGLE_ANGLE_STD
    if jiggle_pos_std is None:
        jiggle_pos_std = constants.TRACKER_JIGGLE_POS_STD
    jiggle_cov = Covariance(np.diag([jiggle_angle_std ** 2] * 3 + [jiggle_pos_std ** 2] * 3))
    eta = sample_normal(np.zeros(6), cov=jiggle_cov)[0]
    F_actual = F_nom * Frame(exp_se3(eta))

    if sensor_noise_std is None:
        sensor_noise_std = constants.SENSOR_NOISE_STD
    sensor_noise_cov = Covariance.eye(3, scale=sensor_noise_std ** 2)

    saved_F = tracker.F
    tracker.F = F_actual
    try:
        results: Dict[str, ObservedBody] = {}
        for placed in placed_bodies:
            observed = tracker.read_marker_body(
                placed.body, sensor_noise_cov=sensor_noise_cov, body_placement=placed.actual_placement,
            )
            frame_est = tracker.procrustes_solver(placed.body.nominal_marker_positions, observed)
            placed.body.observed_frame = frame_est
            placed.body.observed_marker_positions = [o.p for o in observed]
            results[placed.body.name] = ObservedBody(frame=frame_est, markers=observed)
        return results
    finally:
        tracker.F = saved_F
