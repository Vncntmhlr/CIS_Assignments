"""
Rigid registration helpers: point-set registration, frame averaging and
marker-body refinement.
"""

from typing import List

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame

from assignments.assignment1.src.tracker import Tracker
from ..simulation import (
    PlacedBody, sample_marker_bodies, most_recent_observed_markers,
    update_marker_body,
)


def register_points(a: List[vct3], b: List[vct3]) -> Frame:
    """
    Rigid point-set registration: find the least-squares Frame F such that
    F * a[i] ~= b[i] for every corresponding pair. Think about what happens
    with fewer than 3 points, with collinear points, and with a reflection.

    input:
        a - points in the source frame (e.g. a marker body's local layout)
        b - the same points observed in the target frame, same order
    output: Frame mapping a -> b
    """
    if len(a) != len(b) or len(a) < 3:
        raise ValueError("registration needs at least 3 corresponding point pairs")
    A = np.array([point.vec.ravel() for point in a])
    B = np.array([point.vec.ravel() for point in b])
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        raise ValueError("marker coordinates must be finite")
    a_center, b_center = A.mean(axis=0), B.mean(axis=0)
    A0, B0 = A - a_center, B - b_center
    if np.linalg.matrix_rank(A0) < 2 or np.linalg.matrix_rank(B0) < 2:
        raise ValueError("marker points must not be collinear")

    # Arun/Kabsch: fit a proper rotation, excluding a reflection.
    U, _, Vt = np.linalg.svd(A0.T @ B0)
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ correction @ U.T
    return Frame(Rot(R), vct3(b_center - R @ a_center))


def average_frames(frames: List[Frame]) -> Frame:
    """
    Average of several noisy estimates of the same Frame. Rotations cannot
    simply be averaged element-wise -- the result must be a proper
    rotation matrix. Project the mean rotation matrix onto SO(3) using SVD.

    input:
        frames - List[Frame], all estimates of the same transform
    output: the averaged Frame
    """
    if len(frames) == 0:
        raise ValueError("average_frames needs at least one frame")

    R = np.array([frame.R.matrix for frame in frames])  # (N, 3, 3)
    p = np.array([frame.p.vec for frame in frames])     # (N, 3, 1)
    if not np.isfinite(R).all() or not np.isfinite(p).all():
        raise ValueError("frame rotations and translations must be finite")

    p_avg = np.mean(p, axis=0)
    R_mean = np.mean(R, axis=0)

    # Find the closest proper rotation to the elementwise matrix mean.
    U, _, Vt = np.linalg.svd(R_mean)
    correction = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        correction[-1, -1] = -1.0
    R_avg = U @ correction @ Vt

    return Frame(Rot(matrix=R_avg), vct3(p_avg))


def refine_marker_body(trk: Tracker, placed: PlacedBody, n_readings: int = 30) -> List[vct3]:
    """
    "It is possible to improve your estimate of a_m,k through a calibration
    process involving multiple observations of the marker body using the
    tracking camera and then averaging them."

    Take n_readings SampleMarkerBodies readings, use
    most_recent_observed_markers(...) plus your register_points to bring
    each reading's b_m,k back into the body's own frame, average, and push
    the result into the body with update_marker_body(...). Each reading costs
    delta_t_samp of simulated time; assignment 2 has no tracker jiggle.
    The body must already be registered with the tracker and visible.

    Alignment to the original nominal layout fixes the body's coordinate
    convention. The refined shape is recoverable only up to a rigid transform;
    the unknown true body origin/orientation cannot be inferred separately.

    input:
        trk - Tracker the body is tracked by
        placed - PlacedBody whose body's nominal marker positions should be refined
        n_readings - number of readings to average
    output: the new List[vct3] of marker positions (also stored on the body).
        most_recent_observed_frame(placed.body) then returns the refined
        body-to-tracker transform for the LAST observation, recomputed using
        the new layout. This is not an average pose across observations.
    """
    if (isinstance(n_readings, (bool, np.bool_))
            or not isinstance(n_readings, (int, np.integer)) or n_readings < 1):
        raise ValueError("n_readings must be a positive integer")
    if not any(body is placed for body in trk.tracked_bodies):
        raise ValueError("add the placed body to the tracker before refinement")

    nominal = list(placed.body.nominal_marker_positions)
    register_points(nominal, nominal)  # Validate before spending simulated time.
    local_readings = []
    for _ in range(n_readings):
        sample_marker_bodies(trk)
        observed = most_recent_observed_markers(placed.body)
        if observed is None:
            raise ValueError("marker body is not visible; its layout was not updated")
        F_body_to_tracker = register_points(nominal, observed)
        F_tracker_to_body = F_body_to_tracker.inv()
        local_readings.append([
            (F_tracker_to_body * point).vec.ravel() for point in observed
        ])

    refined = [vct3(point) for point in np.mean(local_readings, axis=0)]
    # Refit to the last raw observation so its cached frame matches the new layout.
    F_refined = register_points(refined, observed)
    update_marker_body(placed.body, refined)
    placed.body.observed_frame = F_refined
    return refined
