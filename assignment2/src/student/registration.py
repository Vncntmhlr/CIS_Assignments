"""
Rigid registration helpers: point-set registration, frame averaging and
marker-body refinement.
"""

from typing import List

import numpy as np

from data_types.nominal_types import vct3, Rot, Frame

from assignments.assignment1.src.tracker import Tracker
from ..simulation import PlacedBody


def register_points(a: List[vct3], b: List[vct3]) -> Frame:
    """
    TODO: implement (your own -- do not call the simulator's solver).

    Rigid point-set registration: find the least-squares Frame F such that
    F * a[i] ~= b[i] for every corresponding pair. Think about what happens
    with fewer than 3 points, with collinear points, and with a reflection.

    input:
        a - points in the source frame (e.g. a marker body's local layout)
        b - the same points observed in the target frame, same order
    output: Frame mapping a -> b
    """
    if len(a) != len(b) or len(a) < 3:
        raise ValueError("registration requires at least 3 corresponding point pairs")
    source = np.asarray([p.vec.ravel() for p in a], dtype=np.float64)
    target = np.asarray([p.vec.ravel() for p in b], dtype=np.float64)
    if source.shape != (len(a), 3) or target.shape != (len(b), 3):
        raise ValueError("registration points must have 3 coordinates")
    if not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("registration points must be finite")

    source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    if (np.linalg.matrix_rank(source_centered) < 2
            or np.linalg.matrix_rank(target_centered) < 2):
        raise ValueError("registration requires noncollinear source and target points")

    H = source_centered.T @ target_centered
    U, singular_values, Vt = np.linalg.svd(H)
    tolerance = 3 * np.finfo(float).eps * singular_values[0]
    if singular_values[1] <= tolerance:
        raise ValueError("point correspondences do not constrain a unique rotation")
    correction = np.eye(3)

    correction[2, 2] = -1.0 if np.linalg.det(Vt.T @ U.T) < 0 else 1.0
    rotation = Vt.T @ correction @ U.T
    translation = target_mean - rotation @ source_mean
    return Frame(Rot(matrix=rotation), vct3(translation))


def average_frames(frames: List[Frame]) -> Frame:
    """
    TODO: implement.

    Average of several noisy estimates of the same Frame. Rotations cannot
    simply be averaged element-wise -- the result must be a proper
    rotation matrix.

    input:
        frames - List[Frame], all estimates of the same transform
    output: the averaged Frame
    """
    if len(frames) == 0:
        raise ValueError("cannot average an empty frame sequence")
    rotations = np.asarray([frame.R.matrix for frame in frames], dtype=np.float64)
    translations = np.asarray([frame.p.vec.ravel() for frame in frames], dtype=np.float64)
    if rotations.shape != (len(frames), 3, 3) or translations.shape != (len(frames), 3):
        raise ValueError("frames must contain 3x3 rotations and 3D translations")
    if not np.all(np.isfinite(rotations)) or not np.all(np.isfinite(translations)):
        raise ValueError("frames must be finite")
    if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3),
                        atol=1e-8, rtol=0)
            or not np.allclose(np.linalg.det(rotations), 1.0, atol=1e-8, rtol=0)):
        raise ValueError("input frame rotations must belong to SO(3)")

    U, _, Vt = np.linalg.svd(rotations.mean(axis=0))
    correction = np.eye(3)
    correction[2, 2] = -1.0 if np.linalg.det(U @ Vt) < 0 else 1.0
    rotation = U @ correction @ Vt
    return Frame(Rot(matrix=rotation), vct3(translations.mean(axis=0)))


def refine_marker_body(trk: Tracker, placed: PlacedBody, n_readings: int = 30) -> List[vct3]:
    """
    TODO: implement.

    "It is possible to improve your estimate of a_m,k through a calibration
    process involving multiple observations of the marker body using the
    tracking camera and then averaging them."

    Take n_readings SampleMarkerBodies readings, use
    most_recent_observed_markers(...) plus your register_points to bring
    each reading's b_m,k back into the body's own frame, average, and push
    the result into the body with update_marker_body(...). Note that each
    reading costs delta_t_samp of simulated time, and that the tracker
    jiggles between readings.

    input:
        trk - Tracker the body is tracked by
        placed - PlacedBody whose body's nominal marker positions should be refined
        n_readings - number of readings to average
    output: the new List[vct3] of marker positions (also stored on the body)
    """
    raise NotImplementedError("TODO: implement refine_marker_body")
