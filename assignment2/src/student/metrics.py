"""
Pose-error metrics used to report calibration accuracy.
"""

from typing import Dict, Sequence, Tuple

import numpy as np

from data_types.nominal_types import Frame


def frame_error(F_a: Frame, F_b: Frame) -> Tuple[float, float]:
    """
    TODO: implement.

    The mismatch between two frames is defined as
    dF = F_a^-1 F_b ~= Frame(I + sk(alpha), epsilon); report its size.

    input:
        F_a - reference Frame (e.g. ground truth)
        F_b - Frame to compare (e.g. your prediction)
    output:
        (angle_rad, dist_mm) - ||alpha|| (the rotation angle of dF.R) and
            ||epsilon|| (the norm of dF.p)
    """
    """
    Measure the rotational and positional difference between two poses.

    Both poses must be expressed in the same parent coordinate frame.

    Parameters
    ----------
    F_a : Frame
        Reference pose.
    F_b : Frame
        Pose being compared with the reference.

    Returns
    -------
    angle_rad : float
        Magnitude of the relative rotation, between 0 and pi radians.
    dist_mm : float
        Magnitude of the relative translation, in millimetres.
    """

    # Express pose B relative to pose A.
    relative_frame = F_a.inv() * F_b

    # Calculate the magnitude of the positional error.
    translation_error = relative_frame.p.vec.ravel()
    dist_mm = float(np.linalg.norm(translation_error))

    # Extract the relative rotation matrix.
    R = relative_frame.R.matrix

    # Extract the cosine of the relative rotation angle.
    cos_angle = (np.trace(R) - 1.0) / 2.0

    # Prevent rounding errors from putting the input outside [-1, 1].
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    # Rotation-error magnitude in radians, between 0 and pi.
    angle_rad = float(np.arccos(cos_angle))

    return angle_rad, dist_mm


def summarize_errors(errors: Sequence[Tuple[float, float]]) -> Dict[str, float]:
    """
    TODO: implement.

    input:
        errors - list of (angle_rad, dist_mm) pairs from frame_error
    output:
        dict with at least the keys
            n, rms_angle, max_angle, rms_pos, max_pos
    """
    # Each row contains [angle_rad, dist_mm].
    error_array = np.asarray(errors, dtype=float)

    if error_array.size == 0:
        raise ValueError("At least one error pair is required.")

    if error_array.ndim != 2 or error_array.shape[1] != 2:
        raise ValueError("Expected pairs of (angle_rad, dist_mm).")

    if not np.all(np.isfinite(error_array)):
        raise ValueError("Error values must be finite.")

    # Separate the two types of error.
    angle_errors = error_array[:, 0]
    position_errors = error_array[:, 1]

    # Calculate summary statistics.
    n = int(error_array.shape[0])

    rms_angle = float(np.sqrt(np.mean(angle_errors ** 2)))
    rms_pos = float(np.sqrt(np.mean(position_errors ** 2)))

    max_angle = float(np.max(angle_errors))
    max_pos = float(np.max(position_errors))

    return {
        "n": n,
        "rms_angle": rms_angle,
        "rms_pos": rms_pos,
        "max_angle": max_angle,
        "max_pos": max_pos,
    }
