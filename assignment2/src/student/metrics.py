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
    delta = F_a.inv() * F_b
    R = delta.R.matrix

    sin_axis = 0.5 * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ])
    
    sin_angle = np.linalg.norm(sin_axis)
    cos_angle = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    
    angle = np.arctan2(sin_angle, cos_angle)
    return float(angle), float(delta.p.norm())


def summarize_errors(errors: Sequence[Tuple[float, float]]) -> Dict[str, float]:
    """
    TODO: implement.

    input:
        errors - list of (angle_rad, dist_mm) pairs from frame_error
    output:
        dict with at least the keys
            n, rms_angle, max_angle, rms_pos, max_pos
    """
    values = np.asarray(errors, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] == 0 or values.shape[1] != 2:
        raise ValueError("errors must be a non-empty sequence of (angle_rad, dist_mm) pairs")
    if not np.all(np.isfinite(values)) or np.any(values < 0):
        raise ValueError("error magnitudes must be finite and nonnegative")

    rms = np.sqrt(np.mean(values ** 2, axis=0))
    maxima = np.max(values, axis=0)
    return {
        "n": int(values.shape[0]),
        "rms_angle": float(rms[0]),
        "max_angle": float(maxima[0]),
        "rms_pos": float(rms[1]),
        "max_pos": float(maxima[1]),
    }
