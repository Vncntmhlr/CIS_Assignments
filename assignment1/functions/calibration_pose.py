
import numpy as np
from src import Rot


def rotation_distance_deg(R1, R2):
    """
    Geodesic angular distance between two rotations, in degrees.
    """

    R_relative = R1.matrix.T @ R2.matrix

    cos_theta = (np.trace(R_relative) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    theta = np.arccos(cos_theta)

    return float(np.degrees(theta))


def is_rotation_diverse(
    R_candidate,
    accepted_rotations,
    min_angle_diff=25.0
):
    """
    Return True if the candidate is sufficiently different
    from every previously accepted rotation.
    """

    for R_old in accepted_rotations:
        if rotation_distance_deg(R_candidate, R_old) < min_angle_diff:
            return False

    return True


def sample_diverse_rotation(
    rng,
    accepted_rotations,
    min_angle_diff=25.0
):
    """
    Sample XYZ Euler angles from [-70, 70] degrees
    until a sufficiently diverse rotation is found.

    Returns:
        R_desired, (rx, ry, rz)
    """

    for _ in range(10000):

        rx = int(rng.integers(-70, 71))
        ry = int(rng.integers(-70, 71))
        rz = int(rng.integers(-70, 71))

        R_candidate = Rot.xyz_deg(rx, ry, rz)

        if is_rotation_diverse(
            R_candidate,
            accepted_rotations,
            min_angle_diff
        ):
            return R_candidate, (rx, ry, rz)

    raise RuntimeError("Unable to generate a diverse rotation")
