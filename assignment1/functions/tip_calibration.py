
import numpy as np
from src import vct3


def compute_pivot_calibration(
    F_ptr_list, F_cal_list, sensed_tip_list
):
    # Input consistency
    if not (
        len(F_ptr_list)
        == len(F_cal_list)
        == len(sensed_tip_list)
    ):
        raise ValueError("Input lists must have the same length")

    A_list = []
    b_list = []

    for F_ptr, F_cal, p_sensed in zip(
        F_ptr_list, F_cal_list, sensed_tip_list
    ):
        if p_sensed is None:
            continue

        R_ptr = F_ptr.R.matrix
        R_cal = F_cal.R.matrix

        t_ptr = F_ptr.p.vec.ravel()
        t_cal = F_cal.p.vec.ravel()
        p_s = p_sensed.vec.ravel()

        A_list.append(R_ptr)
        b_list.append(R_cal @ p_s + t_cal - t_ptr)

    if not A_list:
        raise ValueError("No valid sensor observations")

    A = np.vstack(A_list)
    b = np.concatenate(b_list)

    x, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)

    if rank < 3:
        raise ValueError("Calibration system is rank deficient")

    p_tip = vct3(*x)

    # Residual-based covariance estimate
    residual = A @ x - b
    dof = len(b) - rank

    if dof > 0:
        sigma_squared = np.sum(residual ** 2) / dof
        cov_tip = sigma_squared * np.linalg.pinv(A.T @ A)
    else:
        cov_tip = np.full((3, 3), np.nan)

    return p_tip, cov_tip
