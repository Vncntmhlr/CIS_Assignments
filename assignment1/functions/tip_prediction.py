
def predict_tip_position(F_ptr, F_near, p_tip):
    """
    Predict the pointer tip position in the near-reference frame.

    F_ptr: pointer pose relative to tracker
    F_near: near-reference pose relative to tracker
    p_tip: calibrated tip position in pointer coordinates
    """
    return F_near.inv() * F_ptr * p_tip


def position_error_mm(p_predicted, p_actual):
    """Euclidean tip-position error in millimeters."""
    return float((p_predicted - p_actual).norm())


def within_tolerance(error_mm, threshold_mm=2.0):
    """Check whether the error meets the accuracy requirement."""
    return bool(error_mm <= threshold_mm)
