"""
Ground-truth parameters for the assignment 2 simulation (handout of Oct 14).

All lengths are in millimeters, all angles in radians, all times in seconds.

Values shared with assignment 1 (marker manufacturing, body placement,
delta_t_samp) are imported from its constants so the two simulations agree.
Per-sample noise is Gaussian with the std the handout gives. Static "truth"
offsets that the handout bounds as ||x|| <= B (axis tilts, F_we offset,
robot placement, joint distortion terms) are drawn uniformly in direction
with a norm in [BOUND_MIN_FRAC * B, B], so no seed is accidentally easy.
"""

import numpy as np

from assignments.assignment1.src.tracker import Tracker
from assignments.assignment1.src.simulation.constants import (  # noqa: F401 -- re-exported
    SAMPLE_DELTA_T_SEC, MARKER_MANUFACTURING_STD, BODY_PLACEMENT_ANGLE_STD, BODY_PLACEMENT_POS_STD,
)

BOUND_MIN_FRAC = 0.5  # static truth draws never use less than half of their bound

# ---------------------------------------------------------------------------
# Tracker
# ---------------------------------------------------------------------------

# "For the purposes of this assignment, we will (unrealistically) assume that
# the camera has no jiggle": the tracker's actual pose IS its nominal pose.
TRACKER_JIGGLE_ANGLE_STD = 0.0     # sigma_Ta
TRACKER_JIGGLE_POS_STD = 0.0       # sigma_Tp
SENSOR_NOISE_STD = 0.1             # sigma_b: per-marker camera detection noise, mm

# TrackerSystematicError(b) = eps_sys^trk(b): a deterministic, smooth field over
# tracker coordinates, drawn ONCE per tracker (random phase, fixed magnitude):
#     d(v) = rho * (||v|| / D) * Rx(tx) Ry(ty) Rz(tz) [1, 0, 0],  (tx, ty, tz) = (v + p) / D
# so ||d(v)|| = rho ||v|| / D exactly. With R = max ||v|| over the viewing
# volume, the handout's bounds ||eps_sys|| <= delta_sys and ||grad eps_sys|| <= omega_sys
# hold for rho = delta_sys D / R and any D >= sqrt(2) delta_sys / (omega_sys - delta_sys / R).
# D is otherwise free: the field's Jacobian is (nearly) a small rotation of size
# (rho / D)(||v|| / D) that turns over a distance ~D, and that part cannot be
# absorbed by any kinematic parameter -- at D = D_MIN it is 3-4e-4 rad in the
# work volume, so a larger D is used.
TRACKER_SYS_MAX_MM = 0.1           # delta_sys^trk
TRACKER_SYS_GRAD_MAX = 7.5e-4      # omega_sys^trk
TRACKER_SYS_RADIUS_MM = float(np.sqrt(2 * Tracker.FRUSTUM_HALF_WIDTH_FAR ** 2 + Tracker.FRUSTUM_Z_FAR ** 2))  # R
TRACKER_SYS_D_MIN_MM = np.sqrt(2) * TRACKER_SYS_MAX_MM / (TRACKER_SYS_GRAD_MAX - TRACKER_SYS_MAX_MM / TRACKER_SYS_RADIUS_MM)
TRACKER_SYS_D_MM = 1000.0          # D (>= D_MIN ~ 199mm)
TRACKER_SYS_RHO_MM = TRACKER_SYS_MAX_MM * TRACKER_SYS_D_MM / TRACKER_SYS_RADIUS_MM  # rho
assert TRACKER_SYS_D_MM >= TRACKER_SYS_D_MIN_MM, "TRACKER_SYS_D_MM violates the gradient bound"

# ---------------------------------------------------------------------------
# Robot: the handout's values, used as CreateRobot defaults
# ---------------------------------------------------------------------------

ALPHA_MAX_ROB = 0.05               # ||alpha_rob||: base orientation uncertainty, rad
EPS_MAX_ROB = 5.0                  # ||epsilon_rob||: base position uncertainty, mm
THETA_MIN = np.array([0.0, -300.0, 0.0, -0.75 * np.pi, -0.75 * np.pi, -np.pi])
THETA_MAX = np.array([600.0, 300.0, 600.0, 0.75 * np.pi, 0.75 * np.pi, np.pi])
T_SETTLE = 0.05
PHI_MAX = 0.02                     # ||phi_k||: axis direction error for d_2..d_6 (d_1 is exact), rad
THETA_DOT_MAX = np.array([50.0, 50.0, 50.0, 0.1, 0.1, 0.1])        # mm/s, rad/s
THETA_DDOT_MAX = np.array([500.0, 500.0, 500.0, 1.0, 1.0, 1.0])    # mm/s^2, rad/s^2

# theta*_k = theta_k + delta_theta_k,rnd + delta_theta_k,sys on every joint, where theta_k
# is what the encoders / State() report and theta*_k is the physical joint value:
#   delta_theta_k,rnd ~ N(0, sigma_rnd,k^2), fresh on every move
#   delta_theta_k,sys(theta) = c_k + s_k theta + sigma_sys,k sin(A_k theta + B_k)
# i.e. a constant offset, a scale error and a smooth ripple of std sigma_sys,
# drawn once per robot so that the handout's bounds hold over the whole travel:
#   ||delta_theta_k,sys|| <= delta_sys,k        (|c| + |s| theta_max + sigma_sys <= delta_sys)
#   |d/dtheta delta_theta_k,sys| <= omega_sys,k  (|s| + sigma_sys A <= omega_sys)
# HomeRobot() drives the robot to theta* = 0 and returns the encoder values,
# which therefore read -delta_theta_sys(home) - delta_theta_rnd.
SIGMA_RND = np.array([0.005, 0.005, 0.005, 1e-4, 1e-4, 1e-4])      # mm, rad
SIGMA_SYS = np.array([0.01, 0.01, 0.01, 1e-3, 1e-3, 1e-3])         # ripple std, mm / rad
DELTA_SYS = np.array([0.2, 0.2, 0.2, 0.01, 0.01, 0.01])            # ||delta_theta_sys|| bound, mm / rad
OMEGA_SYS = np.array([5e-4, 5e-4, 5e-4, 5e-5, 5e-5, 5e-5])         # slope bound, mm/mm / rad/rad
RIPPLE_SLOPE_SHARE = (0.3, 0.7)                                    # share of omega_sys given to the ripple (sigma A)
DISTORT_PHASE_MAX = np.pi                                          # B_k ~ U(0, pi)

WE_ALPHA_MAX = 0.01                # ||alpha_ee||: tool-holder orientation offset, rad
WE_EPS_MAX = 0.5                   # ||epsilon_ee||: tool-holder position offset, mm

CAL_BODY_MAX_RADIUS_MM = 75.0      # ||a*_k|| bound for the robot calibration body

# ---------------------------------------------------------------------------
# Accuracy targets from the handout ("report the accuracy achieved" if not met)
# ---------------------------------------------------------------------------

TARGET_POS_ERR_MM = 0.2            # ||epsilon_FwdK||, ||epsilon_InvK|| over the work volume
TARGET_ANG_ERR_RAD = 1e-3          # ||alpha_FwdK||, ||alpha_InvK|| over the work volume
TARGET_TIME_SEC = 900.0            # soft simulated-time budget for the calibration (handout: "as short as possible")
