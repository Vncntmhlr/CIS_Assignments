from typing import Callable, List, Optional, Union

import numpy as np

from .utils import sample_normal
from data_types.nominal_types import vct3, Frame, Rot, vct3Array
from data_types.uncertain_types import uvct3, uFrame
from data_types.covariance_types import Covariance
from .marker import MarkerBody
from uncertainty_networks.se3 import exp_se3


class Tracker:

    # Viewing volume (Figure 2 of the handout), in the tracker's own frame: the
    # camera looks along its local +z axis; the usable volume starts 100cm in
    # front of it, is 120cm deep, and widens from a 100cm to a 200cm square.
    FRUSTUM_Z_NEAR = 1000.0
    FRUSTUM_Z_FAR = 2200.0
    FRUSTUM_HALF_WIDTH_NEAR = 500.0
    FRUSTUM_HALF_WIDTH_FAR = 1000.0

    def __init__(self, name, F: Union[Frame, uFrame]):
        self.name = name
        self.F = F  # Frame position of the tracking camera in the world coordinate system; set by students
        self.marker_positions = []  # observed marker positions in the tracker's coordinate frame
        self.tracked_bodies = []    # PlacedBody handles read by every SampleMarkerBodies(trk) call
        self.systematic_error: Optional[Callable[[vct3], vct3]] = None  # TrackerSystematicError(b), if simulated

    def __repr__(self):
        return f"Tracker(name={self.name!r}, F={self.F!r})"

    def set_pose(self, frame: Union[Frame, uFrame]):
        self.F = uFrame(frame)
        return self

    def set_cov(self, cov: Covariance):  # here we might want to pass just np.ndarray, but then we avoid any checks
        self.F = uFrame(self.F, cov)
        return self

    def nominal_pose(self) -> Frame:
        return self.F.F if isinstance(self.F, uFrame) else self.F

    def is_visible(self, points_world: List[vct3], F_tracker: Frame = None) -> bool:
        '''
        input: points_world - points expressed in the workspace frame
               F_tracker - tracker pose to test against (defaults to the
                   nominal pose self.F)
        output: True iff every point lies inside the viewing frustum
        '''
        F_inv = (F_tracker if F_tracker is not None else self.nominal_pose()).inv()
        for p in points_world:
            q = F_inv * (p.p if isinstance(p, uvct3) else p)
            if not (self.FRUSTUM_Z_NEAR <= q.z <= self.FRUSTUM_Z_FAR):
                return False
            t = (q.z - self.FRUSTUM_Z_NEAR) / (self.FRUSTUM_Z_FAR - self.FRUSTUM_Z_NEAR)
            half = self.FRUSTUM_HALF_WIDTH_NEAR + t * (self.FRUSTUM_HALF_WIDTH_FAR - self.FRUSTUM_HALF_WIDTH_NEAR)
            if abs(q.x) > half or abs(q.y) > half:
                return False
        return True

    def read_marker_body(self, marker_body: MarkerBody, jiggle_cov: Covariance = None,
                         sensor_noise_cov: Covariance = None, body_placement: Frame = None,
                         systematic_error: Callable[[vct3], vct3] = None) -> List[uvct3]:
        '''
        Reads marker_body's markers through this tracker, in the tracker's
        own coordinate frame.

        input:
            marker_body - a MarkerBody with a cached manufacturing-noise
                ground truth (see MarkerBody.get_actual_marker_positions)
            jiggle_cov - 6x6 matrix: covariance of this tracker's
                OWN pose perturbation ([alpha; epsilon]). One fresh sample is
                drawn per call and shared across every marker read in that
                call (F*_tracker = F_tracker . Exp(eta)).
            sensor_noise_cov - 3x3 matrix: covariance of the
                camera's per-marker detection noise (sigma_b). An
                independent fresh sample is drawn for each marker, every
                call.
            body_placement - marker_body's current actual pose, expressed in
                whatever frame self.F itself is measured against (e.g. the
                workspace). Composed in before applying this tracker's own
                pose, so bodies at different physical locations/orientations
                correctly register to different Frames. Defaults to None
                (marker_body's local frame is treated as already being in
                that same reference frame -- i.e. no separate placement).
            systematic_error - TrackerSystematicError(b) field, defaulting
                to self.systematic_error (None = no systematic error). The
                handout defines b* = b + eps_sys(b) + eps_b, so the reported
                value is b = b* - eps_sys(b) - eps_b, evaluated at b* to first
                order.
        output: samples - list of observed marker positions (List[uvct3]),
            in the tracker's frame
        '''
        F_nom = self.nominal_pose()
        F_actual = F_nom
        if jiggle_cov is not None:
            eta = sample_normal(np.zeros(6), cov=jiggle_cov)[0]  # [alpha(3); epsilon(3)], one draw for this call
            dF = Frame(exp_se3(eta))  # Monte-Carlo perturbation helper: T_true = T_nom . Exp(eta)
            F_actual = F_nom * dF     # F*_tracker = F_tracker . delta_F_tracker
        if systematic_error is None:
            systematic_error = self.systematic_error

        observed = []
        for actual_pos in marker_body.get_actual_marker_positions():
            local_p = actual_pos.p
            placed_p = (body_placement * local_p) if body_placement is not None else local_p
            p = F_actual.inv() * placed_p  # b*: plain Frame * vct3 -> vct3
            if systematic_error is not None:
                p = p - systematic_error(p)
            if sensor_noise_cov is not None:
                p = sample_normal(p, cov=sensor_noise_cov)  # independent fresh draw per marker
            observed.append(uvct3(p))

        self.marker_positions = observed
        return observed

    def procrustes_solver(self, nominal_points: List[vct3], observed_points: List[uvct3]) -> Frame:
        '''
        Rigid point-set registration via Arun's SVD method: finds the best-fit
        Frame F (in the least-squares sense) such that
        F * nominal_points[i] ~= observed_points[i], for corresponding pairs.

        input:
            nominal_points - marker body's best-known geometry, in its own
                local coordinate frame
            observed_points - this tracker's paired observations of those
                same markers (e.g. from read_marker_body), in the tracker's
                frame, same order/correspondence as nominal_points
        output: the best-fit Frame mapping nominal_points -> observed_points
        '''
        return register_points_arun(nominal_points, observed_points)


def register_points_arun(nominal_points: List[vct3], observed_points: List[Union[vct3, uvct3]]) -> Frame:
    '''
    input: nominal_points, observed_points - paired point sets (>= 3 pairs)
    output: the least-squares Frame F with F * nominal_points[i] ~= observed_points[i]
    '''
    if len(nominal_points) != len(observed_points) or len(nominal_points) < 3:
        raise ValueError("procrustes_solver requires >= 3 paired points")

    P = vct3Array(list(nominal_points))
    Q = vct3Array([o.p if isinstance(o, uvct3) else o for o in observed_points])
    p_bar, q_bar = P.mean(), Q.mean()
    Pc, Qc = (P - p_bar).mat, (Q - q_bar).mat

    U, _, Vt = np.linalg.svd(Pc @ Qc.T)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R_mat = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    R = Rot(matrix=R_mat)
    p_vec = q_bar.vec - R_mat @ p_bar.vec
    p = vct3(p_vec[0, 0], p_vec[1, 0], p_vec[2, 0])
    return Frame(R, p)
