"""
Tracker simulation for assignment 2: AddMarkerBodies, RemoveMarkerBodies,
TrackerSystematicError and the clocked, visibility-aware SampleMarkerBodies(trk).
The camera model itself (jiggle, sensor noise, Procrustes) is assignment 1's
simulation.tracker, which this module wraps.
"""

from typing import Dict, Iterable, List, Optional

import numpy as np

from data_types.nominal_types import Frame, vct3

from assignments.assignment1.src.tracker import Tracker
from assignments.assignment1.src.simulation.tracker import (  # noqa: F401 -- ObservedBody re-exported
    ObservedBody, create_tracker as _create_tracker, sample_marker_bodies as _sample_marker_bodies,
)
from .marker_body import PlacedBody
from .clock import clock
from . import constants


class SystematicField:
    """TrackerSystematicError(b): d(v) = rho (||v||/D) Rx(tx) Ry(ty) Rz(tz) [1,0,0]
    with (tx, ty, tz) = (v + phase) / D, v in tracker coordinates. One
    random phase per tracker; rho and D fix the magnitude / gradient bounds
    (see constants.TRACKER_SYS_*)."""

    def __init__(self, rho: float, D: float, phase):
        self.rho = float(rho)
        self.D = float(D)
        self.phase = np.asarray(phase, dtype=np.float64).reshape(3)

    def __call__(self, b: vct3) -> vct3:
        v = b.vec.ravel()
        tx, ty, tz = (v + self.phase) / self.D
        cx, sx = np.cos(tx), np.sin(tx)
        cy, sy = np.cos(ty), np.sin(ty)
        cz, sz = np.cos(tz), np.sin(tz)
        direction = np.array([cy * cz, cx * sz + sx * sy * cz, sx * sz - cx * sy * cz])
        return vct3(self.rho * (np.linalg.norm(v) / self.D) * direction)


def create_tracker(name: str, F_approx: Frame, placed_bodies: Iterable[PlacedBody] = (),
                   seed: Optional[int] = None) -> Tracker:
    """trk = CreateTracker(F, {mB1, ..., mBN})

    Places the camera approximately at F_approx, starts tracking the listed
    bodies (the list may be empty; see add_marker_bodies) and draws this
    camera's systematic error field.
    """
    trk = _create_tracker(name, F_approx)
    add_marker_bodies(trk, placed_bodies)
    rng = np.random.default_rng(seed)
    phase = rng.uniform(0.0, 2.0 * np.pi * constants.TRACKER_SYS_D_MM, size=3)
    trk.systematic_error = SystematicField(constants.TRACKER_SYS_RHO_MM, constants.TRACKER_SYS_D_MM, phase)
    return trk


def add_marker_bodies(trk: Tracker, placed_bodies: Iterable[PlacedBody]) -> List[PlacedBody]:
    """PreviousList = AddMarkerBodies(trk, {mB1, ..., mBN})"""
    previous = list(trk.tracked_bodies)
    for placed in placed_bodies:
        if placed not in trk.tracked_bodies:
            trk.tracked_bodies.append(placed)
    return previous


def remove_marker_bodies(trk: Tracker, placed_bodies: Iterable[PlacedBody]) -> List[PlacedBody]:
    """PreviousList = RemoveMarkerBodies(trk, {mB1, ..., mBN})"""
    previous = list(trk.tracked_bodies)
    to_remove = list(placed_bodies)
    trk.tracked_bodies = [p for p in trk.tracked_bodies if p not in to_remove]
    return previous


def sample_marker_bodies(trk: Tracker) -> Dict[str, Optional[ObservedBody]]:
    """{F1, ..., FN} = SampleMarkerBodies(trk)

    One camera snapshot of every tracked body, costing delta_t_samp of
    simulated time whether or not anything is visible. A body with any
    marker outside the viewing volume, or a detached calibration body,
    comes back as None; the rest are read by assignment 1's
    sample_marker_bodies (per-marker noise, the systematic field,
    Procrustes against the body's CURRENT nominal layout; this handout has
    no tracker jiggle, so the frames are relative to the tracker's true
    pose, which is its nominal one). Results are keyed by body name and also stored on the bodies
    (see most_recent_observed_frame / most_recent_observed_markers).
    """
    clock.advance(constants.SAMPLE_DELTA_T_SEC)

    results: Dict[str, Optional[ObservedBody]] = {}
    visible = []
    for placed in trk.tracked_bodies:
        body = placed.body
        world_pts = [placed.actual_placement * a.p for a in body.get_actual_marker_positions()]
        if placed.visible and trk.is_visible(world_pts):
            visible.append(placed)
        else:
            body.observed_frame, body.observed_marker_positions = None, None
        results[body.name] = None
    results.update(_sample_marker_bodies(
        trk, visible, sensor_noise_std=constants.SENSOR_NOISE_STD,
        jiggle_angle_std=constants.TRACKER_JIGGLE_ANGLE_STD, jiggle_pos_std=constants.TRACKER_JIGGLE_POS_STD))
    return results
