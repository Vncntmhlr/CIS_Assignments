"""The end-to-end registration workflow: design, patch plan, data collection, registration."""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from data_types.nominal_types import Frame, Rot, vct3

from ..mesh import Mesh, PatchBox
from ..session import Session, SceneDesign, PointerPlacement
from .report import RegistrationReport


_TRACKER_STANDOFF_MM = 1600.0
_REFERENCE_MARKERS_MM = (
    (-42.0, -25.0, 12.0),
    (36.0, -20.0, 18.0),
    (-17.0, 34.0, 28.0),
    (23.0, 11.0, 10.0),
)
_POINTER_MARKERS_MM = (
    (-34.0, -22.0, -8.0),
    (33.0, -18.0, 2.0),
    (-15.0, 27.0, 16.0),
    (16.0, 13.0, -10.0),
)
_POINTER_TIP_OFFSET_MM = 110.0


@dataclass
class SampleSet:
    """Tip positions collected through the tracker."""
    handles: List[PointerPlacement] = field(default_factory=list)
    patch_ids: List[int] = field(default_factory=list)
    d_ref: List[np.ndarray] = field(default_factory=list)   # (3,) tip in reference-body coordinates
    t_collect: float = 0.0

    def __len__(self) -> int:
        return len(self.d_ref)

    def as_array(self) -> np.ndarray:
        return np.array(self.d_ref).reshape(-1, 3)


def design_scene() -> SceneDesign:
    """Return the tracker pose and both body layouts in millimeters.

    The tracker frame maps tracker coordinates to the workspace. The four
    markers on each body and the tip are expressed in their own body frames.
    """
    # With identity rotation the camera looks along workspace +z at the origin.
    tracker_pose = Frame(Rot(), vct3(0, 0, -_TRACKER_STANDOFF_MM))
    return SceneDesign(
        F_tracker_approx=tracker_pose,
        ref_marker_positions=[vct3(*point) for point in _REFERENCE_MARKERS_MM],
        ptr_marker_positions=[vct3(*point) for point in _POINTER_MARKERS_MM],
        p_tip=vct3(0, 0, -_POINTER_TIP_OFFSET_MM),
    )


def plan_patches(boxes: List[PatchBox], n_patches: int, samples_per_patch: int) -> List[int]:
    """
    TODO: implement.
    input: the patch boxes, how many patches to use, samples per patch
    output: ordered list of patch ids to call place_pointer with (repeats allowed)
    """
    raise NotImplementedError("TODO: implement plan_patches")


def collect_samples(session: Session, plan: List[int]) -> SampleSet:
    """
    TODO: implement.
    For every patch id: session.place_pointer, session.sample_marker_bodies,
    d_k = F_ref^-1 F_ptr p_tip; skip (and log) samples where a body was invisible.
    input: session, patch plan
    output: SampleSet
    """
    raise NotImplementedError("TODO: implement collect_samples")


def run_registration(session: Session, mesh: Mesh, boxes: List[PatchBox], **kwargs) -> RegistrationReport:
    """
    TODO: implement -- the headless entry point used for grading:
    plan -> collect -> coarse_registration -> icp -> uncertainty -> RegistrationReport.
    input: session, mesh, boxes, your own keyword options
    output: RegistrationReport
    """
    raise NotImplementedError("TODO: implement run_registration")
