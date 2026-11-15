"""The end-to-end registration workflow: design, patch plan, data collection, registration."""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from ..mesh import Mesh, PatchBox
from ..session import Session, SceneDesign, PointerPlacement
from .report import RegistrationReport
from data_types.nominal_types import vct3
from ..session import Frame, Rot
import time
from . import icp, uncertainty
from .report import residual_statistics

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
    """
    TODO: implement -- return the same values the notebook's Step 1 cell uses
    (grading calls this instead of the notebook).
    """
    F_tracker_approx = Frame(Rot(), vct3(0,0,-1600))     # TODO: Frame, camera pose relative to the workspace (looks along its local +z)
    ref_marker_positions = [    # TODO: >= 3 points, reference-body coordinates
      vct3(-60, -40, -20),  
      vct3( 65, -30, -15),  
      vct3(-35,  60, -10),  
      vct3( 30,  10,  45)
    ]
    ptr_marker_positions = [    # TODO: >= 3 points, pointer coordinates
     vct3(-45, -30, -10),  
     vct3( 50, -25, -10),  
     vct3(-30,  45, -10), 
     vct3( 25,  10,  30),
    ]
    p_tip = vct3(0,0,-120)                # TODO: vct3, tip in pointer coordinates

    design = SceneDesign(F_tracker_approx, ref_marker_positions, ptr_marker_positions, p_tip)
    return design


def plan_patches(boxes: List[PatchBox], n_patches: int, samples_per_patch: int) -> List[int]:
    """
    Consider the first n_patches boxes, skipping normals within 50 degrees
    of parallel or antiparallel to the previous accepted patch's normal.
    input: the patch boxes, how many patches to consider, samples per patch
    output: ordered list of patch ids to call place_pointer with (repeats allowed)
    
    plan_patches selects the first n_patches boxes, skipping those whose normals are within 
    50 degrees of parallel or antiparallel to the previous accepted patch's normal
    """
    if not 0 <= n_patches <= len(boxes):
        raise ValueError("n_patches must be between 0 and len(boxes)")
    if samples_per_patch < 1:
        raise ValueError("samples_per_patch must be at least 1")

    plan = []
    previous_normal = None
    parallel_threshold = np.cos(np.deg2rad(50.0))

    for box in boxes[:n_patches]:
        normal = box.normal  # Unit vector: the box rotation's local z axis.
        if previous_normal is not None:
            if abs(np.dot(normal, previous_normal)) >= parallel_threshold:
                continue
        for _ in range(samples_per_patch):
            plan.append(box.id)
        previous_normal = normal

    return plan


def collect_samples(session: Session, plan: List[int]) -> SampleSet:
    """
    For every patch id: session.place_pointer, session.sample_marker_bodies,
    d_k = F_ref^-1 F_ptr p_tip; skip (and log) samples where a body was invisible.
    input: session, patch plan
    output: SampleSet
    """
    samples = SampleSet()
    start_time = session.elapsed()

    for patch_id in plan:
        handle = session.place_pointer(patch_id)
        observations = session.sample_marker_bodies()
        ref = observations["ref"]
        ptr = observations["ptr"]
        if ref is None or ptr is None:
            print(f"Skipping sample {handle.sample_index} for patch {patch_id}: "
                  "reference or pointer body is invisible")
            continue

        d_ref = ref.frame.inv() * ptr.frame * session.design.p_tip
        samples.handles.append(handle)
        samples.patch_ids.append(patch_id)
        samples.d_ref.append(d_ref.vec.ravel().copy())

    samples.t_collect = session.elapsed() - start_time
    return samples


def run_registration(session: Session, mesh: Mesh, boxes: List[PatchBox], **kwargs) -> RegistrationReport:
    """
    TODO: implement -- the headless entry point used for grading:
    plan -> collect -> coarse_registration -> icp -> uncertainty -> RegistrationReport.
    input: session, mesh, boxes, your own keyword options
    output: RegistrationReport
    """
    """wall_start = time.perf_counter()

    # 1. Plan and collect.
    plan = plan_patches(
        boxes,
        n_patches=kwargs.get("n_patches", len(boxes)),
        samples_per_patch=kwargs.get("samples_per_patch", 4),
    )
    samples = collect_samples(session, plan)

    d = samples.as_array()           # Shape (K, 3), reference coordinates
    patch_ids = np.array(samples.patch_ids)

    # 2. Group measurements by patch for coarse registration.
    d_by_patch = {}
    for patch_id in dict.fromkeys(samples.patch_ids):
        d_by_patch[patch_id] = d[patch_ids == patch_id]

    if len(d_by_patch) < 3:
        raise ValueError("Need valid measurements from at least three patches")

    # 3. Estimate reference -> CT alignment, then refine it.
    F_coarse = icp.coarse_registration(d_by_patch, boxes)

    result = icp.icp(
        d,
        mesh,
        F_coarse,
        max_iter=kwargs.get("max_iter", 100),
        tol=kwargs.get("tol", 1e-4),
        reject="mad",
    )

    # 4. Estimate uncertainty relative to the server's initial guess.
    F0_reg = session.initial_registration_guess()
    mu, C_reg = uncertainty.registration_covariance(result, F0_reg)

    tre = uncertainty.predicted_tre(mesh, result.F_reg, C_reg)
    """
    raise NotImplementedError("TODO: implement run_registration")

