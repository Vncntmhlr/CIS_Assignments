"""The end-to-end registration workflow: design, patch plan, data collection, registration."""

from dataclasses import dataclass, field
from time import perf_counter
from typing import List

import numpy as np

from data_types.nominal_types import Frame, Rot, vct3

from ..mesh import Mesh, PatchBox
from ..session import Session, SceneDesign, PointerPlacement
from .report import RegistrationReport, residual_statistics
from . import icp, uncertainty


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
_MIN_PATCH_NORMAL_ANGLE_DEG = 50.0


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
    """Return repeated patch IDs using Ahmed's consecutive-normal selection.

    Inspect the first n_patches boxes in input order. Accept a box only when
    its unit CT normal is more than 50 degrees from both directions of the
    previous accepted normal. Repeat each accepted ID samples_per_patch times.
    Thus n_patches is a candidate limit; fewer patches may be returned.
    """
    # Adapted from Ahmed783-pixel's workflow.py, commit ca10a27f8d71cfd64076b0b7610546c614517d98.
    if not 0 <= n_patches <= len(boxes):
        raise ValueError("n_patches must be between 0 and len(boxes)")
    if samples_per_patch < 1:
        raise ValueError("samples_per_patch must be at least 1")

    plan = []
    previous_normal = None
    parallel_threshold = np.cos(np.deg2rad(_MIN_PATCH_NORMAL_ANGLE_DEG))
    for box in boxes[:n_patches]:
        normal = box.normal
        if previous_normal is not None:
            if abs(np.dot(normal, previous_normal)) >= parallel_threshold:
                continue
        for _ in range(samples_per_patch):
            plan.append(box.id)
        previous_normal = normal
    return plan


def collect_samples(session: Session, plan: List[int]) -> SampleSet:
    """Collect one tracker reading after each fresh placement in plan.

    Use only Session methods and return a SampleSet of valid reference-frame
    tip positions (mm), original placement handles, and patch IDs. If either
    body is invisible, print the skipped attempt's index/patch and continue.
    t_collect is the simulated clock difference, including skipped attempts.
    """
    samples, _ = _collect_samples_with_log(session, plan)
    return samples


def _collect_samples_with_log(session: Session, plan: List[int]):
    """Collect the same SampleSet plus one report record per placement attempt."""
    # Same frame chain and collection policy as Ahmed's implementation.
    samples = SampleSet()
    attempts = []
    start_time = session.elapsed()
    for patch_id in plan:
        handle = session.place_pointer(patch_id)
        observations = session.sample_marker_bodies()
        ref = observations["ref"]
        ptr = observations["ptr"]
        record = {"sample_index": handle.sample_index, "patch_id": patch_id,
                  "t_sim": handle.t_sim, "d_ref": None, "residual_mm": None,
                  "used": False, "reason": "pending_icp"}
        attempts.append(record)
        if ref is None or ptr is None:
            record["reason"] = ("invisible_ref_and_ptr" if ref is None and ptr is None else
                                "invisible_ref" if ref is None else "invisible_ptr")
            print(f"Skipping sample {handle.sample_index} for patch {patch_id}: "
                  "reference or pointer body is invisible")
            continue

        d_ref = ref.frame.inv() * ptr.frame * session.design.p_tip
        samples.handles.append(handle)
        samples.patch_ids.append(patch_id)
        samples.d_ref.append(d_ref.vec.ravel().copy())
        record["d_ref"] = samples.d_ref[-1].tolist()

    samples.t_collect = session.elapsed() - start_time
    return samples, attempts


def run_registration(session: Session, mesh: Mesh, boxes: List[PatchBox], **kwargs) -> RegistrationReport:
    """Run the headless Reference-to-CT pipeline using documented Session calls.

    Options: n_patches (candidate limit, default len(boxes)), samples_per_patch
    (4), max_iter (100), tol (1e-4 mm), reject ('mad' or None). Return the TA
    RegistrationReport with all placement attempts, distinguishing invisible
    observations from ICP rejection. t_simulated is the session's total clock;
    t_wall covers this function, excluding session creation. No truth oracles.
    """
    wall_start = perf_counter()
    unknown = set(kwargs) - {"n_patches", "samples_per_patch", "max_iter", "tol", "reject"}
    if unknown:
        raise TypeError(f"unknown registration options: {sorted(unknown)}")
    iterations = kwargs.get("max_iter", 100)
    tolerance = kwargs.get("tol", 1e-4)
    rejection = kwargs.get("reject", "mad")
    if (not isinstance(iterations, (int, np.integer)) or iterations < 1
            or not np.isfinite(tolerance) or tolerance <= 0 or rejection not in (None, "mad")):
        raise ValueError("invalid ICP iteration cap, tolerance or rejection option")
    plan = plan_patches(boxes, kwargs.get("n_patches", len(boxes)),
                        kwargs.get("samples_per_patch", 4))
    if not plan:
        raise ValueError("registration requires a nonempty patch plan")
    initial = session.initial_registration_guess()
    samples, attempts = _collect_samples_with_log(session, plan)
    by_patch = {}
    for patch_id, point in zip(samples.patch_ids, samples.d_ref):
        by_patch.setdefault(patch_id, []).append(point)
    coarse = icp.coarse_registration({key: np.array(value) for key, value in by_patch.items()}, boxes)
    result = icp.icp(samples.as_array(), mesh, coarse, max_iter=iterations,
                     tol=tolerance, reject=rejection)
    mu, covariance = uncertainty.registration_covariance(result, initial)
    predicted = uncertainty.predicted_tre(mesh, result.F_reg, covariance)
    valid_indices = {handle.sample_index: i for i, handle in enumerate(samples.handles)}
    for record in attempts:
        index = valid_indices.get(record["sample_index"])
        if index is not None:
            record["residual_mm"] = float(result.residuals[index])
            record["used"] = bool(result.inlier_mask[index])
            record["reason"] = "ok" if record["used"] else "icp_mad"
    patches = []
    for patch_id in dict.fromkeys(plan):
        records = [record for record in attempts if record["patch_id"] == patch_id]
        used = sum(record["used"] for record in records)
        patches.append({"patch_id": patch_id, "n_samples": len(records), "n_used": used,
                        "n_rejected": len(records) - used})
    simulated = session.elapsed()
    return RegistrationReport(
        F_reg=result.F_reg, patches=patches, samples=attempts,
        residual_stats=residual_statistics(result.residuals[result.inlier_mask]),
        t_simulated=simulated, t_wall=perf_counter() - wall_start,
        n_icp_iterations=result.n_iter, converged=result.converged,
        eta_reg={"mu": mu, "C_reg": covariance, "convention": "left_ct",
                 "mu_definition": "Log(F_reg * F0_reg.inv()); correction, not residual bias",
                 "covariance_model": result._covariance_diagnostics,
                 "predicted_tre_definition": "rms over mesh; mean/max/p90 of per-vertex RMS"},
        predicted_tre=predicted)
