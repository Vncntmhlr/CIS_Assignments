"""Rigid registration: point-set registration, coarse initialisation and ICP."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from data_types.nominal_types import vct3, Frame, Rot
from ..mesh import Mesh, PatchBox
from .closest_point import closest_points_on_mesh


_MAD_NORMAL_SCALE = 1.4826
_MAD_CUTOFF = 3.0
# Keep the near-zero-residual gate from collapsing below 0.03 mm.
_MAD_SCALE_FLOOR_MM = 0.01
_MOTION_TOL_FACTOR = 10.0


@dataclass
class IcpResult:
    """Output of icp()."""
    F_reg: Frame                  # reference body -> CT
    residuals: np.ndarray         # (K,) ||F_reg d_k - closest_k|| for every sample
    inlier_mask: np.ndarray       # (K,) bool, False for rejected samples
    closest: np.ndarray           # (K,3) closest mesh points of the last iteration
    normals: np.ndarray           # (K,3) mesh normals at those points
    n_iter: int
    converged: bool
    history: List[float] = field(default_factory=list)   # rms inlier residual per iteration


def _points_array(points: np.ndarray, name: str) -> np.ndarray:
    """Validate finite (N,3) positions in mm and return a float array."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(f"{name} must be a finite (N, 3) array")
    return points


def _rigid_fit(source: np.ndarray, target: np.ndarray) -> Frame:
    """Fit source-to-target Frame by Arun/SVD for paired (N,3) arrays, N>=3."""
    if source.shape != target.shape or len(source) < 3:
        raise ValueError("rigid registration needs at least three paired points")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    if np.linalg.matrix_rank(source_zero) < 2 or np.linalg.matrix_rank(target_zero) < 2:
        raise ValueError("rigid registration needs noncollinear points")
    u, singular_values, vt = np.linalg.svd(source_zero.T @ target_zero)
    if singular_values[1] <= np.finfo(float).eps * max(source.shape) * singular_values[0]:
        raise ValueError("paired geometry does not determine a stable rigid rotation")
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(vt.T @ u.T) >= 0 else -1.0
    rotation = vt.T @ correction @ u.T
    translation = target_center - rotation @ source_center
    return Frame(Rot(matrix=rotation), vct3(translation))


def _inliers(residuals: np.ndarray, reject: Optional[str]) -> np.ndarray:
    """Return a fresh Euclidean-residual MAD mask, or all True for reject=None."""
    if reject is None:
        return np.ones(len(residuals), dtype=bool)
    median = np.median(residuals)
    mad = np.median(np.abs(residuals - median))
    scale = max(_MAD_NORMAL_SCALE * mad, _MAD_SCALE_FLOOR_MM)
    return residuals <= median + _MAD_CUTOFF * scale


def register_points(a: List[vct3], b: List[vct3]) -> Frame:
    """Return the least-squares rigid Frame mapping paired vct3 a to b (mm).

    Require equal lengths, at least three noncollinear pairs, and finite
    coordinates. Arun/SVD includes determinant correction to exclude reflection.
    """
    source = _points_array(np.array([p.vec.ravel() for p in a]).reshape(-1, 3), "a")
    target = _points_array(np.array([p.vec.ravel() for p in b]).reshape(-1, 3), "b")
    return _rigid_fit(source, target)


def coarse_registration(d_by_patch: Dict[int, np.ndarray], boxes: List[PatchBox]) -> Frame:
    """Return a rough Reference-to-CT Frame from patch medians and box centers.

    d_by_patch maps patch IDs to finite (n_i,3) reference-frame positions in mm.
    Empty groups are skipped; unknown IDs or insufficient geometry raise
    ValueError. Componentwise medians approximate patch representatives; the
    CT box centers are approximate correspondences, not known contact points.
    """
    boxes_by_id = {box.id: box for box in boxes}
    source, target = [], []
    for patch_id, points in d_by_patch.items():
        if patch_id not in boxes_by_id:
            raise ValueError(f"unknown patch ID {patch_id}")
        points = _points_array(points, f"patch {patch_id}")
        if len(points) == 0:
            continue
        source.append(np.median(points, axis=0))
        target.append(boxes_by_id[patch_id].p)
    return _rigid_fit(np.asarray(source).reshape(-1, 3), np.asarray(target).reshape(-1, 3))


def icp(d: np.ndarray, mesh: Mesh, F_init: Frame, max_iter: int = 100, tol: float = 1e-4,
        reject: Optional[str] = "mad") -> IcpResult:
    """Register finite reference-frame d (K,3), in mm, to the CT mesh.

    Alternate exact surface matching and point-to-point Arun/SVD fits from
    original d to matched CT points. reject='mad' recomputes a residual gate
    each iteration; None uses all samples. Stop when the mask is unchanged,
    inlier RMS changes by <=tol mm, and RMS movement of all transformed samples
    is <=10*tol mm. max_iter must be positive. Return final matches, Euclidean
    residuals for all samples, mesh normals, mask, and post-update RMS history.
    """
    d = _points_array(d, "d")
    if len(d) < 3:
        raise ValueError("ICP needs at least three samples")
    if reject not in (None, "mad"):
        raise ValueError("reject must be 'mad' or None")
    if not isinstance(max_iter, (int, np.integer)) or max_iter < 1:
        raise ValueError("max_iter must be a positive integer")
    if not np.isfinite(tol) or tol <= 0:
        raise ValueError("tol must be finite and positive (mm)")
    rotation, translation = F_init.R.matrix, F_init.p.vec.ravel()
    if (not np.isfinite(rotation).all() or not np.isfinite(translation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-8, rtol=0)):
        raise ValueError("F_init must be a finite proper rigid frame")

    frame = F_init
    transformed = d @ rotation.T + translation
    closest, triangles, residuals = closest_points_on_mesh(transformed, mesh)
    mask = _inliers(residuals, reject)
    history = []
    converged = False
    for _ in range(max_iter):
        if mask.sum() < 3:
            raise ValueError("fewer than three ICP inliers")
        previous_rms = np.sqrt(np.mean(residuals[mask] ** 2))
        frame = _rigid_fit(d[mask], closest[mask])
        updated = d @ frame.R.matrix.T + frame.p.vec.ravel()
        movement = np.sqrt(np.mean(np.sum((updated - transformed) ** 2, axis=1)))
        closest, triangles, residuals = closest_points_on_mesh(updated, mesh)
        updated_mask = _inliers(residuals, reject)
        if updated_mask.sum() < 3:
            raise ValueError("fewer than three ICP inliers")
        rms = float(np.sqrt(np.mean(residuals[updated_mask] ** 2)))
        history.append(rms)
        stable = (np.array_equal(mask, updated_mask)
                  and abs(previous_rms - rms) <= tol and movement <= _MOTION_TOL_FACTOR * tol)
        transformed, mask = updated, updated_mask
        if stable:
            converged = True
            break

    result = IcpResult(frame, residuals, mask, closest, mesh.face_normals[triangles],
                       len(history), converged, history)
    # Approved private companion data: public dataclass fields remain unchanged.
    result._transformed_points_ct = transformed.copy()
    return result
