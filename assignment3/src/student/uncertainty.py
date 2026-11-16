"""Statistical estimate of the registration error and the predicted TRE."""

from typing import Tuple
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

from data_types.nominal_types import Frame
from ..mesh import Mesh
from .icp import IcpResult


_MAX_SCALED_CONDITION = 1e8
_COVARIANCE_REL_TOL = 1e-10


def _skew(points: np.ndarray) -> np.ndarray:
    """Return (...,3,3) cross-product matrices for (...,3) vectors."""
    points = np.asarray(points, dtype=float)
    x, y, z = np.moveaxis(points, -1, 0)
    zero = np.zeros_like(x)
    return np.stack([zero, -z, y, z, zero, -x, -y, x, zero], axis=-1).reshape(
        points.shape[:-1] + (3, 3))


def _check_frame(frame: Frame) -> None:
    """Reject a nonfinite or nonrigid Frame; units are radians and mm."""
    r = frame.R.matrix
    if (not np.isfinite(r).all() or not np.isfinite(frame.p.vec).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(r), 1.0, atol=1e-8, rtol=0)):
        raise ValueError("expected a finite proper rigid frame")


def _frame_log(frame: Frame) -> np.ndarray:
    """Return the SE(3) log [rotation vector (rad), translation tangent (mm)]."""
    _check_frame(frame)
    alpha = Rotation.from_matrix(frame.R.matrix).as_rotvec()
    theta = np.linalg.norm(alpha)
    a = _skew(alpha)
    if theta < 1e-4:
        b = 0.5 - theta ** 2 / 24 + theta ** 4 / 720
        c = 1 / 6 - theta ** 2 / 120 + theta ** 4 / 5040
    else:
        b = (1 - np.cos(theta)) / theta ** 2
        c = (theta - np.sin(theta)) / theta ** 3
    left_jacobian = np.eye(3) + b * a + c * (a @ a)
    return np.r_[alpha, np.linalg.solve(left_jacobian, frame.p.vec.ravel())]


def _check_covariance(covariance: np.ndarray) -> np.ndarray:
    """Validate a finite, symmetric PSD (6,6) covariance and symmetrize roundoff."""
    covariance = np.asarray(covariance, dtype=float)
    if covariance.shape != (6, 6) or not np.isfinite(covariance).all():
        raise ValueError("C_reg must be a finite (6, 6) matrix")
    scale = np.max(np.abs(covariance))
    if not np.allclose(covariance, covariance.T, atol=_COVARIANCE_REL_TOL * scale, rtol=0):
        raise ValueError("C_reg must be symmetric")
    covariance = (covariance + covariance.T) / 2
    eigenvalues = np.linalg.eigvalsh(covariance)
    if eigenvalues[0] < -_COVARIANCE_REL_TOL * np.max(np.abs(eigenvalues)):
        raise ValueError("C_reg must be positive semidefinite")
    return covariance


def registration_covariance(result: IcpResult, F0_reg: Frame) -> Tuple[np.ndarray, np.ndarray]:
    """Return correction mu (6,) and local left-CT pose covariance C_reg (6,6).

    mu=Log(F_reg*F0_reg.inv()) is an estimated correction, not true residual
    error. Covariance uses inlier signed normal residuals and a local surface
    Jacobian, assuming independent equal-variance errors and fixed local matches.
    Our ICP supplies private transformed CT points. Legacy results lacking them
    use closest points and Euclidean residual norms as a warned approximation.
    Require >6 inliers and six observable directions; do not hide null spaces
    with a pseudoinverse. Diagnostic metadata is private companion data.
    """
    _check_frame(F0_reg)
    _check_frame(result.F_reg)
    closest = np.asarray(result.closest, dtype=float)
    normals = np.asarray(result.normals, dtype=float)
    residuals = np.asarray(result.residuals, dtype=float)
    mask = np.asarray(result.inlier_mask)
    if (closest.ndim != 2 or closest.shape[1] != 3 or normals.shape != closest.shape
            or residuals.shape != (len(closest),) or mask.shape != residuals.shape
            or mask.dtype != np.dtype(bool) or not np.isfinite(closest).all()
            or not np.isfinite(normals).all() or not np.isfinite(residuals).all()
            or np.any(residuals < 0)):
        raise ValueError("inconsistent or nonfinite ICP result arrays")
    count = int(mask.sum())
    if count <= 6:
        raise ValueError("covariance requires more than six inliers")
    q, n = closest[mask], normals[mask]
    lengths = np.linalg.norm(n, axis=1)
    if not np.allclose(lengths, 1.0, atol=1e-6, rtol=0):
        raise ValueError("inlier surface normals must be unit length")
    n = n / lengths[:, None]
    points = getattr(result, "_transformed_points_ct", None)
    approximate = points is None
    if approximate:
        warnings.warn("ICP result lacks transformed samples; using closest-point Jacobians "
                      "and Euclidean-norm noise proxy", RuntimeWarning, stacklevel=2)
        x = q
        squared_error = float(residuals[mask] @ residuals[mask])
    else:
        points = np.asarray(points, dtype=float)
        if points.shape != closest.shape or not np.isfinite(points).all():
            raise ValueError("invalid private transformed CT points")
        x = points[mask]
        signed = np.einsum("ij,ij->i", n, x - q)
        squared_error = float(signed @ signed)

    # Center and scale both parameter blocks into mm before assessing geometry.
    center = x.mean(axis=0)
    lever = float(np.sqrt(np.mean(np.sum((x - center) ** 2, axis=1))))
    if lever <= np.finfo(float).eps:
        raise ValueError("surface geometry does not constrain six pose directions")
    jacobian = np.column_stack([np.cross((x - center) / lever, n), n])
    _, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
    rank_tol = np.finfo(float).eps * max(jacobian.shape) * singular[0]
    if singular[-1] <= rank_tol:
        raise ValueError("rank-deficient surface geometry: fewer than six pose directions")
    condition = float(singular[0] / singular[-1])
    if condition > _MAX_SCALED_CONDITION:
        raise ValueError("surface pose covariance is numerically ill-conditioned")
    sigma_squared = squared_error / (count - 6)
    factor = vt.T / singular
    centered_covariance = sigma_squared * (factor @ factor.T)
    to_ct = np.eye(6)
    to_ct[:3, :3] /= lever
    to_ct[3:, :3] = _skew(center) / lever
    covariance = to_ct @ centered_covariance @ to_ct.T
    covariance = _check_covariance((covariance + covariance.T) / 2)
    result._covariance_diagnostics = {
        "model": "local_point_to_plane", "approximate_input": approximate,
        "point_source": "closest_point_proxy" if approximate else "transformed_samples_ct",
        "noise_scale": "euclidean_norm_proxy" if approximate else "signed_normal_residual",
        "inliers": count, "degrees_of_freedom": count - 6, "rank": 6,
        "scaled_condition": condition, "sigma_squared_mm2": sigma_squared,
        "icp_converged": bool(result.converged),
    }
    mu = _frame_log(result.F_reg * F0_reg.inv())
    return mu, covariance


def predicted_tre(mesh: Mesh, F_reg: Frame, C_reg: np.ndarray) -> dict:
    """Propagate left-CT C_reg (6,6) to mesh CT vertices, returning mm statistics.

    Each vertex has predicted RMS rho=sqrt(trace(G*C_reg*G.T)), G=[-sk(c),I].
    rms=sqrt(mean(rho**2)); mean/max/p90 summarize these per-vertex RMS values,
    not the realized random TRE magnitude distribution. The correction mu is
    not a residual bias. CT vertices are already in CT and are not transformed
    by F_reg again; F_reg is checked for API consistency.
    """
    _check_frame(F_reg)
    covariance = _check_covariance(C_reg)
    vertices = np.asarray(mesh.vertices, dtype=float)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0
            or not np.isfinite(vertices).all()):
        raise ValueError("mesh must contain finite CT vertices")
    jacobian = np.concatenate([-_skew(vertices), np.broadcast_to(np.eye(3),
                              (len(vertices), 3, 3))], axis=2)
    variance = np.einsum("nij,jk,nik->n", jacobian, covariance, jacobian)
    rho = np.sqrt(np.maximum(variance, 0.0))
    return {"rms": float(np.sqrt(np.mean(rho ** 2))), "mean": float(rho.mean()),
            "max": float(rho.max()), "p90": float(np.percentile(rho, 90))}
