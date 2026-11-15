"""Rigid registration: point-set registration, coarse initialisation and ICP."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from data_types.nominal_types import vct3, Frame
from ..mesh import Mesh, PatchBox


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


def register_points(a: List[vct3], b: List[vct3]) -> Frame:
    """
    TODO: implement (your own; do not call the simulator's solver).
    input: paired points a_i, b_i (>= 3)
    output: least-squares Frame F with F * a_i ~= b_i
    """
    raise NotImplementedError("TODO: implement register_points")


def coarse_registration(d_by_patch: Dict[int, np.ndarray], boxes: List[PatchBox]) -> Frame:
    """
    TODO: implement.
    input: {patch id: (n_i,3) tip points in reference coordinates}, the patch boxes
    output: a rough F_reg (reference -> CT) from patch centroids <-> box centres
    """
    raise NotImplementedError("TODO: implement coarse_registration")


def icp(d: np.ndarray, mesh: Mesh, F_init: Frame, max_iter: int = 100, tol: float = 1e-4,
        reject: Optional[str] = "mad") -> IcpResult:
    """
    TODO: implement.
    input: d (K,3) tip points in reference coordinates, mesh, initial F_reg,
           iteration cap, convergence tolerance, outlier-rejection method
    output: IcpResult
    """
    raise NotImplementedError("TODO: implement icp")
