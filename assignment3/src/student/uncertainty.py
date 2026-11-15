"""Statistical estimate of the registration error and the predicted TRE."""

from typing import Tuple

import numpy as np

from data_types.nominal_types import Frame
from ..mesh import Mesh
from .icp import IcpResult


def registration_covariance(result: IcpResult, F0_reg: Frame) -> Tuple[np.ndarray, np.ndarray]:
    """
    TODO: implement.
    Left/CT convention: F*_reg = Exp(delta) . F_reg, eta_reg = [alpha; eps] relative to F0_reg.
    input: IcpResult, the initial guess F0_reg
    output: mu (6,) estimate of eta_reg, C_reg (6,6) covariance of delta
    """
    raise NotImplementedError("TODO: implement registration_covariance")


def predicted_tre(mesh: Mesh, F_reg: Frame, C_reg: np.ndarray) -> dict:
    """
    TODO: implement.
    input: mesh, estimated F_reg, C_reg (6,6)
    output: {"rms", "mean", "max", "p90"} of the predicted TRE over the mesh vertices
    """
    raise NotImplementedError("TODO: implement predicted_tre")
