"""
Public API for the CIS1 assignment 3 package (student side). The simulation
runs on the course server; src.session is the client.
"""

from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from .mesh import Mesh, PatchBox, load_sur, load_patch_boxes
from .session import Session, RemoteSession, Observation, PointerPlacement, SceneDesign, TreSummary

from data_types.nominal_types import vct3, Rot, Frame
from data_types.uncertain_types import uvct3, uRot, uFrame
from data_types.covariance_types import Covariance

__all__ = [
    'Mesh', 'PatchBox', 'load_sur', 'load_patch_boxes',
    'Session', 'RemoteSession', 'Observation', 'PointerPlacement', 'SceneDesign', 'TreSummary',
    'vct3', 'Rot', 'Frame', 'uvct3', 'uRot', 'uFrame', 'Covariance',
]
