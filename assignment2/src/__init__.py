"""
Public API for the CIS1 assignment 2 package. The tracker / marker-body
classes and helpers are the assignment 1 ones (assignments/assignment1/src);
only the robot and the assignment 2 simulation live here.
"""

from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from assignments.assignment1.src.tracker import Tracker
from assignments.assignment1.src.marker import MarkerBody
from assignments.assignment1.src.utils import sample_normal

from .robot import Robot, RobotState
from . import simulation

from data_types.nominal_types import vct3, Rot, Frame
from data_types.uncertain_types import uvct3, uRot, uFrame
from data_types.covariance_types import Covariance

__all__ = [
    'Tracker', 'MarkerBody', 'Robot', 'RobotState', 'sample_normal', 'simulation',
    'vct3', 'Rot', 'Frame', 'uvct3', 'uRot', 'uFrame', 'Covariance',
]
