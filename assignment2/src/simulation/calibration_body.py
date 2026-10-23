"""
Simulation for the one special, exactly-machined marker body mBee that can
be attached to the robot's end effector.
"""

from typing import List

from data_types.nominal_types import vct3, Frame
from data_types.covariance_types import Covariance
from data_types.uncertain_types import uvct3

from assignments.assignment1.src.marker import MarkerBody
from ..robot import Robot
from .marker_body import PlacedBody, place_marker_body
from . import constants


def create_robot_calibration_body(rob: Robot, positions: List[vct3], name: str = "mBee") -> PlacedBody:
    """mBee = CreateRobotCalibrationBody({a*_1, ..., a*_N})

    The given positions are the ACTUAL marker positions (no manufacturing
    error) and the body's coordinate system is exactly F_ee. Can only be
    called once per robot, and every ||a*_k|| must be <= 75mm. The body
    starts attached; its placement follows the robot from then on and the
    definition is saved as an internal variable of the robot.
    """
    if rob.calibration_body is not None:
        raise RuntimeError(f"robot {rob.name!r} already has a calibration body; it cannot be redefined")
    if len(positions) < 3:
        raise ValueError("create_robot_calibration_body: need at least 3 markers")
    for a in positions:
        if a.norm() > constants.CAL_BODY_MAX_RADIUS_MM:
            raise ValueError(f"create_robot_calibration_body: ||a*_k|| = {a.norm():.1f}mm exceeds "
                             f"{constants.CAL_BODY_MAX_RADIUS_MM}mm")

    body = MarkerBody(name, list(positions), world_frame=Frame.eye())
    body.set_manufacturing_cov(Covariance.zeros(3))
    body.actual_marker_positions = [uvct3(vct3(a)) for a in positions]  # a*_k == a_k exactly

    placed = place_marker_body(body, rob.F_rob * rob.commanded_pose(), precision="exact")
    rob.calibration_body = placed
    attach_calibration_marker(rob)
    return placed


def attach_calibration_marker(rob: Robot):
    """AttachCalibrationMarker(Rob)"""
    if rob.calibration_body is None:
        raise RuntimeError("no calibration body has been created for this robot")
    rob.calibration_body_attached = True
    rob.calibration_body.visible = True
    if hasattr(rob, "_sync_calibration_body"):
        rob._sync_calibration_body()


def detach_calibration_marker(rob: Robot):
    """DetachCalibrationMarker(Rob): the body is no longer on the robot, so
    the tracker stops seeing it until it is attached again."""
    if rob.calibration_body is None:
        raise RuntimeError("no calibration body has been created for this robot")
    rob.calibration_body_attached = False
    rob.calibration_body.visible = False
