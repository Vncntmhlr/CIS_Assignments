"""
Non-interactive simulation for assignment 2. Students call these functions
directly; ground-truth kinematic errors and tracker noise are injected here
rather than being something they need to (or can) set themselves -- see
constants.py for every parameter and where its value comes from. Function
names are snake_case versions of the handout's (CreateRobot -> create_robot).
"""

from .clock import clock, SimClock
from .marker_body import (PlacedBody, define_marker_body, place_marker_body, update_marker_body,
                          most_recent_observed_frame, most_recent_observed_markers, actual_frame_of_body)
from .tracker import (ObservedBody, create_tracker, add_marker_bodies, remove_marker_bodies,
                      sample_marker_bodies)
from .robot import RobotSim, create_robot
from .calibration_body import (create_robot_calibration_body, attach_calibration_marker,
                               detach_calibration_marker)
from . import constants

__all__ = [
    "clock", "SimClock",
    "PlacedBody", "define_marker_body", "place_marker_body", "update_marker_body",
    "most_recent_observed_frame", "most_recent_observed_markers", "actual_frame_of_body",
    "ObservedBody", "create_tracker", "add_marker_bodies", "remove_marker_bodies", "sample_marker_bodies",
    "RobotSim", "create_robot",
    "create_robot_calibration_body", "attach_calibration_marker", "detach_calibration_marker",
    "constants",
]
