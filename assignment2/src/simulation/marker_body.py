"""
Marker-body simulation for assignment 2. DefineMarkerBody / PlaceMarkerBody
and PlacedBody are assignment 1's; this module adds UpdateMarkerBody, the
MostRecentObserved* accessors and a ground-truth helper.
"""

from typing import List, Optional

from data_types.nominal_types import vct3, Frame

from assignments.assignment1.src.marker import MarkerBody
from assignments.assignment1.src.tracker import register_points_arun
from assignments.assignment1.src.simulation.marker_body import (  # noqa: F401 -- re-exported
    PlacedBody, define_marker_body, place_marker_body,
)


def update_marker_body(body: MarkerBody, nominal_positions: List[vct3]) -> List[vct3]:
    """PreviousValues = UpdateMarkerBody(mB, {a_m,1, ..., a_m,N})

    Replaces the user's estimate of the marker positions (e.g. after
    averaging several observations). The hidden actual positions the
    tracker simulates from are not touched.
    """
    if len(nominal_positions) != len(body.nominal_marker_positions):
        raise ValueError("update_marker_body: number of markers cannot change")
    previous = list(body.nominal_marker_positions)
    body.set_marker_positions(list(nominal_positions))
    return previous


def actual_frame_of_body(placed: PlacedBody) -> Frame:
    """GROUND TRUTH, for validation/reporting only.

    The actual pose, in the workspace, of the coordinate system DEFINED BY
    the body's current nominal marker layout: the frame F that best maps
    body.nominal_marker_positions onto the markers' true workspace
    positions. After a perfect refinement this is the frame every tracker
    observation of the body is relative to; it differs from
    placed.actual_placement by the small rigid transform between the
    user's layout and the true one (zero for the robot calibration body).
    """
    world_pts = [placed.actual_placement * a.p for a in placed.body.get_actual_marker_positions()]
    return register_points_arun(placed.body.nominal_marker_positions, world_pts)


def most_recent_observed_frame(body: MarkerBody) -> Optional[Frame]:
    """F_m = MostRecentObservedFrame(mB): the body's pose relative to the
    tracker from the last SampleMarkerBodies call, or None if it was not
    visible then."""
    return body.observed_frame


def most_recent_observed_markers(body: MarkerBody) -> Optional[List[vct3]]:
    """{b_m,1, ..., b_m,N} = MostRecentObservedMarkers(mB): the individual
    marker readings (tracker frame) behind that pose, or None."""
    return body.observed_marker_positions
