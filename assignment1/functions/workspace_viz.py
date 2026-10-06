"""Workspace design visualization for CIS I Assignment 1.

Plot the approximate optical-tracker viewing frustum, Step-8 test cube,
calibrator markers, and pointer-marker positions during a few example
calibration orientations. Report nominal marker-visibility margins.

Coordinates are millimeters. By convention the tracker looks along its LOCAL
+X axis; local +Y and +Z are transverse axes. The frustum dimensions are an
approximation of Figure 2 in the assignment handout, not an optical/occlusion
simulation. Nothing in this helper reads hidden simulator ground truth.

Example (from assignment1/notebooks/assignment1_template.ipynb):

    from functions.workspace_viz import plot_workspace
    fig, ax, report = plot_workspace(
        F_tracker_approx, F_cal_approx,
        ptr_local_positions, ptr_tip_approx, cal_local_positions
    )

The TA notebook already adds assignment1/ to sys.path in its first code cell.
"""

from __future__ import annotations

from itertools import product
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Figure 2, approximated as a truncated pyramid:
# 100 cm from tracker to near plane; another 120 cm to far plane;
# near transverse dimensions 100 x 100 cm; far 200 x 200 cm.
NEAR_MM = 1000.0
FAR_MM = 2200.0
HALF_NEAR_MM = 500.0
HALF_FAR_MM = 1000.0


def _vector(value, name="vector") -> np.ndarray:
    """Read a course vct3 or array-like value as a flat (3,) array."""
    arr = getattr(value, "vec", value)
    arr = np.asarray(arr, dtype=float).reshape(-1)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have three elements; got shape {arr.shape}")
    return arr


def _pose(frame, name="Frame") -> tuple[np.ndarray, np.ndarray]:
    """Extract world-from-local R and translation from a course Frame."""
    try:
        R = np.asarray(frame.R.matrix, dtype=float)
        t = _vector(frame.p, f"{name}.p")
    except AttributeError as exc:
        raise TypeError(f"{name} must be a course Frame with .R.matrix and .p") from exc
    if R.shape != (3, 3):
        raise ValueError(f"{name}.R.matrix must be 3x3")
    return R, t


def _world(R, t, points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ R.T + t


def _tracker_coordinates(world_points, tracker_R, tracker_t):
    points = np.asarray(world_points, dtype=float).reshape(-1, 3)
    return (points - tracker_t) @ tracker_R


def _frustum_corners_local():
    corners = []
    for x, half in ((NEAR_MM, HALF_NEAR_MM), (FAR_MM, HALF_FAR_MM)):
        corners.extend([[x, -half, -half], [x, half, -half],
                        [x, half, half], [x, -half, half]])
    return np.array(corners, dtype=float)


def _margins_in_tracker_frame(points):
    """Signed perpendicular distances (mm) to each of six frustum planes.

    Rows are points. Each column is a boundary; >=0 means inside. The
    slanted boundaries use a unit-length plane normal, so a radius can
    be subtracted directly to get a conservative ball-clearance check.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    slope = (HALF_FAR_MM - HALF_NEAR_MM) / (FAR_MM - NEAR_MM)
    intercept = HALF_NEAR_MM - slope * NEAR_MM
    denom = np.sqrt(1.0 + slope**2)
    half = slope * x + intercept
    return np.column_stack((x - NEAR_MM, FAR_MM - x,
                            (half - y) / denom, (half + y) / denom,
                            (half - z) / denom, (half + z) / denom))


def _corners_of_box(box_min, box_max):
    return np.asarray(list(product(*zip(box_min, box_max))), dtype=float)


def _rot_matrix(value):
    # Course Rot or explicit 3x3 NumPy rotation matrix.
    R = np.asarray(getattr(value, "matrix", value), dtype=float)
    if R.shape != (3, 3):
        raise ValueError("Calibration rotations must be Rot objects or 3x3 matrices")
    return R


def _draw_marker_assembly(ax, markers, tip=None, *, color="tab:purple",
                          alpha=1.0, linewidth=2.0, marker_size=58,
                          label=None, spheres=False, sphere_radius=5.0):
    """Sketch a conceptual rigid framework linking the marker positions.

    Each marker is connected to a common hub (their geometric centroid).
    A shaft joins that hub to the tip, when provided. These rods are
    illustrative, not a mechanical or collision model.
    """
    markers = np.asarray(markers, dtype=float).reshape(-1, 3)
    hub = markers.mean(axis=0)

    for point in markers:
        rod = np.stack((hub, point))
        ax.plot(*rod.T, color=color, linewidth=linewidth, alpha=alpha,
                solid_capstyle="round")
    ax.scatter(*hub, s=12, color=color, alpha=alpha, depthshade=True)

    if tip is not None:
        tip = np.asarray(tip, dtype=float).reshape(3)
        shaft = np.stack((hub, tip))
        ax.plot(*shaft.T, color="0.38", linewidth=linewidth * 1.3,
                alpha=alpha, solid_capstyle="round")
        ax.scatter(*tip, marker="*", s=marker_size * 1.45,
                   color="tab:orange", alpha=alpha, depthshade=True)

    # Scatter points remain readable even when the entire tracker volume is
    # several meters across. In the detail view add *actual 3D sphere surfaces*.
    ax.scatter(*markers.T, s=marker_size, color=color, alpha=alpha,
               edgecolors="white" if alpha == 1 else "none", linewidths=0.35,
               depthshade=True, label=label)

    if spheres:
        u = np.linspace(0, 2 * np.pi, 18)
        v = np.linspace(0, np.pi, 12)
        ox = sphere_radius * np.outer(np.cos(u), np.sin(v))
        oy = sphere_radius * np.outer(np.sin(u), np.sin(v))
        oz = sphere_radius * np.outer(np.ones_like(u), np.cos(v))
        for cx, cy, cz in markers:
            ax.plot_surface(cx + ox, cy + oy, cz + oz,
                            color=color, shade=True, alpha=0.95,
                            linewidth=0, antialiased=True)
    return hub


def plot_workspace(
    F_tracker_approx,
    F_cal_approx,
    ptr_local_positions: Sequence,
    ptr_tip_approx,
    cal_local_positions: Sequence,
    calibration_rotations: Sequence | None = None,
    test_box_min=(100.0, 100.0, 100.0),
    test_box_max=(500.0, 500.0, 500.0),
    save_path: str | Path | None = None,
    show: bool = True,
    show_other_poses: bool = False,
    show_detail: bool = True,
    sphere_radius_mm: float = 5.0,
):
    """Draw nominal workspace geometry and evaluate its visibility margins.

    Parameters
    ----------
    F_tracker_approx, F_cal_approx : src.Frame
        Approximate world-frame poses of tracker and calibration object.
    ptr_local_positions : sequence of src.vct3
        Pointer markers in the pointer local frame (mm).
    ptr_tip_approx : src.vct3
        Tip offset in the pointer local frame (mm).
    cal_local_positions : sequence of src.vct3
        Calibration-body markers in calibration-object local frame (mm).
    calibration_rotations : sequence of src.Rot or (3,3) arrays, optional
        Desired pointer orientations relative to the calibrator. Defaults to
        0, +/-30 degrees around pointer X and Y (illustrative only).
    test_box_min, test_box_max : length-3 array-like
        Step 8's nominal *pointer-body origins* (not tip coordinates).
    save_path : optional path
        Save a PNG/SVG/etc. if provided.
    show : bool
        Whether to call plt.show().
    show_other_poses : bool
        Whether to draw all the other example calibration orientations as
        faint wireframes. Visibility checks always include all poses.
    show_detail : bool
        Also draw a zoomed-in 3D view of the pointer marker assembly using
        the pointer's own local coordinates.
    sphere_radius_mm : float
        Marker sphere radius in the detail view, in millimeters.

    Returns
    -------
    fig, ax, report
        A Matplotlib figure/3D axes and a dictionary of visibility checks.

    Notes
    -----
    Uses NOMINAL poses and positions only. Does not model tracker jitter,
    marker manufacturing errors, occlusion, or the simulation's lack of a
    hard field-of-view cutoff. 'Guaranteed' means only for the stated
    spherical orientation-independent bound and nominal geometry, not a
    guarantee about real-world tracking performance.
    """
    Rt, tt = _pose(F_tracker_approx, "F_tracker_approx")
    Rc, tc = _pose(F_cal_approx, "F_cal_approx")
    ptr = np.array([_vector(p, "pointer marker") for p in ptr_local_positions])
    cal = np.array([_vector(p, "calibration marker") for p in cal_local_positions])
    tip = _vector(ptr_tip_approx, "ptr_tip_approx")
    if not len(ptr) or not len(cal):
        raise ValueError("The pointer and calibrator each need marker positions")
    if sphere_radius_mm <= 0:
        raise ValueError("sphere_radius_mm must be positive")
    box_min = _vector(test_box_min, "test_box_min")
    box_max = _vector(test_box_max, "test_box_max")
    if np.any(box_min >= box_max):
        raise ValueError("Each coordinate of test_box_max must exceed test_box_min")

    if calibration_rotations is None:
        a = np.pi / 6
        Rx = lambda t: np.array([[1., 0., 0.], [0., np.cos(t), -np.sin(t)],
                                  [0., np.sin(t), np.cos(t)]])
        Ry = lambda t: np.array([[np.cos(t), 0., np.sin(t)], [0., 1., 0.],
                                  [-np.sin(t), 0., np.cos(t)]])
        rot_list = [np.eye(3), Rx(a), Rx(-a), Ry(a), Ry(-a)]
    else:
        rot_list = [_rot_matrix(R) for R in calibration_rotations]
        if not rot_list:
            raise ValueError("calibration_rotations must not be empty")

    # Frustum faces in world coordinates.
    fc = _world(Rt, tt, _frustum_corners_local())
    test_corners = _corners_of_box(box_min, box_max)
    cal_world = _world(Rc, tc, cal)

    # With the pointer tip at the calibrator origin, marker i in the calibrator
    # frame is R_desired @ (a_i - p_tip). Apply calibration pose to world frame.
    ptr_world_list = [_world(Rc, tc, _world(R, np.zeros(3), ptr - tip))
                      for R in rot_list]

    cal_margins = _margins_in_tracker_frame(_tracker_coordinates(cal_world, Rt, tt))
    ptr_margins = _margins_in_tracker_frame(_tracker_coordinates(
        np.concatenate(ptr_world_list), Rt, tt))
    test_margins = _margins_in_tracker_frame(_tracker_coordinates(test_corners, Rt, tt))

    # Conservative check for all possible pointer rotations about its body
    # origin at every test-cube corner. For a convex frustum, the minimum
    # signed plane margin throughout the box is achieved at a corner.
    radius = float(np.max(np.linalg.norm(ptr, axis=1)))
    nominal_test_clearance = float(np.min(test_margins))
    robust_test_clearance = nominal_test_clearance - radius
    report = {
        "calibrator_markers_inside": bool(np.all(cal_margins >= 0)),
        "calibrator_marker_min_margin_mm": float(np.min(cal_margins)),
        "calibration_pointer_markers_inside": bool(np.all(ptr_margins >= 0)),
        "calibration_pointer_min_margin_mm": float(np.min(ptr_margins)),
        "test_body_origins_inside": bool(np.all(test_margins >= 0)),
        "test_body_origin_min_margin_mm": nominal_test_clearance,
        "pointer_marker_max_radius_from_body_origin_mm": radius,
        "test_markers_guaranteed_inside_any_orientation": bool(robust_test_clearance >= 0),
        "test_marker_conservative_min_margin_mm": robust_test_clearance,
    }

    fig = plt.figure(figsize=(15, 7) if show_detail else (11, 8))
    ax = fig.add_subplot(121 if show_detail else 111, projection="3d")
    near, far = fc[:4], fc[4:]
    faces = [near, far] + [[near[i], near[(i+1)%4], far[(i+1)%4], far[i]]
                           for i in range(4)]
    ax.add_collection3d(Poly3DCollection(faces, alpha=0.06, facecolor="tab:blue",
                                         edgecolor="tab:blue", linewidths=0.75))
    ax.scatter(*tt, marker="^", s=120, label="Tracker", color="tab:blue")
    ax.scatter(*tc, marker="*", s=150, label="Calibrator origin", color="tab:orange")
    _draw_marker_assembly(ax, cal_world, color="tab:green", alpha=0.9,
                          marker_size=55, linewidth=1.4, label="Calibrator markers")

    # The first calibration pose is highlighted as an assembled pointer,
    # instead of drawing an indistinguishable cloud of markers from all poses.
    _draw_marker_assembly(ax, ptr_world_list[0], tip=tc,
                          color="tab:purple", linewidth=2.2, marker_size=85,
                          label="Pointer assembly (pose 1)")
    if show_other_poses:
        for points in ptr_world_list[1:]:
            _draw_marker_assembly(ax, points, tip=tc, color="tab:purple",
                                  linewidth=0.85, marker_size=18, alpha=0.16)

    # Draw the axis-aligned workspace test cube (8 corners, 12 edges).
    corners = test_corners
    for i in range(8):
        for j in range(i+1, 8):
            if np.count_nonzero(np.abs(corners[i] - corners[j]) > 1e-8) == 1:
                ax.plot(*np.stack([corners[i], corners[j]]).T,
                        color="tab:red", linewidth=1.6,
                        label="Step 8 test-origin cube" if i == 0 and j == 4 else None)
    ax.scatter(*test_corners.T, color="tab:red", s=10)

    all_pts = np.vstack([fc, tt[None, :], tc[None, :], cal_world, corners] + ptr_world_list)
    mid = (all_pts.min(axis=0) + all_pts.max(axis=0))/2
    rad = max(float(np.max(np.ptp(all_pts, axis=0))) * 0.55, 100.0)
    ax.set_xlim(mid[0]-rad, mid[0]+rad)
    ax.set_ylim(mid[1]-rad, mid[1]+rad)
    ax.set_zlim(mid[2]-rad, mid[2]+rad)
    ax.set_box_aspect((1,1,1))
    ax.view_init(elev=24, azim=-60)
    ax.set_xlabel("World X (mm)")
    ax.set_ylabel("World Y (mm)")
    ax.set_zlabel("World Z (mm)")
    ax.set_title("Workspace: camera, test cube, marker bodies")
    ax.legend(loc="upper left", fontsize=8)

    if show_detail:
        ax_detail = fig.add_subplot(122, projection="3d")
        _draw_marker_assembly(ax_detail, ptr, tip=tip,
                              color="tab:purple", linewidth=3.5,
                              marker_size=115, spheres=True,
                              sphere_radius=sphere_radius_mm)
        for i, point in enumerate(ptr):
            ax_detail.text(*(point + np.array([sphere_radius_mm * 1.6, 0, 0])),
                           f"M{i+1}", fontsize=9)
        ax_detail.text(*(tip + np.array([sphere_radius_mm * 1.6, 0, 0])),
                       "TIP", fontsize=10, fontweight="bold")
        detail_pts = np.vstack([ptr, tip[None, :]])
        center = 0.5 * (detail_pts.min(axis=0) + detail_pts.max(axis=0))
        detail_extent = max(float(np.ptp(detail_pts, axis=0).max()) * 0.65,
                            15.0, sphere_radius_mm * 3.0)
        ax_detail.set_xlim(center[0] - detail_extent, center[0] + detail_extent)
        ax_detail.set_ylim(center[1] - detail_extent, center[1] + detail_extent)
        ax_detail.set_zlim(center[2] - detail_extent, center[2] + detail_extent)
        ax_detail.set_box_aspect((1, 1, 1))
        ax_detail.view_init(elev=19, azim=-58)
        ax_detail.set_xlabel("Pointer X (mm)")
        ax_detail.set_ylabel("Pointer Y (mm)")
        ax_detail.set_zlabel("Pointer Z (mm)")
        ax_detail.set_title("Close-up: pointer skeleton (local frame)")

    fig.tight_layout()

    print("Nominal geometric FOV checks (mm; negative margin = outside):")
    for name, value in report.items():
        print(f"  {name}: {value}")
    print("Note: approximate frustum only; no occlusion or simulation clipping check.")
    if save_path is not None:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
    if show:
        plt.show()
    return fig, ax, report


__all__ = ["plot_workspace"]    