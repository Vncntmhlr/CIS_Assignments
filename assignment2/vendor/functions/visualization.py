"""Plot nominal scene geometry without sampling or using simulation ground truth."""

from itertools import combinations, product

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from data_types.nominal_types import Frame, Rot, vct3


def _transform(frame, points):
    return np.array([(frame * vct3(*point)).vec.ravel() for point in points])


def _draw_box(ax, corners, **kwargs):
    # product((lo, hi), repeat=3) orders corners by three binary coordinates.
    for i, j in combinations(range(8), 2):
        if (i ^ j).bit_count() == 1:
            ax.plot(*corners[[i, j]].T, **kwargs)


def _draw_frame(ax, frame, label, length=100):
    origin = frame.p.vec.ravel()
    for direction, color in zip(frame.R.matrix.T, ("#d64b45", "#389653", "#386fce")):
        ax.quiver(*origin, *(length * direction), color=color, arrow_length_ratio=0.15)
    ax.text(*origin, "  " + label, fontsize=8)


def _equal_scale(ax, points, padding=0.06):
    low, high = points.min(axis=0), points.max(axis=0)
    center = (low + high) / 2
    radius = max(float(np.max(high - low)), 1.0) * (0.5 + padding)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def visualize_workspace_design(rob, trk, local_markers, *, grid_points=3):
    """Return (figure, axes) for the nominal workspace and local marker layout.

    Samples the same correlated wrist angles as the template's visibility cell;
    the plotted travel box is the wrist-center volume, not the entire EE volume.
    All geometry uses commanded/nominal poses. The simulated clock is unchanged.
    """
    if not isinstance(grid_points, int) or grid_points < 2:
        raise ValueError("grid_points must be an integer >= 2")
    markers = np.array([point.vec.ravel() for point in local_markers])
    if markers.ndim != 2 or markers.shape[1] != 3 or len(markers) < 3:
        raise ValueError("Provide at least three 3D marker positions")

    tracker_pose = trk.nominal_pose()
    near, far = trk.FRUSTUM_Z_NEAR, trk.FRUSTUM_Z_FAR
    widths = (trk.FRUSTUM_HALF_WIDTH_NEAR, trk.FRUSTUM_HALF_WIDTH_FAR)
    frustum_local = np.array([
        [x * half, y * half, depth]
        for depth, half in zip((near, far), widths)
        for x, y in product((-1, 1), repeat=2)
    ])
    frustum = _transform(tracker_pose, frustum_local)

    linear_limits = list(zip(rob.theta_min[:3], rob.theta_max[:3]))
    wrist_local = np.array(list(product(*linear_limits))) + rob.F_CW.p.vec.ravel()
    wrist_corners = _transform(rob.F_rob, wrist_local)
    marker_radius = float(np.linalg.norm(markers, axis=1).max())
    reach = rob.F_WE.p.norm() + marker_radius
    # A conservative sphere around every wrist center bounds all wrist rotations.
    envelope_local = np.array(list(product(*zip(
        wrist_local.min(axis=0) - reach, wrist_local.max(axis=0) + reach,
    ))))
    envelope = _transform(rob.F_rob, envelope_local)

    ee_points, visible = [], []
    linear_samples = [np.linspace(lo, hi, grid_points) for lo, hi in linear_limits]
    for xyz in product(*linear_samples):
        for w in (-1.8, 0.0, 1.8):
            theta = np.array([*xyz, w, -w, w])
            if not rob.within_limits(theta):
                continue
            pose = rob.F_rob * rob.fwd_kins(theta)
            ee_points.append(pose.p.vec.ravel())
            visible.append(trk.is_visible([pose * point for point in local_markers]))
    ee_points, visible = np.array(ee_points), np.array(visible, dtype=bool)
    home = rob.F_rob * rob.fwd_kins(np.zeros(6))
    home_markers = _transform(home, markers)

    fig = plt.figure(figsize=(14, 6.5), layout="constrained")
    scene = fig.add_subplot(121, projection="3d")
    body = fig.add_subplot(122, projection="3d")
    faces = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    scene.add_collection3d(Poly3DCollection(
        [frustum[list(face)] for face in faces],
        facecolor="#5d9cbf", edgecolor="#5687a0", alpha=0.07, linewidth=0.8,
    ))
    _draw_box(scene, wrist_corners, color="#168e94", linewidth=1.4)
    _draw_box(scene, envelope, color="#dc9739", linewidth=1, linestyle="--")
    scene.scatter(*ee_points[visible].T, s=12, c="#168e94", alpha=0.55)
    if np.any(~visible):
        scene.scatter(*ee_points[~visible].T, s=22, c="#c73a39", marker="x")
    scene.scatter(*home_markers.T, color="#8454a7", s=28)
    _draw_frame(scene, tracker_pose, "Tracker", length=180)
    _draw_frame(scene, rob.F_rob, "Robot base", length=130)
    _draw_frame(scene, home, "EE home", length=100)
    scene.set_title(f"Nominal workspace: {visible.sum()}/{len(visible)} sampled poses visible")
    scene.set_xlabel("World x [mm]")
    scene.set_ylabel("World y [mm]")
    scene.set_zlabel("World z [mm]")
    _equal_scale(scene, np.vstack([frustum, envelope, tracker_pose.p.vec.ravel(),
                                  rob.F_rob.p.vec.ravel()]))
    scene.view_init(elev=22, azim=-58)
    scene.legend(handles=[
        Line2D([], [], color="#5687a0", label="Tracker frustum"),
        Line2D([], [], color="#168e94", label="Wrist-center travel box"),
        Line2D([], [], color="#dc9739", linestyle="--", label="All-rotation marker bound"),
        Line2D([], [], color="#168e94", marker=".", linestyle="", label="Visible sampled EE origins"),
        Line2D([], [], color="#8454a7", marker="o", linestyle="", label="Markers at EE home"),
    ], loc="upper left", fontsize=7)

    # The radius constraint is fixed by the handout, independent of chosen layout.
    from src.simulation.constants import CAL_BODY_MAX_RADIUS_MM
    limit = CAL_BODY_MAX_RADIUS_MM
    u, v = np.meshgrid(np.linspace(0, 2 * np.pi, 25), np.linspace(0, np.pi, 13))
    body.plot_wireframe(limit * np.cos(u) * np.sin(v), limit * np.sin(u) * np.sin(v),
                        limit * np.cos(v), color="#aeb8c1", alpha=0.3, linewidth=0.5)
    if len(markers) == 4:
        body.add_collection3d(Poly3DCollection(
            [markers[list(face)] for face in combinations(range(4), 3)],
            facecolor="#8454a7", edgecolor="#8454a7", alpha=0.15,
        ))
    body.scatter(*markers.T, color="#704498", s=40)
    for i, point in enumerate(markers):
        body.text(*point, f"  m{i + 1}", fontsize=9)
    _draw_frame(body, Frame(Rot(), vct3()), "EE", length=45)
    body.set_title(f"Marker layout in EE frame: max radius {marker_radius:.2f} / {limit:g} mm")
    body.set_xlabel("EE x [mm]")
    body.set_ylabel("EE y [mm]")
    body.set_zlabel("EE z [mm]")
    _equal_scale(body, np.array([[-limit] * 3, [limit] * 3]))
    body.view_init(elev=22, azim=35)
    return fig, (scene, body)
