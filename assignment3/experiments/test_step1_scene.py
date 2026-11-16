"""Offline Step 1 checks using the TA tracker and synthetic workspace poses."""

from itertools import combinations
from pathlib import Path

import numpy as np

from src import Frame, Rot, SceneDesign, load_patch_boxes, load_sur, vct3
from src.session import design_to_json
from src.student.workflow import design_scene
from assignments.assignment1.src.tracker import Tracker


DATA = Path(__file__).resolve().parents[1] / "data"


def test_design_scene_headless_contract():
    """Check the scene types, camera frame, and rigid-body marker geometry."""
    design = design_scene()
    assert isinstance(design, SceneDesign)
    assert isinstance(design.F_tracker_approx, Frame)
    assert isinstance(design.p_tip, vct3)

    camera = design.F_tracker_approx
    np.testing.assert_allclose(camera.R.matrix.T @ camera.R.matrix, np.eye(3))
    np.testing.assert_allclose(np.linalg.det(camera.R.matrix), 1.0)
    origin_in_camera = camera.inv() * vct3(0, 0, 0)
    np.testing.assert_allclose(origin_in_camera.vec.ravel(), [0, 0, 1600])
    np.testing.assert_allclose((camera * origin_in_camera).vec, 0)

    for points in (design.ref_marker_positions, design.ptr_marker_positions):
        assert len(points) == 4 and all(isinstance(p, vct3) for p in points)
        coordinates = np.array([point.vec.ravel() for point in points])
        assert np.isfinite(coordinates).all()
        assert np.linalg.matrix_rank(coordinates - coordinates.mean(axis=0)) == 3
        distances = [np.linalg.norm(a - b) for a, b in combinations(coordinates, 2)]
        assert len(np.unique(np.round(distances, 6))) == len(distances)

    assert all(point.z > design.p_tip.z for point in design.ptr_marker_positions)
    assert design_to_json(design_scene()) == design_to_json(design)


def test_scene_frustum_covers_all_patches_in_synthetic_poses():
    """Apply the official frustum to real patch boxes in three mock scenes."""
    design = design_scene()
    tracker = Tracker("offline Step 1", design.F_tracker_approx)
    mesh = load_sur(DATA / "ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(DATA / "box-centroids-rotations-world.json")
    assert len(boxes) == 46

    # Synthetic reference mount 15 mm outward from the +x extreme of the mesh.
    mount_ct = mesh.vertices[np.argmax(mesh.vertices[:, 0])] + [15, 0, 0]
    ct_from_ref = Frame(Rot("y", np.pi / 2), vct3(mount_ct))
    cases = (
        (Frame(Rot(), vct3(0, 0, 0)), 0.0),
        (Frame(Rot("y", 0.3), vct3(50, -50, 50)), 0.1),
        (Frame(Rot("x", -0.3), vct3(-50, 50, -50)), -0.1),
    )
    camera_from_world = design.F_tracker_approx.inv()
    camera_points = []
    for world_from_ct, tilt in cases:
        world_from_ref = world_from_ct * ct_from_ref
        ref_points = [world_from_ref * point for point in design.ref_marker_positions]
        assert tracker.is_visible(ref_points)
        camera_points.extend(camera_from_world * point for point in ref_points)

        for box in boxes:
            # Keep the tip at the patch centre while rotating the pointer.
            world_from_box = world_from_ct * box.F_box
            pointer_rotation = world_from_box.R * Rot("x", tilt)
            pointer_origin = world_from_box.p - pointer_rotation * design.p_tip
            world_from_ptr = Frame(pointer_rotation, pointer_origin)
            np.testing.assert_allclose(
                (world_from_ptr * design.p_tip).vec, world_from_box.p.vec, atol=1e-10
            )
            ptr_points = [world_from_ptr * point for point in design.ptr_marker_positions]
            assert tracker.is_visible(ptr_points), f"patch {box.id}, tilt {tilt} rad"
            camera_points.extend(camera_from_world * point for point in ptr_points)

    coordinates = np.array([point.vec.ravel() for point in camera_points])
    depth = coordinates[:, 2]
    near, far = tracker.FRUSTUM_Z_NEAR, tracker.FRUSTUM_Z_FAR
    half_width = np.interp(
        depth, [near, far], [tracker.FRUSTUM_HALF_WIDTH_NEAR, tracker.FRUSTUM_HALF_WIDTH_FAR]
    )
    side_clearance = (half_width[:, None] - np.abs(coordinates[:, :2])).min()
    depth_clearance = min(depth.min() - near, far - depth.max())
    print(
        f"Synthetic check: {len(cases) * len(boxes)} pointer placements, "
        f"{len(cases)} reference poses, depth {depth.min():.1f}–{depth.max():.1f} mm, "
        f"clearance {depth_clearance:.1f} mm in depth and {side_clearance:.1f} mm laterally"
    )
