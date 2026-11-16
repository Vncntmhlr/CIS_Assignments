"""Offline Step 5 checks; all tracker observations below are synthetic."""

from collections import Counter
from pathlib import Path

import numpy as np
import pytest

from src import Frame, Rot, load_patch_boxes, load_sur, vct3
from src.mesh import PatchBox
from src.session import Observation, PointerPlacement
from src.student.workflow import collect_samples, design_scene, plan_patches
from assignments.assignment1.src.tracker import Tracker


DATA = Path(__file__).resolve().parents[1] / "data"
EXPECTED_PATCHES = [
    0, 1, 3, 5, 7, 8, 10, 11, 12, 13, 14, 15, 16, 18, 21, 22, 23,
    26, 28, 29, 30, 32, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44, 45,
]


@pytest.mark.parametrize(
    "angles, selected",
    [([0, 30, 60], [0, 2]),          # Compare with the last accepted normal.
     ([0, 180, 90], [0, 2]),         # Opposite normals are also redundant.
     ([0, 90, 5], [0, 1, 2])],       # The rule is consecutive, not global.
)
def test_normal_selection_rule(angles, selected):
    boxes = [PatchBox(10 + i, Frame(Rot("y", np.deg2rad(angle)), vct3()))
             for i, angle in enumerate(angles)]
    assert plan_patches(boxes, len(boxes), 2) == [10 + i for i in selected for _ in range(2)]
    assert plan_patches(boxes, 1, 2) == [10, 10]


def test_real_patch_plan():
    boxes = load_patch_boxes(DATA / "box-centroids-rotations-world.json")
    plan = plan_patches(boxes, 46, 4)
    assert plan == [pid for pid in EXPECTED_PATCHES for _ in range(4)]
    assert Counter(plan) == {pid: 4 for pid in EXPECTED_PATCHES}
    assert len(plan) == 132
    assert plan == plan_patches(boxes, 46, 4)


@pytest.mark.parametrize("n_patches, repeats", [(-1, 4), (2, 4), (1, 0), (1, -1)])
def test_plan_rejects_invalid_counts(n_patches, repeats):
    boxes = [PatchBox(0, Frame(Rot(), vct3()))]
    with pytest.raises(ValueError):
        plan_patches(boxes, n_patches, repeats)


def test_empty_candidate_plan():
    assert plan_patches([], 0, 4) == []
    assert plan_patches([PatchBox(0, Frame(Rot(), vct3()))], 0, 4) == []


class MockSession:
    """Synthetic clock/observations with deliberately nonconsecutive sample IDs."""

    def __init__(self, observations):
        self.design = design_scene()
        self.observations = observations
        self.clock = 17.0
        self.calls = []
        self.attempt = -1

    def elapsed(self):
        return self.clock

    def place_pointer(self, patch_id):
        self.attempt += 1
        self.clock += 1.0
        self.calls.append(("place", patch_id))
        return PointerPlacement(patch_id, 10 + 3 * self.attempt, self.clock)

    def sample_marker_bodies(self):
        self.clock += 0.03
        self.calls.append(("sample", None))
        return self.observations[self.attempt]


def synthetic_observation_pair(common_jiggle=None):
    """Construct tracker poses whose tip position in the reference is known."""
    tip = design_scene().p_tip.vec.ravel()
    expected = np.array([7.0, -2.0, 9.0])
    ref_rotation = (Rot("x", 0.4) * Rot("z", -0.3)).matrix
    ref_translation = np.array([80.0, -50.0, 1200.0])
    ptr_rotation = (Rot("y", -0.6) * Rot("z", 0.2)).matrix
    ptr_translation = ref_rotation @ expected + ref_translation - ptr_rotation @ tip
    ref = Frame(Rot(matrix=ref_rotation), vct3(ref_translation))
    ptr = Frame(Rot(matrix=ptr_rotation), vct3(ptr_translation))
    if common_jiggle is not None:
        ref, ptr = common_jiggle * ref, common_jiggle * ptr
    return {"ref": Observation(ref, []), "ptr": Observation(ptr, [])}, expected


@pytest.mark.parametrize("jiggle", [None, Frame(Rot("y", 0.3), vct3(4, -3, 2))])
def test_collection_frame_chain_and_common_jiggle(jiggle):
    observations, expected = synthetic_observation_pair(jiggle)
    session = MockSession([observations, observations])
    samples = collect_samples(session, [4, 4])
    np.testing.assert_allclose(samples.as_array(), np.tile(expected, (2, 1)), atol=1e-12)
    assert samples.patch_ids == [4, 4]
    assert [h.sample_index for h in samples.handles] == [10, 13]
    assert [h.t_sim for h in samples.handles] == pytest.approx([18.0, 19.03])
    assert samples.t_collect == pytest.approx(2.06)
    assert session.calls == [("place", 4), ("sample", None)] * 2
    assert all(p.shape == (3,) for p in samples.d_ref)
    assert not np.shares_memory(samples.d_ref[0], samples.d_ref[1])


def test_collection_skips_missing_bodies_and_preserves_indices(capsys):
    pair, expected = synthetic_observation_pair()
    missing_ref = {"ref": None, "ptr": pair["ptr"]}
    missing_ptr = {"ref": pair["ref"], "ptr": None}
    session = MockSession([pair, missing_ref, missing_ptr, {"ref": None, "ptr": None}, pair])
    plan = [0, 1, 2, 3, 4]
    samples = collect_samples(session, plan)
    assert len(samples) == len(samples.handles) == len(samples.patch_ids) == 2
    assert samples.patch_ids == [0, 4]
    assert [h.sample_index for h in samples.handles] == [10, 22]
    np.testing.assert_allclose(samples.as_array(), np.tile(expected, (2, 1)), atol=1e-12)
    assert samples.t_collect == pytest.approx(5.15)
    assert session.calls == [call for pid in plan for call in (("place", pid), ("sample", None))]
    log = capsys.readouterr().out
    for index, pid in ((13, 1), (16, 2), (19, 3)):
        assert f"Skipping sample {index} for patch {pid}" in log


def test_empty_and_all_invisible_collection(capsys):
    empty = MockSession([])
    samples = collect_samples(empty, [])
    assert len(samples) == 0 and samples.as_array().shape == (0, 3)
    assert samples.t_collect == 0 and empty.calls == []

    invisible = MockSession([{"ref": None, "ptr": None}])
    samples = collect_samples(invisible, [5])
    assert len(samples) == 0 and samples.as_array().shape == (0, 3)
    assert samples.handles == samples.patch_ids == []
    assert samples.t_collect == pytest.approx(1.03)
    assert "Skipping sample 10 for patch 5" in capsys.readouterr().out


def test_real_plan_collection_with_synthetic_observations():
    boxes = load_patch_boxes(DATA / "box-centroids-rotations-world.json")
    plan = plan_patches(boxes, 46, 4)
    pair, expected = synthetic_observation_pair()
    session = MockSession([pair] * len(plan))
    samples = collect_samples(session, plan)
    assert len(samples) == 132
    assert samples.patch_ids == plan
    assert samples.t_collect == pytest.approx(135.96)
    np.testing.assert_allclose(samples.as_array(), np.tile(expected, (132, 1)), atol=1e-12)


@pytest.mark.parametrize("axis, angle, translation, tilt", [
    ("x", 0.0, [0, 0, 0], 0.0),
    ("y", 0.3, [50, -50, 50], 0.1),
    ("x", -0.3, [-50, 50, -50], -0.1),
])
def test_selected_patch_visibility_in_synthetic_scenes(axis, angle, translation, tilt):
    design = design_scene()
    mesh = load_sur(DATA / "ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(DATA / "box-centroids-rotations-world.json")
    selected = set(plan_patches(boxes, 46, 4))
    tracker = Tracker("Step 5 synthetic visibility", design.F_tracker_approx)
    world_from_ct = Frame(Rot(axis, angle), vct3(translation))
    mount = mesh.vertices[np.argmax(mesh.vertices[:, 0])] + [15, 0, 0]
    world_from_ref = world_from_ct * Frame(Rot("y", np.pi / 2), vct3(mount))
    assert tracker.is_visible([world_from_ref * marker for marker in design.ref_marker_positions])
    for box in boxes:
        if box.id not in selected:
            continue
        world_from_box = world_from_ct * box.F_box
        ptr_rotation = world_from_box.R * Rot("x", tilt)
        world_from_ptr = Frame(ptr_rotation, world_from_box.p - ptr_rotation * design.p_tip)
        assert tracker.is_visible([world_from_ptr * marker for marker in design.ptr_marker_positions])
