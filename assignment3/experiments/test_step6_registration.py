"""Step 6 numerical checks with known synthetic transforms and real CT geometry."""

from pathlib import Path

import numpy as np
import pytest

from src import Frame, Rot, load_patch_boxes, load_sur, vct3
from src.mesh import PatchBox
from src.student.closest_point import closest_points_on_mesh
from src.student.icp import coarse_registration, icp, register_points


DATA = Path(__file__).resolve().parents[1] / "data"


def apply(frame, points):
    """Apply a source-to-target Frame to an (N,3) array."""
    return points @ frame.R.matrix.T + frame.p.vec.ravel()


def surface_case(seed=11, count=80):
    """Return a real mesh, exact synthetic surface points, transform and samples."""
    mesh = load_sur(DATA / "ProximalFemurMeshFile.sur")
    rng = np.random.default_rng(seed)
    indices = rng.choice(mesh.n_triangles, count, p=mesh.areas / mesh.areas.sum())
    uv = rng.uniform(size=(count, 2))
    flip = uv.sum(axis=1) > 1
    uv[flip] = 1 - uv[flip]
    weights = np.column_stack([1 - uv.sum(axis=1), uv])
    surface = np.einsum("ki,kij->kj", weights, mesh.tri_vertices[indices])
    truth = Frame(Rot(vct3(1, 2, 3), angle=0.24), vct3(12, -7, 15))
    return mesh, surface, truth, apply(truth.inv(), surface)


@pytest.mark.parametrize("planar", [False, True])
def test_rigid_fit_known_transform_and_planar_geometry(planar):
    source = np.random.default_rng(21).normal(size=(12, 3)) * 20
    if planar:
        source[:, 2] = 0
    truth = Frame(Rot(vct3(2, -1, 3), angle=0.7), vct3(30, -10, 9))
    target = apply(truth, source)
    result = register_points([vct3(p) for p in source], [vct3(p) for p in target])
    np.testing.assert_allclose(apply(result, source), target, atol=1e-12)
    np.testing.assert_allclose(result.R.matrix.T @ result.R.matrix, np.eye(3), atol=1e-12)
    assert np.linalg.det(result.R.matrix) == pytest.approx(1)


def test_rigid_fit_excludes_reflection():
    source = np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]], dtype=float)
    target = source * [-1, 1, 1]
    result = register_points([vct3(p) for p in source], [vct3(p) for p in target])
    assert np.linalg.det(result.R.matrix) == pytest.approx(1)
    assert np.sum((apply(result, source) - target) ** 2) == pytest.approx(16)


@pytest.mark.parametrize("kind", ["too_few", "mismatch", "collinear", "nonfinite"])
def test_rigid_fit_rejects_invalid_geometry(kind):
    source = np.eye(3)
    target = source.copy()
    if kind == "too_few":
        source, target = source[:2], target[:2]
    elif kind == "mismatch":
        target = target[:2]
    elif kind == "collinear":
        source = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
    else:
        source[0, 0] = np.nan
    with pytest.raises(ValueError):
        register_points([vct3(p) for p in source], [vct3(p) for p in target])


def test_patch_median_coarse_fit_with_extreme_single_outlier_per_patch():
    boxes = load_patch_boxes(DATA / "box-centroids-rotations-world.json")
    selected = [boxes[i] for i in (0, 8, 21, 34, 44)]
    truth = Frame(Rot("y", 0.3), vct3(14, -9, 7))
    by_patch = {}
    for box in selected:
        center = apply(truth.inv(), box.p[None, :])[0]
        points = np.tile(center, (4, 1))
        points[-1] += [400, -300, 200]
        by_patch[box.id] = points
    by_patch[1] = np.empty((0, 3))
    result = coarse_registration(by_patch, boxes)
    np.testing.assert_allclose(result.R.matrix, truth.R.matrix, atol=1e-12)
    np.testing.assert_allclose(result.p.vec, truth.p.vec, atol=1e-12)
    # This fixture deliberately has three identical clean contacts per patch;
    # it verifies robustness in a controlled case, not real patch-center accuracy.


@pytest.mark.parametrize("kind", ["unknown", "insufficient", "collinear", "bad_shape"])
def test_coarse_fit_rejects_unusable_inputs(kind):
    boxes = [PatchBox(i + 10, Frame(Rot(), vct3(p))) for i, p in enumerate(np.eye(3))]
    groups = {box.id: box.p[None, :] for box in boxes}
    if kind == "unknown":
        groups[100] = np.zeros((1, 3))
    elif kind == "insufficient":
        groups.pop(12)
    elif kind == "collinear":
        groups = {10 + i: np.array([[i, 0, 0]]) for i in range(3)}
    else:
        groups[10] = np.zeros(3)
    with pytest.raises(ValueError):
        coarse_registration(groups, boxes)


@pytest.mark.parametrize("fraction", [0.0, 0.1, 0.25])
def test_mad_uses_residuals_instead_of_a_fixed_outlier_count(fraction):
    mesh, surface, truth, samples = surface_case()
    count_bad = round(len(samples) * fraction)
    samples[:count_bad] += [200, 300, -200]
    result = icp(samples, mesh, truth)
    assert result.converged
    assert not result.inlier_mask[:count_bad].any()
    assert result.inlier_mask[count_bad:].all()
    np.testing.assert_allclose(apply(result.F_reg, samples[count_bad:]), surface[count_bad:], atol=1e-10)


def test_final_result_fields_after_iteration_cap_and_reject_none():
    mesh, _, truth, samples = surface_case()
    original = samples.copy()
    initial = Frame(Rot("z", 0.04), vct3(1, -1, 0.5)) * truth
    result = icp(samples, mesh, initial, max_iter=1, tol=1e-12, reject=None)
    assert not result.converged and result.n_iter == len(result.history) == 1
    assert result.inlier_mask.all()
    points, indices, distances = closest_points_on_mesh(apply(result.F_reg, samples), mesh)
    np.testing.assert_allclose(result.closest, points, atol=1e-12)
    np.testing.assert_allclose(result.residuals, distances, atol=1e-12)
    np.testing.assert_allclose(result.normals, mesh.face_normals[indices], atol=1e-12)
    assert result.history[-1] == pytest.approx(np.sqrt(np.mean(distances ** 2)))
    np.testing.assert_array_equal(samples, original)


@pytest.mark.parametrize("seed", [11, 33])
def test_small_clean_residuals_are_retained_after_perturbed_alignment(seed):
    # Regression: a near-zero MAD scale previously rejected 5/10 clean points.
    mesh, surface, truth, samples = surface_case(seed, count=120)
    rng = np.random.default_rng(seed + 100)
    bad = np.zeros(len(samples), dtype=bool)
    bad[rng.choice(len(samples), 12, replace=False)] = True
    samples[bad] += rng.normal(size=(12, 3)) * 20
    initial = Frame(Rot("z", 0.04), vct3(1, -1, 0.5)) * truth
    result = icp(samples, mesh, initial)
    assert result.converged
    np.testing.assert_array_equal(result.inlier_mask, ~bad)
    assert np.linalg.norm(apply(result.F_reg, samples[~bad]) - surface[~bad], axis=1).max() < 0.05


@pytest.mark.parametrize("kwargs", [{"reject": "trimmed"}, {"max_iter": 0},
                                    {"max_iter": 1.5}, {"tol": 0}, {"tol": np.nan}])
def test_icp_rejects_invalid_options(kwargs):
    mesh, _, truth, samples = surface_case()
    with pytest.raises(ValueError):
        icp(samples, mesh, truth, **kwargs)


def test_icp_rejects_too_few_finite_samples_and_inliers():
    mesh, surface, truth, samples = surface_case()
    with pytest.raises(ValueError, match="at least three"):
        icp(samples[:2], mesh, truth)
    with pytest.raises(ValueError, match="finite"):
        icp(np.full((3, 3), np.nan), mesh, truth)
    points = surface[:3].copy()
    points[-1] += [300, 300, 300]
    with pytest.raises(ValueError, match="fewer than three"):
        icp(points, mesh, Frame(Rot(), vct3()))
