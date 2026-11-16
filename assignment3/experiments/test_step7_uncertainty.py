"""Step 7 numerical tests; synthetic covariance models are explicitly isolated."""

from dataclasses import asdict, fields, replace
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.linalg import expm

from src import Frame, Rot, load_patch_boxes, load_sur, vct3
from src.mesh import Mesh
from src.session import Observation, PointerPlacement, frame_from_json
from src.student import icp, report, uncertainty, workflow


ROOT = Path(__file__).resolve().parents[1]


def skew(vector):
    """Independent explicit 3x3 cross-product matrix for test equations."""
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def pose_exp(tangent):
    """Use a general matrix exponential as an independent SE(3) reference."""
    algebra = np.zeros((4, 4))
    algebra[:3, :3] = skew(tangent[:3])
    algebra[:3, 3] = tangent[3:]
    return frame_from_json(expm(algebra))


def surface_result(seed=71, count=80):
    """Build a full-rank synthetic fixed-normal model, not an ICP simulation."""
    rng = np.random.default_rng(seed)
    points = rng.normal(size=(count, 3)) * 25 + [41, -20, 18]
    normals = rng.normal(size=(count, 3))
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    errors = rng.normal(scale=0.02, size=count)
    result = icp.IcpResult(Frame(Rot(), vct3()), np.abs(errors), np.ones(count, dtype=bool),
                           points - errors[:, None] * normals, normals, 1, True, [0.02])
    result._transformed_points_ct = points.copy()
    return result, points, normals, errors


def test_centered_covariance_matches_independent_ct_formula():
    result, points, normals, errors = surface_result()
    jacobian = np.column_stack([np.cross(points, normals), normals])
    sigma_squared = errors @ errors / (len(errors) - 6)
    expected = sigma_squared * np.linalg.inv(jacobian.T @ jacobian)
    mu, covariance = uncertainty.registration_covariance(result, Frame(Rot(), vct3()))
    np.testing.assert_array_equal(mu, np.zeros(6))
    np.testing.assert_allclose(covariance, expected, rtol=1e-10, atol=1e-16)
    np.testing.assert_allclose(covariance, covariance.T, atol=1e-16)
    assert np.linalg.eigvalsh(covariance).min() > 0
    assert result._covariance_diagnostics["rank"] == 6
    assert not result._covariance_diagnostics["approximate_input"]


@pytest.mark.parametrize("angle", [0.0, 1e-9, 0.2, np.pi - 1e-6])
def test_mu_is_exact_left_correction_not_inverse_or_plain_translation(angle):
    result, _, _, _ = surface_result()
    expected = np.array([angle, 0, 0, 3, -4, 5])
    initial = Frame(Rot("z", 0.3), vct3(20, -10, 30))
    result.F_reg = pose_exp(expected) * initial
    mu, _ = uncertainty.registration_covariance(result, initial)
    np.testing.assert_allclose(mu, expected, rtol=1e-8, atol=1e-8)


def test_left_ct_jacobian_against_finite_differences():
    _, points, normals, _ = surface_result(count=12)
    jacobian = np.column_stack([np.cross(points, normals), normals])
    h = 1e-7
    for column in range(6):
        step = np.eye(6)[column] * h
        perturbation = pose_exp(step)
        moved = points @ perturbation.R.matrix.T + perturbation.p.vec.ravel()
        difference = np.einsum("ij,ij->i", normals, moved - points) / h
        np.testing.assert_allclose(difference, jacobian[:, column], atol=1e-5, rtol=1e-6)


def test_legacy_result_warns_and_agrees_for_pure_normal_residuals():
    exact, _, _, _ = surface_result()
    legacy = replace(exact)
    _, expected = uncertainty.registration_covariance(exact, Frame(Rot(), vct3()))
    with pytest.warns(RuntimeWarning, match="lacks transformed samples"):
        _, approximate = uncertainty.registration_covariance(legacy, Frame(Rot(), vct3()))
    np.testing.assert_allclose(approximate, expected, rtol=1e-10, atol=1e-16)
    assert legacy._covariance_diagnostics["approximate_input"]
    assert legacy._covariance_diagnostics["noise_scale"] == "euclidean_norm_proxy"
    assert set(asdict(exact)) == {field.name for field in fields(icp.IcpResult)}
    assert "_transformed_points_ct" not in asdict(exact)


def test_legacy_proxy_difference_is_visible_for_tangential_residuals():
    exact, points, normals, errors = surface_result()
    tangent = np.cross(normals, np.array([0, 0, 1])) * 0.2
    exact.closest = points - errors[:, None] * normals - tangent
    exact.residuals = np.linalg.norm(points - exact.closest, axis=1)
    legacy = replace(exact)
    _, covariance = uncertainty.registration_covariance(exact, Frame(Rot(), vct3()))
    with pytest.warns(RuntimeWarning):
        _, proxy = uncertainty.registration_covariance(legacy, Frame(Rot(), vct3()))
    assert np.trace(proxy) > 2 * np.trace(covariance)


def test_icp_private_points_match_final_pose_and_keep_public_fields():
    from experiments.test_step6_registration import surface_case
    mesh, _, truth, samples = surface_case(count=30)
    result = icp.icp(samples, mesh, truth)
    expected = samples @ result.F_reg.R.matrix.T + result.F_reg.p.vec.ravel()
    np.testing.assert_allclose(result._transformed_points_ct, expected, atol=1e-12)
    assert not np.shares_memory(result._transformed_points_ct, samples)
    assert set(asdict(result)) == {"F_reg", "residuals", "inlier_mask", "closest",
                                  "normals", "n_iter", "converged", "history"}


def test_nearly_parallel_normals_are_reported_as_ill_conditioned():
    result, _, normals, _ = surface_result()
    normals = normals * 1e-10 + [0, 0, 1]
    result.normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)
    with pytest.raises(ValueError, match="ill-conditioned"):
        uncertainty.registration_covariance(result, Frame(Rot(), vct3()))


@pytest.mark.parametrize("kind", ["few", "plane", "zero_normals", "bad_points", "bad_mask"])
def test_covariance_rejects_invalid_or_unobservable_geometry(kind):
    result, _, _, _ = surface_result(count=12)
    if kind == "few":
        result.inlier_mask[6:] = False
    elif kind == "plane":
        result.normals[:] = [0, 0, 1]
    elif kind == "zero_normals":
        result.normals[:] = 0
    elif kind == "bad_points":
        result._transformed_points_ct = np.zeros((2, 3))
    else:
        result.inlier_mask = np.ones(12, dtype=int)
    with pytest.raises(ValueError):
        uncertainty.registration_covariance(result, Frame(Rot(), vct3()))


def test_ct_coordinate_translation_changes_covariance_but_not_predicted_displacement():
    original, points, _, _ = surface_result()
    _, covariance = uncertainty.registration_covariance(original, Frame(Rot(), vct3()))
    offset = np.array([500, -300, 200])
    shifted = replace(original, closest=original.closest + offset)
    shifted._transformed_points_ct = points + offset
    _, shifted_covariance = uncertainty.registration_covariance(shifted, Frame(Rot(), vct3()))
    adjoint = np.eye(6)
    adjoint[3:, :3] = skew(offset)
    np.testing.assert_allclose(shifted_covariance, adjoint @ covariance @ adjoint.T,
                               rtol=1e-10, atol=1e-15)
    empty_triangles = np.empty((0, 3), dtype=int)
    first = uncertainty.predicted_tre(Mesh(points, empty_triangles), original.F_reg, covariance)
    second = uncertainty.predicted_tre(Mesh(points + offset, empty_triangles), shifted.F_reg,
                                       shifted_covariance)
    np.testing.assert_allclose(list(first.values()), list(second.values()), rtol=1e-10)


def test_tre_translation_and_rotation_have_known_units_and_vertex_statistics():
    mesh = Mesh(np.array([[1, 0, 0], [2, 0, 0], [0, 0, 3.0]]), np.empty((0, 3), dtype=int))
    frame = Frame(Rot("x", 0.8), vct3(100, 200, 300))
    translation_covariance = np.diag([0, 0, 0, 1, 4, 9])
    predicted = uncertainty.predicted_tre(mesh, frame, translation_covariance)
    np.testing.assert_allclose(list(predicted.values()), np.sqrt(14))
    rotation_covariance = np.diag([0, 0, 1e-4, 0, 0, 0])
    predicted = uncertainty.predicted_tre(mesh, frame, rotation_covariance)
    expected = np.array([0.01, 0.02, 0])
    np.testing.assert_allclose(list(predicted.values()), [np.sqrt(np.mean(expected ** 2)),
        expected.mean(), expected.max(), np.percentile(expected, 90)])
    assert set(uncertainty.predicted_tre(mesh, frame, np.zeros((6, 6))).values()) == {0.0}


@pytest.mark.parametrize("kind", ["shape", "nonsymmetric", "negative", "nan"])
def test_tre_rejects_invalid_covariance(kind):
    mesh = Mesh(np.ones((1, 3)), np.empty((0, 3), dtype=int))
    covariance = np.eye(6)
    if kind == "shape":
        covariance = np.eye(3)
    elif kind == "nonsymmetric":
        covariance[0, 1] = 0.1
    elif kind == "negative":
        covariance[0, 0] = -1
    else:
        covariance[0, 0] = np.nan
    with pytest.raises(ValueError):
        uncertainty.predicted_tre(mesh, Frame(Rot(), vct3()), covariance)


class ReplaySession:
    """Session-only mock: manufacture tracker frames for saved measured d_ref."""

    def __init__(self, missing=()):
        self.design = workflow.design_scene()
        self.n_patches = 46
        self.rows = json.loads((ROOT / "experiments/results/step5_practice.json").read_text())["samples"]
        self.initial = frame_from_json(json.loads((ROOT / "experiments/results/step6_validation.json").read_text())
                                       ["practice_replay"]["F_coarse_reference_to_ct"])
        self.clock, self.index = 17.0, -1
        self.missing = set(missing)
        self.calls = []

    def initial_registration_guess(self):
        self.calls.append("initial_registration_guess")
        return self.initial

    def elapsed(self):
        return self.clock

    def place_pointer(self, patch_id):
        self.index += 1
        assert self.rows[self.index]["patch_id"] == patch_id
        self.clock += 1
        self.calls.append("place_pointer")
        return PointerPlacement(patch_id, 100 + 7 * self.index, self.clock)

    def sample_marker_bodies(self):
        self.clock += 0.03
        self.calls.append("sample_marker_bodies")
        ref = Frame(Rot("x", 0.15), vct3(30, -20, 1400))
        point = np.array(self.rows[self.index]["d_ref_mm"])
        ptr_origin = ref.R.matrix @ point + ref.p.vec.ravel() - self.design.p_tip.vec.ravel()
        pair = {"ref": Observation(ref, []), "ptr": Observation(Frame(Rot(), vct3(ptr_origin)), [])}
        if self.index in self.missing:
            pair["ref" if self.index % 2 == 0 else "ptr"] = None
        return pair

    def evaluate_tre(self, *args):
        raise AssertionError("truth scoring must not be called by the workflow")

    def true_outliers(self):
        raise AssertionError("true outlier labels must not be called by the workflow")


@pytest.mark.parametrize("missing", [(), (2, 5, 10)])
def test_headless_report_and_missing_attempt_accounting(missing, tmp_path, capsys):
    mesh = load_sur(ROOT / "data/ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(ROOT / "data/box-centroids-rotations-world.json")
    session = ReplaySession(missing)
    result = workflow.run_registration(session, mesh, boxes)
    assert isinstance(result, report.RegistrationReport)
    assert result.converged and len(result.samples) == 132 and len(result.patches) == 33
    assert result.t_simulated == pytest.approx(17 + 135.96)
    assert result.t_wall > 0
    assert session.calls.count("initial_registration_guess") == 1
    assert session.calls.count("place_pointer") == session.calls.count("sample_marker_bodies") == 132
    assert len({row["sample_index"] for row in result.samples}) == 132
    for index in missing:
        row = result.samples[index]
        assert row["sample_index"] == 100 + 7 * index
        assert row["reason"].startswith("invisible_") and not row["used"]
        assert row["d_ref"] is None and row["residual_mm"] is None
    assert all(row["reason"] in {"ok", "icp_mad"} for i, row in enumerate(result.samples) if i not in missing)
    assert sum(p["n_samples"] for p in result.patches) == 132
    assert sum(p["n_used"] for p in result.patches) == result.residual_stats["n_used"]
    assert all(p["n_samples"] == p["n_used"] + p["n_rejected"] for p in result.patches)
    assert result.eta_reg["convention"] == "left_ct"
    assert not result.eta_reg["covariance_model"]["approximate_input"]
    assert np.isfinite(list(result.predicted_tre.values())).all()
    result.to_json(tmp_path / "report.json")
    restored = report.RegistrationReport.from_json(tmp_path / "report.json")
    assert restored.to_dict() == result.to_dict()
    assert "Predicted TRE" in restored.to_text()
    capsys.readouterr()


@pytest.mark.parametrize("options", [{"unknown": 1}, {"max_iter": 0}, {"tol": 0},
                                     {"reject": "trimmed"}, {"n_patches": 0}])
def test_invalid_workflow_options_do_not_spend_sampling_time(options):
    session = ReplaySession()
    mesh = load_sur(ROOT / "data/ProximalFemurMeshFile.sur")
    boxes = load_patch_boxes(ROOT / "data/box-centroids-rotations-world.json")
    with pytest.raises((TypeError, ValueError)):
        workflow.run_registration(session, mesh, boxes, **options)
    assert session.calls == [] and session.clock == 17
