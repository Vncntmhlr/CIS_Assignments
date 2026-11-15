"""
Unit tests for YOUR components (src/student). The complete tests below fail
with NotImplementedError until you implement what they call; the ones
marked TODO are yours to write.

Run from assignments/assignment3:  python -m pytest tests -q
"""

import numpy as np
import pytest

from pathlib import Path

from src import vct3, Rot, Frame, load_sur, load_patch_boxes

MESH_FILE = Path(__file__).resolve().parents[1] / "data" / "ProximalFemurMeshFile.sur"
from src.student import closest_point, icp, uncertainty, report


def random_frame(rng, angle=1.0, dist=100.0):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    return Frame(Rot(vct3(axis), angle=rng.uniform(-angle, angle)), vct3(rng.uniform(-dist, dist, 3)))


def apply(F, P):
    return P @ F.R.matrix.T + F.p.vec.ravel()


@pytest.fixture(scope="module")
def mesh():
    return load_sur(MESH_FILE)


@pytest.fixture(scope="module")
def surface_points(mesh):
    rng = np.random.default_rng(0)
    tri = rng.choice(mesh.n_triangles, size=150, p=mesh.areas / mesh.areas.sum())
    r1, r2 = rng.uniform(size=(2, 150))
    flip = r1 + r2 > 1
    r1[flip], r2[flip] = 1 - r1[flip], 1 - r2[flip]
    bary = np.stack([1 - r1 - r2, r1, r2], axis=1)
    return np.einsum("kj,kjd->kd", bary, mesh.tri_vertices[tri])


# ---------------------------------------------------------------- closest point

def test_closest_point_on_triangle_regions():
    a, b, c = vct3(0, 0, 0), vct3(10, 0, 0), vct3(0, 10, 0)
    assert np.allclose(closest_point.closest_point_on_triangle(vct3(2, 3, 5), a, b, c).vec.ravel(), [2, 3, 0])   # face
    assert np.allclose(closest_point.closest_point_on_triangle(vct3(5, -4, 1), a, b, c).vec.ravel(), [5, 0, 0])  # edge ab
    assert np.allclose(closest_point.closest_point_on_triangle(vct3(-3, -3, 2), a, b, c).vec.ravel(), [0, 0, 0]) # vertex a
    assert np.allclose(closest_point.closest_point_on_triangle(vct3(8, 8, 0), a, b, c).vec.ravel(), [5, 5, 0])   # edge bc


def test_closest_points_on_mesh_matches_brute_force(mesh):
    rng = np.random.default_rng(1)
    q = rng.uniform(-70, 70, size=(40, 3))
    closest, tri, dist = closest_point.closest_points_on_mesh(q, mesh)
    assert closest.shape == (40, 3) and tri.shape == (40,) and dist.shape == (40,)
    for k in range(40):
        best = min((closest_point.closest_point_on_triangle(vct3(q[k]), *[vct3(v) for v in mesh.tri_vertices[t]]) - vct3(q[k])).norm()
                   for t in range(mesh.n_triangles))
        assert np.isclose(dist[k], best, atol=1e-6)
        assert np.isclose(np.linalg.norm(closest[k] - q[k]), dist[k])


def test_closest_points_on_surface_points_have_zero_distance(mesh, surface_points):
    _, _, dist = closest_point.closest_points_on_mesh(surface_points, mesh)
    assert dist.max() < 1e-6


# ---------------------------------------------------------------- registration

def test_register_points_recovers_known_transform():
    rng = np.random.default_rng(2)
    F = random_frame(rng)
    a = [vct3(rng.uniform(-50, 50, 3)) for _ in range(6)]
    b = [F * p for p in a]
    F_est = icp.register_points(a, b)
    assert np.allclose(F_est.R.matrix, F.R.matrix, atol=1e-9) and np.allclose(F_est.p.vec, F.p.vec, atol=1e-9)


def test_icp_recovers_transform_from_surface_samples(mesh, surface_points):
    rng = np.random.default_rng(3)
    F_true = random_frame(rng, angle=0.5, dist=20.0)             # reference -> CT
    d = apply(F_true.inv(), surface_points)                        # samples in reference coordinates
    F_init = Frame(Rot(vct3(0, 0, 1), angle=0.15), vct3(5, -5, 3)) * F_true
    res = icp.icp(d, mesh, F_init, max_iter=100)
    assert res.converged
    err = np.linalg.norm(apply(res.F_reg, d) - surface_points, axis=1)
    assert err.max() < 0.05


def test_icp_rejects_injected_outliers(mesh, surface_points):
    rng = np.random.default_rng(4)
    F_true = random_frame(rng, angle=0.5, dist=20.0)
    d = apply(F_true.inv(), surface_points)
    bad = rng.choice(len(d), size=15, replace=False)
    d[bad] += rng.normal(size=(15, 3)) * 20.0
    res = icp.icp(d, mesh, F_true, max_iter=100, reject="mad")
    assert not res.inlier_mask[bad].any()
    assert res.inlier_mask.sum() >= len(d) - 15 - 5
    err = np.linalg.norm(apply(res.F_reg, d[res.inlier_mask]) - surface_points[res.inlier_mask], axis=1)
    assert err.max() < 0.05


# ---------------------------------------------------------------- report (provided)

def test_report_round_trip(tmp_path):
    F = Frame(Rot('z', 0.3), vct3(1, 2, 3))
    r = report.RegistrationReport(F_reg=F, patches=[{"patch_id": 0, "n_samples": 2, "n_used": 1, "n_rejected": 1}],
                                  samples=[{"sample_index": 0, "patch_id": 0, "t_sim": 1.0, "d_ref": [0, 0, 0],
                                            "residual_mm": 0.1, "used": True, "reason": "ok"}],
                                  residual_stats=report.residual_statistics(np.array([0.1])),
                                  t_simulated=1.03, t_wall=0.5, n_icp_iterations=3, converged=True)
    r.to_json(tmp_path / "r.json")
    back = report.RegistrationReport.from_json(tmp_path / "r.json")
    assert np.allclose(back.F_reg.R.matrix, F.R.matrix) and np.allclose(back.F_reg.p.vec, F.p.vec)
    assert "Valid samples (1)" in back.to_text()


# ---------------------------------------------------------------- TODO: yours

def test_registration_covariance_is_consistent():
    pytest.skip("TODO: check that eta_true is within the predicted covariance (Mahalanobis ~ chi2_6)")


def test_patch_plan_keeps_bodies_visible():
    pytest.skip("TODO: check every planned patch keeps pointer and reference inside the frustum")
