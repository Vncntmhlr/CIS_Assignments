"""
Unit tests for student components (src/student). These tests fail with
NotImplementedError until the corresponding algorithms are implemented.
Ground truth appears only in controlled test fixtures, never calibration code.

Run from the assignment2 folder:  python -m pytest tests -q
"""

import numpy as np
import pytest

from src import simulation, vct3, Rot, Frame
from src.student import metrics, registration, workflow, kinematics


def random_theta(rng):
    return np.array([rng.uniform(0, 600), rng.uniform(-300, 300), rng.uniform(0, 600),
                     rng.uniform(-1.3, 1.3), rng.uniform(-1.3, 1.3), rng.uniform(-2.5, 2.5)])


def random_frame(rng, angle=1.0, dist=100.0):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    return Frame(Rot(vct3(axis), angle=rng.uniform(-angle, angle)), vct3(rng.uniform(-dist, dist, 3)))


# ---------------------------------------------------------------- metrics

def test_frame_error_known_values():
    F_a = Frame(Rot(), vct3(1, 2, 3))
    F_b = Frame(Rot('z', 0.1), vct3(1, 2, 3))
    angle, dist = metrics.frame_error(F_a, F_b)
    assert np.isclose(angle, 0.1) and np.isclose(dist, 0.0)
    angle, dist = metrics.frame_error(F_a, Frame(Rot(), vct3(4, 6, 3)))
    assert np.isclose(angle, 0.0) and np.isclose(dist, 5.0)


def test_frame_error_is_invariant_to_common_left_factor():
    rng = np.random.default_rng(1)
    G, F_a, F_b = random_frame(rng), random_frame(rng), random_frame(rng)
    assert np.allclose(metrics.frame_error(F_a, F_b), metrics.frame_error(G * F_a, G * F_b))


def test_summarize_errors():
    s = metrics.summarize_errors([(0.0, 3.0), (0.0, 4.0)])
    assert s["n"] == 2 and np.isclose(s["rms_pos"], np.sqrt(12.5)) and np.isclose(s["max_pos"], 4.0)


# ---------------------------------------------------------------- registration

def test_register_points_recovers_known_transform():
    rng = np.random.default_rng(2)
    F = random_frame(rng)
    a = [vct3(rng.uniform(-50, 50, 3)) for _ in range(6)]
    b = [F * p for p in a]
    F_est = registration.register_points(a, b)
    assert np.allclose(F_est.R.matrix, F.R.matrix, atol=1e-9)
    assert np.allclose(F_est.p.vec, F.p.vec, atol=1e-9)


def test_register_points_rejects_degenerate_input():
    with pytest.raises(ValueError):
        registration.register_points([vct3(0, 0, 0), vct3(1, 0, 0)], [vct3(0, 0, 0), vct3(1, 0, 0)])


def test_average_frames_of_identical_frames():
    rng = np.random.default_rng(3)
    F = random_frame(rng)
    F_avg = registration.average_frames([F] * 5)
    assert np.allclose(F_avg.R.matrix, F.R.matrix, atol=1e-9)
    assert np.allclose(F_avg.p.vec, F.p.vec, atol=1e-9)


def test_average_frames_is_a_proper_rotation():
    rng = np.random.default_rng(4)
    F_avg = registration.average_frames([random_frame(rng, angle=0.2) for _ in range(10)])
    R = F_avg.R.matrix
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(R), 1.0)


def test_refine_marker_body_reduces_layout_error(monkeypatch):
    """Refinement must improve body shape, preserve truth, and account for time."""
    from src import uvct3
    from scipy.spatial.transform import Rotation

    # Make camera noise deterministic without changing NumPy's global RNG state.
    rng = np.random.default_rng(11)
    monkeypatch.setattr(np.random, "multivariate_normal", rng.multivariate_normal)
    simulation.clock.reset()
    tracker = simulation.create_tracker("refine_tracker", Frame(Rot(), vct3(0, 0, -1600)), seed=11)
    nominal = [vct3(-30, -30, -30), vct3(30, 30, -30),
               vct3(30, -30, 30), vct3(-30, 30, 30)]
    body = simulation.define_marker_body("refine_body", nominal)
    # A known manufacturing deformation is test data, not available to the solver.
    offsets = [vct3(-1, 2, 1), vct3(2, -1, -2), vct3(-2, 1, -1), vct3(1, -2, 2)]
    truth = [p + offset for p, offset in zip(nominal, offsets)]
    body.actual_marker_positions = [uvct3(p) for p in truth]
    placed = simulation.place_marker_body(body, Frame(Rot('z', 0.2), vct3()), precision="exact")
    simulation.add_marker_bodies(tracker, [placed])
    truth_before = np.array([p.vec.ravel() for p in truth])

    def shape_error(layout):
        # Independent proper-rotation alignment removes gauge but rejects reflection.
        points = np.array([p.vec.ravel() for p in layout])
        centered = points - points.mean(axis=0)
        reference = truth_before - truth_before.mean(axis=0)
        rotation, _ = Rotation.align_vectors(reference, centered)
        residual = rotation.apply(centered) - reference
        return float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

    before = shape_error(nominal)
    started = simulation.clock.elapsed
    n_readings = 60
    refined = registration.refine_marker_body(tracker, placed, n_readings=n_readings)
    assert len(refined) == len(nominal)
    assert shape_error(refined) < 0.25 * before
    assert np.allclose([p.vec for p in refined], [p.vec for p in body.nominal_marker_positions])
    assert np.array_equal(truth_before, [p.p.vec.ravel() for p in body.get_actual_marker_positions()])
    assert np.isclose(simulation.clock.elapsed - started,
                      n_readings * simulation.constants.SAMPLE_DELTA_T_SEC)


# ---------------------------------------------------------------- workflow

def test_estimate_base_frame_on_synthetic_data():
    """With perfect nominal data, the estimate must reproduce F_X exactly."""
    rob = simulation.create_robot("rob", Frame.eye(), seed=0)
    rng = np.random.default_rng(5)
    F_X = random_frame(rng, angle=0.3, dist=300.0)
    data = workflow.CalibrationData()
    for _ in range(10):
        th = random_theta(rng)
        data.thetas.append(th)
        data.frames.append(F_X * rob.fwd_kins(th))
    F_est = workflow.estimate_base_frame(data, rob)
    assert np.allclose(F_est.R.matrix, F_X.R.matrix, atol=1e-9)
    assert np.allclose(F_est.p.vec, F_X.p.vec, atol=1e-6)


def test_planned_poses_are_visible_and_within_limits():
    """A useful plan stays visible/in limits and excites every joint without moving."""
    simulation.clock.reset()
    rob = simulation.create_robot("plan_robot", Frame(Rot(), vct3(-800, 0, 1300)), seed=12)
    trk = simulation.create_tracker("plan_tracker", Frame.eye(), seed=12)
    s = 37.5
    markers = [vct3(s, s, s), vct3(s, -s, -s), vct3(-s, s, -s), vct3(-s, -s, s)]
    before_theta = rob.state().theta.copy()
    poses = workflow.plan_calibration_poses(
        rob, trk, markers, desired_accuracy=simulation.constants.TARGET_ANG_ERR_RAD)
    poses = np.asarray(poses, dtype=float)
    assert poses.ndim == 2 and poses.shape[0] > 0 and poses.shape[1] == 6
    assert np.all(np.isfinite(poses))
    assert np.all(np.ptp(poses, axis=0) > 1e-6), "Every joint needs excitation"
    for theta in poses:
        assert rob.within_limits(theta)
        F_ee = rob.F_rob * rob.fwd_kins(theta)
        assert trk.is_visible([F_ee * p for p in markers])
    assert simulation.clock.elapsed == 0.0
    assert np.array_equal(rob.state().theta, before_theta)


def test_measure_home_offsets_matches_encoders():
    rob = simulation.create_robot("rob", Frame.eye(), seed=9)
    offsets = workflow.measure_home_offsets(rob, n_homings=20)
    enc = rob.home_robot()
    assert offsets.shape == (6,)
    assert np.allclose(-offsets, enc, atol=4 * simulation.constants.SIGMA_RND.max())


# ---------------------------------------------------------------- kinematics

def test_improved_kinematics_initial_params_match_nominal():
    rob = simulation.create_robot("rob", Frame.eye(), seed=0)
    rng = np.random.default_rng(6)
    F_X = random_frame(rng, angle=0.3, dist=300.0)
    improved = kinematics.ImprovedKinematics(rob, F_X)
    for _ in range(10):
        th = random_theta(rng)
        F_nom, F_imp = rob.fwd_kins(th), improved.fwd_kins(th)
        assert np.allclose(F_nom.R.matrix, F_imp.R.matrix, atol=1e-9)
        assert np.allclose(F_nom.p.vec, F_imp.p.vec, atol=1e-9)
    assert np.allclose(improved.base_frame().p.vec, F_X.p.vec, atol=1e-9)


def test_improved_inv_kins_round_trip():
    rob = simulation.create_robot("rob", Frame.eye(), seed=0)
    improved = kinematics.ImprovedKinematics(rob, Frame.eye())
    improved.params = improved.initial_params() + 1e-3 * np.random.default_rng(7).normal(size=improved.params.shape)
    rng = np.random.default_rng(8)
    for _ in range(10):
        th = random_theta(rng)
        F = improved.fwd_kins(th)
        th_back = improved.inv_kins(F, seed=th)
        F_back = improved.fwd_kins(th_back)
        assert metrics.frame_error(F, F_back)[1] < 1e-6
        assert metrics.frame_error(F, F_back)[0] < 1e-8


def test_fit_recovers_synthetic_parameters():
    """Fit noiseless synthetic data and recover held-out poses, allowing gauge freedom."""
    from scipy.spatial.transform import Rotation

    rob = simulation.create_robot("fit_robot", Frame.eye(), seed=13)
    F_X = Frame(Rot('z', 0.2), vct3(50, -20, 100))
    rng = np.random.default_rng(13)
    generating = kinematics.ImprovedKinematics(rob, F_X)
    generating.params = generating.initial_params() + 1e-3 * rng.normal(size=generating.params.shape)
    data = workflow.CalibrationData()
    for _ in range(60):
        theta = random_theta(rng)
        data.thetas.append(theta)
        data.frames.append(generating.predict_in_tracker(theta))
    held_out = [random_theta(rng) for _ in range(20)]
    fitted = kinematics.ImprovedKinematics(rob, F_X)

    def prediction_errors(model):
        errors = []
        for theta in held_out:
            truth = generating.predict_in_tracker(theta)
            estimate = model.predict_in_tracker(theta)
            # Independent oracle avoids depending on the unimplemented metrics module.
            angle = Rotation.from_matrix(truth.R.matrix.T @ estimate.R.matrix).magnitude()
            distance = (truth.p - estimate.p).norm()
            errors.append((angle, distance))
        return np.asarray(errors)

    angle_tol, position_tol = 1e-5, 1e-3  # rad, mm; synthetic noise-free tolerances
    before = prediction_errors(fitted)
    assert np.any(before.max(axis=0) > 10 * np.array([angle_tol, position_tol]))
    fitted.fit(data)
    after = prediction_errors(fitted)
    assert np.all(np.isfinite(after))
    assert after[:, 0].max() < angle_tol
    assert after[:, 1].max() < position_tol
