"""
Unit tests for YOUR components (src/student). The complete tests below fail
with NotImplementedError until you implement the function they exercise;
the ones marked TODO are yours to write -- the handout requires you to
describe your unit tests and report their results.

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
    """TODO: write this test.

    Hint: define a body, place it in front of a tracker, measure how far the
    nominal layout is from body.get_actual_marker_positions() (ground truth,
    allowed in a test) up to a rigid transform, refine, and show the
    distance shrank. Also check how much simulated time it cost.
    """
    # Reproducible manufacturing errors and tracker noise.
    np.random.seed(0)

    # Nominal marker coordinates in the body's local frame (mm).
    s = 30
    a_k = [
        vct3( s,  s,  s),
        vct3( s, -s, -s),
        vct3(-s,  s, -s),
        vct3(-s, -s,  s),
    ]

    # Ordinary body: actual marker positions have manufacturing error.
    body = simulation.define_marker_body("body", a_k)

    # Ground-truth marker coordinates in the body's local frame.
    # Used only to evaluate the test, never inside refinement.
    a_star = [
        vct3(marker.p)
        for marker in body.get_actual_marker_positions()
    ]

    # Preserve the initial layout before refinement updates the body.
    a_before = [vct3(a) for a in a_k]

    # Tracker at workspace origin, looking along local +z.
    trk = simulation.create_tracker(
        "trk_refinement", Frame.eye(), seed=0
    )

    # Place the body 1500 mm in front of the tracker.
    placed = simulation.place_marker_body(
        body,
        Frame(Rot(), vct3(0, 0, 1500)),
        precision="exact",
    )
    simulation.add_marker_bodies(trk, [placed])

    assert trk.is_visible([
        placed.actual_placement * a for a in a_star
    ])

    def aligned_rms_error(a_est, a_star):
        """
        Inputs:
            a_est  : estimated local marker coordinates
            a_star : actual local marker coordinates
        Output:
            RMS layout error after rigid alignment, in mm.
        """
        F = registration.register_points(a_est, a_star)

        squared_errors = [
            ((F * a) - a_true).norm() ** 2
            for a, a_true in zip(a_est, a_star)
        ]
        return float(np.sqrt(np.mean(squared_errors)))

    # Initial error, after aligning nominal layout to actual layout.
    error_before = aligned_rms_error(a_before, a_star)

    # Refine using repeated tracker observations.
    n_readings = 30
    t_start = simulation.clock.elapsed

    a_hat = registration.refine_marker_body(
        trk, placed, n_readings=n_readings
    )

    t_refinement = simulation.clock.elapsed - t_start

    # Refined error, after aligning refined layout to actual layout.
    error_after = aligned_rms_error(a_hat, a_star)

    print(f"Before refinement: {error_before:.4f} mm RMS")
    print(f"After refinement:  {error_after:.4f} mm RMS")
    print(f"Refinement time:   {t_refinement:.3f} s")

    assert len(a_hat) == len(a_star)
    assert np.isfinite(error_after)
    assert error_after < error_before, (
        f"Layout did not improve: "
        f"{error_before:.4f} -> {error_after:.4f} mm RMS"
    )

    expected_time = (
        n_readings * simulation.constants.SAMPLE_DELTA_T_SEC
    )
    assert np.isclose(t_refinement, expected_time)


   

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
    """TODO: write this test for your plan_calibration_poses."""
    # Workspace origin coincides with the tracker.
    F_tracker = Frame.eye()
    F_robot = Frame(Rot(), vct3(-900, 0, 1300))

    rob = simulation.create_robot("rob", F_robot, seed=0)
    trk = simulation.create_tracker("trk", F_tracker, seed=0)

    # Four noncoplanar markers, each within the 75 mm radius limit.
    s = 30.0
    markers = [
        vct3( s,  s,  s),
        vct3( s, -s, -s),
        vct3(-s,  s, -s),
        vct3(-s, -s,  s),
    ]
    placed = simulation.create_robot_calibration_body(rob, markers)
    simulation.add_marker_bodies(trk, [placed])
    

    # Call the function being tested.
    poses = workflow.plan_calibration_poses(
        rob,
        trk,
        markers,
        desired_accuracy=simulation.constants.TARGET_ANG_ERR_RAD,
    )

    assert len(poses) > 0, "The calibration plan is empty"

    for i, pose in enumerate(poses):
        theta = np.asarray(pose, dtype=float)

        # Each pose must contain six finite joint values.
        assert theta.shape == (6,), f"Pose {i} must contain six joint values"
        assert np.all(np.isfinite(theta)), f"Pose {i} contains NaN or infinity"

        # Check the robot's joint limits.
        assert np.all(theta >= rob.theta_min), (
            f"Pose {i} falls below a joint's minimum"
        )
        assert np.all(theta <= rob.theta_max), (
            f"Pose {i} exceeds a joint's maximum"
        )

        # Transform body-local markers into workspace coordinates.
        F_ee_world = rob.F_rob * rob.fwd_kins(theta)
        markers_world = [F_ee_world * a for a in markers]

        # is_visible() converts workspace points to tracker coordinates.
        assert trk.is_visible(markers_world), (
            f"Pose {i} places one or more markers outside the viewing volume"
        )


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
    """TODO: write this test.

    Hint: generate CalibrationData from your own model with a known,
    non-zero parameter vector (no noise), fit from zero, and check that
    predict_in_tracker agrees with the generating model on new poses. Note that
    the parameters themselves need not match if your model has a null
    direction -- compare poses, not parameters.
    """
    rng = np.random.default_rng(10)

    rob = simulation.create_robot(
        "rob_fit", Frame.eye(), seed=0
    )

    # Known robot-base pose relative to the tracker.
    F_X = Frame(
        Rot.xyz(0.05, -0.03, 0.02),
        vct3(100, -50, 200),
    )

    # Create the model that generates the synthetic observations.
    known = kinematics.ImprovedKinematics(rob, F_X)

    P_known = known.initial_params().copy()
    P_known += rng.normal(
        loc=0.0,
        scale=1e-3,
        size=P_known.shape,
    )
    known.params = P_known

    # Collect paired commanded coordinates and synthetic tracker poses.
    data = workflow.CalibrationData()

    for _ in range(80):
        theta_cmd = random_theta(rng)
        F_ee_meas = known.predict_in_tracker(theta_cmd)

        data.thetas.append(theta_cmd.copy())
        data.frames.append(F_ee_meas)

    # Separate configurations for validation
    theta_cmd_test = [
        random_theta(rng) for _ in range(20)
    ]
    F_ee_reference = [
        known.predict_in_tracker(theta_cmd)
        for theta_cmd in theta_cmd_test
    ]

    # Initialize the model without the generating corrections
    estimated = kinematics.ImprovedKinematics(rob, F_X)

    errors_before = np.array([
        metrics.frame_error(
            F_reference,
            estimated.predict_in_tracker(theta_cmd),
        )
        for theta_cmd, F_reference
        in zip(theta_cmd_test, F_ee_reference)
    ])

    # Fit using training observations only.
    estimated.fit(data)

    # Evaluate predictions at the held-out configurations.
    errors_after = np.array([
        metrics.frame_error(
            F_reference,
            estimated.predict_in_tracker(theta_cmd),
        )
        for theta_cmd, F_reference
        in zip(theta_cmd_test, F_ee_reference)
    ])

    # Columns: orientation error (rad), position error (mm).
    rms_before = np.sqrt(np.mean(errors_before ** 2, axis=0))
    rms_after = np.sqrt(np.mean(errors_after ** 2, axis=0))

    max_angle = np.max(errors_after[:, 0])
    max_position = np.max(errors_after[:, 1])

    print(
        f"Position RMS: {rms_before[1]:.6f}"
        f" -> {rms_after[1]:.6f} mm"
    )
    print(
        f"Angle RMS: {rms_before[0]:.3e}"
        f" -> {rms_after[0]:.3e} rad"
    )

    assert np.all(np.isfinite(estimated.params))
    assert np.all(np.isfinite(errors_after))

    # Strict numerical tolerances for this noiseless synthetic test.
    assert max_position < 1e-3, (
        f"Maximum position error: {max_position:.3e} mm"
    )
    assert max_angle < 1e-6, (
        f"Maximum angle error: {max_angle:.3e} rad"
    )
