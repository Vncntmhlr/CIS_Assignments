"""
Provided tests for the simulator API. They pass as shipped; read them as
examples of how to unit-test a component against known inputs/outputs.
"""

import numpy as np
import pytest

from src import simulation, vct3, Rot, Frame
from src.simulation import clock, constants


def make_robot(seed=0):
    return simulation.create_robot("rob", Frame.eye(), seed=seed)


def square_body(size=60.0):
    return [vct3(0, 0, 0), vct3(size, 0, 0), vct3(0, size, 0), vct3(0, 0, size)]


def random_theta(rng):
    return np.array([rng.uniform(0, 600), rng.uniform(-300, 300), rng.uniform(0, 600),
                     rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4), rng.uniform(-np.pi, np.pi)])


# ---------------------------------------------------------------- nominal kinematics

def test_fwd_kins_home():
    rob = make_robot()
    F = rob.fwd_kins(np.zeros(6))
    assert np.allclose(F.R.matrix, np.eye(3))
    assert np.allclose(F.p.vec.ravel(), [600.0, 0.0, 0.0])


def test_rot_axis_sign_matches_exp_so3():
    from uncertainty_networks.se3 import exp_so3
    for axis, vec in (("x", [1, 0, 0]), ("y", [0, 1, 0]), ("z", [0, 0, 1])):
        assert np.allclose(Rot(axis, 0.7).matrix, exp_so3(0.7 * np.array(vec, dtype=float)))


def test_inv_kins_round_trip_random():
    rob = make_robot()
    rng = np.random.default_rng(0)
    for _ in range(100):
        th = random_theta(rng)
        F = rob.fwd_kins(th)
        th_back = rob.inv_kins(F, seed=th)
        assert np.allclose(th, th_back, atol=1e-9)


def test_inv_kins_both_wrist_branches_reproduce_rotation():
    rob = make_robot()
    th = np.array([10.0, -20.0, 30.0, 0.4, -0.8, 1.9])
    F = rob.fwd_kins(th)
    sols = rob.wrist_solutions(F.R)
    assert len(sols) == 2
    for w in sols:
        assert np.allclose(rob.fwd_kins(np.concatenate([th[:3], w])).R.matrix, F.R.matrix, atol=1e-9)


def test_inv_kins_near_wrist_singularity():
    rob = make_robot()
    F = rob.fwd_kins([300, 0, 300, 0.3, np.pi / 2, 0.2])
    th = rob.inv_kins(F)
    assert np.allclose(rob.fwd_kins(th).R.matrix, F.R.matrix, atol=1e-6)
    assert np.allclose(rob.fwd_kins(th).p.vec, F.p.vec, atol=1e-6)


def test_inv_kins_rejects_unreachable():
    rob = make_robot()
    with pytest.raises(ValueError):
        rob.inv_kins(Frame(Rot(), vct3(5000, 0, 0)))
    assert not rob.is_reachable(Frame(Rot(), vct3(5000, 0, 0)))


# ---------------------------------------------------------------- limits, timing, clock

def test_move_joints_rejects_limit_violation():
    rob = make_robot()
    with pytest.raises(ValueError):
        rob.move_joints([650, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        rob.move_joints([-10, 0, 0, 0, 0, 0])
    with pytest.raises(ValueError):
        rob.move_joints([0, 0, 0, 0, 2.5, 0])


def test_motion_time_profiles():
    rob = make_robot()
    # long move: trapezoid, d/v + v/a; short move: triangle, 2 sqrt(d/a)
    assert np.isclose(rob.motion_time(np.zeros(6), [300, 0, 0, 0, 0, 0]), 0.05 + 300 / 50 + 50 / 500)
    assert np.isclose(rob.motion_time(np.zeros(6), [1, 0, 0, 0, 0, 0]), 0.05 + 2 * np.sqrt(1 / 500))
    assert np.isclose(rob.motion_time(np.zeros(6), np.zeros(6)), 0.05)
    assert np.isclose(rob.motion_time(np.zeros(6), [0, 0, 0, 0, np.pi / 2, 0]), 0.05 + np.pi / 2 / 0.1 + 0.1 / 1.0)


def test_clock_advances_on_moves_and_samples():
    clock.reset()
    rob = make_robot()
    dt = rob.move_joints([50, 0, 0, 0, 0, 0])
    assert np.isclose(clock.elapsed, dt)
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    simulation.sample_marker_bodies(trk)  # nothing tracked, still costs time
    assert np.isclose(clock.elapsed, dt + constants.SAMPLE_DELTA_T_SEC)
    assert rob.state().t_elapsed == clock.elapsed


def test_home_robot_returns_encoders_and_oracle_is_silent():
    rob = make_robot(seed=3)
    rob.move_joints([100, 50, 50, 0.5, 0.2, 0.1])
    assert np.any(np.abs(rob._theta_star - rob.theta_cmd) > 1e-6)  # distortion + noise present
    enc = rob.home_robot()
    assert np.all(rob._theta_star == 0.0)                        # physically at home ...
    assert np.allclose(enc, rob.state().theta)                   # ... and State() reports the encoder values
    assert np.any(np.abs(enc) > 1e-4)                            # which are NOT zero: -delta_theta_sys(home) - rnd
    assert np.allclose(enc + rob._delta_theta_sys(enc), 0.0, atol=3 * constants.SIGMA_RND.max() + 1e-9)
    t = clock.elapsed
    F = rob.actual_ee_pose([10, 20, 30, 0.1, 0.2, 0.3])
    assert isinstance(F, Frame) and clock.elapsed == t


def test_robot_starts_at_physical_home():
    rob = make_robot(seed=6)
    assert np.all(rob._theta_star == 0.0) and np.any(rob.state().theta != 0.0)


def test_joint_distortion_respects_handout_bounds():
    rob = make_robot(seed=4)
    thetas = np.linspace(rob.theta_min, rob.theta_max, 2000)
    sys_vals = np.array([rob._delta_theta_sys(th) for th in thetas])
    slope = np.abs(np.diff(sys_vals, axis=0) / np.diff(thetas, axis=0))
    assert np.all(slope.max(axis=0) <= constants.OMEGA_SYS * 1.001)
    assert np.all(np.abs(sys_vals).max(axis=0) <= constants.DELTA_SYS * 1.001)
    gt = rob.ground_truth_parameters()
    assert np.allclose(gt["axis_tilts"][0], 0.0) and np.allclose(gt["axes"][0], [1, 0, 0])  # d*_1 = d_1
    assert all(np.linalg.norm(gt["axis_tilts"][k]) <= constants.PHI_MAX for k in range(1, 6))


def test_actual_kinematics_differ_from_nominal():
    rob = make_robot(seed=5)
    th = np.array([150, 150, 150, 0.5, 0.5, 0.5])
    nominal = rob.F_rob * rob.fwd_kins(th)
    actual = rob.actual_ee_pose(th)
    assert (nominal.p - actual.p).norm() > 1.0


# ---------------------------------------------------------------- tracker, bodies

def test_frustum_visibility():
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    assert trk.is_visible([vct3(0, 0, 0), vct3(300, -300, 100)])
    assert not trk.is_visible([vct3(0, 0, 700)])        # beyond 2200mm
    assert not trk.is_visible([vct3(0, 0, -700)])       # closer than 1000mm
    assert not trk.is_visible([vct3(900, 0, 0)])        # outside the 1600mm-deep cross-section


def test_systematic_field_respects_handout_bounds():
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)), seed=0)
    field = trk.systematic_error
    assert field(vct3()).norm() == 0.0
    rng = np.random.default_rng(1)
    for _ in range(300):
        z = rng.uniform(trk.FRUSTUM_Z_NEAR, trk.FRUSTUM_Z_FAR)
        half = trk.FRUSTUM_HALF_WIDTH_NEAR + (z - trk.FRUSTUM_Z_NEAR) / 1200.0 * 500.0
        v = vct3(rng.uniform(-half, half), rng.uniform(-half, half), z)
        d = field(v)
        assert d.norm() <= constants.TRACKER_SYS_MAX_MM + 1e-9
        J = np.column_stack([(field(v + vct3(*e)) - d).vec.ravel() / 1e-3 for e in np.eye(3) * 1e-3])
        assert np.linalg.norm(J, 2) <= constants.TRACKER_SYS_GRAD_MAX
    # the field is a property of the camera: same input, same output, but different cameras differ
    assert np.allclose(field(vct3(100, 50, 1500)).vec, field(vct3(100, 50, 1500)).vec)
    other = simulation.create_tracker("trk2", Frame(Rot(), vct3(0, 0, -1600)), seed=1).systematic_error
    assert not np.allclose(field(vct3(100, 50, 1500)).vec, other(vct3(100, 50, 1500)).vec)


def test_sample_returns_frame_near_truth_and_stores_on_body():
    clock.reset()
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    mb = simulation.define_marker_body("mb", square_body())
    placed = simulation.place_marker_body(mb, Frame(Rot('z', 0.3), vct3(100, 50, 0)))
    simulation.add_marker_bodies(trk, [placed])
    obs = simulation.sample_marker_bodies(trk)
    assert obs["mb"] is not None and len(obs["mb"].markers) == 4
    assert simulation.most_recent_observed_frame(mb) is obs["mb"].frame
    # the body's pose relative to the tracker is close to truth up to the ~5mm jiggle / manufacturing error
    F_true_rel = trk.nominal_pose().inv() * placed.actual_placement
    assert (F_true_rel.p - obs["mb"].frame.p).norm() < 30.0


def test_no_jiggle_frames_are_in_the_nominal_tracker_frame():
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    a = simulation.place_marker_body(simulation.define_marker_body("a", square_body()), Frame(Rot('y', 0.2), vct3(50, 0, 0)))
    a.body.actual_marker_positions = None  # defeat manufacturing error for this test
    a.body.set_manufacturing_cov(np.zeros((3, 3)))
    simulation.add_marker_bodies(trk, [a])
    truth = trk.nominal_pose().inv() * a.actual_placement
    for _ in range(5):
        obs = simulation.sample_marker_bodies(trk)["a"].frame
        assert (obs.p - truth.p).norm() < 0.5   # sigma_b = 0.1mm plus the <= 0.1mm systematic field, no jiggle
        assert np.allclose(obs.R.matrix, truth.R.matrix, atol=5e-3)


def test_invisible_body_returns_none():
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    far = simulation.place_marker_body(simulation.define_marker_body("far", square_body()), Frame(Rot(), vct3(0, 0, 900)))
    simulation.add_marker_bodies(trk, [far])
    assert simulation.sample_marker_bodies(trk)["far"] is None
    assert simulation.most_recent_observed_markers(far.body) is None


def test_update_marker_body_returns_previous_and_keeps_truth():
    mb = simulation.define_marker_body("mb", square_body())
    truth_before = [a.p.vec.copy() for a in mb.get_actual_marker_positions()]
    new = [p + vct3(1, 0, 0) for p in mb.nominal_marker_positions]
    prev = simulation.update_marker_body(mb, new)
    assert prev[1].x == 60.0 and mb.nominal_marker_positions[1].x == 61.0
    assert all(np.allclose(a.p.vec, t) for a, t in zip(mb.get_actual_marker_positions(), truth_before))
    with pytest.raises(ValueError):
        simulation.update_marker_body(mb, new[:3])


def test_add_remove_marker_bodies():
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(0, 0, -1600)))
    a = simulation.place_marker_body(simulation.define_marker_body("a", square_body()), Frame.eye())
    assert simulation.add_marker_bodies(trk, [a]) == []
    assert simulation.remove_marker_bodies(trk, [a]) == [a]
    assert trk.tracked_bodies == []


# ---------------------------------------------------------------- calibration body

def test_calibration_body_rules():
    rob = make_robot()
    with pytest.raises(ValueError):
        simulation.create_robot_calibration_body(rob, [vct3(0, 0, 0), vct3(80, 0, 0), vct3(0, 80, 0)])
    mbee = simulation.create_robot_calibration_body(rob, square_body(50))
    with pytest.raises(RuntimeError):
        simulation.create_robot_calibration_body(rob, square_body(50))
    # a*_k == a_k exactly, and the definition lives on the robot
    assert rob.calibration_body is mbee
    assert all(np.allclose(a.p.vec, n.vec) for a, n in zip(mbee.body.get_actual_marker_positions(), mbee.body.nominal_marker_positions))


def test_calibration_body_follows_robot_and_detaches():
    rob = make_robot(seed=2)
    mbee = simulation.create_robot_calibration_body(rob, square_body(50))
    trk = simulation.create_tracker("trk", Frame(Rot(), vct3(600, 0, -1700)), [mbee])
    rob.move_joints([100, 0, 0, 0, 0, 0])
    assert np.allclose(rob.state().theta, [100, 0, 0, 0, 0, 0])
    assert np.allclose(mbee.actual_placement.p.vec, (rob._truth.F_rob_actual * rob._actual_fwd_kins(rob._theta_star)).p.vec)
    assert simulation.sample_marker_bodies(trk)["mBee"] is not None
    simulation.detach_calibration_marker(rob)
    assert simulation.sample_marker_bodies(trk)["mBee"] is None
    simulation.attach_calibration_marker(rob)
    assert simulation.sample_marker_bodies(trk)["mBee"] is not None


def test_actual_frame_of_body_equals_placement_for_exact_body():
    rob = make_robot()
    mbee = simulation.create_robot_calibration_body(rob, square_body(50))
    F = simulation.actual_frame_of_body(mbee)
    assert np.allclose(F.R.matrix, mbee.actual_placement.R.matrix, atol=1e-9)
    assert np.allclose(F.p.vec, mbee.actual_placement.p.vec, atol=1e-9)
