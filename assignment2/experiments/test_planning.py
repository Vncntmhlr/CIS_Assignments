"""Student-owned Step 7 design checks; no TA assertions are modified."""

from collections import Counter

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src import Frame, Rot, simulation, vct3
from src.student import workflow
from uncertainty_networks.se3 import exp_se3, exp_so3


def scene(seed=1):
    robot = simulation.create_robot('planning', Frame(Rot(), vct3(-800, 0, 1300)), seed=seed)
    tracker = simulation.create_tracker('camera', Frame.eye(), seed=seed)
    s = 37.5
    markers = [vct3(s, s, s), vct3(s, -s, -s), vct3(-s, s, -s), vct3(-s, -s, s)]
    placed = simulation.create_robot_calibration_body(robot, markers)
    simulation.add_marker_bodies(tracker, [placed])
    return robot, tracker, markers, placed


@pytest.fixture(autouse=True)
def reset_clock():
    simulation.clock.reset()
    yield
    simulation.clock.reset()


def plan(robot, tracker, markers, accuracy=1e-3, **kwargs):
    return workflow.plan_calibration_poses(robot, tracker, markers, accuracy, **kwargs)


def test_default_plan_geometry_spacing_repetitions_and_no_side_effects():
    robot, tracker, markers, _ = scene()
    before = robot.state().theta.copy()
    rng_before = np.random.get_state()
    poses = np.asarray(plan(robot, tracker, markers))
    assert poses.shape == (765, 6)
    counts = Counter(map(tuple, poses))
    assert len(counts) == 69
    assert counts[(0.,) * 6] == 17  # 11 initial readings + six single returns.
    assert set(count for theta, count in counts.items() if any(theta)) == {11}
    assert np.array_equal(poses[0], np.zeros(6))
    assert np.array_equal(poses[-1], np.zeros(6))
    assert np.all(np.count_nonzero(poses, axis=1) <= 1)
    assert np.allclose(poses.min(axis=0), robot.theta_min)
    assert np.allclose(poses.max(axis=0), robot.theta_max)
    for k in range(6):
        differences = np.diff(np.unique(poses[:, k]))
        assert differences.max() <= (37.5 + 1e-10 if k < 3 else np.pi / 4 + 1e-10)
    # A closed single-axis traversal must cover each range twice. This
    # verifies the minimum travel distance within the Method 1 sweep family.
    assert np.allclose(np.abs(np.diff(poses, axis=0)).sum(axis=0),
                       2 * (robot.theta_max - robot.theta_min))
    for theta in counts:
        theta = np.array(theta)
        assert robot.within_limits(theta)
        frame = robot.F_rob * robot.fwd_kins(theta)
        assert tracker.is_visible([frame * p for p in markers])
    assert simulation.clock.elapsed == 0
    assert np.array_equal(robot.state().theta, before)
    rng_after = np.random.get_state()
    assert rng_before[0] == rng_after[0]
    assert np.array_equal(rng_before[1], rng_after[1])
    assert rng_before[2:] == rng_after[2:]


def test_tighter_accuracy_increases_sampling_and_repetitions():
    robot, tracker, markers, _ = scene()
    baseline = np.asarray(plan(robot, tracker, markers))
    tighter = np.asarray(plan(robot, tracker, markers, accuracy=5e-4))
    assert len(tighter) > len(baseline)
    assert len(np.unique(tighter, axis=0)) > len(np.unique(baseline, axis=0))
    assert max(Counter(map(tuple, tighter)).values()) >= 44


def test_marker_geometry_changes_required_repetitions():
    robot, tracker, markers, _ = scene()
    baseline = plan(robot, tracker, markers)
    spread = [p * (75 / p.norm()) for p in markers]
    larger = plan(robot, tracker, spread)
    assert len(larger) < len(baseline)
    assert max(count for theta, count in Counter(map(tuple, larger)).items() if any(theta)) == 9


@pytest.mark.parametrize('accuracy', [0, -1e-3, np.nan, np.inf])
def test_invalid_accuracy_is_rejected(accuracy):
    robot, tracker, markers, _ = scene()
    with pytest.raises(ValueError):
        plan(robot, tracker, markers, accuracy=accuracy)
    assert simulation.clock.elapsed == 0


@pytest.mark.parametrize('markers', [[], [vct3()] * 4,
                                   [vct3(0, 0, 0), vct3(1, 0, 0), vct3(2, 0, 0)],
                                   [vct3(np.nan, 0, 0)] * 4,
                                   [vct3(100, 0, 0), vct3(0, 1, 0), vct3(0, 0, 1)]])
def test_invalid_marker_geometry_is_rejected(markers):
    robot, tracker, _, _ = scene()
    with pytest.raises(ValueError):
        plan(robot, tracker, markers)


def test_invisible_scene_fails_without_silently_removing_required_poses():
    robot, _, markers, _ = scene()
    tracker = simulation.create_tracker('misplaced_camera', Frame(Rot(), vct3(0, 0, 5000)), seed=1)
    with pytest.raises(ValueError, match='not visible'):
        plan(robot, tracker, markers)
    assert simulation.clock.elapsed == 0


def test_planning_resource_limit_does_not_silently_cap_requested_accuracy():
    robot, tracker, markers, _ = scene()
    with pytest.raises(ValueError, match='planning limit'):
        plan(robot, tracker, markers, accuracy=1e-5)


@pytest.mark.parametrize('seed', [1, 7, 61])
def test_public_collection_matches_predicted_time_and_preserves_all_samples(seed, monkeypatch):
    rng = np.random.default_rng(700 + seed)
    monkeypatch.setattr(np.random, 'multivariate_normal', rng.multivariate_normal)
    robot, tracker, markers, placed = scene(seed)
    # Match the public commanded state at the end of the Step 6 baseline.
    robot.move_joints(np.array([300., 0., 300., 0., 0., 1.5]))
    poses = plan(robot, tracker, markers)
    previous = robot.state().theta
    motion_time = 0.0
    for theta in poses:
        motion_time += robot.motion_time(previous, theta)
        previous = theta
    predicted = motion_time + len(poses) * simulation.constants.SAMPLE_DELTA_T_SEC
    assert np.isclose(predicted, 469.8592653589808)
    start = simulation.clock.elapsed
    data = workflow.collect_calibration_data(robot, tracker, placed, poses)
    assert len(data) == len(data.frames) == len(poses) == 765
    assert np.array_equal(data.thetas, poses)
    assert np.isclose(data.t_collect, predicted, atol=1e-8)
    assert np.isclose(simulation.clock.elapsed - start, predicted, atol=1e-8)
    assert np.array_equal(robot.state().theta, np.zeros(6))


def example_model_pose(theta, parameters):
    """Independent rank fixture for the TA's example model, not Step 8 code.

    Home offsets are fixed; five axes have two perpendicular tilt components;
    all six joint maps have normalized linear/quadratic corrections. No
    hidden robot axes or other ground-truth APIs enter this diagnostic.
    """
    axes = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]] * 2)
    perpendicular = [(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)]
    for k, basis in enumerate(perpendicular, 1):
        phi = np.zeros(3)
        phi[list(basis)] = parameters[6 + 2 * (k - 1):8 + 2 * (k - 1)]
        axes[k] = exp_so3(phi) @ axes[k]
    scales = np.array([600., 300., 600., .75 * np.pi, .75 * np.pi, np.pi])
    joints = theta + np.sum(parameters[22:].reshape(6, 2)
                            * (theta / scales)[:, None] ** [1, 2], axis=1)
    frame = np.eye(4)
    frame[:3, 3] = joints[:3] @ axes[:3] + [500., 0., 0.]
    for k in range(3, 6):
        frame[:3, :3] = frame[:3, :3] @ exp_so3(axes[k] * joints[k])
    tool = np.eye(4)
    tool[0, 3] = 100.
    return exp_se3(parameters[:6]) @ frame @ tool @ exp_se3(parameters[16:22])


def test_prospective_step8_local_rank_has_only_the_expected_gauge():
    robot, tracker, markers, _ = scene()
    poses = np.unique(plan(robot, tracker, markers), axis=0)
    scales = np.r_[[.01] * 3, [1.] * 3, [.01] * 10, [.01] * 3, [.5] * 3,
                   np.repeat([.2, .2, .2, .01, .01, .01], 2)]
    columns = []
    for i in range(34):
        change = np.zeros(34)
        change[i] = 1e-5 * scales[i]
        column = []
        for theta in poses:
            plus = example_model_pose(theta, change)
            minus = example_model_pose(theta, -change)
            angular = Rotation.from_matrix(minus[:3, :3].T @ plus[:3, :3]).as_rotvec()
            position = plus[:3, 3] - minus[:3, 3]
            column.extend(np.r_[angular / .001, position / .2] / (2e-5))
        columns.append(column)
    jacobian = np.array(columns).T
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    assert np.count_nonzero(singular_values > singular_values[0] * 1e-7) == 33
    # Fix base rotation about the exact d_1 axis; the other 33 are locally
    # identifiable. An actual Step 8 model must repeat this audit after fitting.
    fixed = np.linalg.svd(np.delete(jacobian, 0, axis=1), compute_uv=False)
    assert fixed[-1] > fixed[0] * 1e-7
    assert np.isclose(fixed[-1], 0.3611455504, rtol=1e-5)
