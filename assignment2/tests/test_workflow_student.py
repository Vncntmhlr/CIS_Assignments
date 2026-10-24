"""Student-owned Step 6 checks; original TA assertions are preserved."""

from types import SimpleNamespace

import numpy as np
import pytest

from src import Frame, Rot, simulation, vct3
from src.student import registration, workflow


@pytest.fixture(autouse=True)
def reset_clock():
    simulation.clock.reset()
    yield
    simulation.clock.reset()


def test_home_offset_sign_average_and_reused_encoder_buffer():
    readings = np.array([[0.1, -0.2, 0.3, -0.004, 0.005, -0.006],
                         [0.3, -0.4, 0.5, -0.006, 0.007, -0.008]])
    buffer = np.zeros(6)
    calls = []

    def home_robot():
        buffer[:] = readings[len(calls)]
        calls.append(True)
        simulation.clock.advance(0.05)
        return buffer

    result = workflow.measure_home_offsets(SimpleNamespace(home_robot=home_robot), n_homings=2)
    assert result.shape == (6,)
    assert np.allclose(result, -readings.mean(axis=0), atol=1e-15, rtol=0)
    assert len(calls) == 2
    assert np.isclose(simulation.clock.elapsed, 0.1)


@pytest.mark.parametrize("count", [0, -1, 1.5, True])
def test_home_rejects_invalid_count_without_moving(count):
    with pytest.raises(ValueError):
        workflow.measure_home_offsets(SimpleNamespace(), n_homings=count)
    assert simulation.clock.elapsed == 0


@pytest.mark.parametrize("encoders", [np.zeros(5), np.full(6, np.nan)])
def test_home_rejects_invalid_encoder_reading(encoders):
    with pytest.raises(ValueError):
        workflow.measure_home_offsets(SimpleNamespace(home_robot=lambda: encoders), 1)


def test_collection_pairs_reported_encoders_skips_missing_and_counts_all_time(monkeypatch):
    # Minimal public interface only; requested commands intentionally differ
    # from the reported state. State and tracker buffers are reused in place.
    encoder_buffer = np.zeros(6)
    commands, snapshots = [], []
    controller_offset = np.array([0.2, -0.1, 0.3, 0.001, -0.002, 0.003])

    def move_joints(command):
        commands.append(command.copy())
        encoder_buffer[:] = command + controller_offset
        simulation.clock.advance(0.7)

    robot = SimpleNamespace(move_joints=move_joints,
                            state=lambda: SimpleNamespace(theta=encoder_buffer))
    placed = SimpleNamespace(body=SimpleNamespace(name='calibration'))
    tracker = object()
    shared_frame = Frame.eye()
    outcomes = ['valid', 'none', 'absent', 'valid', 'valid', 'none']

    def sample_marker_bodies(trk):
        assert trk is tracker
        i = len(snapshots)
        snapshots.append(encoder_buffer.copy())
        simulation.clock.advance(simulation.constants.SAMPLE_DELTA_T_SEC)
        if outcomes[i] == 'none':
            return {'calibration': None}
        if outcomes[i] == 'absent':
            return {}
        shared_frame.R.matrix[:] = Rot('z', 0.01 * i).matrix
        shared_frame.p.vec[:, 0] = [encoder_buffer[0], i, 1600]
        return {'calibration': simulation.ObservedBody(shared_frame, [])}

    monkeypatch.setattr(simulation, 'sample_marker_bodies', sample_marker_bodies)
    poses = [np.zeros(6), np.ones(6)]
    simulation.clock.advance(7.0)  # Collection must exclude previous work.
    data = workflow.collect_calibration_data(robot, tracker, placed, poses, repeats=3)
    assert len(commands) == 2 and len(snapshots) == 6
    assert np.array_equal(commands, poses)
    assert len(data) == len(data.frames) == 3
    for theta, frame, index in zip(data.thetas, data.frames, [0, 3, 4]):
        assert np.array_equal(theta, snapshots[index])
        assert np.allclose(frame.R.matrix, Rot('z', 0.01 * index).matrix)
        assert np.array_equal(frame.p.vec.ravel(), [snapshots[index][0], index, 1600])
    assert np.isclose(data.t_collect, 2 * 0.7 + 6 * 0.03)
    assert np.isclose(simulation.clock.elapsed, 7.0 + data.t_collect)
    assert np.array_equal(poses, [np.zeros(6), np.ones(6)])


@pytest.mark.parametrize("repeats", [0, -1, 1.5, True])
def test_collection_rejects_invalid_repeat_count(repeats):
    with pytest.raises(ValueError):
        workflow.collect_calibration_data(None, None, None, [], repeats=repeats)
    assert simulation.clock.elapsed == 0


@pytest.mark.parametrize("bad_pose", [np.zeros(5), np.full(6, np.inf)])
def test_collection_validates_poses_before_moving(bad_pose):
    with pytest.raises(ValueError):
        workflow.collect_calibration_data(None, None, None, [np.zeros(6), bad_pose])
    assert simulation.clock.elapsed == 0


def test_collection_empty_plan_is_empty_and_costs_no_time():
    data = workflow.collect_calibration_data(None, None, None, [])
    assert len(data) == len(data.frames) == 0
    assert data.t_collect == simulation.clock.elapsed == 0


def build_scene():
    robot = simulation.create_robot('workflow_robot', Frame(Rot(), vct3(-800, 0, 1300)), seed=61)
    tracker = simulation.create_tracker('workflow_tracker', Frame.eye(), seed=61)
    s = 37.5
    markers = [vct3(s, s, s), vct3(s, -s, -s), vct3(-s, s, -s), vct3(-s, -s, s)]
    placed = simulation.create_robot_calibration_body(robot, markers)
    simulation.add_marker_bodies(tracker, [placed])
    return robot, tracker, placed


def test_collection_with_public_simulator_and_detached_body(monkeypatch):
    rng = np.random.default_rng(61)
    monkeypatch.setattr(np.random, 'multivariate_normal', rng.multivariate_normal)
    robot, tracker, placed = build_scene()
    poses = [np.array([300, 0, 300, 0, 0, 0]), np.array([350, 20, 320, 0.1, 0.2, -0.1])]
    previous = robot.state().theta
    expected_time = 0.0
    for pose in poses:
        expected_time += robot.motion_time(previous, pose) + 2 * simulation.constants.SAMPLE_DELTA_T_SEC
        previous = pose
    data = workflow.collect_calibration_data(robot, tracker, placed, poses, repeats=2)
    assert len(data) == len(data.frames) == 4
    assert np.array_equal(data.thetas, np.repeat(poses, 2, axis=0))
    assert np.isclose(data.t_collect, expected_time)
    assert np.isclose(data.t_collect, simulation.clock.elapsed)
    assert all(np.all(np.isfinite(frame.p.vec)) for frame in data.frames)

    # There is a previous valid cached observation. Detachment must not reuse it.
    assert simulation.most_recent_observed_frame(placed.body) is not None
    simulation.detach_calibration_marker(robot)
    missing = workflow.collect_calibration_data(robot, tracker, placed, [poses[-1]], repeats=2)
    assert len(missing) == len(missing.frames) == 0
    assert np.isclose(missing.t_collect, robot.t_settle + 2 * simulation.constants.SAMPLE_DELTA_T_SEC)
    with pytest.raises(ValueError):
        workflow.estimate_base_frame(missing, robot)


def test_base_estimation_composition_order_and_reuses_frame_average(monkeypatch):
    base = Frame(Rot('x', 0.4) * Rot('z', -0.3), vct3(30, -40, 1500))
    nominal_poses = [Frame(Rot('y', 0.5), vct3(600, 20, 10)),
                     Frame(Rot('z', -0.7), vct3(750, -100, 200))]
    candidates = []
    data = workflow.CalibrationData()
    for i, nominal in enumerate(nominal_poses):
        theta = np.zeros(6)
        theta[0] = i
        data.thetas.append(theta)
        data.frames.append(base * nominal)

    def fwd_kins(theta):
        return nominal_poses[int(theta[0])]

    original_average = registration.average_frames

    def average_frames(frames):
        candidates.extend(frames)
        return original_average(frames)

    monkeypatch.setattr(registration, 'average_frames', average_frames)
    result = workflow.estimate_base_frame(data, SimpleNamespace(fwd_kins=fwd_kins))
    assert len(candidates) == 2
    for candidate in [*candidates, result]:
        assert np.allclose(candidate.R.matrix, base.R.matrix, atol=1e-12, rtol=0)
        assert np.allclose(candidate.p.vec, base.p.vec, atol=1e-12, rtol=0)
    assert simulation.clock.elapsed == 0


def test_base_estimation_noisy_candidates_uses_projected_rotation_mean():
    base = Frame(Rot('x', 0.3), vct3(10, 20, 30))
    nominal = Frame(Rot('y', -0.4), vct3(600, 0, 300))
    candidates = [base * Frame(Rot('z', angle), vct3(offset, 0, 0))
                  for angle, offset in [(-0.1, -2), (0.1, 2)]]
    data = workflow.CalibrationData(thetas=[np.zeros(6)] * 2,
                                    frames=[candidate * nominal for candidate in candidates])
    result = workflow.estimate_base_frame(data, SimpleNamespace(fwd_kins=lambda _: nominal))
    assert np.allclose(result.R.matrix, base.R.matrix, atol=1e-12, rtol=0)
    assert np.allclose(result.p.vec, base.p.vec, atol=1e-12, rtol=0)
    assert np.isclose(np.linalg.det(result.R.matrix), 1.0)


@pytest.mark.parametrize("thetas,frames", [
    ([], []),
    ([np.zeros(6)], []),
    ([], [Frame.eye()]),
    ([np.zeros(5)], [Frame.eye()]),
    ([np.full(6, np.nan)], [Frame.eye()]),
    ([np.zeros(6)], [None]),
])
def test_base_estimation_rejects_invalid_samples(thetas, frames):
    data = workflow.CalibrationData(thetas=thetas, frames=frames)
    with pytest.raises(ValueError):
        workflow.estimate_base_frame(data, SimpleNamespace())
