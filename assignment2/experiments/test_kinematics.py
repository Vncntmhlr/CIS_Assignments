"""Student model, fitting and identifiability regressions (Steps 8 A–C)."""


# test_kinematics_phase_a
import numpy as np
import pytest
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

from src import Frame, Rot, vct3
from src.robot import Robot
from src.simulation import clock, constants
from src.student.kinematics import ImprovedKinematics


def nominal_robot():
    # No sampling, homing, motion, or hidden simulator data is needed here.
    return Robot('phase_a', Frame.eye(), constants.THETA_MIN, constants.THETA_MAX,
                 constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)


def matrix(frame):
    result = np.eye(4)
    result[:3, :3] = frame.R.matrix
    result[:3, 3] = frame.p.vec.ravel()
    return result


def twist_matrix(xi):
    # Independent matrix exponential oracle, rather than exp_se3/exp_so3.
    x, y, z = xi[:3]
    result = np.zeros((4, 4))
    result[:3, :3] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
    result[:3, 3] = xi[3:]
    return result


@pytest.mark.parametrize('degree', [0, 1, 2, 3])
def test_zero_parameters_reproduce_nominal_over_full_joint_ranges(degree):
    robot = nominal_robot()
    base = Frame(Rot('z', 0.4), vct3(10, 20, 30))
    model = ImprovedKinematics(robot, base, n_poly=degree)
    assert model.params.shape == (22 + 6 * degree,)
    assert np.array_equal(model.params, np.zeros_like(model.params))
    rng = np.random.default_rng(808)
    for theta in rng.uniform(robot.theta_min, robot.theta_max, size=(100, 6)):
        assert np.allclose(matrix(model.fwd_kins(theta)), matrix(robot.fwd_kins(theta)),
                           atol=1e-12, rtol=0)
        assert np.allclose(matrix(model.predict_in_tracker(theta)),
                           matrix(base * robot.fwd_kins(theta)), atol=1e-12, rtol=0)
    fresh = model.initial_params()
    fresh[0] = 1
    assert model.params[0] == 0


def test_base_correction_is_right_composed_in_initial_base_coordinates():
    base = Frame(Rot('z', np.pi / 2), vct3(10, 20, 30))
    model = ImprovedKinematics(nominal_robot(), base)
    params = model.initial_params()
    params[3] = 1.0  # +1 mm along the initial base's x, which is tracker +y.
    assert np.allclose(model.base_frame(params).p.vec.ravel(), [10, 21, 30], atol=1e-12)
    params[:6] = [0.02, -0.03, 0.01, 0.3, -0.4, 0.5]
    expected = matrix(base) @ expm(twist_matrix(params[:6]))
    wrong_order = expm(twist_matrix(params[:6])) @ matrix(base)
    assert not np.allclose(expected, wrong_order)
    assert np.allclose(matrix(model.base_frame(params)), expected, atol=1e-12, rtol=0)
    assert np.array_equal(model.params, np.zeros(34))


@pytest.mark.parametrize('axis_index,component', [(k, j) for k in range(1, 6) for j in range(2)])
def test_each_perpendicular_axis_tilt_has_the_expected_geometry(axis_index, component):
    robot = nominal_robot()
    model = ImprovedKinematics(robot, Frame.eye())
    theta, params = np.zeros(6), model.initial_params()
    tilt = 0.02
    params[6 + 2 * (axis_index - 1) + component] = tilt
    perpendicular = [(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)]
    rotation_vector = np.zeros(3)
    rotation_vector[perpendicular[axis_index - 1][component]] = tilt
    axis = Rotation.from_rotvec(rotation_vector).apply(np.eye(3)[axis_index % 3])
    if axis_index < 3:
        theta[axis_index] = 100.0
        expected_position = np.array([600., 0., 0.]) + 100 * axis
        expected_rotation = np.eye(3)
    else:
        theta[axis_index] = 0.6
        expected_rotation = Rotation.from_rotvec(0.6 * axis).as_matrix()
        expected_position = np.array([500., 0., 0.]) + expected_rotation @ [100., 0., 0.]
    actual = model.fwd_kins(theta, params)
    assert np.allclose(actual.R.matrix, expected_rotation, atol=1e-12, rtol=0)
    assert np.allclose(actual.p.vec.ravel(), expected_position, atol=1e-12, rtol=0)
    # The inherited nominal axes must never be mutated by a tilted model.
    assert np.array_equal([axis.vec.ravel() for axis in robot.AXES], np.tile(np.eye(3), (2, 1)))


def test_home_offsets_are_added_once_and_variable_corrections_vanish_at_home():
    robot = nominal_robot()
    offsets = np.array([0.1, -0.2, 0.3, 0.001, -0.002, 0.003])
    model = ImprovedKinematics(robot, Frame.eye(), joint_offsets=offsets)
    theta = np.array([300., -150., 250., 0.3, -0.4, 0.5])
    assert np.allclose(matrix(model.fwd_kins(theta)), matrix(robot.fwd_kins(theta + offsets)),
                       atol=1e-12, rtol=0)
    params = model.initial_params()
    params[22:] = np.random.default_rng(809).normal(scale=0.01, size=12)
    assert np.allclose(matrix(model.fwd_kins(-offsets, params)),
                       matrix(robot.fwd_kins(np.zeros(6))), atol=1e-12, rtol=0)


def test_normalized_linear_and_quadratic_distortion_has_joint_units():
    robot = nominal_robot()
    offsets = np.array([0.1, -0.2, 0.3, 0.001, -0.002, 0.003])
    model = ImprovedKinematics(robot, Frame.eye(), joint_offsets=offsets)
    theta = np.array([300., -150., 250., 0.3, -0.4, 0.5])
    coefficients = np.array([[0.12, -0.03]] * 3 + [[0.002, -0.001]] * 3)
    params = model.initial_params()
    params[22:] = coefficients.ravel()
    u = theta + offsets
    normalized = u / np.maximum(np.abs(robot.theta_min), np.abs(robot.theta_max))
    effective = u + coefficients[:, 0] * normalized + coefficients[:, 1] * normalized ** 2
    assert np.allclose(matrix(model.fwd_kins(theta, params)), matrix(robot.fwd_kins(effective)),
                       atol=1e-12, rtol=0)


def test_tool_correction_is_right_composed_after_the_wrist():
    robot = nominal_robot()
    model = ImprovedKinematics(robot, Frame.eye())
    params = model.initial_params()
    params[16:19] = [0, 0, 0.02]
    at_home = model.fwd_kins(np.zeros(6), params)
    # Rotating the EE frame about its own origin must not rotate the 100 mm
    # wrist-to-EE translation around the wrist center.
    assert np.allclose(at_home.p.vec.ravel(), [600, 0, 0], atol=1e-12)
    theta = np.array([100., 20., 300., 0.3, -0.5, 0.7])
    params[16:22] = [0.02, -0.01, 0.03, 0.2, -0.3, 0.4]
    expected = matrix(robot.fwd_kins(theta)) @ expm(twist_matrix(params[16:22]))
    assert np.allclose(matrix(model.fwd_kins(theta, params)), expected, atol=1e-12, rtol=0)


def test_all_corrections_preserve_so3_and_do_not_change_simulated_time():
    robot = nominal_robot()
    model = ImprovedKinematics(robot, Frame(Rot('y', 0.4), vct3(10, 20, 30)))
    rng = np.random.default_rng(810)
    started = clock.elapsed
    for _ in range(100):
        params = rng.normal(scale=0.005, size=34)
        theta = rng.uniform(robot.theta_min, robot.theta_max)
        for frame in [model.base_frame(params), model.fwd_kins(theta, params),
                      model.base_frame(params) * model.fwd_kins(theta, params)]:
            rotation = frame.R.matrix
            assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
            assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12, rtol=0)
    assert clock.elapsed == started


def test_combined_model_matches_independent_matrix_exponential_reference():
    robot = nominal_robot()
    offsets = np.array([0.1, -0.2, 0.3, 0.001, -0.002, 0.003])
    base = Frame(Rot('z', 0.2), vct3(10, 20, 30))
    model = ImprovedKinematics(robot, base, joint_offsets=offsets)
    rng = np.random.default_rng(811)
    errors = []
    for _ in range(100):
        theta = rng.uniform(robot.theta_min, robot.theta_max)
        params = rng.normal(scale=0.005, size=34)
        axes = np.tile(np.eye(3), (2, 1))
        for k, components in enumerate([(0, 2), (0, 1), (1, 2), (0, 2), (0, 1)], 1):
            phi = np.zeros(3)
            phi[list(components)] = params[6 + 2 * (k - 1):8 + 2 * (k - 1)]
            axes[k] = expm(twist_matrix(np.r_[phi, np.zeros(3)]))[:3, :3] @ axes[k]
        u = theta + offsets
        x = u / np.maximum(np.abs(robot.theta_min), np.abs(robot.theta_max))
        coefficients = params[22:].reshape(6, 2)
        q = u + coefficients[:, 0] * x + coefficients[:, 1] * x ** 2
        expected = matrix(base) @ expm(twist_matrix(params[:6]))
        translation = np.eye(4)
        translation[:3, 3] = q[:3] @ axes[:3]
        expected = expected @ translation @ matrix(robot.F_CW)
        for k in range(3, 6):
            expected = expected @ expm(twist_matrix(np.r_[q[k] * axes[k], np.zeros(3)]))
        expected = expected @ matrix(robot.F_WE) @ expm(twist_matrix(params[16:22]))
        actual = matrix(model.base_frame(params) * model.fwd_kins(theta, params))
        angle = Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude()
        distance = np.linalg.norm(expected[:3, 3] - actual[:3, 3])
        orthogonality = np.max(np.abs(actual[:3, :3].T @ actual[:3, :3] - np.eye(3)))
        errors.append((angle, distance, orthogonality))
        assert angle < 1e-12 and distance < 1e-9 and orthogonality < 1e-12
    maximum = np.max(errors, axis=0)
    print(f"100 combined poses, seed 811: max angle={maximum[0]:.3e} rad, "
          f"position={maximum[1]:.3e} mm, orthogonality={maximum[2]:.3e}")


def test_explicit_parameters_override_default_without_mutating_inputs():
    model = ImprovedKinematics(nominal_robot(), Frame.eye())
    model.params[3] = 2.0
    model.params[22] = 0.1
    theta = np.array([300., 0., 300., 0.1, 0.2, 0.3])
    values = model.initial_params()
    saved_theta, saved_values, saved_default = theta.copy(), values.copy(), model.params.copy()
    assert np.allclose(model.base_frame(values).p.vec, np.zeros((3, 1)))
    assert np.allclose(matrix(model.fwd_kins(theta, values)), matrix(nominal_robot().fwd_kins(theta)))
    assert np.array_equal(theta, saved_theta)
    assert np.array_equal(values, saved_values)
    assert np.array_equal(model.params, saved_default)


@pytest.mark.parametrize('degree', [-1, 1.5, True])
def test_invalid_polynomial_degree_is_rejected(degree):
    with pytest.raises(ValueError):
        ImprovedKinematics(nominal_robot(), Frame.eye(), n_poly=degree)


def test_nonfinite_home_offsets_are_rejected():
    with pytest.raises(ValueError):
        ImprovedKinematics(nominal_robot(), Frame.eye(), joint_offsets=np.full(6, np.nan))


@pytest.mark.parametrize('params', [np.zeros(33), np.zeros((34, 1)), np.full(34, np.inf)])
def test_invalid_parameter_vectors_are_rejected(params):
    model = ImprovedKinematics(nominal_robot(), Frame.eye())
    with pytest.raises(ValueError):
        model.base_frame(params)
    with pytest.raises(ValueError):
        model.fwd_kins(np.zeros(6), params)


@pytest.mark.parametrize('theta', [np.zeros(5), np.zeros((6, 1)), np.full(6, np.nan)])
def test_invalid_encoder_vectors_are_rejected(theta):
    model = ImprovedKinematics(nominal_robot(), Frame.eye())
    with pytest.raises(ValueError):
        model.fwd_kins(theta)


# test_kinematics_phase_b
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from src import Frame, Rot, vct3
from src.robot import Robot
from src.simulation import clock, constants
from src.student.kinematics import ImprovedKinematics
from src.student.workflow import CalibrationData


def robot():
    return Robot('phase_b', Frame.eye(), constants.THETA_MIN, constants.THETA_MAX,
                 constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)


def synthetic(seed=821, degree=2, n_train=24, n_test=40):
    rng = np.random.default_rng(seed)
    base = Frame(Rot('z', 0.2) * Rot('y', -0.1), vct3(30, -20, 1500))
    offsets = np.array([.03, -.02, .04, .001, -.002, .003])
    truth = ImprovedKinematics(robot(), base, joint_offsets=offsets, n_poly=degree)
    scales = np.r_[[.01] * 3, [1.] * 3, [.01] * 10, [.01] * 3, [.5] * 3,
                   np.repeat([.2] * 3 + [.01] * 3, degree)]
    truth.params = .2 * scales * rng.normal(size=scales.size)
    truth.params[0] = .003  # Different from the fitter's zero-valued gauge.
    fitted = ImprovedKinematics(robot(), base, joint_offsets=offsets, n_poly=degree)
    low = np.array([0., -300., 0., -1.2, -1.2, -2.5])
    high = np.array([600., 300., 600., 1.2, 1.2, 2.5])
    train = CalibrationData()
    for theta in rng.uniform(low, high, size=(n_train, 6)):
        train.thetas.append(theta.copy())
        train.frames.append(truth.predict_in_tracker(theta))
    held_out = rng.uniform(low, high, size=(n_test, 6))
    return truth, fitted, train, held_out


def pose_summary(truth, model, thetas):
    errors = []
    for theta in thetas:
        reference, prediction = truth.predict_in_tracker(theta), model.predict_in_tracker(theta)
        angular = Rotation.from_matrix(reference.R.matrix.T @ prediction.R.matrix).magnitude()
        position = np.linalg.norm(reference.p.vec - prediction.p.vec)
        errors.append([angular, position])
    values = np.array(errors)
    return {'rms_rad': float(np.sqrt(np.mean(values[:, 0] ** 2))),
            'max_rad': float(values[:, 0].max()),
            'rms_mm': float(np.sqrt(np.mean(values[:, 1] ** 2))),
            'max_mm': float(values[:, 1].max())}


@pytest.mark.parametrize('angle', [0.01, -0.01, 1e-10, -1e-10])
def test_residuals_are_signed_weighted_relative_frame_components(angle):
    model = ImprovedKinematics(robot(), Frame(Rot('z', .7), vct3(10, 20, 30)))
    theta = np.array([300., -10., 200., .2, -.1, .3])
    prediction = model.predict_in_tracker(theta)
    delta = Frame(Rot('z', angle), vct3(.2, -.4, .6))
    data = CalibrationData(thetas=[theta], frames=[prediction * delta.inv()])
    result = model.residuals(model.params, data)
    expected = np.array([0., 0., angle / .001, 1., -2., 3.])
    assert result.shape == (6,)
    assert np.allclose(result, expected, atol=1e-9, rtol=0)


def test_residuals_are_invariant_to_a_common_left_transform():
    truth, model, data, _ = synthetic(n_train=6)
    transform = Frame(Rot('x', .4), vct3(100, -200, 300))
    transformed_model = ImprovedKinematics(robot(), transform * model.F_X_init,
                                           joint_offsets=model.joint_offsets)
    transformed = CalibrationData(thetas=data.thetas,
                                   frames=[transform * frame for frame in data.frames])
    assert np.allclose(model.residuals(model.params, data),
                       transformed_model.residuals(transformed_model.params, transformed),
                       atol=1e-9, rtol=0)
    assert np.max(np.abs(truth.residuals(truth.params, data))) < 1e-9


@pytest.mark.parametrize('seed,degree', [(821, 2), (822, 1)])
def test_noiseless_fit_improves_training_and_independent_predictions(seed, degree):
    truth, fitted, train, held_out = synthetic(seed=seed, degree=degree)
    initial_parameters = fitted.params.copy()
    initial_train = pose_summary(truth, fitted, train.thetas)
    initial_test = pose_summary(truth, fitted, held_out)
    initial_cost = .5 * np.sum(fitted.residuals(fitted.params, train) ** 2)
    saved_thetas = np.array(train.thetas).copy()
    saved_positions = np.array([frame.p.vec for frame in train.frames]).copy()
    saved_rotations = np.array([frame.R.matrix for frame in train.frames]).copy()
    started = clock.elapsed
    # Historical fixtures deliberately exceed the physical distortion bounds.
    # Keep this regression of the explicitly available unconstrained solver.
    info = fitted.fit(train, physical_constraints=False)
    final_train = pose_summary(truth, fitted, train.thetas)
    final_test = pose_summary(truth, fitted, held_out)
    assert info['success'] and info['solver_success'] and info['params_updated']
    assert info['initial_cost'] == pytest.approx(initial_cost)
    assert info['final_cost'] < initial_cost * 1e-8
    assert initial_test['max_mm'] > 0.1 and initial_test['max_rad'] > 1e-3
    assert final_test['max_mm'] < initial_test['max_mm'] * 1e-4
    assert final_test['max_rad'] < initial_test['max_rad'] * 1e-4
    for summary in [final_train, final_test]:
        assert summary['max_mm'] < 1e-5 and summary['max_rad'] < 1e-7
    assert info['x'].shape == fitted.params.shape == (22 + 6 * degree,)
    assert info['n_free_params'] == len(fitted.params) - 1
    assert info['fixed_parameter_indices'] == [0]
    assert fitted.params[0] == initial_parameters[0] == 0.
    # Prediction recovery must not be confused with identifying all physical
    # coefficients: the generating model uses another gauge representative.
    assert truth.params[0] == .003
    assert np.linalg.norm(fitted.params - truth.params) > .003
    assert info['iterations'] > 0 and info['nfev'] >= info['iterations']
    assert info['residual_calls'] > info['nfev'] and info['wall_time_s'] > 0
    assert clock.elapsed == started
    assert np.array_equal(saved_thetas, train.thetas)
    assert np.array_equal(saved_positions, [frame.p.vec for frame in train.frames])
    assert np.array_equal(saved_rotations, [frame.R.matrix for frame in train.frames])
    print('PHASE_B_SYNTHETIC', json.dumps({
        'seed': seed, 'degree': degree, 'n_train': len(train), 'n_test': len(held_out),
        'initial_train': initial_train, 'final_train': final_train,
        'initial_test': initial_test, 'final_test': final_test,
        'initial_cost': info['initial_cost'], 'final_cost': info['final_cost'],
        'status': info['status'], 'message': info['message'], 'optimality': info['optimality'],
        'iterations': info['iterations'], 'nfev': info['nfev'], 'njev': info['njev'],
        'residual_calls': info['residual_calls'], 'wall_s': info['wall_time_s'],
        'parameter_difference_norm': float(np.linalg.norm(fitted.params - truth.params))}))


def test_nonconverged_fit_preserves_previous_model():
    _, model, train, _ = synthetic(n_train=8)
    before = model.params.copy()
    info = model.fit(train, max_nfev=1)
    assert not info['success'] and not info['solver_success'] and not info['params_updated']
    assert info['status'] == 0
    assert np.array_equal(model.params, before)


def test_fit_honors_nonzero_starting_gauge_and_explicit_physical_coordinate_bounds():
    truth, fitted, train, _ = synthetic(degree=0, n_train=12)
    # Make the data exactly representable with an explicitly chosen gauge.
    truth.params[0] = fitted.params[0] = .003
    train.frames = [truth.predict_in_tracker(theta) for theta in train.thetas]
    scales = np.r_[[.01] * 3, [1.] * 3, [.01] * 10, [.01] * 3, [.5] * 3]
    lower, upper = -10 * scales, 10 * scales
    lower[0] = upper[0] = .003  # Equality is allowed for the already fixed component.
    info = fitted.fit(train, parameter_scales=scales, bounds=(lower, upper))
    assert info['success'] and fitted.params[0] == .003
    assert info['fixed_parameter_values'] == [.003]
    assert np.all(fitted.params >= lower) and np.all(fitted.params <= upper)


@pytest.mark.parametrize('kind', ['empty', 'unpaired', 'missing_frame', 'nonfinite_frame',
                                 'reflection', 'nonfinite_theta'])
def test_residuals_reject_invalid_measurements(kind):
    model = ImprovedKinematics(robot(), Frame.eye())
    data = CalibrationData(thetas=[np.zeros(6)], frames=[Frame.eye()])
    if kind == 'empty':
        data = CalibrationData()
    elif kind == 'unpaired':
        data.frames = []
    elif kind == 'missing_frame':
        data.frames = [None]
    elif kind == 'nonfinite_frame':
        data.frames = [Frame(Rot(), vct3(np.nan, 0, 0))]
    elif kind == 'reflection':
        data.frames = [Frame(Rot(matrix=np.diag([-1, 1, 1])), vct3())]
    else:
        data.thetas = [np.full(6, np.inf)]
    with pytest.raises(ValueError):
        model.residuals(model.params, data)


def test_fit_rejects_too_few_observations_without_changing_parameters():
    _, model, train, _ = synthetic(n_train=2)
    before = model.params.copy()
    with pytest.raises(ValueError, match='too few'):
        model.fit(train)
    assert np.array_equal(model.params, before)


@pytest.mark.parametrize('options', [{'parameter_scales': np.zeros(34)},
                                    {'parameter_scales': np.ones(33)},
                                    {'bounds': (np.ones(34), np.ones(34) * 2)},
                                    {'unknown_option': 1}])
def test_fit_rejects_invalid_configuration_without_mutation(options):
    _, model, train, _ = synthetic(n_train=6)
    before = model.params.copy()
    with pytest.raises((ValueError, TypeError)):
        model.fit(train, **options)
    assert np.array_equal(model.params, before)


# test_kinematics_phase_c
import numpy as np
import pytest

from src import Frame, Rot, simulation, vct3
from src.robot import Robot
from src.student import workflow
from src.student.kinematics import ImprovedKinematics
from src.student.workflow import CalibrationData


@pytest.fixture(scope='module')
def cases():
    model, _, random_data, _ = synthetic(seed=821)
    controller = Robot('phase_c_plan', Frame(Rot(), vct3(-800, 0, 1300)),
                       model.theta_min, model.theta_max, model.t_settle,
                       model.theta_dot_max, model.theta_ddot_max)
    tracker = simulation.create_tracker('phase_c_camera', Frame.eye(), seed=1)
    s = 37.5
    markers = [vct3(s, s, s), vct3(s, -s, -s), vct3(-s, s, -s), vct3(-s, -s, s)]
    poses = np.unique(workflow.plan_calibration_poses(controller, tracker, markers, 1e-3), axis=0)
    data = CalibrationData(thetas=list(poses), frames=[model.predict_in_tracker(p) for p in poses])
    before = model.params.copy()
    clock_before = simulation.clock.elapsed
    random_report = model.jacobian_rank_report(random_data)
    plan_report = model.jacobian_rank_report(data)
    assert np.array_equal(model.params, before)
    assert simulation.clock.elapsed == clock_before
    return model, random_data, data, random_report, plan_report


@pytest.mark.parametrize('which', [3, 4])
def test_full_and_gauge_fixed_rank_are_stable_and_reported_separately(cases, which):
    report = cases[which]
    assert report['rank'] == report['full']['rank'] == 33
    assert report['n_params'] == report['full']['n_params'] == 34
    assert report['full']['nullity'] == 1
    assert np.isinf(report['full']['condition_number'])
    assert report['reduced']['rank'] == report['reduced']['n_params'] == 33
    assert report['reduced']['nullity'] == 0
    assert np.isfinite(report['reduced']['condition_number'])
    assert np.array_equal(report['reduced']['column_indices'], np.arange(1, 34))
    for result in [report['full'], report['reduced']]:
        assert result['coarse_step_rank'] == result['rank']
        assert result['fd_relative_difference'] < 1e-7
        assert result['rank_threshold'] >= 5 * result['fd_difference_norm']
        for relative_threshold in [1e-8, 1e-7, 1e-6]:
            threshold = max(relative_threshold * result['singular_values'][0],
                            5 * result['fd_difference_norm'])
            assert np.count_nonzero(result['singular_values'] > threshold) == result['rank']


def test_expected_physical_gauge_explains_the_only_null_direction(cases):
    for report in cases[3:]:
        assert report['gauge_verified']
        assert report['gauge_nullspace_alignment'] > 1 - 1e-8
        assert report['gauge_relative_response'] < 1e-7
        assert abs(report['gauge_direction_scaled'][0]) > 0.1


def test_actual_69_pose_design_has_no_additional_null_but_has_weak_modes(cases):
    report = cases[4]
    assert report['n_distinct_poses'] == report['n_samples'] == 69
    singular = report['reduced']['singular_values']
    assert np.isclose(singular[0], 305.124951, rtol=1e-5)
    assert np.isclose(singular[-1], .361120426, rtol=1e-5)
    assert 800 < report['reduced']['condition_number'] < 900
    assert report['reduced']['weak_relative_count'] >= 3
    for mode in report['reduced']['weak_directions']:
        vector = mode['scaled_direction']
        matrix = report['scaled_jacobian'][:, 1:]
        assert np.isclose(np.linalg.norm(vector), 1)
        assert np.isclose(np.linalg.norm(matrix @ vector), mode['singular_value'], rtol=1e-7)
        assert np.allclose(mode['physical_direction'], report['parameter_scales'][1:] * vector)
    # Do not assert vector signs: SVD signs and bases within close clusters
    # are arbitrary. Check the physically relevant dominant coefficient pair.
    weakest_names = [{entry['name'] for entry in mode['dominant_parameters'][:2]}
                     for mode in report['reduced']['weak_directions'][-2:]]
    assert {'joint1_c1', 'joint1_c2'} in weakest_names
    assert {'joint3_c1', 'joint3_c2'} in weakest_names


def test_returned_jacobian_differentiates_the_actual_residual_function(cases):
    model, data, _, report, _ = cases
    rng = np.random.default_rng(831)
    direction = rng.normal(size=34)
    direction /= np.linalg.norm(direction)
    step = 2.5e-5
    delta = step * report['parameter_scales'] * direction
    actual = (model.residuals(model.params + delta, data)
              - model.residuals(model.params - delta, data)) / (2 * step)
    expected = report['scaled_jacobian'] @ direction
    assert np.allclose(actual, expected, atol=1e-6, rtol=1e-6)
    assert np.allclose(report['jacobian'] * report['parameter_scales'], report['scaled_jacobian'])
    assert report['residual_calls'] == 137


def test_repeated_rows_scale_singular_values_without_increasing_rank(cases):
    model, data, _, report, _ = cases
    repeated = CalibrationData(thetas=[theta for theta in data.thetas for _ in range(3)],
                               frames=[frame for frame in data.frames for _ in range(3)])
    result = model.jacobian_rank_report(repeated)
    for key in ['full', 'reduced']:
        assert result[key]['rank'] == report[key]['rank']
        assert np.allclose(result[key]['singular_values'][:33],
                           np.sqrt(3) * report[key]['singular_values'][:33], rtol=1e-6, atol=1e-6)
        assert np.allclose(result[key]['singular_values_per_sqrt_sample'][:33],
                           report[key]['singular_values_per_sqrt_sample'][:33], rtol=1e-6, atol=1e-6)


@pytest.mark.parametrize('count', [1, 12])
def test_one_configuration_remains_underexcited_even_if_repeated(cases, count):
    model, data, _, _, _ = cases
    single = CalibrationData(thetas=[data.thetas[0]] * count, frames=[data.frames[0]] * count)
    report = model.jacobian_rank_report(single)
    assert report['full']['rank'] == report['reduced']['rank'] == 6
    assert len(report['full']['null_directions']) == 28
    assert len(report['reduced']['null_directions']) == 27
    assert not report['gauge_verified']  # Gauge exists, but it is not the only problem.


def test_linear_sweeps_alone_do_not_identify_wrist_geometry(cases):
    model, _, data, _, _ = cases
    selected = [i for i, theta in enumerate(data.thetas) if np.all(np.asarray(theta)[3:] == 0)]
    partial = CalibrationData(thetas=[data.thetas[i] for i in selected],
                              frames=[data.frames[i] for i in selected])
    report = model.jacobian_rank_report(partial)
    assert report['rank'] < 33 and report['reduced']['rank'] < 33
    assert not report['gauge_verified']
    assert np.isinf(report['reduced']['condition_number'])


def test_nominal_analytic_gauge_and_explicit_parameters_do_not_mutate_the_model(cases):
    model, _, data, _, _ = cases
    zero = ImprovedKinematics(model, model.F_X_init)
    nominal_data = CalibrationData(thetas=data.thetas,
                                   frames=[zero.predict_in_tracker(theta) for theta in data.thetas])
    before = zero.params.copy()
    report = zero.jacobian_rank_report(nominal_data, params=before)
    gauge = np.zeros(34)
    gauge[0] = 1
    gauge[[6, 8, 12, 14, 16]] = -1
    gauge /= report['parameter_scales']
    gauge /= np.linalg.norm(gauge)
    assert abs(gauge @ report['gauge_direction_scaled']) > 1 - 1e-10
    assert report['gauge_verified']
    assert np.array_equal(zero.params, before)


@pytest.mark.parametrize('params', [np.zeros(33), np.full(34, np.nan)])
def test_invalid_parameters_are_rejected(cases, params):
    model, data, _, _, _ = cases
    with pytest.raises(ValueError):
        model.jacobian_rank_report(data, params)


def test_empty_or_unpaired_data_are_rejected(cases):
    model, _, _, _, _ = cases
    for data in [CalibrationData(), CalibrationData(thetas=[np.zeros(6)])]:
        with pytest.raises(ValueError):
            model.jacobian_rank_report(data)


def test_rotation_log_branch_cut_is_not_reported_as_smooth_information(cases):
    model, data, _, _, _ = cases
    theta = data.thetas[0]
    observed = model.predict_in_tracker(theta) * Frame(Rot('x', np.pi), vct3())
    with pytest.raises(ValueError, match='close to pi'):
        model.jacobian_rank_report(CalibrationData(thetas=[theta], frames=[observed]))
