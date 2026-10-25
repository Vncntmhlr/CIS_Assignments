"""Student saved-data, physical-constraint and reporting regressions."""


# test_kinematics_phase_d
import json
from pathlib import Path

import numpy as np
import pytest

from experiments.step8_phase_d import groups, load_data, mean_data, summary
from src import Frame
from src.robot import Robot
from src.simulation import clock, constants
from src.student.kinematics import ImprovedKinematics


@pytest.fixture(scope='module')
def experiment():
    folder = Path(__file__).resolve().parents[1] / 'results/step8_phase_d'
    if not (folder / 'summary.json').exists():
        pytest.skip('Run the student Phase D experiment to create measurement evidence')
    training, extra = load_data(folder / 'calibration.npz')
    holdout, _ = load_data(folder / 'holdout.npz')
    report = json.loads((folder / 'summary.json').read_text())
    nominal = Robot('phase_d_regression', Frame.eye(), constants.THETA_MIN, constants.THETA_MAX,
                    constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)
    model = ImprovedKinematics(nominal, Frame(extra['base_frame']),
                               joint_offsets=extra['home_offsets'])
    with np.load(folder / 'fit.npz', allow_pickle=False) as saved:
        before, after = saved['initial_params'].copy(), saved['fitted_params'].copy()
    return model, training, holdout, before, after, report


def test_holdout_contains_only_new_configurations_and_training_matches_step7(experiment):
    _, training, holdout, _, _, _ = experiment
    assert len(training) == 765 and len(groups(training)) == 69
    assert sorted(map(len, groups(training).values())) == [11] * 68 + [17]
    assert len(holdout) == 396 and len(groups(holdout)) == 36
    assert set(map(len, groups(holdout).values())) == {11}
    assert not (set(groups(training)) & set(groups(holdout)))
    distinct = np.array(list(groups(holdout)))
    assert np.count_nonzero(np.count_nonzero(distinct, axis=1) == 1) == 12
    assert np.count_nonzero(np.count_nonzero(distinct, axis=1) > 1) == 24


def test_archived_full_sample_objective_and_fixed_gauge_are_consistent(experiment):
    model, training, _, before, after, report = experiment
    assert before.shape == after.shape == (34,)
    assert before[0] == after[0] == 0
    info = report['fit']
    assert info['success'] and info['solver_success'] and info['params_updated']
    assert info['n_free_params'] == 33 and info['fixed_parameter_indices'] == [0]
    for params, key in [(before, 'initial_cost'), (after, 'final_cost')]:
        residual = model.residuals(params, training)
        assert .5 * residual @ residual == pytest.approx(info[key], rel=1e-10)
    assert info['final_cost'] < info['initial_cost'] * .1
    assert info['nfev'] >= info['iterations'] > 0
    assert info['residual_calls'] > info['nfev']


@pytest.mark.parametrize('which', ['training', 'holdout'])
def test_fit_meaningfully_improves_predictions_without_claiming_truth_accuracy(experiment, which):
    model, training, holdout, before, after, report = experiment
    data = training if which == 'training' else holdout
    elapsed = clock.elapsed
    for observations in (data, mean_data(data)):
        initial, final = summary(model, observations, before), summary(model, observations, after)
        assert final['rms_mm'] < initial['rms_mm'] * .5
        assert final['rms_rad'] < initial['rms_rad'] * .5
    final = summary(model, data, after)
    assert final['rms_mm'] == pytest.approx(report['errors'][which + '_raw']['after']['rms_mm'])
    assert clock.elapsed == elapsed  # Evaluation does not acquire or move anything.


def test_frame_means_preserve_configuration_identity_and_valid_rotations(experiment):
    _, training, holdout, _, _, _ = experiment
    for data in (training, holdout):
        means = mean_data(data)
        assert set(groups(means)) == set(groups(data))
        for frame in means.frames:
            assert np.allclose(frame.R.matrix.T @ frame.R.matrix, np.eye(3), atol=1e-12)
            assert np.linalg.det(frame.R.matrix) == pytest.approx(1, abs=1e-12)


# test_kinematics_phase_e
import json
from pathlib import Path

import numpy as np
import pytest

from experiments.step8_phase_d import interval_bounds
from experiments.step8_phase_e import saved_model
from src import Frame
from src.simulation import clock
from vendor.functions._fit_constraints import distortion_bounds
from src.student.metrics import frame_error
from experiments.test_kinematics import synthetic


@pytest.mark.parametrize('degree', [0, 1, 2, 3])
def test_parameter_report_has_correct_nominal_units_axes_and_scales(degree):
    _, model, _, _ = synthetic(degree=degree)
    model.joint_offsets[:] = 0
    report = model.parameters_report()
    assert report['axes'].shape == report['axis_tilts'].shape == (6, 3)
    assert np.allclose(report['axes'], np.eye(3)[[0, 1, 2, 0, 1, 2]])
    assert np.allclose(report['linear_axis_angles_deg'], (1-np.eye(3))*90)
    assert np.allclose(report['wrist_axis_angles_deg'], (1-np.eye(3))*90)
    assert np.array_equal(report['joint_offset'], np.zeros(6))
    assert np.array_equal(report['joint_scale'], np.zeros(6))
    assert np.array_equal(report['joint_scale_factor'], np.ones(6))
    assert report['physical_joint_bounds_satisfied']
    assert report['joint_polynomial_coefficients'].shape == (6, degree)


def test_report_uses_effective_encoder_zero_quantities_and_actual_se3_translation():
    truth, _, _, _ = synthetic()
    before = truth.params.copy()
    report = truth.parameters_report()
    o, s = truth.joint_offsets, report['joint_normalization_scales']
    a, b = truth.params[22:].reshape(6, 2).T
    assert np.allclose(report['joint_offset'], o + a*(o/s) + b*(o/s)**2)
    assert np.allclose(report['joint_scale'], (a + 2*b*o/s)/s)
    assert np.allclose(report['joint_home_scale_factor'], 1 + a/s)
    assert np.allclose(report['tool_correction_translation_mm'], report['F_we_correction'].p.vec.ravel())
    assert not np.allclose(report['tool_correction_translation_mm'], truth.params[19:22], atol=1e-8)
    assert np.allclose(report['axes'] @ report['axes'].T,
                       np.cos(np.radians(report['axis_angles_deg'])), atol=1e-12)
    assert report['limitations'] and report['gauge_dependent'] and report['gauge_invariant']
    report['joint_home_correction'][:] = 99
    report['F_we_twist'][:] = 99
    assert np.array_equal(truth.params, before)
    assert np.max(abs(truth.joint_offsets)) < 1


def test_report_separates_gauge_coordinates_from_invariant_geometry():
    _, model, data, _ = synthetic()
    model.params[:] = 0
    first = model.parameters_report()
    predictions = [model.predict_in_tracker(theta) for theta in data.thetas[:3]]
    gamma = .015
    model.params[0] = gamma
    model.params[[6, 8, 12, 14, 16]] = -gamma
    second = model.parameters_report()
    assert not np.allclose(first['axes'], second['axes'])
    assert np.allclose(first['axis_angles_deg'], second['axis_angles_deg'], atol=1e-6)
    assert np.allclose(first['axes_in_tracker_at_joint_zero'], second['axes_in_tracker_at_joint_zero'])
    assert np.array_equal(first['joint_offset'], second['joint_offset'])
    for theta, prediction in zip(data.thetas[:3], predictions):
        angle, position = frame_error(prediction, model.predict_in_tracker(theta))
        assert angle < 1e-12 and position < 1e-10


def test_integrated_bound_calculus_matches_the_validated_diagnostic():
    truth, _, _, _ = synthetic()
    expected, margin, jacobian = interval_bounds(truth, truth.params)
    actual, actual_margin, actual_jacobian = distortion_bounds(truth, truth.params)
    assert np.allclose(actual_margin, margin, atol=1e-10)
    assert np.allclose(actual_jacobian, jacobian, atol=1e-10)
    for a, b in zip(actual, expected):
        for key in ['minimum', 'maximum', 'max_absolute_derivative']:
            assert a[key] == pytest.approx(b[key])


@pytest.fixture(scope='module')
def adopted():
    root = Path(__file__).resolve().parents[1]
    folder = root / 'results/step8_phase_e'
    if not (folder/'summary.json').exists():
        pytest.skip('Run the Phase E saved-data experiment first')
    model, data = saved_model()
    with np.load(folder/'fit.npz', allow_pickle=False) as saved:
        model.params = saved['fitted_params'].copy()
    return model, data, json.loads((folder/'summary.json').read_text())


def test_normal_default_fit_reproduces_candidate_and_preserves_bound_compliance(adopted):
    model, _, report = adopted
    for label in ['warm_fit', 'cold_fit']:
        info = report[label]
        assert info['success'] and info['params_updated'] and info['physical_constraints']
        assert info['solver'] == 'SLSQP' and info['n_params'] == 34 and info['n_free_params'] == 33
        assert info['minimum_normalized_margin'] >= -1e-7
    for case in report['candidate_reproduction'].values():
        assert case['max_position_mm'] < 1e-5 and case['max_angle_rad'] < 1e-7
    assert model.parameters_report()['physical_joint_bounds_satisfied']
    assert report['additional_simulated_time_s'] == 0


@pytest.mark.parametrize('wrist', [(-1.8, -.9, -2.4), (.7, .6, 2.2), (-.6, -.4, -1.), (1.9, 1.1, .5)])
def test_provided_inverse_consistency_across_regular_wrist_configurations(adopted, wrist):
    model, _, _ = adopted
    theta = np.r_[300., 0., 300., wrist]
    elapsed = clock.elapsed
    goal = model.fwd_kins(theta)
    result = model.inv_kins(goal, seed=theta + [0, 0, 0, .05, -.03, .04])
    angle, position = frame_error(goal, model.fwd_kins(result))
    assert model.within_limits(result) and angle < 1e-8 and position < 1e-6
    assert clock.elapsed == elapsed


@pytest.mark.parametrize('case', [
    pytest.param('near_singular', marks=pytest.mark.xfail(strict=True, reason='Provided IK can return an unconverged pole solution')),
    pytest.param('joint2_upper', marks=pytest.mark.xfail(strict=True, reason='Provided IK can reject a reachable boundary target')),
    pytest.param('joint6_upper', marks=pytest.mark.xfail(strict=True, reason='Provided IK wraps a nonperiodic joint-distortion map after solving')),
])
def test_documented_provided_inverse_edge_limitations(adopted, case):
    model, _, report = adopted
    theta = np.array([300., 0., 300., .3, .5, -.4])
    if case == 'near_singular':
        theta[3], theta[4] = .7, report['ik']['fitted_wrist_poles'][1]['theta5_rad']
    elif case == 'joint2_upper':
        theta[1] = model.theta_max[1]
    else:
        theta[5] = model.theta_max[5]
    goal = model.fwd_kins(theta)
    back = model.inv_kins(goal, seed=theta)
    angle, position = frame_error(goal, model.fwd_kins(back))
    assert model.within_limits(back) and angle < 1e-8 and position < 1e-6


def test_existing_public_ik_controls_resolve_the_reachable_boundary_case(adopted):
    model, _, _ = adopted
    theta = np.array([300., model.theta_max[1], 300., .3, .5, -.4])
    target = model.fwd_kins(theta)
    back = model.inv_kins(target, seed=theta, max_iter=200, tol=1e-12)
    angle, position = frame_error(target, model.fwd_kins(back))
    assert model.within_limits(back) and angle < 1e-8 and position < 1e-6


def test_saved_audit_distinguishes_default_failures_from_public_control_retries(adopted):
    _, _, report = adopted
    audit = report['ik']
    assert audit['categories']['heldout']['strict_pass'] == 36
    assert audit['categories']['near_singular']['strict_pass'] < audit['categories']['near_singular']['n']
    assert audit['public_controls']['rescued'] > 0
    assert audit['public_controls']['combined_target_sized_consistency_pass'] == len(audit['rows'])


# test_bound_diagnostic
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from experiments.step8_phase_d import constrained_fit, interval_bounds
from src.student.workflow import CalibrationData
from experiments.test_kinematics import synthetic


@pytest.mark.parametrize('a,b,vertex', [(0., .1, True), (.04, -.1, True),
                                       (.1, 0., False), (.1, .01, False)])
def test_exact_bounds_find_endpoint_and_interior_extrema(a, b, vertex):
    _, model, _, _ = synthetic()
    model.joint_offsets[:] = 0
    params = np.zeros(34)
    params[24:26] = [a, b]  # Joint 2 has normalized interval [-1, 1].
    reports, _, _ = interval_bounds(model, params)
    report = reports[1]
    x = np.linspace(-1, 1, 20001)
    values = a * x + b * x**2
    assert report['minimum'] == pytest.approx(values.min(), abs=1e-9)
    assert report['maximum'] == pytest.approx(values.max(), abs=1e-9)
    assert report['max_absolute_derivative'] == pytest.approx(max(abs(a-2*b), abs(a+2*b))/300)
    locations = [report['theta_at_minimum'], report['theta_at_maximum']]
    assert any(-300 < theta < 300 for theta in locations) == vertex


def test_home_offset_is_included_and_constraint_jacobian_is_correct():
    truth, model, _, _ = synthetic()
    params = truth.params.copy()
    reports, margin, jacobian = interval_bounds(model, params)
    assert len(margin) == 36 and jacobian.shape == (36, 34)
    for k in range(22, 34):
        delta = np.zeros(34)
        delta[k] = 1e-7
        numeric = (interval_bounds(model, params+delta)[1]
                   - interval_bounds(model, params-delta)[1]) / 2e-7
        assert np.allclose(numeric, jacobian[:, k], rtol=2e-6, atol=1e-6)
    zero = interval_bounds(model, np.zeros(34))[0]
    for k, report in enumerate(zero):
        assert report['minimum'] == report['maximum'] == model.joint_offsets[k]
        assert report['max_absolute_derivative'] == 0


def test_constrained_fit_recovers_a_feasible_noiseless_model_without_mutation():
    truth, fitted, data, heldout = synthetic(seed=859, n_train=12, n_test=16)
    truth.params[22:28] = [.02, -.01, -.015, .01, .025, -.005]
    truth.params[28:] = [1e-5, -1e-5, -1e-5, 1e-5, 2e-5, 0]
    assert min(interval_bounds(truth, truth.params)[1]) > 0
    data.frames = [truth.predict_in_tracker(theta) for theta in data.thetas]
    before = fitted.params.copy()
    candidate, info, _ = constrained_fit(fitted, data, before, maxiter=20, verbose=False)
    assert info['success'] and info['minimum_normalized_margin'] >= -1e-7
    assert info['final_cost'] < info['initial_cost'] * 1e-9
    assert np.array_equal(fitted.params, before) and candidate[0] == before[0]
    independent = CalibrationData(thetas=list(heldout),
                                  frames=[truth.predict_in_tracker(theta) for theta in heldout])
    errors = fitted.residuals(candidate, independent).reshape(-1, 6)
    assert np.max(np.linalg.norm(errors[:, :3] * .001, axis=1)) < 1e-7
    assert np.max(np.linalg.norm(errors[:, 3:] * .2, axis=1)) < 1e-5


def test_saved_candidate_is_feasible_and_original_archives_unchanged():
    root = Path(__file__).resolve().parents[1]
    output = root / 'results/step8_bound_diagnostic'
    if not (output / 'summary.json').exists():
        pytest.skip('Full archived-data diagnostic has not been run yet')
    report = json.loads((output / 'summary.json').read_text())
    for path, digest in report['archive_sha256'].items():
        assert hashlib.sha256((root/path).read_bytes()).hexdigest() == digest
    assert report['fit']['success'] and not report['fit']['model_params_updated']
    assert report['additional_simulated_time_s'] == 0
    for bound in report['bounds_constrained']:
        assert bound['magnitude_ratio'] <= 1+1e-7
        assert bound['derivative_ratio'] <= 1+1e-7


@pytest.mark.parametrize('visible_x,expected_count', [(None, 729), (400., 486)])
def test_step9_nonidentity_frames_known_errors_and_fixed_visible_grid(monkeypatch, visible_x, expected_count):
    """Known right-frame errors detect a reversed/missing tracker transform."""
    from itertools import product
    from types import SimpleNamespace
    from experiments.step9_forward import execute_step9
    from src import Rot, vct3, simulation
    from src.student import metrics

    def forbidden(*args, **kwargs):
        pytest.fail('Step 9 must not move, sample, fit, or run inverse kinematics')

    tracker_frame = Frame(Rot('z', .7)*Rot('y', -.3), vct3(120, -250, 1700))
    nominal_base = Frame(Rot('x', -.2), vct3(-400, 30, 800))

    def reference(theta):
        return Frame(Rot('x', .4+.1*theta[3])*Rot('y', .2*theta[4]), vct3(theta[:3]))

    nominal_delta = Frame(Rot('z', .015), vct3(.6, .8, 0))
    improved_delta = Frame(Rot('x', .0007), vct3(.03, .04, 0))
    oracle_calls = []

    def actual(theta):
        oracle_calls.append(theta.copy())
        return tracker_frame*reference(theta)

    robot = SimpleNamespace(actual_ee_pose=actual, fwd_kins=lambda th: nominal_base.inv()*reference(th)*nominal_delta,
                            move_joints=forbidden, home_robot=forbidden)
    tracker = SimpleNamespace(nominal_pose=lambda: tracker_frame,
                              is_visible=lambda pts: visible_x is None or pts[0].vec[0, 0] < visible_x)
    model = SimpleNamespace(predict_in_tracker=lambda th: reference(th)*improved_delta,
                            fit=forbidden, inv_kins=forbidden)
    namespace = dict(np=np, rob=robot, trk=tracker, improved=model, F_X_nom=nominal_base,
                     metrics=metrics, constants=simulation.constants,
                     mbee_markers_world=lambda th: [vct3(th[:3])])
    simulation.clock.reset()
    simulation.clock.advance(19.25)
    monkeypatch.setattr(simulation.clock, 'advance', forbidden)
    monkeypatch.setattr(simulation, 'sample_marker_bodies', forbidden)
    before_random = np.random.get_state()
    execute_step9(namespace)
    expected = np.array(list(product([100., 300., 500.], [-200., 0., 200.], [100., 300., 500.],
                                    [-1.3, 0., 1.3], [-1.3, 0., 1.3], [-2., 0., 2.])))
    if visible_x is not None:
        expected = expected[expected[:, 0] < visible_x]
    assert len(expected) == expected_count
    assert np.array_equal(namespace['test_grid'], expected)
    assert np.array_equal(oracle_calls, expected)
    assert np.allclose(namespace['fwd_err_nominal'], [.015, 1.], atol=1e-11, rtol=0)
    assert np.allclose(namespace['fwd_err_improved'], [.0007, .05], atol=1e-11, rtol=0)
    assert namespace['fwd_summary_improved']['n'] == expected_count
    assert simulation.clock.elapsed == 19.25
    after_random = np.random.get_state()
    assert before_random[0] == after_random[0] and np.array_equal(before_random[1], after_random[1])
    assert before_random[2:] == after_random[2:]


def test_step9_frozen_calibration_generalizes_without_acquisition_or_refitting(monkeypatch):
    from experiments.step9_forward import evaluate_saved
    from src import simulation
    from src.student.kinematics import ImprovedKinematics

    def forbidden(*args, **kwargs):
        pytest.fail('Step 9 may only read the deterministic pose oracle')

    monkeypatch.setattr(simulation.RobotSim, 'move_joints', forbidden)
    monkeypatch.setattr(simulation.RobotSim, 'home_robot', forbidden)
    monkeypatch.setattr(simulation.RobotSim, 'ground_truth_parameters', forbidden)
    monkeypatch.setattr(simulation, 'sample_marker_bodies', forbidden)
    monkeypatch.setattr(ImprovedKinematics, 'fit', forbidden)
    monkeypatch.setattr(ImprovedKinematics, 'inv_kins', forbidden)
    report, poses, nominal, improved = evaluate_saved()
    assert report['candidate_poses'] == report['visible_poses'] == len(poses) == 729
    assert report['training_configuration_overlap'] == report['tracker_holdout_configuration_overlap'] == 0
    assert report['additional_simulated_evaluation_time_s'] == 0
    assert report['additional_simulated_acquisition_time_s'] == 0
    assert not report['fitted_parameters_changed']
    assert report['improved']['rms_pos'] < report['nominal']['rms_pos']*.1
    assert report['improved']['rms_angle'] < report['nominal']['rms_angle']*.1
    assert np.max(improved[:, 0]) < simulation.constants.TARGET_ANG_ERR_RAD
    assert np.max(improved[:, 1]) < simulation.constants.TARGET_POS_ERR_MM
    archived = json.loads((Path(__file__).resolve().parents[1]/'results/step9_forward_summary.json').read_text())
    for label in ['nominal', 'improved']:
        for key in ['rms_pos', 'max_pos', 'rms_angle', 'max_angle']:
            assert report[label][key] == pytest.approx(archived[label][key], abs=1e-12)
    with np.load(Path(__file__).resolve().parents[1]/'results/step9_forward_errors.npz', allow_pickle=False) as saved:
        assert np.array_equal(poses, saved['thetas'])
        assert np.allclose(nominal, saved['nominal_errors'], atol=1e-12, rtol=0)
        assert np.allclose(improved, saved['improved_errors'], atol=1e-12, rtol=0)
