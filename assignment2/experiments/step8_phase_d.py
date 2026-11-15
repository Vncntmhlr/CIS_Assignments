"""Student-owned Phase D experiment; no TA cells, solver, or truth APIs changed.

Run from assignment2: venv/bin/python experiments/step8_phase_d.py
Acquisitions are cached in results/step8_phase_d/ so reruns do not move/sample
the simulator again. The holdout data never enter base estimation or fitting.
"""

import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src import Frame, Rot, vct3, simulation
from src.robot import Robot
from src.simulation import clock, constants
from src.student import registration, workflow
from src.student.kinematics import ImprovedKinematics
from src.student.workflow import CalibrationData


def frame_array(frame):
    matrix = np.eye(4)
    matrix[:3, :3] = frame.R.matrix
    matrix[:3, 3] = frame.p.vec.ravel()
    return matrix


def save_data(path, data, **extra):
    np.savez_compressed(path, thetas=np.asarray(data.thetas),
                        frames=np.asarray([frame_array(f) for f in data.frames]),
                        t_collect=data.t_collect, **extra)


def load_data(path):
    with np.load(path, allow_pickle=False) as saved:
        data = CalibrationData(thetas=[x.copy() for x in saved['thetas']],
                               frames=[Frame(x) for x in saved['frames']],
                               t_collect=float(saved['t_collect']))
        extra = {k: saved[k].copy() for k in saved.files
                 if k not in {'thetas', 'frames', 't_collect'}}
    return data, extra


def groups(data):
    """Exact command identities: repetitions cannot leak across train/test."""
    result = {}
    for i, theta in enumerate(data.thetas):
        result.setdefault(tuple(theta), []).append(i)
    return result


def mean_data(data):
    return CalibrationData(
        thetas=[np.asarray(theta) for theta in groups(data)],
        frames=[registration.average_frames([data.frames[i] for i in rows])
                for rows in groups(data).values()])


def subset(data, rows):
    return CalibrationData(thetas=[data.thetas[i] for i in rows],
                           frames=[data.frames[i] for i in rows])


def summary(model, data, params):
    residual = model.residuals(params, data).reshape(-1, 6)
    angles = np.linalg.norm(residual[:, :3] * .001, axis=1)
    positions = np.linalg.norm(residual[:, 3:] * .2, axis=1)
    return dict(n=len(data), rms_mm=float(np.sqrt(np.mean(positions**2))),
                max_mm=float(positions.max()), rms_rad=float(np.sqrt(np.mean(angles**2))),
                max_rad=float(angles.max()), weighted_cost=float(.5 * np.sum(residual**2)))


def check_data(data, robot, expected_rows, expected_distinct):
    if len(data) != expected_rows or len(groups(data)) != expected_distinct:
        raise ValueError('Missing/extra observations: inspect acquisition before fitting')
    if len(data.thetas) != len(data.frames):
        raise ValueError('Unpaired observations')
    for theta, frame in zip(data.thetas, data.frames):
        matrix = frame_array(frame)
        if (not robot.within_limits(theta) or not np.isfinite(theta).all()
                or not np.isfinite(matrix).all()
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10)
                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-10)):
            raise ValueError('Invalid encoder or observed frame')


def public_time(robot, poses):
    previous = robot.state().theta.copy()
    motion = 0.
    for theta in poses:
        motion += robot.motion_time(previous, theta)
        previous = theta
    return dict(motion_s=float(motion), sampling_s=len(poses) * .03,
                total_s=float(motion + len(poses) * .03))


def validation_poses(robot, tracker, markers, train):
    """12 new linear midpoints and 24 new mixed poses, chosen before fitting."""
    rng = np.random.default_rng(840)
    poses = []
    for axis in (0, 2):
        for value in (18.75, 93.75, 206.25, 318.75, 431.25, 543.75):
            theta = np.zeros(6)
            theta[axis] = value
            poses.append(theta)
    while len(poses) < 36:
        theta = rng.uniform(robot.theta_min, robot.theta_max)
        nominal = robot.F_rob * robot.fwd_kins(theta)
        if tracker.is_visible([nominal * p for p in markers]):
            poses.append(theta)
    if set(map(tuple, poses)) & set(groups(train)):
        raise ValueError('Holdout configurations overlap fitting data')
    # Greedy public motion-time ordering reduces validation acquisition cost;
    # it is not claimed to be the global shortest route or a new Step 7 plan.
    ordered, previous = [], robot.state().theta.copy()
    while poses:
        i = min(range(len(poses)), key=lambda i: robot.motion_time(previous, poses[i]))
        previous = poses.pop(i)
        ordered.append(previous)
    return [theta.copy() for theta in ordered for _ in range(11)]


def sweep_analysis(model, data, before, after):
    """Signed residuals and high-order contrasts; no correction is fitted here."""
    reports, profiles = {}, {}
    grouped = groups(data)
    for axis in range(6):
        commands = sorted((theta for theta in grouped
                           if all(theta[k] == 0 for k in range(6) if k != axis)),
                          key=lambda theta: theta[axis])
        rows = [i for theta in commands for i in grouped[theta]]
        part = subset(data, rows)
        means = mean_data(part)
        xyz = np.array([(model.predict_in_tracker(theta, after).p.vec - frame.p.vec).ravel()
                        for theta, frame in zip(means.thetas, means.frames)])
        angular = np.array([Rotation.from_matrix(frame.R.matrix.T
                            @ model.predict_in_tracker(theta, after).R.matrix).as_rotvec()
                           for theta, frame in zip(means.thetas, means.frames)])
        record = {'before': summary(model, part, before), 'after': summary(model, part, after),
                  'pose_means_after': summary(model, means, after),
                  'n_configurations': len(commands)}
        if axis < 3:
            direction = (model.base_frame(after).R * model.AXES[axis]).vec.ravel()
            axial = xyz @ direction
            mean_variance = []
            for theta in commands:
                positions = np.array([data.frames[i].p.vec.ravel() for i in grouped[theta]])
                mean_variance.append(np.var(positions @ direction, ddof=1) / len(positions))
            x = np.array([theta[axis] for theta in commands])
            x = 2 * (x - x.min()) / np.ptp(x) - 1
            design = np.polynomial.legendre.legvander(x, 4)
            variance = np.array(mean_variance)
            projection = np.linalg.pinv(design / np.sqrt(variance[:, None])) / np.sqrt(variance)
            coefficients = projection @ axial
            standard_errors = np.sqrt(np.sum(projection**2 * variance, axis=1))
            record['axial_mean_range_mm'] = [float(axial.min()), float(axial.max())]
            record['high_order_contrasts'] = {
                'legendre_c3_mm': float(coefficients[3]), 'c3_noise_se_mm': float(standard_errors[3]),
                'legendre_c4_mm': float(coefficients[4]), 'c4_noise_se_mm': float(standard_errors[4]),
                'c3_z': float(coefficients[3] / standard_errors[3]),
                'c4_z': float(coefficients[4] / standard_errors[4])}
            # Noise-only contrast SE excludes model-parameter uncertainty and
            # multiple testing; these are diagnostics, not calibrated p-values.
        reports[str(axis + 1)] = record
        profiles[str(axis + 1)] = dict(command=np.array(commands)[:, axis],
                                       position_xyz_mm=xyz, rotation_xyz_rad=angular)
    return reports, profiles


def noise_estimate(data):
    """Within-command scatter, including tracker and re-command repeatability.

    Small-angle rotation scatter and independence of repetitions are assumed.
    Fixed bias is absent from this statistic; it is not an accuracy bound.
    """
    position_sum = rotation_sum = 0.
    mean_position_variance, mean_rotation_variance = [], []
    for rows in groups(data).values():
        frames = [data.frames[i] for i in rows]
        mean = registration.average_frames(frames)
        dp = np.array([(frame.p.vec - mean.p.vec).ravel() for frame in frames])
        dr = np.array([Rotation.from_matrix(mean.R.matrix.T @ frame.R.matrix).as_rotvec()
                       for frame in frames])
        position_sum += np.sum(dp**2)
        rotation_sum += np.sum(dr**2)
        mean_position_variance.append(np.sum(dp**2) / ((len(rows) - 1) * len(rows)))
        mean_rotation_variance.append(np.sum(dr**2) / ((len(rows) - 1) * len(rows)))
    degrees = len(data) - len(groups(data))
    return dict(single_observation_position_rms_mm=float(np.sqrt(position_sum / degrees)),
                single_observation_orientation_rms_rad=float(np.sqrt(rotation_sum / degrees)),
                pose_mean_position_noise_rms_mm=float(np.sqrt(np.mean(mean_position_variance))),
                pose_mean_orientation_noise_rms_rad=float(np.sqrt(np.mean(mean_rotation_variance))))


def plot_profiles(profiles, destination, orientation=False):
    import os
    import tempfile
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'cis-phase-d-matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(11, 6), layout='constrained')
    for axis, panel in enumerate(axes.flat, start=1):
        profile = profiles[str(axis)]
        values = profile['rotation_xyz_rad'] * 1000 if orientation else profile['position_xyz_mm']
        for component, label in enumerate('xyz'):
            panel.plot(profile['command'], values[:, component], '.-', label=label)
        panel.axhline(0, color='black', lw=.5)
        panel.set(title=f'Joint {axis}', xlabel='Encoder (mm)' if axis <= 3 else 'Encoder (rad)',
                  ylabel=('Observed-EE rotation vector (mrad)' if orientation
                          else 'Signed tracker position residual (mm)'))
        panel.legend(ncol=3, fontsize=8)
    fig.suptitle('Phase D: ' + ('signed orientation errors against mean observed Frames' if orientation
                              else 'predicted minus mean observed position at each calibration configuration'))
    fig.savefig(destination, dpi=160)
    plt.close(fig)


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def run(analyze_only=False):
    started = perf_counter()
    output = ROOT / 'results' / 'step8_phase_d'
    output.mkdir(parents=True, exist_ok=True)
    if analyze_only and not all((output / name).exists() for name in
                                ['calibration.npz', 'holdout.npz', 'fit.npz', 'summary.json']):
        raise ValueError('--analyze-only requires the completed experiment archives')
    notebook = ROOT / 'notebooks' / 'assignment2_template.ipynb'
    cells = json.loads(notebook.read_text())['cells']
    namespace = {'__name__': '__phase_d_acquisition__'}
    # Execute original cell source without changing/saving the notebook.
    exec(compile(''.join(cells[1]['source']), str(notebook) + ':cell1', 'exec'), namespace)
    np.random.seed(606)
    for i in (3, 5):
        exec(compile(''.join(cells[i]['source']), str(notebook) + f':cell{i}', 'exec'), namespace)
    robot, tracker, body = (namespace[k] for k in ('rob', 'trk', 'mbee_placed'))
    train_path, holdout_path = output / 'calibration.npz', output / 'holdout.npz'
    provenance_path = output / 'acquisition.json'
    provenance = json.loads(provenance_path.read_text()) if provenance_path.exists() else {}
    if not train_path.exists():
        for i in (16, 18):
            exec(compile(''.join(cells[i]['source']), str(notebook) + f':cell{i}', 'exec'), namespace)
        train, base, offsets = (namespace[k] for k in ('data', 'F_X_nom', 'home_offsets'))
        save_data(train_path, train, base_frame=frame_array(base), home_offsets=offsets)
        provenance['training'] = dict(simulated_total_including_step6_s=simulation.clock.elapsed,
                                      step7_s=train.t_collect, rows=len(train), distinct=len(groups(train)),
                                      robot_tracker_seed=1, sensor_noise_seed=606,
                                      original_cells=[1, 3, 5, 16, 18])
        provenance_path.write_text(json.dumps(provenance, indent=2))
    train, saved = load_data(train_path)
    check_data(train, robot, 765, 69)
    base, offsets = Frame(saved['base_frame']), saved['home_offsets']
    if not holdout_path.exists():
        poses = validation_poses(robot, tracker, namespace['mbee_local_positions'], train)
        timing = public_time(robot, poses)
        np.random.seed(2604)
        holdout = workflow.collect_calibration_data(robot, tracker, body, poses, repeats=1)
        check_data(holdout, robot, 396, 36)
        save_data(holdout_path, holdout)
        provenance['holdout'] = dict(rows=len(holdout), distinct=len(groups(holdout)),
                                     timing_prediction=timing, measured_s=holdout.t_collect,
                                     pose_seed=840, sensor_noise_seed=2604,
                                     design='12 J1/J3 midpoints + 24 mixed; 11 re-commands each')
        provenance_path.write_text(json.dumps(provenance, indent=2))
    holdout, _ = load_data(holdout_path)
    check_data(holdout, robot, 396, 36)
    assert not (set(groups(train)) & set(groups(holdout)))
    model = ImprovedKinematics(robot, base, joint_offsets=offsets)
    before = model.params.copy()
    print('ACQUISITION', json.dumps(provenance), flush=True)
    print('INITIAL', json.dumps(summary(model, train, before)), flush=True)
    clock_before = simulation.clock.elapsed
    previous_report = json.loads((output / 'summary.json').read_text()) if analyze_only else None
    if analyze_only:
        with np.load(output / 'fit.npz', allow_pickle=False) as saved_fit:
            if not np.array_equal(before, saved_fit['initial_params']):
                raise ValueError('Saved fit has another initial model')
            model.params = saved_fit['fitted_params'].copy()
        info = previous_report['fit']
    else:
        # Preserve the historical unconstrained comparison after Phase E
        # makes the physically constrained fit the normal default.
        info = model.fit(train, verbose=2, physical_constraints=False)
    if not info['success']:
        raise RuntimeError(f'Fit failed; parameters left unchanged: {info}')
    assert simulation.clock.elapsed == clock_before
    after = model.params.copy()
    if not analyze_only:
        np.savez_compressed(output / 'fit.npz', initial_params=before, fitted_params=after)
    cases = {'training_raw': train, 'training_pose_means': mean_data(train),
             'holdout_raw': holdout, 'holdout_pose_means': mean_data(holdout)}
    for label, predicate in [('midpoints', lambda t: np.count_nonzero(t) == 1),
                             ('mixed', lambda t: np.count_nonzero(t) > 1)]:
        part = subset(holdout, [i for i, theta in enumerate(holdout.thetas) if predicate(theta)])
        cases['holdout_' + label + '_raw'] = part
        cases['holdout_' + label + '_pose_means'] = mean_data(part)
    sweep_reports, profiles = sweep_analysis(model, train, before, after)
    # Gauge-independent joint-map monotonicity is a plausibility check, not
    # Phase E's physical parameter report or a comparison with hidden truth.
    scales = np.maximum(abs(robot.theta_min), abs(robot.theta_max))
    coefficients = after[22:].reshape(6, 2)
    endpoints = np.stack([robot.theta_min, robot.theta_max]) + offsets
    slopes = 1 + coefficients[:, 0] / scales + 2 * coefficients[:, 1] * endpoints / scales**2
    report = dict(acquisition=provenance, fit=info,
                  errors={key: {'before': summary(model, data, before),
                                'after': summary(model, data, after)} for key, data in cases.items()},
                  sweeps=sweep_reports,
                  within_pose_noise={key: noise_estimate(data)
                                     for key, data in [('training', train), ('holdout', holdout)]},
                  joint_map_slope_min=slopes.min(axis=0),
                  joint_map_slope_max=slopes.max(axis=0),
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in [notebook, ROOT / 'src/student/kinematics.py',
                                           ROOT / 'src/student/workflow.py']},
                  experiment_wall_s=(previous_report['experiment_wall_s'] if analyze_only
                                     else perf_counter() - started),
                  evaluation='Tracker observations only; no official robot ground-truth evaluation')
    (output / 'summary.json').write_text(json.dumps(report, default=json_default, indent=2) + '\n')
    plot_profiles(profiles, output / 'sweep_residuals.png')
    plot_profiles(profiles, output / 'sweep_orientation_residuals.png', orientation=True)
    print('FIT', json.dumps(info, default=json_default), flush=True)
    print('ERRORS', json.dumps(report['errors']), flush=True)
    print('RESULTS', output, flush=True)


def parameter_scales():
    return np.r_[[.01] * 3, [1.] * 3, [.01] * 10, [.01] * 3, [.5] * 3,
                 np.repeat([.2] * 3 + [.01] * 3, 2)]

def interval_bounds(model, params):
    """Exact quadratic extrema, including the fixed home correction.

    Rows of the Jacobian are normalized margins >= 0: magnitude upper/lower,
    then derivative upper/lower at each endpoint. Interior-vertex gradients
    follow the envelope theorem: d(delta(x_vertex))/d(a,b)=(x,x^2).
    """
    if model.n_poly != 2:
        raise ValueError('This diagnostic is specifically for the Phase A quadratic')
    values = np.asarray(params, dtype=float)
    if values.shape != (34,) or not np.isfinite(values).all():
        raise ValueError('Expected the unchanged finite 34-parameter vector')
    scales = np.maximum(abs(model.theta_min), abs(model.theta_max))
    reports, margins, jacobian = [], [], []
    for k, (a, b) in enumerate(values[22:].reshape(6, 2)):
        lower, upper = (np.array([model.theta_min[k], model.theta_max[k]])
                        + model.joint_offsets[k]) / scales[k]
        candidates = [lower, upper]
        if b != 0:
            vertex = -a / (2 * b)
            if lower < vertex < upper:
                candidates.append(vertex)
        x = np.asarray(candidates)
        distortion = model.joint_offsets[k] + a * x + b * x**2
        imin, imax = int(np.argmin(distortion)), int(np.argmax(distortion))
        endpoints = np.array([lower, upper])
        derivative = (a + 2 * b * endpoints) / scales[k]
        magnitude_limit, derivative_limit = constants.DELTA_SYS[k], constants.OMEGA_SYS[k]
        margin = [1 - distortion[imax] / magnitude_limit,
                  1 + distortion[imin] / magnitude_limit]
        gradients = [-np.array([x[imax], x[imax]**2]) / magnitude_limit,
                     np.array([x[imin], x[imin]**2]) / magnitude_limit]
        for endpoint, slope in zip(endpoints, derivative):
            margin.extend([1 - slope / derivative_limit, 1 + slope / derivative_limit])
            gradient = np.array([1., 2 * endpoint]) / (scales[k] * derivative_limit)
            gradients.extend([-gradient, gradient])
        block = np.zeros((6, 34))
        block[:, 22 + 2 * k:24 + 2 * k] = gradients
        jacobian.append(block)
        margins.extend(margin)
        reports.append(dict(joint=k + 1, units='mm' if k < 3 else 'rad',
                            interval=[float(model.theta_min[k]), float(model.theta_max[k])],
                            minimum=float(distortion[imin]), maximum=float(distortion[imax]),
                            theta_at_minimum=float(x[imin] * scales[k] - model.joint_offsets[k]),
                            theta_at_maximum=float(x[imax] * scales[k] - model.joint_offsets[k]),
                            max_absolute=float(np.max(abs(distortion))),
                            magnitude_limit=float(magnitude_limit),
                            magnitude_ratio=float(np.max(abs(distortion)) / magnitude_limit),
                            endpoint_derivatives=derivative.tolist(),
                            max_absolute_derivative=float(max(abs(derivative))),
                            derivative_limit=float(derivative_limit),
                            derivative_ratio=float(max(abs(derivative)) / derivative_limit)))
    return reports, np.asarray(margins), np.vstack(jacobian)

def finite_jacobian(model, data, params):
    """Scaled central differences in the same physical coordinates as Phase C."""
    scales = parameter_scales()
    jacobian = np.empty((6 * len(data), 33))
    step = 1e-4 * np.maximum(1., abs(params / scales))
    for column, k in enumerate(range(1, 34)):
        delta = np.zeros(34)
        delta[k] = step[k] * scales[k]
        jacobian[:, column] = (model.residuals(params + delta, data)
                              - model.residuals(params - delta, data)) / (2 * step[k])
    return jacobian

def sampling_uncertainty(model, data, params, jacobian):
    """Linearized sandwich covariance with per-command empirical noise blocks.

    Conditional on measured home offsets and the quadratic model. Excludes
    fixed tracker bias, model mismatch and homing uncertainty; not a truth CI.
    """
    u, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
    influence = (parameter_scales()[1:, None] * (vt.T / singular)) @ u.T
    influence = influence.reshape(33, len(data), 6)
    residuals = model.residuals(params, data).reshape(-1, 6)
    covariance = np.zeros((34, 34))
    for rows in groups(data).values():
        noise = np.cov(residuals[rows], rowvar=False, ddof=1)
        part = influence[:, rows, :]
        covariance[1:, 1:] += np.einsum('aik,kl,bil->ab', part, noise, part)
    reports, _, _ = interval_bounds(model, params)
    scales = np.maximum(abs(model.theta_min), abs(model.theta_max))
    for k, report in enumerate(reports):
        errors = []
        for theta in [model.theta_min[k], model.theta_max[k]]:
            x = (theta + model.joint_offsets[k]) / scales[k]
            gradient = np.zeros(34)
            gradient[22 + 2 * k:24 + 2 * k] = np.array([1., 2 * x]) / scales[k]
            errors.append(float(np.sqrt(max(0., gradient @ covariance @ gradient))))
        active = int(np.argmax(abs(np.array(report['endpoint_derivatives']))))
        report['endpoint_derivative_noise_se'] = errors
        report['selected_endpoint_excess_in_noise_se'] = (
            (report['max_absolute_derivative'] - report['derivative_limit']) / errors[active])
    return reports, covariance

def noise_couplings(covariance):
    """Largest local random-error correlations of wrist polynomial terms."""
    names = ([f'base_{part}_{axis}' for part in ['rot', 'trans'] for axis in 'xyz']
             + [f'axis{k}_{axis}' for k, axes in zip(range(2, 7), ['xz', 'xy', 'yz', 'xz', 'xy'])
                for axis in axes]
             + [f'tool_{part}_{axis}' for part in ['rot', 'trans'] for axis in 'xyz']
             + [f'joint{k}_c{j}' for k in range(1, 7) for j in [1, 2]])
    deviations = np.sqrt(np.maximum(0., np.diag(covariance)))
    denominator = np.outer(deviations, deviations)
    correlation = np.divide(covariance, denominator, out=np.zeros_like(covariance),
                            where=denominator > 0)
    result = {}
    for k in range(28, 34):
        indices = sorted((j for j in range(1, 34) if j != k),
                         key=lambda j: -abs(correlation[k, j]))[:3]
        result[names[k]] = dict(noise_se=float(deviations[k]),
                               correlations=[dict(parameter=names[j], value=float(correlation[k, j]))
                                             for j in indices])
    return result

def constrained_fit(model, data, initial, maxiter=30, verbose=True):
    """Compare a candidate using the production fitter; leave caller unchanged."""
    from copy import copy
    candidate_model = copy(model)
    candidate_model.params = np.asarray(initial, dtype=float).copy()
    info = candidate_model.fit(data, max_nfev=maxiter, ftol=1e-11, verbose=int(verbose))
    info['model_params_updated'] = False  # Only the separate candidate was fitted.
    return candidate_model.params.copy(), info, finite_jacobian(model, data, initial)

def run_bound_diagnostic():
    archive = ROOT / 'results/step8_phase_d'
    output = ROOT / 'results/step8_bound_diagnostic'
    output.mkdir(parents=True, exist_ok=True)
    paths = [archive / name for name in ['calibration.npz', 'holdout.npz', 'fit.npz', 'summary.json']]
    original_hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    train, extra = load_data(archive / 'calibration.npz')
    holdout, _ = load_data(archive / 'holdout.npz')
    if set(groups(train)) & set(groups(holdout)):
        raise ValueError('Holdout command identities overlap training commands')
    nominal = Robot('bound_diagnostic', Frame.eye(), constants.THETA_MIN, constants.THETA_MAX,
                    constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)
    model = ImprovedKinematics(nominal, Frame(extra['base_frame']), joint_offsets=extra['home_offsets'])
    with np.load(archive / 'fit.npz', allow_pickle=False) as saved:
        initial = saved['fitted_params'].copy()
    before_clock = clock.elapsed
    print('UNCONSTRAINED_BOUNDS', json.dumps(interval_bounds(model, initial)[0]), flush=True)
    candidate, info, jacobian = constrained_fit(model, train, initial)
    if not info['success']:
        raise RuntimeError(f'Constrained candidate did not converge feasibly: {info}')
    noise, covariance = sampling_uncertainty(model, train, initial, jacobian)
    cases = dict(training_raw=train, training_pose_means=mean_data(train),
                 holdout_raw=holdout, holdout_pose_means=mean_data(holdout))
    comparison = {name: dict(unconstrained=summary(model, data, initial),
                             constrained=summary(model, data, candidate))
                  for name, data in cases.items()}
    report = dict(fit=info, bounds_unconstrained=noise,
                  bounds_constrained=interval_bounds(model, candidate)[0], errors=comparison,
                  wrist_noise_coupling=noise_couplings(covariance),
                  original_unconstrained_fit=json.loads((archive / 'summary.json').read_text())['fit'],
                  archive_sha256=original_hashes, additional_simulated_time_s=clock.elapsed-before_clock,
                  covariance_scope='Linearized random sampling only; fixed home/model; no bias uncertainty',
                  estimator='SLSQP; exact full-interval quadratic magnitude and endpoint derivative bounds')
    assert clock.elapsed == before_clock
    for p in paths:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == original_hashes[str(p.relative_to(ROOT))]
    np.savez_compressed(output / 'candidate.npz', unconstrained_params=initial,
                        constrained_params=candidate, unconstrained_sampling_covariance=covariance)
    (output / 'summary.json').write_text(json.dumps(report, default=json_default, indent=2) + '\n')
    print('CONSTRAINED_FIT', json.dumps(info, default=json_default), flush=True)
    print('COMPARISON', json.dumps(comparison), flush=True)


if __name__ == '__main__':
    # Original relative imports depend on notebook cwd. No protected file is edited.
    import os
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analyze-only', action='store_true', help='No acquisition or fitting; use archives')
    parser.add_argument('--bounds', action='store_true', help='Saved-data physical-bound/covariance comparison using the production fitter')
    args = parser.parse_args()
    os.chdir(ROOT / 'notebooks')
    if args.bounds:
        run_bound_diagnostic()
    else:
        run(analyze_only=args.analyze_only)
