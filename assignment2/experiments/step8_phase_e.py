"""Adopt constrained fitting, physical reporting, and supplied-IK audit.

Uses saved measurements only; does not invoke Steps 9--10 or truth APIs.
Run from assignment2: venv/bin/python experiments/step8_phase_e.py
"""

import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.step8_phase_d import frame_array, groups, json_default, load_data, mean_data, summary
from src import Frame, Rot, vct3
from src.robot import Robot
from src.simulation import clock, constants
from src.student.kinematics import ImprovedKinematics
from src.student.metrics import frame_error


def saved_model():
    train, extra = load_data(ROOT / 'results/step8_phase_d/calibration.npz')
    robot = Robot('phase_e', Frame.eye(), constants.THETA_MIN, constants.THETA_MAX,
                  constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)
    return ImprovedKinematics(robot, Frame(extra['base_frame']), joint_offsets=extra['home_offsets']), train


def ik_audit(model, heldout):
    """Pose round trips, not joint-vector equality or physical robot accuracy."""
    started = perf_counter()
    cases = [('heldout', f'heldout_{i}', np.array(theta)) for i, theta in enumerate(groups(heldout))]
    center = np.array([300., 0., 300., .3, .5, -.4])
    for k in range(6):
        for side, limit in [('lower', model.theta_min[k]), ('upper', model.theta_max[k])]:
            theta = center.copy()
            theta[k] = limit
            cases.append(('limits', f'joint{k+1}_{side}', theta))
            near = theta.copy()
            near[k] += (1 if side == 'lower' else -1) * (1e-3 if k < 3 else 1e-5)
            cases.append(('near_limits', f'joint{k+1}_{side}_interior', near))
    for side, theta in [('all_lower', model.theta_min), ('all_upper', model.theta_max)]:
        cases.append(('limits', side, theta.copy()))
    report = model.parameters_report()
    axes = report['axes']
    scale = report['joint_normalization_scales'][4]
    poly = np.polynomial.Polynomial(np.r_[model.joint_offsets[4], model.params[22:].reshape(6, model.n_poly)[4]])

    def wrist_matrix(theta5):
        q5 = theta5 + poly((theta5 + model.joint_offsets[4])/scale)
        last_axis = (Rot(vct3(axes[4]), angle=q5) * vct3(axes[5])).vec.ravel()
        # A common R4 rotation does not affect singular values or determinant.
        return np.column_stack([axes[3], axes[4], last_axis])

    poles = []
    for sign in [-1, 1]:
        nominal = sign * np.pi/2
        root = brentq(lambda t: np.linalg.det(wrist_matrix(t)), nominal-.15, nominal+.15, xtol=1e-14)
        poles.append(dict(theta5_rad=float(root), smallest_axis_singular_value=float(np.linalg.svd(wrist_matrix(root), compute_uv=False)[-1])))
        for delta in [-.01, -.001, -.0001, 0., .0001, .001, .01]:
            theta = center.copy()
            theta[3], theta[4] = .7, root+delta
            cases.append(('near_singular', f'pole_{sign}_offset_{delta}', theta))
        theta = center.copy()
        theta[3], theta[4] = .7, nominal
        cases.append(('nominal_poles', f'nominal_pole_{sign}', theta))
    rows = []
    for category, name, theta in cases:
        goal = model.fwd_kins(theta)
        seed = np.clip(theta + np.array([.1, -.1, .1, .05, -.03, .04]), model.theta_min, model.theta_max)
        row = dict(category=category, name=name, theta=theta.copy())
        try:
            recovered = model.inv_kins(goal, seed=seed)
            angle, position = frame_error(goal, model.fwd_kins(recovered))
            inside = model.within_limits(recovered)
            row.update(recovered=recovered, angle_rad=angle, position_mm=position, in_limits=inside,
                       strict_pass=bool(inside and angle < 1e-8 and position < 1e-6),
                       target_sized_consistency_pass=bool(inside and angle <= .001 and position <= .2))
        except ValueError as error:
            row.update(strict_pass=False, target_sized_consistency_pass=False,
                       error=str(error).split(' for ')[0])
        rows.append(row)
    categories = {}
    for category in sorted({r['category'] for r in rows}):
        part = [r for r in rows if r['category'] == category]
        returned = [r for r in part if 'angle_rad' in r]
        categories[category] = dict(n=len(part), strict_pass=sum(r['strict_pass'] for r in part),
                                    target_sized_consistency_pass=sum(r['target_sized_consistency_pass'] for r in part),
                                    exceptions=sum('error' in r for r in part),
                                    max_angle_rad=max((r['angle_rad'] for r in returned), default=None),
                                    max_position_mm=max((r['position_mm'] for r in returned), default=None))
    # Assess existing public solver controls without changing the supplied IK.
    retries = []
    for row in rows:
        if row['strict_pass']:
            continue
        theta = row['theta']
        seed = np.clip(theta + np.array([.1, -.1, .1, .05, -.03, .04]), model.theta_min, model.theta_max)
        retry = dict(name=row['name'], strict_pass=False, target_sized_consistency_pass=False)
        try:
            back = model.inv_kins(model.fwd_kins(theta), seed=seed, max_iter=200, tol=1e-12)
            angle, position = frame_error(model.fwd_kins(theta), model.fwd_kins(back))
            retry.update(angle_rad=angle, position_mm=position,
                         strict_pass=bool(model.within_limits(back) and angle < 1e-8 and position < 1e-6),
                         target_sized_consistency_pass=bool(model.within_limits(back) and angle <= .001 and position <= .2))
        except ValueError as error:
            retry['error'] = str(error).split(' for ')[0]
        retries.append(retry)
    return dict(categories=categories, fitted_wrist_poles=poles, rows=rows,
                public_controls=dict(max_iter=200, tol=1e-12, retried=len(retries),
                                     rescued=sum(r['strict_pass'] for r in retries), rows=retries,
                                     combined_strict_pass=sum(r['strict_pass'] for r in rows)+sum(r['strict_pass'] for r in retries),
                                     combined_target_sized_consistency_pass=sum(r['strict_pass'] for r in rows)+sum(r['target_sized_consistency_pass'] for r in retries)),
                wall_time_s=perf_counter()-started,
                interpretation='Improved-model FK/IK consistency only; no robot ground-truth evaluation')


def encode(value):
    return frame_array(value).tolist() if isinstance(value, Frame) else json_default(value)


def run(audit_only=False):
    output = ROOT / 'results/step8_phase_e'
    output.mkdir(parents=True, exist_ok=True)
    originals = [ROOT / 'results/step8_phase_d' / name for name in ['calibration.npz', 'holdout.npz', 'fit.npz', 'summary.json']]
    originals += [ROOT / 'results/step8_bound_diagnostic' / name for name in ['candidate.npz', 'summary.json']]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    heldout, _ = load_data(ROOT / 'results/step8_phase_d/holdout.npz')
    warm, train = saved_model()
    with np.load(ROOT / 'results/step8_phase_d/fit.npz', allow_pickle=False) as saved:
        warm.params = saved['fitted_params'].copy()
    with np.load(ROOT / 'results/step8_bound_diagnostic/candidate.npz', allow_pickle=False) as saved:
        reference = saved['constrained_params'].copy()
    start_clock = clock.elapsed
    cold, _ = saved_model()
    if audit_only:
        previous = json.loads((output/'summary.json').read_text())
        with np.load(output/'fit.npz', allow_pickle=False) as saved:
            cold.params, warm.params = saved['fitted_params'].copy(), saved['warm_params'].copy()
        warm_info, cold_info = previous['warm_fit'], previous['cold_fit']
    else:
        print('VERIFY INTEGRATED WARM FIT', flush=True)
        warm_info = warm.fit(train, verbose=1)
        if not warm_info['success']:
            raise RuntimeError(f'Integrated warm fit failed: {warm_info}')
        print('VERIFY NORMAL COLD FIT', flush=True)
        cold_info = cold.fit(train, verbose=1)
        if not cold_info['success']:
            raise RuntimeError(f'Normal cold fit failed: {cold_info}')
    cases = dict(training_raw=train, training_pose_means=mean_data(train),
                 holdout_raw=heldout, holdout_pose_means=mean_data(heldout))
    errors = {key: summary(cold, data, cold.params) for key, data in cases.items()}
    difference = {}
    for label, params in [('integrated_warm', warm.params), ('normal_cold', cold.params)]:
        angle, position = [], []
        for theta in list(groups(train)) + list(groups(heldout)):
            a, p = frame_error(cold.predict_in_tracker(theta, reference), cold.predict_in_tracker(theta, params))
            angle.append(a)
            position.append(p)
        difference[label] = dict(max_angle_rad=max(angle), max_position_mm=max(position),
                                 max_parameter_difference=float(max(abs(params-reference))))
    parameters = cold.parameters_report()
    print('RUN PROVIDED IK AUDIT', flush=True)
    audit = ik_audit(cold, heldout)
    report = dict(warm_fit=warm_info, cold_fit=cold_info, candidate_reproduction=difference,
                  adopted_fit='normal cold-start constrained fit', errors=errors, ik=audit,
                  original_archive_sha256=hashes, additional_simulated_time_s=clock.elapsed-start_clock)
    if not audit_only:
        np.savez_compressed(output / 'fit.npz', fitted_params=cold.params, warm_params=warm.params,
                            reference_params=reference)
    (output / 'parameters.json').write_text(json.dumps(parameters, default=encode, indent=2)+'\n')
    (output / 'summary.json').write_text(json.dumps(report, default=encode, indent=2)+'\n')
    for p in originals:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p.relative_to(ROOT))]
    assert clock.elapsed == start_clock
    print('FIT REPRODUCTION', difference, flush=True)
    print('IK AUDIT', audit['categories'], flush=True)


def verify_saved_fit():
    """Re-fit saved public observations without overwriting historical evidence."""
    paths = [p for p in (ROOT/'results').rglob('*') if p.is_file()]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    model, train = saved_model()
    heldout, _ = load_data(ROOT/'results/step8_phase_d/holdout.npz')
    with np.load(ROOT/'results/step8_phase_d/fit.npz', allow_pickle=False) as saved:
        model.params = saved['fitted_params'].copy()
    with np.load(ROOT/'results/step8_phase_e/fit.npz', allow_pickle=False) as saved:
        reference = saved['fitted_params'].copy()
    started = clock.elapsed
    info = model.fit(train, verbose=1)
    assert info['success'] and info['physical_constraints'] and model.params[0] == 0
    differences = [frame_error(model.predict_in_tracker(q, reference), model.predict_in_tracker(q))
                   for q in list(groups(train)) + list(groups(heldout))]
    maximum = np.max(differences, axis=0)
    assert maximum[0] < 1e-7 and maximum[1] < 1e-5
    report = model.parameters_report()
    assert report['physical_joint_bounds_satisfied'] and clock.elapsed == started
    for path, digest in hashes.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == digest
    result = dict(fit=info, training_raw=summary(model, train, model.params),
                  heldout_raw=summary(model, heldout, model.params),
                  heldout_pose_means=summary(model, mean_data(heldout), model.params),
                  max_difference_from_adopted=dict(angle_rad=maximum[0], position_mm=maximum[1]),
                  physical_bounds_satisfied=True, additional_simulated_time_s=clock.elapsed-started,
                  archive_hashes_preserved=True)
    print(json.dumps(result, default=encode, indent=2), flush=True)
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-only', action='store_true', help='Reuse integrated fits; no acquisition or fitting')
    parser.add_argument('--verify-fit', action='store_true', help='Verify the default fit on saved data; do not overwrite archives')
    args = parser.parse_args()
    if args.verify_fit:
        verify_saved_fit()
    else:
        run(audit_only=args.audit_only)
