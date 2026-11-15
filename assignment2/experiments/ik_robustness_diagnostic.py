"""Instrumented IK replay and ablations only; production IK is never edited.

Run from assignment2: venv/bin/python experiments/ik_robustness_diagnostic.py
Reads the adopted parameters and the original 78 FK-generated test commands.
"""

import hashlib
import gzip
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from experiments.step8_phase_e import encode, saved_model
from src.student.metrics import frame_error
from src import Rot, vct3
from src.simulation import clock
from uncertainty_networks.se3 import log_so3

STRICT_ANGLE, STRICT_POSITION = 1e-8, 1e-6


def pose_metrics(model, goal, theta):
    angle, position = frame_error(goal, model.fwd_kins(theta))
    return dict(angle_rad=angle, position_mm=position,
                strict=bool(angle < STRICT_ANGLE and position < STRICT_POSITION))


def violation(model, theta):
    return np.maximum(np.maximum(model.theta_min-theta, theta-model.theta_max), 0.)


def target_jacobian(model, theta):
    """Analytic body pose-error derivative at the generating solution.

    Angular columns are joint axes; positional columns for wrist joints are
    axis cross tool lever. Rotate both into the EE frame and negate for
    residual FK(theta)^-1 * target. This uses the fitted model only.
    """
    report = model.parameters_report()
    axes, scales = report['axes'], report['joint_normalization_scales']
    u = np.asarray(theta)+model.joint_offsets
    coefficients = model.params[22:].reshape(6,model.n_poly)
    powers = np.arange(1,model.n_poly+1)
    q = u+np.sum(coefficients*(u/scales)[:,None]**powers,axis=1)
    gain = 1+np.sum(coefficients*powers*(u/scales)[:,None]**(powers-1),axis=1)/scales
    r4 = Rot(vct3(axes[3]),angle=q[3]).matrix
    r5 = Rot(vct3(axes[4]),angle=q[4]).matrix
    wrist_axes = model.F_CW.R.matrix @ np.column_stack([axes[3],r4@axes[4],r4@r5@axes[5]])
    frame = model.fwd_kins(theta)
    center = q[:3] @ axes[:3]+model.F_CW.p.vec.ravel()
    lever = frame.p.vec.ravel()-center
    matrix = np.zeros((6,6))
    matrix[3:,:3] = axes[:3].T*gain[:3]
    matrix[:3,3:] = wrist_axes*gain[3:]
    matrix[3:,3:] = np.column_stack([np.cross(wrist_axes[:,i],lever)*gain[i+3] for i in range(3)])
    matrix[:3] = -frame.R.matrix.T @ matrix[:3]
    matrix[3:] = -frame.R.matrix.T @ matrix[3:]
    return matrix


def classify_failures(model, failures):
    counts, missed_valid = {}, []
    for case in failures:
        run = case['runs']['default']
        chosen = run['selected_branch']
        if chosen is None:
            kind = 'numerical_boundary_rejection'
        else:
            branch = run['branches'][chosen]
            kind = ('nonperiodic_wrist_wrapping' if branch['before_metrics']['strict']
                    and not branch['final_metrics']['strict'] else 'iteration_limit_without_final_validation')
            accurate = [i for i,b in enumerate(run['branches']) if b['final_in_limits'] and b['final_metrics']['strict']]
            if accurate:
                missed_valid.append(case['name'])
        counts[kind] = counts.get(kind,0)+1
        matrix = target_jacobian(model,np.asarray(case['target']))
        balanced = matrix*np.r_[[1000.]*3,[5.]*3][:,None]*np.r_[[50.]*3,[.3]*3][None,:]
        singular = np.linalg.svd(balanced,compute_uv=False)
        case['classification'] = kind
        case['target_conditioning'] = dict(balanced_singular_values=singular,
            balanced_condition_number=float(singular[0]/singular[-1]),
            numerical_rank=int(np.count_nonzero(singular > singular[0]*1e-10)))
    return dict(primary_counts=counts, inaccurate_selection_despite_accurate_candidate=missed_valid,
                forced_acceptances_observed=sum(b['forced_acceptances'] for c in failures for b in c['runs']['default']['branches']))


def replay(model, goal, seed, max_iter=50, tol=1e-10, include_seed=False,
           wrap=True, gate=False, snap=False, feasible=False, augmented=False,
           weighted=False, history=False, early_seed=False):
    """Faithful default replay, with individually labelled diagnostic switches.

    The no-switch path preserves the provided arithmetic, stopping, damping,
    wrapping and selection order. Additional metrics never alter iterates.
    None of the alternative branches is installed as a production solver.
    """
    started = perf_counter()
    p_w = goal.p - goal.R * model.F_WE.p - model.F_CW.p
    guesses = [(f'nominal_{i}', np.r_[p_w.x, p_w.y, p_w.z, wrist])
               for i, wrist in enumerate(model.wrist_solutions(goal.R))]
    if include_seed:
        guesses.insert(0, ('caller_seed', np.asarray(seed).copy()))
    step_max = np.r_[[50.] * 3, [.3] * 3]
    h = np.r_[[1e-3] * 3, [1e-6] * 3]
    weights = np.r_[[1/.001] * 3, [1/.2] * 3] if weighted else np.ones(6)

    def delta(theta):
        difference = model.fwd_kins(theta).inv() * goal
        return np.r_[log_so3(difference.R.matrix), difference.p.vec.ravel()]

    branches, candidates = [], []
    for label, guess in guesses:
        theta = np.clip(guess, model.theta_min, model.theta_max) if feasible else guess.copy()
        raw = delta(theta)
        lam = 1e-4
        branch = dict(label=label, initial_guess=guess.copy(), initial_used=theta.copy(),
                      initial_metrics=pose_metrics(model, goal, theta),
                      initial_limit_violation=violation(model, theta), iterations=0,
                      rejected_steps=0, forced_acceptances=0, accepted_nonreducing_steps=0,
                      out_of_limit_iterates=0, maximum_limit_violation=violation(model, theta),
                      raw_condition_max=0., balanced_condition_max=0.,
                      smallest_balanced_singular_value=float('inf'),
                      lambda_min=lam, lambda_max=lam, reason='iteration_limit', history=[])
        for iteration in range(max_iter):
            residual = raw * weights
            cost = float(residual @ residual)
            if np.sqrt(float(raw @ raw)) < tol:
                branch['reason'] = 'raw_residual_tolerance'
                break
            jacobian = np.zeros((6, 6))
            for k in range(6):
                shifted = theta.copy()
                increment = -h[k] if feasible and theta[k]+h[k] > model.theta_max[k] else h[k]
                shifted[k] += increment
                jacobian[:, k] = (delta(shifted)-raw) / increment
            raw_singular = np.linalg.svd(jacobian, compute_uv=False)
            balanced = jacobian * np.r_[[1000.] * 3, [5.] * 3][:, None] * step_max[None, :]
            balanced_singular = np.linalg.svd(balanced, compute_uv=False)
            raw_condition = float(raw_singular[0] / raw_singular[-1])
            balanced_condition = float(balanced_singular[0] / balanced_singular[-1])
            branch['raw_condition_max'] = max(branch['raw_condition_max'], raw_condition)
            branch['balanced_condition_max'] = max(branch['balanced_condition_max'], balanced_condition)
            branch['smallest_balanced_singular_value'] = min(branch['smallest_balanced_singular_value'], float(balanced_singular[-1]))
            branch['last_jacobian'] = jacobian
            lam_before = lam
            while True:
                # Weighted ablation uses dimensionless residual/joint coordinates.
                matrix = weights[:, None] * jacobian
                matrix = matrix * step_max[None, :] if weighted else matrix
                normal = matrix.T @ matrix
                diagonal = np.diag(normal) + 1e-12
                if augmented:
                    step = np.linalg.lstsq(np.vstack([matrix, np.diag(np.sqrt(lam * diagonal))]),
                                           np.r_[-residual, np.zeros(6)], rcond=None)[0]
                else:
                    step = -np.linalg.solve(normal + lam * np.diag(diagonal), matrix.T @ residual)
                if weighted:
                    step = step * step_max
                trial = theta + np.clip(step, -step_max, step_max)
                if feasible:
                    trial = np.clip(trial, model.theta_min, model.theta_max)
                trial_raw = delta(trial)
                trial_cost = float((trial_raw * weights) @ (trial_raw * weights))
                branch['lambda_max'] = max(branch['lambda_max'], lam)
                if trial_cost < cost or lam > 1e6:
                    forced = lam > 1e6
                    branch['forced_acceptances'] += int(forced)
                    branch['accepted_nonreducing_steps'] += int(trial_cost >= cost)
                    theta, raw, lam = trial, trial_raw, max(lam/10., 1e-12)
                    break
                branch['rejected_steps'] += 1
                lam *= 10.
            branch['iterations'] += 1
            branch['lambda_min'] = min(branch['lambda_min'], lam)
            exceeded = violation(model, theta)
            branch['maximum_limit_violation'] = np.maximum(branch['maximum_limit_violation'], exceeded)
            branch['out_of_limit_iterates'] += int(np.any(exceeded > 1e-9))
            if history:
                branch['history'].append(dict(iteration=iteration+1, theta=theta.copy(),
                                               raw_cost=float(raw @ raw), angle_norm_rad=float(np.linalg.norm(raw[:3])),
                                               position_norm_mm=float(np.linalg.norm(raw[3:])),
                                               lambda_before=lam_before, lambda_after=lam,
                                               raw_condition=raw_condition, balanced_condition=balanced_condition,
                                               raw_singular_values=raw_singular,
                                               balanced_singular_values=balanced_singular,
                                               limit_violation=exceeded.copy(), forced_acceptance=forced))
        branch['before_postprocessing'] = theta.copy()
        branch['before_metrics'] = pose_metrics(model, goal, theta)
        branch['before_in_limits'] = model.within_limits(theta)
        branch['before_limit_violation'] = violation(model, theta)
        if wrap:
            theta[3] = (theta[3]+np.pi) % (2*np.pi)-np.pi
            theta[5] = (theta[5]+np.pi) % (2*np.pi)-np.pi
        if snap and np.all(violation(model, theta) <= np.r_[[1e-6]*3, [1e-8]*3]):
            theta = np.clip(theta, model.theta_min, model.theta_max)
        branch['final_theta'] = theta.copy()
        branch['final_metrics'] = pose_metrics(model, goal, theta)
        branch['final_in_limits'] = model.within_limits(theta)
        branch['final_limit_violation'] = violation(model, theta)
        branch['selection_distance'] = float(np.linalg.norm(theta-seed))
        branch['accepted_candidate'] = bool(model.within_limits(theta) and
                                            (not gate or branch['final_metrics']['strict']))
        branches.append(branch)
        if branch['accepted_candidate']:
            candidates.append(len(branches)-1)
            if early_seed and label == 'caller_seed' and branch['final_metrics']['strict']:
                break
    selected = min(candidates, key=lambda i: branches[i]['selection_distance']) if candidates else None
    result = branches[selected]['final_theta'] if selected is not None else None
    return dict(branches=branches, selected_branch=selected, result=result,
                strict_pass=bool(selected is not None and branches[selected]['final_metrics']['strict']),
                target_sized_pass=bool(selected is not None and branches[selected]['final_metrics']['angle_rad'] <= .001
                                      and branches[selected]['final_metrics']['position_mm'] <= .2),
                wall_time_s=perf_counter()-started)


def summarize(rows):
    valid = [r for r in rows if r['selected_branch'] is not None]
    metrics = [r['branches'][r['selected_branch']]['final_metrics'] for r in valid]
    return dict(n=len(rows), strict_pass=sum(r['strict_pass'] for r in rows),
                target_sized_pass=sum(r['target_sized_pass'] for r in rows),
                no_candidate=len(rows)-len(valid),
                max_angle_rad=max((m['angle_rad'] for m in metrics), default=None),
                max_position_mm=max((m['position_mm'] for m in metrics), default=None),
                wall_time_s=sum(r['wall_time_s'] for r in rows))


def write_failed_report(failures, destination):
    lines = ['# Complete failed-case IK trace', '',
             'Units: first three joint entries mm, last three rad. All targets are reachable by construction.',
             'Each branch includes its initial guess, pre/post-wrap result, limits and termination reason.',
             'Full iteration/Jacobian/damping histories are in `traces.json.gz`.', '']
    for case in failures:
        run = case['runs']['default']
        lines.extend([f"## {case['name']} ({case['category']})", '',
                      f"Target: `{np.array2string(np.array(case['target']), precision=12, separator=', ')}`", '',
                      f"Caller seed: `{np.array2string(np.array(case['seed']), precision=12, separator=', ')}`", '',
                      f"Selected branch: `{run['selected_branch']}`; strict pass: `{run['strict_pass']}`.", '',
                      f"Classification: **{case.get('classification','see summary')}**. The generating command is an in-limit exact solution.", '',
                      '| Setting | Returned joints | Position (mm) | Orientation (rad) | Termination / limits |',
                      '| --- | --- | ---: | ---: | --- |'])
        for label in ['default','iterations_200','iterations_200_tight','minimal_weighted_fast']:
            result=case['runs'][label]
            index=result['selected_branch']
            if index is None:
                lines.append(f'| {label} | no candidate | — | — | no accepted in-limit candidate |')
            else:
                b=result['branches'][index]
                joints=np.array2string(np.array(b['final_theta']),precision=10,separator=', ',max_line_width=1000)
                lines.append(f"| {label} | `{joints}` | {b['final_metrics']['position_mm']:.10g} | "
                             f"{b['final_metrics']['angle_rad']:.10g} | {b['reason']}; in limits: {b['final_in_limits']} |")
        lines.append('')
        for i, branch in enumerate(run['branches']):
            def vector(key):
                return np.array2string(np.array(branch[key]), precision=12, separator=', ')
            lines.extend([f"### Branch {i}: {branch['label']}", '', '```text',
                          'initial guess: '+vector('initial_guess'),
                          'before wrap : '+vector('before_postprocessing'),
                          'final joints: '+vector('final_theta'),
                          'limit excess: '+vector('final_limit_violation'), '```', '',
                          f"Termination: {branch['reason']}; iterations: {branch['iterations']}; final in limits: {branch['final_in_limits']}.", '',
                          f"Before wrap: {branch['before_metrics']['position_mm']:.12g} mm / {branch['before_metrics']['angle_rad']:.12g} rad. "
                          f"After wrap: **{branch['final_metrics']['position_mm']:.12g} mm / {branch['final_metrics']['angle_rad']:.12g} rad**.", '',
                          f"Max balanced Jacobian condition: {branch['balanced_condition_max']:.6g}; damping range "
                          f"{branch['lambda_min']:.6g}...{branch['lambda_max']:.6g}; rejected steps: {branch['rejected_steps']}; "
                          f"forced accepts: {branch['forced_acceptances']}.", ''])
        lines.extend(['Other solver settings and ablations: see this case in `traces.json.gz`.', ''])
    destination.write_text('\n'.join(lines).rstrip()+'\n')


def run(selected_settings=None):
    output = ROOT / 'results/ik_robustness_diagnostic'
    output.mkdir(parents=True, exist_ok=True)
    model, _ = saved_model()
    fit_path = ROOT / 'results/step8_phase_e/fit.npz'
    source_path = ROOT / 'src/student/kinematics.py'
    source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
    fit_hash = hashlib.sha256(fit_path.read_bytes()).hexdigest()
    with np.load(fit_path, allow_pickle=False) as saved:
        model.params = saved['fitted_params'].copy()
    original = model.params.copy()
    phase_e = json.loads((ROOT/'results/step8_phase_e/summary.json').read_text())
    cases = phase_e['ik']['rows']
    failed_names = {r['name'] for r in cases if not r['strict_pass']}
    settings = {
        'default': {},
        'iterations_200': dict(max_iter=200),
        'iterations_200_tight': dict(max_iter=200, tol=1e-12),
        'add_caller_seed': dict(include_seed=True),
        'no_wrap': dict(wrap=False),
        'seed_unwrapped_gated': dict(include_seed=True, wrap=False, gate=True, snap=True),
        'seed_unwrapped_feasible': dict(include_seed=True, wrap=False, gate=True, feasible=True, snap=True),
        'seed_feasible_augmented': dict(include_seed=True, wrap=False, gate=True, feasible=True, snap=True, augmented=True),
        'seed_feasible_weighted': dict(include_seed=True, wrap=False, gate=True, feasible=True, snap=True, augmented=True, weighted=True),
        'seed_unwrapped_weighted': dict(include_seed=True, wrap=False, gate=True, snap=True, augmented=True, weighted=True),
        'feasible_weighted_without_seed': dict(wrap=False, gate=True, feasible=True, snap=True, augmented=True, weighted=True),
        'seed_feasible_weighted_normal': dict(include_seed=True, wrap=False, gate=True, feasible=True, snap=True, weighted=True),
        'minimal_weighted_fast': dict(include_seed=True, wrap=False, gate=True, feasible=True, snap=True, augmented=True, weighted=True, early_seed=True),
    }
    traces = [dict(name=c['name'], category=c['category'], target=c['theta'], runs={}) for c in cases]
    previous = {}
    if selected_settings:
        previous = json.loads((output/'summary.json').read_text())
        if (output/'traces.json.gz').exists():
            with gzip.open(output/'traces.json.gz','rt') as handle:
                old_traces = json.load(handle)
        else:
            old_traces = json.loads((output/'traces.json').read_text())
        cached = {entry['name']:entry for entry in old_traces}
        for entry in traces:
            if entry['name'] in cached:
                entry.update(cached[entry['name']])
    executing = {k:settings[k] for k in selected_settings} if selected_settings else settings
    start_clock = clock.elapsed
    for setting, options in executing.items():
        print('SETTING', setting, flush=True)
        for entry in traces:
            theta = np.asarray(entry['target'])
            goal = model.fwd_kins(theta)
            seed = np.clip(theta+np.array([.1,-.1,.1,.05,-.03,.04]),model.theta_min,model.theta_max)
            entry['seed'] = seed
            result = replay(model, goal, seed, history=setting=='default' and entry['name'] in failed_names, **options)
            if setting == 'default':
                try:
                    actual = model.inv_kins(goal, seed=seed)
                    assert result['result'] is not None and np.allclose(actual,result['result'],atol=1e-11,rtol=0)
                except ValueError:
                    assert result['result'] is None
                entry['generating_solution'] = dict(in_limits=model.within_limits(theta), metrics=pose_metrics(model,goal,theta))
            entry['runs'][setting] = result
        print('SUMMARY', setting, summarize([e['runs'][setting] for e in traces]), flush=True)
    summaries = previous.get('summaries', {}).copy()
    summaries.update({label: summarize([r['runs'][label] for r in traces]) for label in executing})
    groups = previous.get('categories', {}).copy()
    for label in executing:
        groups[label] = {category: summarize([e['runs'][label] for e in traces if e['category']==category])
                         for category in sorted({e['category'] for e in traces})}
    failures = [e for e in traces if e['name'] in failed_names]
    classification = classify_failures(model,failures)
    (output/'summary.json').write_text(json.dumps(dict(settings={k:settings[k] for k in summaries}, summaries=summaries, categories=groups,
        failed_case_names=sorted(failed_names), cases=len(traces), source_sha256=source_hash,
        classification=classification, fit_sha256=fit_hash, additional_simulated_time_s=clock.elapsed-start_clock),default=encode,indent=2)+'\n')
    serialized = json.dumps(failures,default=encode,indent=2)+'\n'
    with gzip.open(output/'traces.json.gz','wt') as handle:
        handle.write(serialized)
    if (output/'traces.json').exists():
        (output/'traces.json').unlink()  # Superseded file generated by this diagnostic only.
    write_failed_report(json.loads(serialized), output/'failed_cases.md')
    assert clock.elapsed == start_clock and np.array_equal(model.params,original)
    assert hashlib.sha256(source_path.read_bytes()).hexdigest()==source_hash
    assert hashlib.sha256(fit_path.read_bytes()).hexdigest()==fit_hash


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', nargs='+', help='Run only named ablations, preserving earlier traces')
    run(parser.parse_args().settings)
