"""Run the original Step 10 protocol on frozen calibration, without changing IK.

From assignment2: venv/bin/python experiments/step10_inverse.py
Public motion replay restores the archived post-calibration robot state/RNG.
No tracker samples, new calibration data, refitting or hidden parameters.
"""

import ast
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.step8_phase_d import json_default
from experiments.step9_forward import execute_step9, notebook_cells, saved_scene
from src.simulation import clock, constants
from src.student.metrics import summarize_errors


def replay_calibration_motion(namespace, train, acquisition):
    """Restore noise draws using public actions, without sampling or fitting."""
    cells, _ = notebook_cells()
    source = ast.parse(''.join(cells[16]['source']))
    home_call = source.body[0].value
    n_homings = ast.literal_eval(next(k.value for k in home_call.keywords if k.arg == 'n_homings'))
    assignments = [node for node in source.body if isinstance(node, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id in {'center', 'baseline_poses'} for t in node.targets)]
    exec(compile(ast.Module(body=assignments, type_ignores=[]), 'notebook:original-baseline-poses', 'exec'), namespace)
    robot = namespace['rob']
    started = clock.elapsed
    readings = np.array([robot.home_robot() for _ in range(n_homings)])
    if not np.allclose(-readings.mean(axis=0), namespace['improved'].joint_offsets, atol=1e-12, rtol=0):
        raise ValueError('Public homing replay differs from the saved calibration; do not mix robot states')
    for theta in namespace['baseline_poses']:
        robot.move_joints(theta)
    for theta in train.thetas:
        robot.move_joints(theta)
    elapsed = clock.elapsed-started
    omitted = (len(namespace['baseline_poses'])+len(train))*constants.SAMPLE_DELTA_T_SEC
    expected = acquisition['training']['simulated_total_including_step6_s']-omitted
    assert np.isclose(elapsed, expected, atol=1e-9, rtol=0)
    assert np.array_equal(robot.state().theta, train.thetas[-1])
    return dict(simulated_motion_replay_s=elapsed, expected_motion_replay_s=expected,
                homings=n_homings, baseline_moves=len(namespace['baseline_poses']), calibration_moves=len(train),
                omitted_tracker_sampling_time_s=omitted, tracker_samples=0,
                home_offsets_match=True, initial_testing_command=robot.state().theta.tolist(),
                interpretation='Reconstruction overhead in a fresh simulator, not new calibration or Step 10 testing')


def execute_step10(namespace):
    cells, _ = notebook_cells()
    heading = next(i for i, c in enumerate(cells)
                   if c['cell_type'] == 'markdown' and ''.join(c['source']).startswith('## Step 10:'))
    started = perf_counter()
    exec(compile(''.join(cells[heading+1]['source']), f'assignment2_template.ipynb:cell{heading+1}', 'exec'), namespace)
    return perf_counter()-started


def summarize_attempts(records):
    moved = [r for r in records if r['status'] == 'moved']
    errors = np.asarray([r['actual_error'] for r in moved])
    inaccurate = [r for r in records if r['model_error'] is not None
                  and (r['model_error'][0] > 1e-8 or r['model_error'][1] > 1e-6)]
    result = dict(attempts=len(records), executed=len(moved),
                  ik_exceptions=sum(r['status'] == 'ik_exception' for r in records),
                  move_exceptions=sum(r['status'] == 'move_exception' for r in records),
                  returned_outside_limits=sum(r['in_limits'] is False for r in records),
                  solver_reported_no_in_limit_candidate=sum(r['status'] == 'ik_exception'
                      and 'within limits' in r.get('message', '') for r in records),
                  returned_model_inconsistent=len(inaccurate),
                  simulated_motion_time_s=sum(r['simulated_time_s'] for r in records),
                  errors=summarize_errors(errors) if moved else None)
    if moved:
        result.update(position_pass=int(np.sum(errors[:, 1] <= constants.TARGET_POS_ERR_MM)),
                      orientation_pass=int(np.sum(errors[:, 0] <= constants.TARGET_ANG_ERR_RAD)),
                      both_pass=int(np.sum((errors[:, 1] <= constants.TARGET_POS_ERR_MM)
                                           & (errors[:, 0] <= constants.TARGET_ANG_ERR_RAD))))
        for label, column in [('position', 1), ('orientation', 0)]:
            row = moved[int(np.argmax(errors[:, column]))]
            result['worst_'+label] = {k:row[k] for k in ['target_index', 'grid_index', 'target_theta',
                                                       'seed', 'command', 'actual_error', 'model_error']}
    result['exception_cases'] = [r for r in records if r['status'] != 'moved']
    result['model_inconsistent_cases'] = inaccurate
    result['accuracy_failed_cases'] = [r for r in moved if r['actual_error'][0] > constants.TARGET_ANG_ERR_RAD
                                      or r['actual_error'][1] > constants.TARGET_POS_ERR_MM]
    return result


def evaluate_saved():
    inputs = [ROOT/'results'/name for name in ['step8_phase_d/calibration.npz', 'step8_phase_d/acquisition.json',
                                               'step8_phase_e/fit.npz', 'step9_forward_errors.npz']]
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    namespace, train, acquisition = saved_scene()
    params = namespace['improved'].params.copy()
    preparation = replay_calibration_motion(namespace, train, acquisition)
    execute_step9(namespace)
    started = clock.elapsed
    wall = execute_step10(namespace)
    assert np.isclose(namespace['t_testing'], clock.elapsed-started, atol=1e-10, rtol=0)
    assert np.array_equal(params, namespace['improved'].params)
    grid = namespace['test_grid']
    expected = np.random.default_rng(0).choice(len(grid), size=min(40, len(grid)), replace=False)
    assert np.array_equal(namespace['targets'], np.asarray(grid)[expected])
    records = namespace['inv_diagnostics']
    assert len(records) == 2*len(expected)
    for i, record in enumerate(records):
        record.update(target_index=i//2, grid_index=int(expected[i//2]))
        assert record['model'] == ('nominal' if i%2 == 0 else 'improved')
        assert np.array_equal(record['seed'], record['target_theta'])
    assert np.isclose(sum(r['simulated_time_s'] for r in records), namespace['t_testing'], atol=1e-9, rtol=0)
    for p in inputs:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p.relative_to(ROOT))]
    report = dict(seed=namespace['SEED'], target_selection_seed=0, n_targets=len(expected),
                  target_grid_indices=expected.tolist(), order='nominal then improved for each target; no retries',
                  nominal=summarize_attempts([r for r in records if r['model'] == 'nominal']),
                  improved=summarize_attempts([r for r in records if r['model'] == 'improved']),
                  simulated_testing_time_s=namespace['t_testing'], evaluation_wall_time_s=wall,
                  preparation=preparation, additional_tracker_samples=0, calibration_refitted=False,
                  historical_calibration_time_s=acquisition['training']['simulated_total_including_step6_s'],
                  full_notebook_calibration_plus_testing_s=acquisition['training']['simulated_total_including_step6_s']+namespace['t_testing'],
                  input_sha256=hashes, attempts=records,
                  model_consistency_thresholds=dict(position_mm=1e-6, orientation_rad=1e-8),
                  interpretation='Physical executed-move errors include robot repeatability noise. '
                                 'RMS/max cover executed attempts only; exceptions remain in the attempted-target denominator. '
                                 'Returned model-residual checks are not internal solver termination flags. '
                                 'All targets have known in-limit generating configurations; no hidden parameters enter IK.')
    return report


def run():
    report = evaluate_saved()
    output = ROOT/'results'
    (output/'step10_inverse_summary.json').write_text(json.dumps(report, default=json_default, indent=2)+'\n')
    records = report['attempts']
    np.savez_compressed(output/'step10_inverse_errors.npz',
        target_grid_indices=np.asarray(report['target_grid_indices']),
        target_thetas=np.array([r['target_theta'] for r in records]),
        commands=np.array([r['command'] if r['command'] is not None else np.full(6, np.nan) for r in records]),
        model_errors=np.array([r['model_error'] if r['model_error'] is not None else [np.nan]*2 for r in records]),
        actual_errors=np.array([r['actual_error'] if r['actual_error'] is not None else [np.nan]*2 for r in records]),
        status=np.array([r['status'] for r in records]), models=np.array([r['model'] for r in records]),
        simulated_time_s=np.array([r['simulated_time_s'] for r in records]))
    print(json.dumps({k:v for k,v in report.items() if k != 'attempts'}, default=json_default, indent=2))


if __name__ == '__main__':
    run()
