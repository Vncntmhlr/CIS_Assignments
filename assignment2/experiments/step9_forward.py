"""Report-only evaluation of the frozen Step 8 fit on the TA Step 9 grid.

From assignment2: venv/bin/python experiments/step9_forward.py
Uses existing results/ only; never collects data, fits parameters or runs IK.
"""

import hashlib
import json
import os
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.step8_phase_d import frame_array, groups, load_data
from src import Frame
from src.simulation import clock, constants
from src.student.kinematics import ImprovedKinematics
from src.student.metrics import summarize_errors


def notebook_cells():
    cells = json.loads((ROOT/'notebooks/assignment2_template.ipynb').read_text())['cells']
    heading = next(i for i, c in enumerate(cells)
                   if c['cell_type'] == 'markdown' and ''.join(c['source']).startswith('## Step 9:'))
    return cells, heading + 1


def saved_scene():
    """Reconstruct the archived seed's scene, without motion or acquisition."""
    cells, _ = notebook_cells()
    namespace = {'__name__': '__step9_forward__'}
    previous_cwd = Path.cwd()
    try:
        os.chdir(ROOT/'notebooks')
        # Original imports, design, scene construction and nominal visibility.
        # No homing, collection, fitting, truth-parameter or IK cells are run.
        for i in (1, 3, 5, 8):
            exec(compile(''.join(cells[i]['source']), f'assignment2_template.ipynb:cell{i}', 'exec'), namespace)
    finally:
        os.chdir(previous_cwd)
    acquisition = json.loads((ROOT/'results/step8_phase_d/acquisition.json').read_text())
    if namespace['SEED'] != acquisition['training']['robot_tracker_seed']:
        raise ValueError('Notebook seed does not match the saved calibration; do not cross-use fitted models')
    # These are the public design values used to acquire this historical data.
    expected_robot = np.eye(4)
    expected_robot[:3, 3] = [-800., 0., 1300.]
    expected_markers = 37.5*np.array([[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]])
    if (not np.array_equal(frame_array(namespace['F_rob_approx']), expected_robot)
            or not np.array_equal(frame_array(namespace['F_tracker_approx']), np.eye(4))
            or not np.array_equal([p.vec.ravel() for p in namespace['mbee_local_positions']], expected_markers)):
        raise ValueError('Scene design differs from the saved-data provenance; acquire a matching calibration separately')
    train, extra = load_data(ROOT/'results/step8_phase_d/calibration.npz')
    namespace['F_X_nom'] = Frame(extra['base_frame'])
    model = ImprovedKinematics(namespace['rob'], namespace['F_X_nom'], joint_offsets=extra['home_offsets'])
    with np.load(ROOT/'results/step8_phase_e/fit.npz', allow_pickle=False) as saved:
        model.params = saved['fitted_params'].copy()
    namespace['improved'] = model
    return namespace, train, acquisition


def execute_step9(namespace):
    """Execute the actual notebook TODO and fixed grid, without saving outputs."""
    cells, index = notebook_cells()
    started = perf_counter()
    exec(compile(''.join(cells[index]['source']), f'assignment2_template.ipynb:cell{index}', 'exec'), namespace)
    return perf_counter() - started


def describe(errors, poses):
    values = np.asarray(errors)
    result = summarize_errors(errors)
    result.update(median_angle=float(np.median(values[:, 0])), p95_angle=float(np.percentile(values[:, 0], 95)),
                  median_pos=float(np.median(values[:, 1])), p95_pos=float(np.percentile(values[:, 1], 95)))
    angle_ok = values[:, 0] <= constants.TARGET_ANG_ERR_RAD
    position_ok = values[:, 1] <= constants.TARGET_POS_ERR_MM
    result.update(orientation_pass=int(angle_ok.sum()), position_pass=int(position_ok.sum()),
                  both_pass=int((angle_ok & position_ok).sum()), all_poses_pass=bool((angle_ok & position_ok).all()))
    for label, column in [('orientation', 0), ('position', 1)]:
        index = int(np.argmax(values[:, column]))
        result['worst_'+label] = dict(index=index, theta=np.asarray(poses[index]).tolist(),
                                     angle_rad=float(values[index, 0]), position_mm=float(values[index, 1]))
    return result


def evaluate_saved():
    paths = [ROOT/'results'/name for name in ['step8_phase_d/calibration.npz', 'step8_phase_d/holdout.npz',
                                             'step8_phase_d/acquisition.json', 'step8_phase_e/fit.npz']]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    namespace, train, acquisition = saved_scene()
    robot, model = namespace['rob'], namespace['improved']
    params, encoders = model.params.copy(), robot.state().theta.copy()
    body_pose = frame_array(namespace['mbee_placed'].actual_placement)
    before, random_state = clock.elapsed, np.random.get_state()
    wall_time = execute_step9(namespace)
    assert clock.elapsed == before and np.array_equal(model.params, params)
    assert np.array_equal(robot.state().theta, encoders)
    assert np.array_equal(frame_array(namespace['mbee_placed'].actual_placement), body_pose)
    after_random = np.random.get_state()
    assert random_state[0] == after_random[0] and np.array_equal(random_state[1], after_random[1])
    assert random_state[2:] == after_random[2:]
    for p in paths:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p.relative_to(ROOT))]
    poses = np.asarray(namespace['test_grid'])
    holdout, _ = load_data(ROOT/'results/step8_phase_d/holdout.npz')
    levels = namespace['test_levels']
    report = dict(seed=namespace['SEED'], candidate_poses=int(np.prod([len(v) for v in levels.values()])),
                  visible_poses=len(poses), grid_levels=levels,
                  nominal=describe(namespace['fwd_err_nominal'], poses),
                  improved=describe(namespace['fwd_err_improved'], poses),
                  target_pos_mm=constants.TARGET_POS_ERR_MM, target_angle_rad=constants.TARGET_ANG_ERR_RAD,
                  training_configuration_overlap=len(set(map(tuple, poses)) & set(groups(train))),
                  tracker_holdout_configuration_overlap=len(set(map(tuple, poses)) & set(groups(holdout))),
                  additional_simulated_evaluation_time_s=clock.elapsed-before,
                  additional_simulated_acquisition_time_s=0., evaluation_wall_time_s=wall_time,
                  historical_calibration_time_s=acquisition['training']['simulated_total_including_step6_s'],
                  historical_tracker_holdout_time_s=acquisition['holdout']['measured_s'],
                  calibration_archive_sha256=hashes, fitted_parameters_changed=False,
                  interpretation='Deterministic physical FK oracle on the fixed visible TA grid, frozen seed-1 fit; '
                                 'not tracker residuals, random motion repeatability, continuous-volume or multi-seed certification')
    return report, poses, np.asarray(namespace['fwd_err_nominal']), np.asarray(namespace['fwd_err_improved'])


def plot_errors(nominal, improved, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')
    for ax, column, target, label in zip(axes, [1, 0], [.2, .001], ['Position error (mm)', 'Orientation error (rad)']):
        for values, name in [(nominal, 'Nominal'), (improved, 'Improved, constrained')]:
            ax.step(np.sort(values[:, column]), np.arange(1, len(values)+1)/len(values), where='post', label=name)
        ax.axvline(target, color='black', linestyle='--', label='Assignment target')
        ax.set(xscale='log', xlabel=label, ylabel='Fraction of grid poses', ylim=(0, 1.02))
        ax.grid(True, alpha=.25)
        ax.legend(loc='lower right', fontsize=8)
    fig.suptitle(f'Step 9: frozen seed-1 model, {len(improved)} visible TA grid poses')
    fig.savefig(destination, dpi=170)
    plt.close(fig)


def run():
    report, poses, nominal, improved = evaluate_saved()
    output = ROOT/'results'
    (output/'step9_forward_summary.json').write_text(json.dumps(report, indent=2)+'\n')
    np.savez_compressed(output/'step9_forward_errors.npz', thetas=poses,
                        nominal_errors=nominal, improved_errors=improved)
    plot_errors(nominal, improved, output/'step9_forward_errors.png')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    run()
