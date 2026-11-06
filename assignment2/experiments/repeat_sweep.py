#!/usr/bin/env python3
"""Run the current notebook pipeline with repeats 1–6, without editing it."""
import argparse
import ast
import csv
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import time
import traceback

ASSIGNMENT = Path(__file__).resolve().parents[1]
NOTEBOOK = ASSIGNMENT / 'notebooks/assignment2_Draft_2.ipynb'
# The setup, scene, visibility, baseline, collection, fit and evaluation cells.
CELL_IDS = (1, 3, 5, 7, 15, 17, 19, 21, 23)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def worker(repeats, output):
    import numpy as np
    os.chdir(ASSIGNMENT / 'notebooks')
    notebook = json.loads((output / 'input_notebook.ipynb').read_text())
    scene = ast.parse(''.join(notebook['cells'][5]['source']))
    seed = next(ast.literal_eval(node.value) for node in scene.body
                if isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'SEED' for t in node.targets))
    np.random.seed(seed)  # Assignment 1 tracker noise uses this global RNG.
    random.seed(seed)
    ns = {'__name__': '__main__'}
    row = {'repeats': repeats, 'status': 'failed', 'error': ''}
    started = time.perf_counter()
    try:
        for index in CELL_IDS:
            tree = ast.parse(''.join(notebook['cells'][index]['source']))
            if index == 17:
                calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                         and isinstance(n.func, ast.Attribute)
                         and n.func.attr == 'collect_calibration_data']
                assert len(calls) == 1
                kw = next(k for k in calls[0].keywords if k.arg == 'repeats')
                kw.value = ast.Constant(repeats)
                ast.fix_missing_locations(tree)
            print(f'Executing notebook cell {index}', flush=True)
            exec(compile(tree, str(NOTEBOOK) + f':cell{index}', 'exec'), ns)
        constants = ns['constants']
        row['calibration_time_s'] = ns['t_calibration']
        row['testing_time_s'] = ns['t_testing']
        row['fit_success'] = bool(ns['fit_info']['success'])
        row['collected_observations'] = len(ns['data'])
        row['skipped_observations'] = repeats * len(ns['poses']) - len(ns['data'])
        for kind, key, total in (
            ('forward', 'fwd_err_improved', len(ns['test_grid'])),
            ('inverse', 'inv_err_improved', len(ns['targets'])),
        ):
            errors = np.asarray(ns[key], dtype=float)
            assert errors.shape == (total, 2) and total > 0
            assert np.isfinite(errors).all(), f'Nonfinite {kind} errors'
            passed = int(np.sum((errors[:, 0] <= constants.TARGET_ANG_ERR_RAD)
                               & (errors[:, 1] <= constants.TARGET_POS_ERR_MM)))
            row[f'{kind}_passed'] = passed
            row[f'{kind}_evaluated'] = total
            row[f'{kind}_accuracy_pct'] = 100.0 * passed / total
        # Keep the measured errors to verify the counts without another fit.
        np.savez(output / f'repeat_{repeats}_errors.npz',
                 forward=np.asarray(ns['fwd_err_improved']),
                 inverse=np.asarray(ns['inv_err_improved']),
                 test_grid=np.asarray(ns['test_grid']), targets=np.asarray(ns['targets']))
        row['status'] = 'ok' if row['fit_success'] else 'fit_not_converged'
        settings = {
            'seed': seed, 'global_noise_seed': seed,
            'planned_poses': np.asarray(ns['poses']).tolist(),
            'test_grid': np.asarray(ns['test_grid']).tolist(),
            'targets': np.asarray(ns['targets']).tolist(),
            'markers': [p.vec.ravel().tolist() for p in ns['mbee_local_positions']],
            'n_poly': ns['improved'].n_poly,
            'angular_target_rad': constants.TARGET_ANG_ERR_RAD,
            'position_target_mm': constants.TARGET_POS_ERR_MM,
            'calibration_budget_s': constants.TARGET_TIME_SEC,
        }
        (output / f'repeat_{repeats}_settings.json').write_text(json.dumps(settings, indent=2))
    except Exception as error:
        row['error'] = f'{type(error).__name__}: {error}'
        traceback.print_exc()
        # Partial counts are diagnostic only; no partial run gets a pass rate.
        for kind, key in [('forward', 'fwd_err_improved'), ('inverse', 'inv_err_improved')]:
            row[f'{kind}_evaluated'] = len(ns.get(key, []))
    row['wall_time_s'] = time.perf_counter() - started
    (output / f'repeat_{repeats}.json').write_text(json.dumps(row, indent=2))


def plot(rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    valid = [r for r in rows if r['status'] == 'ok']
    fig, ax = plt.subplots(figsize=(10, 6))
    for kind, color, marker, offset in [('forward', '#1769aa', 'o', 11),
                                        ('inverse', '#d2691e', 's', -18)]:
        x = [r.get('calibration_time_s', float('nan')) for r in rows]
        y = [r.get(f'{kind}_accuracy_pct', float('nan')) if r['status'] == 'ok'
             else float('nan') for r in rows]
        ax.plot(x, y, marker=marker, color=color, linewidth=2, label=f'Improved {kind}')
        for r in valid:
            ax.annotate(f"r={r['repeats']}", (r['calibration_time_s'], r[f'{kind}_accuracy_pct']),
                        xytext=(0, offset), textcoords='offset points', ha='center',
                        fontsize=9, color=color)
    ax.axvline(900, color='#a22d35', linestyle='--', label='900 s soft calibration budget')
    ax.set(xlabel='Simulated calibration time (seconds)',
           ylabel='Poses meeting both accuracy targets (%)', ylim=(-8, 112),
           title='Calibration repeats: accuracy versus simulated time')
    ax.set_yticks(range(0, 101, 20))
    ax.grid(alpha=0.2)
    ax.legend(loc='lower right')
    failed = [str(r['repeats']) for r in rows if r['status'] != 'ok']
    caption = ('Position ≤ 0.2 mm AND orientation ≤ 0.001 rad | '
               'Fixed seeds; one run per repeat count\n'
               'Forward: 729 poses; inverse: 40 targets. Labels indicate repeats (r).')
    if failed:
        caption += '\nFailed configurations (see CSV/logs): ' + ', '.join(failed)
    fig.text(0.5, 0.025, caption, ha='center', fontsize=9)
    fig.tight_layout(rect=(0, 0.10, 1, 1))
    for suffix in ('png', 'svg'):
        fig.savefig(output / f'accuracy_vs_calibration_time.{suffix}', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeat', type=int, choices=range(1, 7), help=argparse.SUPPRESS)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.repeat:
        worker(args.repeat, output)
        return
    output.mkdir(parents=True, exist_ok=False)  # Never overwrite an earlier sweep.
    (output / 'input_notebook.ipynb').write_bytes(NOTEBOOK.read_bytes())
    inputs = [NOTEBOOK, *sorted((ASSIGNMENT / 'src').rglob('*.py')),
              *sorted((ASSIGNMENT / 'vendor/course').rglob('*.py'))]
    hashes = {str(p.relative_to(ASSIGNMENT)): digest(p) for p in inputs}
    git = lambda *cmd: subprocess.check_output(['git', *cmd], cwd=ASSIGNMENT, text=True)
    assert git('branch', '--show-current').strip() == 'dev/ahmed'
    (output / 'input_changes.patch').write_text(git('diff', '--', 'src', 'notebooks'))
    (output / 'provenance.json').write_text(json.dumps({
        'commit': git('rev-parse', 'HEAD').strip(), 'branch': 'dev/ahmed',
        'input_sha256': hashes, 'python': sys.version,
        'command': [sys.executable, *sys.argv],
        'note': 'Only Step 7 repeats changes. Global NumPy/Python RNG seeded from existing SEED. '
                'All scene objects recreated per run. Notebook and sources preserved.',
    }, indent=2))
    rows = []
    for repeats in range(1, 7):
        assert all(digest(ASSIGNMENT / p) == h for p, h in hashes.items()), 'Inputs changed during sweep'
        print(f'Starting repeats={repeats}', flush=True)
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', MPLCONFIGDIR='/tmp/cis-repeat-mpl')
        with (output / f'repeat_{repeats}.log').open('w') as log:
            completed = subprocess.run([sys.executable, '-B', str(Path(__file__).resolve()),
                                        '--output', str(output), '--repeat', str(repeats)],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)
        result_path = output / f'repeat_{repeats}.json'
        row = json.loads(result_path.read_text()) if result_path.exists() else {
            'repeats': repeats, 'status': 'failed', 'error': f'Worker exit {completed.returncode}; see log'}
        if completed.returncode:
            row.update(status='failed', error=f'Worker exit {completed.returncode}; see log')
        rows.append(row)
        fields = list(dict.fromkeys(key for r in rows for key in r))
        with (output / 'results.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        print(json.dumps(row), flush=True)
    settings = [json.loads((output / f"repeat_{r['repeats']}_settings.json").read_text())
                for r in rows if r['status'] == 'ok']
    assert all(s == settings[0] for s in settings), 'Settings or evaluation targets changed across runs'
    assert all(digest(ASSIGNMENT / p) == h for p, h in hashes.items()), 'Input files changed'
    plot(rows, output)
    print(f'CSV, plots, and logs saved to {output}', flush=True)


if __name__ == '__main__':
    main()
