"""Frozen-model, single-observation evaluation; the default runs a small pilot.

All new artifacts use results/comprehensive_v1_*; legacy files are read-only.
No experiment edits the notebook, simulator, calibrated model or provided IK.
"""

import argparse
import ast
from contextlib import contextmanager
import hashlib
from itertools import product
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter

import numpy as np
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiments.step8_phase_d import frame_array, load_data, save_data
from experiments.step9_forward import notebook_cells, saved_scene
from experiments.step10_inverse import replay_calibration_motion
from src import Frame, Rot, vct3, simulation
from src.robot import Robot
from src.simulation import clock, constants
from src.student import workflow, metrics
from src.student.kinematics import ImprovedKinematics

VERSION = 'comprehensive_v1'
CATEGORIES = ('random', 'limits', 'boundary', 'singularity')
FULL_FK = dict(zip(CATEGORIES, [1000, 120, 104, 96]))
FULL_IK = dict(zip(CATEGORIES, [80, 20, 20, 20]))
PILOT_FK = dict(zip(CATEGORIES, [40, 24, 16, 16]))
PILOT_IK = dict(zip(CATEGORIES, [8, 4, 4, 4]))
CAL_NOISE = {1: 606, 7: 1607, 61: 1661}
REFERENCE = np.array([300., 0., 300., 0., 0., 0.])


def path(seed, suffix):
    return ROOT/'results'/f'{VERSION}_s{seed:03d}_{suffix}'


def digest(filename):
    return hashlib.sha256(Path(filename).read_bytes()).hexdigest()


def builtin(value):
    if isinstance(value, np.ndarray):
        return builtin(value.tolist())
    if isinstance(value, np.generic):
        return builtin(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k):builtin(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [builtin(v) for v in value]
    return value


def write_once(filename, value):
    with Path(filename).open('x') as handle:
        json.dump(builtin(value), handle, indent=2, allow_nan=False)
        handle.write('\n')


def write_atomic(filename, value):
    filename = Path(filename)
    temporary = filename.with_suffix(filename.suffix+'.tmp')
    temporary.write_text(json.dumps(builtin(value),indent=2,allow_nan=False)+'\n')
    temporary.replace(filename)


def save_run_summary(reports, stage, seeds, figures_complete=False):
    """Small review file and seed-level resume checkpoint; no individual records."""
    completed = [report['seed'] for report in reports]
    summary = dict(stage=stage,units=dict(position='mm',orientation='rad',time='seconds'),
        completed_seeds=completed,pending_seeds=[s for s in seeds if s not in completed],
        figures_complete=figures_complete,seeds={})
    for report in reports:
        summary['seeds'][str(report['seed'])] = dict(
            result_file=path(report['seed'],stage+'.json').name,
            preparation=report['preparation'],costs=report['costs'],
            evaluation_wall_s=report['evaluation_wall_s'],
            simulated_evaluation_s=report['simulated_evaluation_s'],
            statistics={key:{k:v for k,v in values.items() if k!='worst_attempt'}
                        for key,values in report['summary'].items()})
    write_atomic(ROOT/'results'/f'{VERSION}_{stage}_summary.json',summary)
    write_atomic(ROOT/'results'/f'{VERSION}_{stage}_checkpoint.json',dict(
        completed_seeds=completed,pending_seeds=summary['pending_seeds'],
        figures_complete=figures_complete,
        resume='Rerun the same command. Completed seed results and frozen models are reused; '
               'an interrupted evaluation restarts only its unfinished seed.'))


@contextmanager
def sensor_stream(seed):
    """The TA camera uses NumPy's public global RNG; keep phases separate."""
    state = np.random.get_state()
    np.random.seed(seed)
    try:
        yield
    finally:
        np.random.set_state(state)


def geometry_rng(seed, category):
    return np.random.default_rng(np.random.SeedSequence([20261025, seed, category]))


def baseline_design(namespace):
    cells, _ = notebook_cells()
    tree = ast.parse(''.join(cells[16]['source']))
    assignments = [n for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id in {'center', 'baseline_poses'} for t in n.targets)]
    exec(compile(ast.Module(body=assignments, type_ignores=[]), 'original-baseline-design', 'exec'), namespace)
    return namespace['baseline_poses']


def make_scene(seed):
    clock.reset()  # A new independently initialized simulation, before any cost is incurred.
    cells, _ = notebook_cells()
    ns = dict(np=np, Frame=Frame, Rot=Rot, vct3=vct3)
    exec(compile(''.join(cells[3]['source']), 'original-public-scene-design', 'exec'), ns)
    rob = simulation.create_robot(f'comprehensive_{seed}', ns['F_rob_approx'], seed=seed)
    trk = simulation.create_tracker(f'comprehensive_{seed}', ns['F_tracker_approx'], seed=seed)
    body = simulation.create_robot_calibration_body(rob, ns['mbee_local_positions'])
    simulation.add_marker_bodies(trk, [body])
    ns.update(rob=rob, trk=trk, mbee_placed=body, SEED=seed)
    return ns


def source_hashes():
    return {name:digest(ROOT/name) for name in ['src/student/kinematics.py', 'src/student/workflow.py',
            'vendor/functions/_fit_constraints.py', 'notebooks/assignment2_template.ipynb']}


def fit_worker(seed):
    """This process sees observed calibration data only, never a pose oracle."""
    data_file = path(seed, 'calibration.npz')
    data, extra = load_data(data_file)
    meta = json.loads(path(seed, 'acquisition.json').read_text())
    robot = Robot('calibration_only', Frame(meta['public_robot_frame']), constants.THETA_MIN,
                  constants.THETA_MAX, constants.T_SETTLE, constants.THETA_DOT_MAX, constants.THETA_DDOT_MAX)
    model = ImprovedKinematics(robot, Frame(extra['base_frame']), joint_offsets=extra['home_offsets'])
    info = model.fit(data)  # Unchanged normal constrained fitter; no ground truth.
    write_once(path(seed, 'fit_info.json'), info)
    if not info['success']:
        raise RuntimeError('Calibration did not converge; no model will be frozen')
    target = path(seed, 'model.npz')
    with target.open('xb') as handle:
        np.savez_compressed(handle, params=model.params, base_frame=extra['base_frame'],
                            home_offsets=extra['home_offsets'])
    write_once(path(seed, 'freeze.json'), dict(seed=seed, model_sha256=digest(target),
        data_sha256=digest(data_file), source_sha256=source_hashes(), fit_info=info,
        historical_seed1=False, frozen_before_ground_truth=True))


def validate_freeze(seed):
    meta = json.loads(path(seed, 'freeze.json').read_text())
    if meta['seed'] != seed or digest(path(seed, 'model.npz')) != meta['model_sha256']:
        raise ValueError('Frozen model identity/hash mismatch')
    if meta['source_sha256'] != source_hashes():
        raise ValueError('Source changed after freezing; review explicitly before reuse')
    data_file = ROOT/'results/step8_phase_d/calibration.npz' if seed == 1 else path(seed, 'calibration.npz')
    if digest(data_file) != meta['data_sha256']:
        raise ValueError('Frozen calibration data hash mismatch')
    return meta


def load_frozen(seed, namespace):
    meta = validate_freeze(seed)
    with np.load(path(seed, 'model.npz'), allow_pickle=False) as saved:
        model = ImprovedKinematics(namespace['rob'], Frame(saved['base_frame']), joint_offsets=saved['home_offsets'])
        model.params = saved['params'].copy()
    namespace.update(improved=model, F_X_nom=model.F_X_init)
    return meta


def prepare_seed(seed):
    started = perf_counter()
    if seed == 1:
        ns, data, acquisition = saved_scene()
        if not path(seed, 'freeze.json').exists():
            target = path(seed, 'model.npz')
            with target.open('xb') as handle:
                np.savez_compressed(handle, params=ns['improved'].params,
                    base_frame=frame_array(ns['F_X_nom']), home_offsets=ns['improved'].joint_offsets)
            info = json.loads((ROOT/'results/step8_phase_e/summary.json').read_text())['cold_fit']
            write_once(path(seed, 'freeze.json'), dict(seed=seed, model_sha256=digest(target),
                data_sha256=digest(ROOT/'results/step8_phase_d/calibration.npz'),
                source_sha256=source_hashes(), fit_info=info, historical_seed1=True,
                frozen_before_ground_truth=True,
                caveat='Seed 1 was developed/validated historically; supplementary geometry is new, not a blind model-development seed'))
        freeze = load_frozen(seed, ns)
        replay = replay_calibration_motion(ns, data, acquisition)
        return ns, data, freeze, dict(replay_s=replay['simulated_motion_replay_s'],
            acquisition_s=0., acquisition_wall_s=0., fit_wall_s=0.,
            historical_calibration_s=acquisition['training']['simulated_total_including_step6_s'],
            initialization_wall_s=perf_counter()-started, new_tracker_samples=0)
    ns = make_scene(seed)
    new_acquisition = not path(seed, 'calibration.npz').exists()
    acquisition_wall = 0.
    if new_acquisition:
        collect_start = perf_counter()
        with sensor_stream(CAL_NOISE[seed]):
            offsets = workflow.measure_home_offsets(ns['rob'], n_homings=5)
            baseline = workflow.collect_calibration_data(ns['rob'], ns['trk'], ns['mbee_placed'], baseline_design(ns))
            poses = workflow.plan_calibration_poses(ns['rob'], ns['trk'], ns['mbee_local_positions'], constants.TARGET_ANG_ERR_RAD)
            data = workflow.collect_calibration_data(ns['rob'], ns['trk'], ns['mbee_placed'], poses)
        if len(data) != len(poses) or len(baseline) != 10:
            raise RuntimeError('Missing calibration observations; preserve failure rather than silently retrying')
        base = workflow.estimate_base_frame(data, ns['rob'])
        save_data(path(seed, 'calibration.npz'), data, base_frame=frame_array(base), home_offsets=offsets,
                  baseline_thetas=np.asarray(baseline.thetas), baseline_frames=np.asarray([frame_array(f) for f in baseline.frames]))
        acquisition_wall = perf_counter()-collect_start
        write_once(path(seed, 'acquisition.json'), dict(seed=seed, sensor_seed=CAL_NOISE[seed],
            public_robot_frame=frame_array(ns['F_rob_approx']), acquisition_wall_s=acquisition_wall,
            training=dict(simulated_total_including_step6_s=clock.elapsed, rows=len(data), distinct=len(set(map(tuple,data.thetas)))),
            tracker_samples=len(poses)+10))
    acquisition = json.loads(path(seed, 'acquisition.json').read_text())
    fit_wall = 0.
    if not path(seed, 'freeze.json').exists():
        if path(seed, 'fit_info.json').exists() or path(seed, 'fit_timeout.json').exists():
            raise RuntimeError('An earlier fit attempt exists; review its diagnostics before another attempt')
        print(f'Seed {seed}: independently fitting saved public observations (300 s wall limit)', flush=True)
        start_fit = perf_counter()
        try:
            subprocess.run([sys.executable, str(Path(__file__).resolve()), '--fit-worker', str(seed)],
                           check=True, timeout=300)
        except subprocess.TimeoutExpired:
            write_once(path(seed, 'fit_timeout.json'), dict(seed=seed, timeout_s=300))
            raise RuntimeError('Pilot fitting exceeded 300 s; stop for review')
        fit_wall = perf_counter()-start_fit
    freeze = load_frozen(seed, ns)
    data, _ = load_data(path(seed, 'calibration.npz'))
    replay_s = 0.
    if not new_acquisition:
        replay = replay_calibration_motion(ns, data, acquisition)
        replay_s = replay['simulated_motion_replay_s']
    return ns, data, freeze, dict(replay_s=replay_s,
        acquisition_s=acquisition['training']['simulated_total_including_step6_s'] if new_acquisition else 0.,
        acquisition_wall_s=acquisition_wall, fit_wall_s=fit_wall,
        historical_calibration_s=acquisition['training']['simulated_total_including_step6_s'],
        initialization_wall_s=perf_counter()-started,
        new_tracker_samples=acquisition['tracker_samples'] if new_acquisition else 0)


def pole_locations(model):
    info = model.parameters_report()
    axes, scale = info['axes'], info['joint_normalization_scales'][4]
    coefficients = np.r_[model.joint_offsets[4], model.params[22:].reshape(6,model.n_poly)[4]]
    def determinant(theta):
        q = theta+np.polynomial.polynomial.polyval((theta+model.joint_offsets[4])/scale, coefficients)
        return np.linalg.det(np.column_stack([axes[3], axes[4], (Rot(vct3(axes[4]),angle=q)*vct3(axes[5])).vec.ravel()]))
    return [brentq(determinant, sign*np.pi/2-.2, sign*np.pi/2+.2, xtol=1e-14) for sign in [-1,1]]


def generate_design(seed, model):
    lower, upper = model.theta_min, model.theta_max
    pools = {}; ik_ids = {}; pilot_ids = {}; pilot_ik = {}
    pools['random'] = geometry_rng(seed,0).uniform(lower, upper, size=(1000,6))
    permutation = geometry_rng(seed,1).permutation(1000)
    ik_ids['random'] = permutation[:80]; pilot_ids['random'] = permutation[:40]; pilot_ik['random'] = permutation[:8]
    contexts = geometry_rng(seed,2).uniform(lower+.15*(upper-lower),upper-.15*(upper-lower),size=(2,6))
    limits = []
    for k, side, j, context in product(range(6), range(2), range(5), range(2)):
        offset = ([0., .001, .1, 1., 5.] if k<3 else [0.,1e-5,1e-3,.01,.05])[j]
        q = contexts[context].copy(); q[k] = lower[k]+offset if side==0 else upper[k]-offset
        limits.append(q)
    pools['limits'] = np.asarray(limits)
    exact = [20*k+10*side for k,side in product(range(6),range(2))]
    near = [20*k+10*side+3 for k,side in product(range(6),range(2))]
    ik_ids['limits'] = np.array(exact+near[:8]); pilot_ids['limits'] = np.array(exact+near)
    pilot_ik['limits'] = np.array([30,110,80,70])
    positions = [np.array(p) for p in product([0.,.5,1.],repeat=3) if p != (.5,.5,.5)]
    wrists = geometry_rng(seed,3).uniform(lower[3:]*.8,upper[3:]*.8,size=(4,3))
    pools['boundary'] = np.array([np.r_[lower[:3]+p*(upper[:3]-lower[:3]),w] for p,w in product(positions,wrists)])
    corners = [4*i for i,p in enumerate(positions) if np.all(p!=.5)]
    faces = [4*i for i,p in enumerate(positions) if np.count_nonzero(p!=.5)==1]
    edges = [4*i for i,p in enumerate(positions) if np.count_nonzero(p!=.5)==2]
    ik_ids['boundary'] = np.array(corners+[i+1 for i in corners]+faces[:4])
    pilot_ids['boundary'] = np.array(corners+faces+edges[:2]); pilot_ik['boundary'] = np.array([corners[0],corners[-1],faces[0],edges[0]])
    poles = pole_locations(model)
    backgrounds = geometry_rng(seed,4).uniform(lower+.15*(upper-lower),upper-.15*(upper-lower),size=(6,6))
    offsets = [-.01,-.001,-.0001,0.,.0001,.001,.01]
    singular = []; tags = []
    for sign, j, context in product(range(2),range(7),range(6)):
        q=backgrounds[context].copy();q[4]=poles[sign]+offsets[j]
        singular.append(q);tags.append(('fitted',sign,j,context))
    for sign,context in product(range(2),range(6)):
        q=backgrounds[context].copy();q[4]=[-1,1][sign]*np.pi/2
        singular.append(q);tags.append(('nominal',sign,3,context))
    pools['singularity'] = np.array(singular)
    select = lambda keys: np.array([tags.index(k) for k in keys],dtype=int)
    zero = [('fitted',s,3,c) for s,c in product(range(2),range(2))]
    close = [('fitted',s,j,2) for s,j in product(range(2),[1,2,4,5])]
    farther = [('fitted',s,j,3) for s,j in product(range(2),[0,6])]
    nominal = [('nominal',s,3,c) for s,c in product(range(2),range(2))]
    ik_ids['singularity'] = select(zero+close+farther+nominal)
    pilot_ik['singularity'] = select([('fitted',s,3,0) for s in range(2)]+[('nominal',s,3,0) for s in range(2)])
    pilot_ids['singularity'] = select([('fitted',s,j,0) for s,j in product(range(2),range(7))]+[('nominal',s,3,0) for s in range(2)])
    for cat in CATEGORIES:
        assert len(pools[cat]) == FULL_FK[cat] and len(ik_ids[cat]) == FULL_IK[cat]
        assert all(model.within_limits(q) for q in pools[cat])
    return dict(pools=pools, ik=ik_ids, pilot=pilot_ids, pilot_ik=pilot_ik,
                tracker_ids=permutation[80:180], pilot_tracker_ids=permutation[80:88], poles=poles)


def summarize(rows):
    valid = [r for r in rows if r.get('error') is not None]
    values = np.asarray([r['error'] for r in valid])
    result = dict(attempted=len(rows), scored=len(valid), unavailable=len(rows)-len(valid),
        exceptions=sum(r['status']=='exception' for r in rows),
        out_of_limits=sum(r.get('in_limits') is False for r in rows),
        solver_no_in_limit_candidate=sum(r['status']=='exception' and 'within limits' in r.get('message','') for r in rows),
        inaccurate_returns=sum(r.get('model_error') is not None and
            (r['model_error'][0]>1e-8 or r['model_error'][1]>1e-6) for r in rows))
    if valid:
        result.update(metrics.summarize_errors(values))
        result.update(median_angle=float(np.median(values[:,0])), p95_angle=float(np.percentile(values[:,0],95)),
                      median_pos=float(np.median(values[:,1])), p95_pos=float(np.percentile(values[:,1],95)))
        legal=np.array([r.get('in_limits') is not False for r in valid])
        passed=(values[:,0]<=.001)&(values[:,1]<=.2)&legal
        result.update(both_pass=int(passed.sum()), simultaneous_pass_rate=float(passed.sum()/len(rows)))
        strict=(values[:,0]<=1e-8)&(values[:,1]<=1e-6)&legal
        result.update(strict_consistency_pass=int(strict.sum()), strict_consistency_rate=float(strict.sum()/len(rows)))
        worst = int(np.argmax(np.maximum(values[:,0]/.001,values[:,1]/.2)))
        result['worst_attempt'] = valid[worst]
    else:
        result.update(both_pass=0, simultaneous_pass_rate=0.,strict_consistency_pass=0,strict_consistency_rate=0.)
    result['statistics_scope']='RMS/median/p95/max condition on scored attempts; rates include every attempted target'
    return result


def ordinary(q, poles):
    margin=np.minimum(q-constants.THETA_MIN,constants.THETA_MAX-q)
    return bool(np.all(margin>=np.r_[[5.]*3,[.05]*3]) and min(abs(q[4]-p) for p in list(poles)+[-np.pi/2,np.pi/2])>=.05)


def solve_attempt(robot, kins, base, target, seed, tracker_frame, body, execute):
    row=dict(command=None, initial_guess=np.asarray(seed).copy(),in_limits=None, error=None, model_error=None, message=None)
    start, wall=clock.elapsed,perf_counter()
    try:
        q=np.asarray(kins.inv_kins(base.inv()*target,seed=seed),dtype=float)
        row.update(command=q.copy(),in_limits=bool(robot.within_limits(q)))
        row['model_error']=metrics.frame_error(target,base*kins.fwd_kins(q))
        if execute:
            robot.move_joints(q)  # No filtering/clipping/retry, even for inaccurate returned candidates.
            row['error']=metrics.frame_error(target,tracker_frame.inv()*body.actual_placement)
        else:
            row['error']=row['model_error']
        row['status']='scored'
    except (ValueError,np.linalg.LinAlgError) as exc:
        row.update(status='exception',message=str(exc))
    row.update(wall_s=perf_counter()-wall,simulated_s=clock.elapsed-start)
    return row


def finalize_report(report):
    """Derive display statistics from raw attempts, never repeat measurements."""
    for row in report['rows']:
        if 'initial_guess' not in row and row['family'].startswith(('numerical','physical_ik')):
            q=np.asarray(row['theta'])
            row['initial_guess']=(REFERENCE if row.get('guess')=='center' else
                np.clip(q+[20,-20,20,.3,-.2,.3],constants.THETA_MIN,constants.THETA_MAX)
                if row.get('guess')=='perturbed' else q).tolist()
    groups={}
    for key in report['summary']:
        family,model,category=key.split('/')
        rows=[r for r in report['rows'] if r['family']==family and r['model']==model and
              (r['category']==category or category=='ordinary_subset' and r['category']=='random' and r['ordinary'])]
        groups[key]=summarize(rows)
    report['summary']=groups
    report['analysis_source_sha256']={name:digest(ROOT/name) for name in
        ['experiments/comprehensive_evaluation.py','experiments/comprehensive_figures.py']}
    report['metric_thresholds']=dict(physical_position_mm=.2,physical_orientation_rad=.001,
                                    numerical_consistency_position_mm=1e-6,numerical_consistency_orientation_rad=1e-8)
    return builtin(report)


def evaluate(seed, stage, ns, train, freeze, preparation):
    model,robot,tracker,body=ns['improved'],ns['rob'],ns['trk'],ns['mbee_placed']
    frozen=model.params.copy();model_file_hash=digest(path(seed,'model.npz'))
    design=generate_design(seed,model)
    training_configurations=set(map(tuple,train.thetas))
    if any(set(map(tuple,pool))&training_configurations for pool in design['pools'].values()):
        raise ValueError('Evaluation design overlaps calibration configurations')
    pilot=stage=='pilot';fk_ids=design['pilot'] if pilot else {c:np.arange(FULL_FK[c]) for c in CATEGORIES}
    ik_ids=design['pilot_ik'] if pilot else design['ik']
    rows=[]; models=[('nominal',robot,ns['F_X_nom']),('improved',model,model.base_frame())]
    tracker_frame=tracker.nominal_pose()
    costs={}; wall_start=perf_counter(); simulation_start=clock.elapsed
    stage_wall=perf_counter()
    for cat in CATEGORIES:
        for i in fk_ids[cat]:
            q=design['pools'][cat][i];truth=tracker_frame.inv()*robot.actual_ee_pose(q)
            shared=dict(seed=seed,category=cat,index=int(i),theta=q,ordinary=ordinary(q,design['poles']),
                        workspace_position=(tracker_frame*truth).p.vec.ravel())
            for label,kins,base in models:
                rows.append(dict(shared,family='physical_fk',model=label,status='scored',
                    error=metrics.frame_error(truth,base*kins.fwd_kins(q))))
    costs['physical_fk']=dict(wall_s=perf_counter()-stage_wall,simulated_s=0.)
    stage_wall=perf_counter();start=clock.elapsed
    with sensor_stream(3100+seed):
        for i in design['pilot_tracker_ids'] if pilot else design['tracker_ids']:
            q=design['pools']['random'][i];robot.move_joints(q)
            encoders=robot.state().theta.copy()
            observed=simulation.sample_marker_bodies(tracker).get(body.body.name)
            for label,kins,base in models:
                rows.append(dict(seed=seed,category='random',index=int(i),theta=encoders,
                    ordinary=ordinary(q,design['poles']),family='tracker_prediction',model=label,
                    status='missing' if observed is None else 'scored',
                    error=None if observed is None else metrics.frame_error(observed.frame,base*kins.fwd_kins(encoders))))
    costs['tracker_prediction']=dict(wall_s=perf_counter()-stage_wall,simulated_s=clock.elapsed-start,
        samples=8 if pilot else 100,sampling_s=(8 if pilot else 100)*constants.SAMPLE_DELTA_T_SEC)
    stage_wall=perf_counter()
    for label,kins,base in models:
        for cat in CATEGORIES:
            for i in ik_ids[cat]:
                q=design['pools'][cat][i];target=base*kins.fwd_kins(q)
                result=solve_attempt(robot,kins,base,target,q,tracker_frame,body,False)
                rows.append(dict(result,seed=seed,category=cat,index=int(i),theta=q,
                    ordinary=ordinary(q,design['poles']),family='numerical_consistency',model=label,guess='generating'))
        selected=design['pilot_ik']['singularity'] if pilot else geometry_rng(seed,5).choice(96,32,replace=False)
        for i in selected:
            q=design['pools']['singularity'][i];target=base*kins.fwd_kins(q)
            for guess,initial in [('center',REFERENCE),('perturbed',np.clip(q+[20,-20,20,.3,-.2,.3],robot.theta_min,robot.theta_max))]:
                result=solve_attempt(robot,kins,base,target,initial,tracker_frame,body,False)
                rows.append(dict(result,seed=seed,category='singularity',index=int(i),theta=q,
                    ordinary=False,family='numerical_seed_sensitivity',model=label,guess=guess))
    costs['numerical_consistency']=dict(wall_s=perf_counter()-stage_wall,simulated_s=0.)
    stage_wall=perf_counter();start=clock.elapsed;reset_s=0.
    for label,kins,base in models:
        for cat in CATEGORIES:
            reset_start=clock.elapsed;robot.move_joints(REFERENCE);reset_s+=clock.elapsed-reset_start
            for i in ik_ids[cat]:
                q=design['pools'][cat][i];target=tracker_frame.inv()*robot.actual_ee_pose(q)
                result=solve_attempt(robot,kins,base,target,q,tracker_frame,body,True)
                rows.append(dict(result,seed=seed,category=cat,index=int(i),theta=q,
                    ordinary=ordinary(q,design['poles']),family='physical_ik',model=label,guess='generating'))
    costs['physical_ik']=dict(wall_s=perf_counter()-stage_wall,simulated_s=clock.elapsed-start,reset_s=reset_s,
        target_motion_s=sum(r['simulated_s'] for r in rows if r['family']=='physical_ik'))
    assert np.array_equal(model.params,frozen) and digest(path(seed,'model.npz'))==model_file_hash
    groups={}
    for family in sorted({r['family'] for r in rows}):
        for label,_,_ in models:
            for cat in CATEGORIES:
                part=[r for r in rows if r['family']==family and r['model']==label and r['category']==cat]
                if part:groups[f'{family}/{label}/{cat}']=summarize(part)
            part=[r for r in rows if r['family']==family and r['model']==label and r['category']=='random' and r['ordinary']]
            if part:groups[f'{family}/{label}/ordinary_subset']=summarize(part)
    return builtin(dict(version=VERSION,stage=stage,seed=seed,freeze_sha256=digest(path(seed,'freeze.json')),
        design_sha256=hashlib.sha256(b''.join(design['pools'][c].tobytes() for c in CATEGORIES)).hexdigest(),
        fk_counts={c:len(fk_ids[c]) for c in CATEGORIES},ik_counts={c:len(ik_ids[c]) for c in CATEGORIES},
        preparation=preparation,costs=costs,summary=groups,rows=rows,
        evaluation_wall_s=perf_counter()-wall_start,simulated_evaluation_s=clock.elapsed-simulation_start,
        noise=dict(calibration_sensor_seed=CAL_NOISE[seed],evaluation_sensor_seed=3100+seed,
            geometry_seed=[20261025,seed,'category_id'],
            robot='Public seed-specific simulator RNG continues after calibration motions; no private RNG edits or artificial burn-in',
            historical_caveat='Seed 1 may repeat robot-noise draws from historical development runs; new poses are not a blind model-development claim' if seed==1 else 'New evaluation uses post-calibration draws, disjoint from its calibration draws'),
        definition='Error pairs are (rad,mm); tracker observations are single snapshots; physical moves are scored individually.'))


def project(reports):
    """Measured per-category scaling; include failed calls, not only successes."""
    detail=[]
    for report in reports:
        seed=report['seed'];costs=report['costs'];projection_wall=0.;projection_sim=0.;slow_call_wall=0.
        fk_scale=sum(FULL_FK.values())/sum(report['fk_counts'].values())
        projection_wall+=costs['physical_fk']['wall_s']*fk_scale
        slow_call_wall+=costs['physical_fk']['wall_s']*fk_scale
        tracker_n=costs['tracker_prediction']['samples'];projection_wall+=costs['tracker_prediction']['wall_s']*100/tracker_n
        slow_call_wall+=costs['tracker_prediction']['wall_s']*100/tracker_n
        projection_sim+=costs['tracker_prediction']['simulated_s']*100/tracker_n
        for family in ['numerical_consistency','physical_ik']:
            for cat in CATEGORIES:
                part=[r for r in report['rows'] if r['family']==family and r['category']==cat]
                projection_wall+=sum(r['wall_s'] for r in part)*FULL_IK[cat]/report['ik_counts'][cat]
                for model in ['nominal','improved']:
                    slow_call_wall+=max(r['wall_s'] for r in part if r['model']==model)*FULL_IK[cat]
                if family=='physical_ik':projection_sim+=sum(r['simulated_s'] for r in part)*FULL_IK[cat]/report['ik_counts'][cat]
        sensitivity=[r for r in report['rows'] if r['family']=='numerical_seed_sensitivity']
        projection_wall+=sum(r['wall_s'] for r in sensitivity)*128/len(sensitivity)
        for model in ['nominal','improved']:
            slow_call_wall+=max(r['wall_s'] for r in sensitivity if r['model']==model)*64
        projection_sim+=costs['physical_ik']['reset_s']
        detail.append(dict(seed=seed,estimated_full_evaluation_wall_s=projection_wall,
            observed_slowest_call_scaled_wall_s=slow_call_wall,
            estimated_full_evaluation_simulated_s=projection_sim))
    # Only Seed 61 still needs a new fit after a successful Seed 7 pilot.
    measured_fit=max(r['preparation']['fit_wall_s'] for r in reports)
    estimate_wall=3*max(d['estimated_full_evaluation_wall_s'] for d in detail)+measured_fit+45
    estimate_sim=3*max(d['estimated_full_evaluation_simulated_s'] for d in detail)
    return builtin(dict(category_scaled=detail,remaining_new_calibration_seed=61,
        measured_seed7_fit_process_wall_s=measured_fit,
        conservative_three_seed_evaluation_plus_remaining_fit_and_plot_wall_s=estimate_wall,
        slow_call_three_seed_evaluation_plus_remaining_fit_and_plot_wall_s=
            3*max(d['observed_slowest_call_scaled_wall_s'] for d in detail)+measured_fit+45,
        remaining_new_fit_wall_timeout_s=300,
        approximate_three_seed_simulated_evaluation_s=estimate_sim,
        seed61_calibration_additional_simulated_s=554.1,
        note='Linear pilot extrapolation is not a guarantee; tiny boundary samples/branch changes can alter time. '
             'Cached Seed 7 fit is reused. Replay/initialization overhead is additional and separately itemized. '
             'Full coverage remains 1320 FK, 140 physical IK targets/model/seed; no automatic coverage reduction.',
        review_gate='STOP: full evaluation has not been run; user review required regardless of projected expense'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fit-worker',type=int,choices=[7,61])
    parser.add_argument('--stage',choices=['pilot','full'],default='pilot')
    parser.add_argument('--approve-full',action='store_true')
    args=parser.parse_args()
    if args.fit_worker is not None:
        fit_worker(args.fit_worker);return
    if args.stage=='full' and not args.approve_full:
        parser.error('Full evaluation is gated pending pilot review; --approve-full must be explicit')
    seeds=[1,7] if args.stage=='pilot' else [1,7,61]
    reports=[]
    for seed in seeds:
        output=path(seed,args.stage+'.json')
        if output.exists():
            validate_freeze(seed)
            report=json.loads(output.read_text())
            if report['freeze_sha256']!=digest(path(seed,'freeze.json')):
                raise ValueError('Cached evaluation is not paired with this frozen model')
        else:
            print(f'Preparing seed {seed} for {args.stage}',flush=True)
            ns,data,freeze,preparation=prepare_seed(seed)
            report=evaluate(seed,args.stage,ns,data,freeze,preparation)
            write_once(output,report)
        report=finalize_report(report)
        write_atomic(output,report)
        reports.append(report)
        save_run_summary(reports,args.stage,seeds)
        print(f'Seed {seed}: evaluation {report["evaluation_wall_s"]:.3f}s wall / '
              f'{report["simulated_evaluation_s"]:.3f}s simulated',flush=True)
    if args.stage=='pilot':
        filename=ROOT/'results'/f'{VERSION}_pilot_projection.json'
        filename.write_text(json.dumps(project(reports),indent=2,allow_nan=False)+'\n')
    from experiments.comprehensive_figures import generate_figures
    plot_start=perf_counter()
    generate_figures(reports,args.stage)
    save_run_summary(reports,args.stage,seeds,figures_complete=True)
    if args.stage=='pilot':
        projection=json.loads(filename.read_text());projection['actual_figure_generation_wall_s']=perf_counter()-plot_start
        filename.write_text(json.dumps(projection,indent=2,allow_nan=False)+'\n')
    print('STOP after pilot; full evaluation remains separately gated.' if args.stage=='pilot' else 'Full evaluation finished.',flush=True)


if __name__=='__main__':
    main()
