"""Cohort contracts, failure accounting and frozen pilot evidence; no new fits."""

import ast
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from experiments import comprehensive_evaluation as evaluation
from experiments.comprehensive_figures import empirical_cdf
from src import Frame, Rot, vct3, simulation
from src.robot import Robot
from src.student.kinematics import ImprovedKinematics


@pytest.fixture
def nominal_model():
    c=simulation.constants
    robot=Robot('test_design',Frame.eye(),c.THETA_MIN,c.THETA_MAX,c.T_SETTLE,c.THETA_DOT_MAX,c.THETA_DDOT_MAX)
    return ImprovedKinematics(robot,Frame.eye())


@pytest.mark.parametrize('seed',[1,7,61])
def test_exact_full_and_pilot_counts_continuous_sampling_and_limits(seed,nominal_model):
    before=np.random.get_state();clock=simulation.clock.elapsed
    a=evaluation.generate_design(seed,nominal_model);b=evaluation.generate_design(seed,nominal_model)
    for category in evaluation.CATEGORIES:
        assert len(a['pools'][category])==evaluation.FULL_FK[category]
        assert len(a['ik'][category])==evaluation.FULL_IK[category]
        assert len(a['pilot'][category])==evaluation.PILOT_FK[category]
        assert len(a['pilot_ik'][category])==evaluation.PILOT_IK[category]
        assert len(set(a['ik'][category]))==len(a['ik'][category])
        assert np.array_equal(a['pools'][category],b['pools'][category])
        assert all(nominal_model.within_limits(q) for q in a['pools'][category])
    assert sum(map(len,a['pools'].values()))==1320
    assert sum(map(len,a['ik'].values()))==140
    assert len(set(map(tuple,a['pools']['random'])))==1000
    normalized=(a['pools']['random']-nominal_model.theta_min)/(nominal_model.theta_max-nominal_model.theta_min)
    assert np.all(np.std(normalized,axis=0)>.25)
    assert not np.any(np.all(np.isclose(normalized,.5),axis=1))
    assert len(a['tracker_ids'])==100 and not set(a['tracker_ids'])&set(a['ik']['random'])
    after=np.random.get_state()
    assert np.array_equal(before[1],after[1]) and before[2:]==after[2:]
    assert simulation.clock.elapsed==clock


def test_failures_and_invalid_returns_remain_in_denominators():
    rows=[dict(status='scored',error=[.0001,.1],in_limits=True),
          dict(status='exception',error=None,message='no joint solution within limits'),
          dict(status='scored',error=[0.,0.],in_limits=False)]
    result=evaluation.summarize(rows)
    assert result['attempted']==3 and result['scored']==2 and result['unavailable']==1
    assert result['both_pass']==1 and result['simultaneous_pass_rate']==pytest.approx(1/3)
    assert result['strict_consistency_pass']==0
    assert result['out_of_limits']==result['solver_no_in_limit_candidate']==1
    x,y=empirical_cdf(rows,1)
    assert np.array_equal(x,[0.,.1]) and y[-1]==pytest.approx(2/3)


def test_actual_placement_is_scored_and_inaccurate_returns_are_executed():
    clock=simulation.clock;clock.reset();body=SimpleNamespace(actual_placement=Frame.eye());commands=[]
    camera=Frame(Rot('y',.4),vct3(80,30,1300));target=Frame.eye()
    def move(q):
        commands.append(q.copy());clock.advance(.5)
        body.actual_placement=camera*Frame(Rot('z',.002),vct3(.3,0,0))
    robot=SimpleNamespace(within_limits=lambda q:True,move_joints=move)
    kins=SimpleNamespace(inv_kins=lambda F,seed:np.ones(6),
                         fwd_kins=lambda q:Frame(Rot('z',.1),vct3(3,0,0)))
    row=evaluation.solve_attempt(robot,kins,Frame.eye(),target,np.zeros(6),camera,body,True)
    assert row['status']=='scored' and len(commands)==1
    assert np.allclose(row['model_error'],[.1,3.])
    assert np.allclose(row['error'],[.002,.3]) and row['simulated_s']==.5
    assert evaluation.summarize([row])['inaccurate_returns']==1
    assert np.array_equal(row['initial_guess'],np.zeros(6))


def test_calibration_functions_have_no_ground_truth_calls_or_artificial_burn_in():
    tree=ast.parse(Path(evaluation.__file__).read_text())
    for node in tree.body:
        if isinstance(node,ast.FunctionDef) and node.name in ['fit_worker','prepare_seed','make_scene']:
            attributes={n.attr for n in ast.walk(node) if isinstance(n,ast.Attribute)}
            assert not {'actual_ee_pose','ground_truth_parameters','_truth','_rng'}&attributes
    assert '500' not in Path(evaluation.__file__).read_text()


def test_frozen_model_identity_cannot_be_reused_for_another_seed(monkeypatch,tmp_path,nominal_model):
    monkeypatch.setattr(evaluation,'path',lambda seed,suffix:tmp_path/suffix)
    np.savez_compressed(tmp_path/'model.npz',params=nominal_model.params,
                        base_frame=np.eye(4),home_offsets=np.zeros(6))
    (tmp_path/'freeze.json').write_text(json.dumps(dict(seed=7,
        model_sha256=evaluation.digest(tmp_path/'model.npz'),source_sha256=evaluation.source_hashes())))
    with pytest.raises(ValueError,match='identity'):
        evaluation.load_frozen(61,dict(rob=nominal_model))


def test_full_execution_is_explicitly_gated():
    result=subprocess.run([sys.executable,str(Path(evaluation.__file__)),'--stage','full'],
                           capture_output=True,text=True)
    assert result.returncode==2 and 'gated pending pilot review' in result.stderr


def test_compact_summary_and_seed_checkpoint(monkeypatch,tmp_path):
    monkeypatch.setattr(evaluation,'ROOT',tmp_path)
    (tmp_path/'results').mkdir()
    report=dict(seed=1,preparation={},costs={},evaluation_wall_s=2.,simulated_evaluation_s=30.,
        summary={'physical_fk/improved/random':dict(attempted=40,both_pass=40,worst_attempt={'large':'record'})})
    evaluation.save_run_summary([report],'full',[1,7,61])
    summary=json.loads((tmp_path/'results/comprehensive_v1_full_summary.json').read_text())
    checkpoint=json.loads((tmp_path/'results/comprehensive_v1_full_checkpoint.json').read_text())
    assert summary['completed_seeds']==[1] and checkpoint['pending_seeds']==[7,61]
    assert 'worst_attempt' not in summary['seeds']['1']['statistics']['physical_fk/improved/random']
    assert not checkpoint['figures_complete']
    evaluation.save_run_summary([report],'full',[1],figures_complete=True)
    assert json.loads((tmp_path/'results/comprehensive_v1_full_checkpoint.json').read_text())['figures_complete']
    assert not list((tmp_path/'results').glob('*.tmp'))


def test_cached_results_reject_changed_calibration_data(monkeypatch,tmp_path):
    monkeypatch.setattr(evaluation,'path',lambda seed,suffix:tmp_path/suffix)
    (tmp_path/'model.npz').write_bytes(b'frozen model')
    data=tmp_path/'calibration.npz';data.write_bytes(b'observed pairs')
    meta=dict(seed=7,model_sha256=evaluation.digest(tmp_path/'model.npz'),
              data_sha256=evaluation.digest(data),source_sha256=evaluation.source_hashes())
    (tmp_path/'freeze.json').write_text(json.dumps(meta))
    assert evaluation.validate_freeze(7)==meta
    data.write_bytes(b'changed observed pairs')
    with pytest.raises(ValueError,match='calibration data hash'):
        evaluation.validate_freeze(7)


@pytest.mark.parametrize('seed',[1,7])
def test_saved_pilot_has_exact_attempt_counts_and_preserves_failed_targets(seed):
    report=json.loads(evaluation.path(seed,'pilot.json').read_text())
    assert report['fk_counts']==evaluation.PILOT_FK and report['ik_counts']==evaluation.PILOT_IK
    assert evaluation.digest(evaluation.path(seed,'freeze.json'))==report['freeze_sha256']
    for model in ['nominal','improved']:
        rows=[r for r in report['rows'] if r['model']==model]
        assert sum(r['family']=='physical_fk' for r in rows)==96
        assert sum(r['family']=='physical_ik' for r in rows)==20
        assert sum(r['family']=='numerical_consistency' for r in rows)==20
        assert sum(r['family']=='numerical_seed_sensitivity' for r in rows)==8
        assert sum(r['family']=='tracker_prediction' for r in rows)==8
        for category in evaluation.CATEGORIES:
            subset=[r for r in rows if r['family']=='physical_ik' and r['category']==category]
            stats=report['summary'][f'physical_ik/{model}/{category}']
            assert stats['attempted']==len(subset)
            assert stats['simultaneous_pass_rate']==pytest.approx(stats['both_pass']/len(subset))
    freeze=json.loads(evaluation.path(seed,'freeze.json').read_text())
    assert freeze['frozen_before_ground_truth'] and freeze['seed']==seed
    assert evaluation.digest(evaluation.path(seed,'model.npz'))==freeze['model_sha256']
    assert report['costs']['tracker_prediction']['samples']==8
    assert report['noise']['evaluation_sensor_seed']!=report['noise']['calibration_sensor_seed']
    assert report['preparation']['new_tracker_samples']==(0 if seed==1 else 775)
    data_file=evaluation.ROOT/'results/step8_phase_d/calibration.npz' if seed==1 else evaluation.path(seed,'calibration.npz')
    train,_=evaluation.load_data(data_file)
    training_configurations=set(map(tuple,train.thetas))
    assert all(tuple(r['theta']) not in training_configurations for r in report['rows']
               if r['family']=='physical_fk')
