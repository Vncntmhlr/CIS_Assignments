"""Diagnostic replay fidelity and evidence checks; no production IK patch."""

import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from experiments.ik_robustness_diagnostic import replay, target_jacobian
from experiments.step8_phase_e import saved_model
from uncertainty_networks.se3 import log_so3


@pytest.fixture(scope='module')
def evidence():
    root = Path(__file__).resolve().parents[1]
    folder = root/'results/ik_robustness_diagnostic'
    if not (folder/'traces.json.gz').exists():
        pytest.skip('Run the separate IK robustness diagnostic first')
    with gzip.open(folder/'traces.json.gz','rt') as handle:
        cases = json.load(handle)
    summary = json.loads((folder/'summary.json').read_text())
    model, _ = saved_model()
    with np.load(root/'results/step8_phase_e/fit.npz',allow_pickle=False) as saved:
        model.params = saved['fitted_params'].copy()
    return root, model, {c['name']:c for c in cases}, summary


@pytest.mark.parametrize('name', ['joint2_upper','joint6_upper','pole_-1_offset_0.01','nominal_pole_1'])
def test_instrumented_default_matches_the_unchanged_live_solver(evidence, name):
    _, model, cases, _ = evidence
    case = cases[name]
    goal = model.fwd_kins(case['target'])
    trace = replay(model,goal,np.array(case['seed']))
    if trace['result'] is None:
        with pytest.raises(ValueError):
            model.inv_kins(goal,seed=case['seed'])
    else:
        assert np.allclose(trace['result'],model.inv_kins(goal,seed=case['seed']),atol=1e-11,rtol=0)


def test_all_failures_have_reachable_witnesses_and_complete_branch_records(evidence):
    _, model, cases, summary = evidence
    assert len(cases) == 19 and set(cases) == set(summary['failed_case_names'])
    for case in cases.values():
        assert model.within_limits(case['target']) and case['generating_solution']['metrics']['strict']
        assert case['generating_solution']['in_limits']
        assert not case['runs']['default']['strict_pass']
        for branch in case['runs']['default']['branches']:
            assert len(branch['initial_guess']) == len(branch['final_theta']) == 6
            assert branch['reason'] in {'raw_residual_tolerance','iteration_limit'}
            assert len(branch['history']) == branch['iterations']
            assert branch['final_metrics']['position_mm'] >= 0
            assert branch['final_metrics']['angle_rad'] >= 0


def test_wrapping_invalidates_an_already_converged_calibrated_solution(evidence):
    _, _, cases, _ = evidence
    case = cases['joint6_upper']['runs']['default']
    branch = case['branches'][case['selected_branch']]
    assert branch['before_metrics']['strict'] and not branch['final_metrics']['strict']
    assert abs(abs(branch['final_theta'][5]-branch['before_postprocessing'][5])-2*np.pi) < 1e-12
    assert branch['final_in_limits'] and not branch['before_in_limits']


def test_boundary_rejection_is_numerical_not_proof_of_unreachability(evidence):
    _, _, cases, _ = evidence
    case = cases['joint2_upper']['runs']['default']
    assert case['result'] is None
    assert any(b['final_metrics']['strict'] and 1e-9 < max(b['final_limit_violation']) < 1e-6
               for b in case['branches'])


def test_actual_caller_seed_is_not_a_provided_initial_guess(evidence):
    _, model, cases, _ = evidence
    case = cases['pole_-1_offset_0.01']
    theta = np.array(case['target'])
    baseline = replay(model,model.fwd_kins(theta),theta)
    assert all(not np.allclose(b['initial_guess'],theta) for b in baseline['branches'])
    seeded = replay(model,model.fwd_kins(theta),theta,include_seed=True,wrap=False,gate=True)
    assert seeded['branches'][0]['label'] == 'caller_seed'
    assert seeded['branches'][0]['iterations'] == 0 and seeded['branches'][0]['final_metrics']['strict']


def test_gated_variants_never_return_an_inaccurate_candidate(evidence):
    _, _, cases, summary = evidence
    for case in cases.values():
        for label, options in summary['settings'].items():
            run = case['runs'][label]
            if options.get('gate') and run['result'] is not None:
                branch = run['branches'][run['selected_branch']]
                assert branch['final_metrics']['strict'] and branch['final_in_limits']


def test_production_source_and_adopted_fit_were_not_changed(evidence):
    root, _, _, summary = evidence
    # Historical evidence predates the import-only helper relocation. Normalize
    # exactly that approved path change, preserving the original archive hashes.
    source = (root/'src/student/kinematics.py').read_text().replace(
        'from vendor.functions._fit_constraints import', 'from ._fit_constraints import')
    assert hashlib.sha256(source.encode()).hexdigest() == summary['source_sha256']
    assert hashlib.sha256((root/'results/step8_phase_e/fit.npz').read_bytes()).hexdigest() == summary['fit_sha256']
    assert summary['additional_simulated_time_s'] == 0


def test_analytic_target_jacobian_matches_central_differences(evidence):
    _,model,_,_ = evidence
    theta=np.array([300.,-10.,250.,.7,.5,-.4])
    goal=model.fwd_kins(theta)
    def residual(q):
        difference=model.fwd_kins(q).inv()*goal
        return np.r_[log_so3(difference.R.matrix),difference.p.vec.ravel()]
    numeric=np.zeros((6,6))
    for k,step in enumerate([1e-3]*3+[1e-6]*3):
        delta=np.eye(6)[k]*step
        numeric[:,k]=(residual(theta+delta)-residual(theta-delta))/(2*step)
    assert np.allclose(target_jacobian(model,theta),numeric,atol=2e-6,rtol=1e-6)


def test_singular_rank_and_primary_failure_classes_are_reported(evidence):
    _,_,cases,summary=evidence
    assert summary['classification']['primary_counts']=={
        'numerical_boundary_rejection':1,'nonperiodic_wrist_wrapping':3,
        'iteration_limit_without_final_validation':15}
    for sign in [-1,1]:
        assert cases[f'pole_{sign}_offset_0.0']['target_conditioning']['numerical_rank']==5
    assert summary['classification']['inaccurate_selection_despite_accurate_candidate']==['pole_-1_offset_0.01']
