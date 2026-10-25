# Focused IK robustness investigation — production solver unchanged

All 19 Phase E strict failures were reproduced. An instrumented replay matches
the live provided solver on all 78 original cases, including its rejected case.
Every target has an in-limit generating command that reproduces it by improved
FK; none is genuinely unreachable. No acquisition, fitting, or Steps 9–10 work
is performed here.

- [Every failed case](failed_cases.md): target/caller seed, both nominal initial
  guesses, final joints before/after wrapping, limits, residuals and termination
  reasons; per-case comparisons against increased iteration limits and the
  diagnostic proposed path.
- `traces.json.gz`: compressed machine-readable branch/iteration records for
  all 19 failures and all ablations. Default traces include Jacobian singular
  values/conditioning, damping, accepted/rejected steps and intermediate limits.
- `summary.json`: all 13 ablations, category counts/timings, classification and
  production-source/adopted-fit hashes.
- [Exact proposed changes for approval](PROPOSED_CHANGE.md). No patch is applied.

The trace script is [student-owned](../../experiments/ik_robustness_diagnostic.py).
From `assignment2/` in the [existing environment](../../SETUP.md):

```bash
venv/bin/python experiments/ik_robustness_diagnostic.py
venv/bin/python -m pytest experiments/test_ik_robustness.py -q
```

Strict checks: position <1e-6 mm and orientation <1e-8 rad. These are model
FK/IK checks, not robot ground-truth accuracy. Target-sized checks use 0.2 mm /
0.001 rad. The original default loop stops on its unscaled raw norm <1e-10.
All comparisons reuse Phase E's same nearby caller seeds (0.1 mm linear and
at most 0.05 rad wrist perturbations, clipped to limits). No alternative solver
is given the generating configuration as a privileged initial guess. Success
on these cases is not a global guarantee for arbitrary distant seeds.

| Setting | Strict passes / 78 | No candidate | Worst returned angle (rad) | Replay wall time (s) |
| --- | ---: | ---: | ---: | ---: |
| Provided defaults: 50 iterations | 59 | 1 | 0.0105345 | 3.82 |
| Only increase to 200 iterations | 71 | 1 | 0.000196793 | 7.71 |
| 200 iterations and tol=1e-12 | 72 | 0 | 0.000196793 | 7.95 |
| Add actual caller seed as a start | 73 | 0 | 0.000196793 | 4.80 |
| Seed + no wrap + final gate + tiny snap | 74 | 4 | 9.44e-11 | 4.82 |
| Above + feasible iterates | 74 | 4 | 7.97e-11 | 8.59 |
| Above + augmented least squares | 74 | 4 | 7.97e-11 | 8.86 |
| Above + balanced residuals | 78 | 0 | 1.76e-11 | 6.07 |
| Proposed path, verified seed attempt returns early | 78 | 0 | 1.76e-11 | 0.58 |

Alternative isolating tests: balanced residuals without the caller seed reach
76/78; weighted normal equations and weighted augmented least squares both
reach 78/78. Thus direct normal equations are a numerical risk, but changing
the linear solve alone is not the observed cure. Balancing rotation/translation
and honoring the supplied initialization are material changes.

On the 36 ordinary held-out cases, the proposed path retains 36/36 strict
passes, with maximum errors 8.91e-11 mm / 2.63e-12 rad. Instrumented replay time
is 0.223 s versus 0.908 s for defaults. These single-run timings include tracing
overhead and are not production performance guarantees.

The three independent failure classes are:

1. **15 unchecked iteration-limit returns.** Nominal Euler starts are far
   from the calibrated solution near poles; the caller seed is used only to
   rank candidates. The loop mixes mm/rad without scaling and can spend 50
   iterations making small damped improvements. It appends an in-limit result
   even if the residual never converged. In `pole_-1_offset_0.01`, it selects a
   candidate with 0.0105345 rad error while another in-limit candidate already
   has 1.41e-13 rad / 7.05e-12 mm error, because selection ranks seed distance
   without an accuracy gate.
2. **3 nonperiodic wrapping failures.** A converged unwrapped encoder is changed
   by +/-2*pi after refinement. For q(theta)=theta+delta(theta),
   q(theta+2*pi)-q(theta)=2*pi+delta(theta+2*pi)-delta(theta); the correction is
   nonperiodic. This creates about 1.968e-4 rad orientation error and 0.01969 mm
   position error at the approximately 100 mm tool lever. More iterations
   cannot repair a mutation performed after convergence.
3. **1 numerical boundary rejection.** J2-upper has a branch with about
   2.86e-11 rad / 7.70e-12 mm error but encoder overshoot 2.393e-9 mm, exceeding
   `within_limits`' 1e-9 margin. This is a tolerance/limit-handling mismatch,
   not proof that the target is unreachable.

Intermediate guesses/iterates are unconstrained in the provided solver; among
the failed traces, angular-limit excursions reach 0.657 rad. The target
Jacobian has rank 5 at both fitted poles; normalized condition numbers are
about 1e16–1e17 there and 2.32e4 at a 1e-4-rad offset. Scaling choices are
explicit: residual scales 0.001 rad / 0.2 mm; joint scales 50 mm / 0.3 rad.
The analytic body Jacobian is independently checked against central differences.
Normal equations square conditioning. Many near-pole traces show repeated
damping rejections; the source also permits an uphill acceptance at lambda>1e6,
but **zero such forced acceptances occurred** in these failed traces. That
guard is a safety concern, not an observed explanation for this dataset.

The provided inverse, existing assertions and adopted parameters are unchanged.
The later organizational cleanup only relocated the constrained-fit helper
import. The regression normalizes exactly that import change when checking the
historical full-source hash; the archived diagnostic evidence remains intact. Additional simulated time is exactly 0 s. All experimental switches
are confined to the diagnostic script and await approval before adoption.
