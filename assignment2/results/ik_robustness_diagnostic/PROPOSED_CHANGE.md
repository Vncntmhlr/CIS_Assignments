# Proposed minimal IK patch — approval required; not applied

Change only `ImprovedKinematics.inv_kins()` if approved. Keep its signature,
calibrated forward model, parameters, frame convention, joint limits, default
50 iterations, finite-difference magnitudes, step caps and caller arguments.
Do not change simulation, notebook content or original assertions.

1. **Honor the caller seed as the first numerical start.** Validate finiteness,
   project it into the permitted encoder box, and refine it before the two
   existing nominal starts. A seed already reproducing the target returns
   immediately. A refined seed solution may return early only after the full
   final checks below. Preserve nominal starts as fallbacks if it fails. This
   makes the seed a preferred local branch, rather than only a post-hoc rank.
   Among fallback solutions that pass validation, retain nearest-seed selection.
   Early acceptance prefers the validated local seed solution; it does not
   promise a global closest solution over all nonlinear inverse branches.

2. **Balance the signed pose residual and Jacobian rows.** Keep the original
   `dF=FK(theta)^-1*target`, with its rotation vector and body translation.
   Minimize `r=[alpha/0.001; epsilon/0.2]`, and apply the same row scales to J.
   Use joint-coordinate scales S=diag(50,50,50,0.3,0.3,0.3) for the linear step.
   Keep `tol` as the requested raw residual convergence tolerance; normalized
   quantities control optimization, not a relaxation of final accuracy.

3. **Keep trial encoders feasible and unwrapped.** Project nominal initial
   guesses and each trial into the existing joint box, re-evaluating FK/cost
   after projection. Use an inward finite difference at an active upper
   limit, avoiding derivative probes outside the modeled interval. Remove the
   final modulo operations on encoder joints 4 and 6. Equivalent ideal angles
   are not automatically equivalent calibrated encoder commands.

4. **Use the damped augmented least-squares system.** With A=W*J*S and
   D=diag(sqrt(diag(A.T*A)+1e-12)), solve
   `[A; sqrt(lambda)*D] dz = [-r; 0]` by `np.linalg.lstsq`, then map back
   `dtheta=S*dz`. Retain the existing physical step caps and basic damping
   updates. This avoids explicitly solving squared normal equations near
   rank loss. The ablation shows this is numerical hardening; balancing and
   seed handling provide the observed major improvement.

5. **Validate every candidate after all processing.** Recompute FK and check
   permitted encoders, requested raw `tol`, and angular/position residuals
   separately in rad/mm. Do not append an unconverged in-limit vector merely
   because its iteration budget expired. Any optional numerical snap is
   limited to 1e-6 mm / 1e-8 rad overruns and followed by the same residual
   checks; it cannot justify clipping a genuinely infeasible solution.

6. **Return an honest failure if no validated candidate exists.** Keep the
   `ValueError` interface, but report that no converged in-limit solution was
   found; do not imply mathematical unreachability from numerical failure.
   At a damping cap, stop that start as stalled rather than accepting an
   uphill step unconditionally. This safety condition was not triggered in
   the reproduced failures and needs its own regression check when patched.

The separate diagnostic tested this structure without modifying production:
78/78 strict round trips at 50 iterations, including all 19 prior failures.
The ordinary 36 cases remain strict passes. This is finite test evidence,
not proof of global convergence for every possible reachable pose. Trial
projection is a local LM safeguard, not a general constrained global solver.

If approved, add live regression assertions for all 78 cases, final-residual
rejection, boundary/wrapping correctness, singular-pose behavior and damping
failure handling. Remove an expected-failure marker only after its original
strict assertion passes; do not weaken any original assertion. Re-run the
existing full suite and a larger deterministic ordinary-pose FK/IK sample.

No patch is applied here. This method is protected TA non-TODO code under
`AGENTS.md`; user approval is required before editing it.
