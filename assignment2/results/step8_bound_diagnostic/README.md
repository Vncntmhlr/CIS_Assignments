# Joint-bound diagnostic: separate candidate

The original [Phase D data and fit](../step8_phase_d/README.md) remain unchanged.
No additional measurements were acquired and no hidden robot values were used.
`candidate.npz` preserves the diagnostic comparison. Its method has now been
adopted in the [normal fitter and Phase E](../step8_phase_e/README.md); the archive
is retained unchanged as the reproduction reference.

- `candidate.npz`: original and constrained 34-parameter vectors, plus the
  unconstrained fit's linearized random-sampling covariance.
- `summary.json`: full-interval distortion/magnitude/derivative checks for all
  joints, solver/KKT diagnostics, training/held-out error comparisons, sampling
  uncertainty/coupling estimates and hashes of the original Phase D archives.

The full correction is `delta(theta)=o+a*x+b*x^2`, `x=(theta+o)/s`, including
the measured home correction. Magnitude extrema occur at interval endpoints
and an interior vertex when present; derivative extrema are at the endpoints.
All six joints satisfy both prescribed bounds in the constrained candidate.

The experiment minimizes the same target-weighted residual cost over all 765
training observations, normalized by 765 solely for numerical scaling. SLSQP
handles the coupled exact interval constraints. A fixed SVD transformation
preconditions the 33 free solver coordinates; the original parameter layout,
fixed home offsets and fixed `params[0]` are preserved. The 396 observations at
36 held-out configurations are used only for comparison.

From `assignment2/`, in the environment documented in [SETUP.md](../../SETUP.md):

```bash
venv/bin/python experiments/step8_phase_d.py --bounds
venv/bin/python -m pytest experiments/test_calibration.py -q
```

The superseded standalone driver/solver was consolidated into the Phase D
driver; `--bounds` delegates to the normal fitter on a separate candidate model
and retains the independent extrema/covariance calculations. Running it can
replace this historical candidate archive; use the read-only verification
command in [Phase E](../step8_phase_e/README.md) for routine checks.

The original constrained fit was warm-started from the saved unconstrained solution.
Its 26.70 s runtime includes Jacobian/SVD preparation. The archived 80.55 s
unconstrained fit started from zero corrections, so these times are not a
controlled solver-speed comparison. Additional simulated acquisition time: 0 s.

Training cost rises by 0.042%; held-out mean position/orientation RMS improves
slightly, while maximum orientation error rises slightly. This is evidence
against a large accuracy penalty, not proof of statistical superiority or
official robot accuracy. Linearized covariance assumes independent random
repetitions, a fixed quadratic model and fixed measured home offsets; it omits
systematic tracker bias, model mismatch and homing uncertainty.

See [REPORT_NOTES.md](../../REPORT_NOTES.md) for numerical results. This diagnostic
did not modify the production fitter; integration occurred subsequently in
Phase E. TA notebook content, original assertions, simulator and inverse solver
remain unchanged. Steps 9–10 have not been performed.
