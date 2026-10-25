# Step 8 Phase D evidence

Public robot/tracker measurements only. No ground-truth pose or parameter
oracle is used in acquisition, fitting, or evaluation. This is student-owned
experimental evidence; the TA notebook, solver, simulator and tests are unchanged.

- `calibration.npz`: 765 paired encoder vectors / observed 4x4 Frames,
  collection time, measured home offsets and initial base estimate (69 commands).
- `holdout.npz`: 396 independent observations at 36 new commands, never fitted.
- `fit.npz`: initial and fitted 34-parameter vectors; coordinate 0 stays fixed.
- `acquisition.json`: seeds, source-cell indices and simulated acquisition times.
- `summary.json`: optimizer diagnostics, before/after errors, per-sweep summaries,
  within-pose noise estimates, slope checks and protected-source hashes.
- `sweep_residuals.png`: signed per-command mean position residuals in tracker axes.
- `sweep_orientation_residuals.png`: signed rotation-vector residuals, expressed
  in the observed EE frame (mrad).

All NPZ arrays load with `allow_pickle=False`. Encoder units are mm (1–3) and rad
(4–6); Frame translations are mm. Frame matrices map EE coordinates to tracker
coordinates. Costs use target scales 0.2 mm / 0.001 rad, not noise whitening.
Configuration means are descriptive validation statistics, not replacement
samples for fitting. The shared zero reference appears in each sweep diagnostic.

From `assignment2/`, using the environment documented in [SETUP.md](../../SETUP.md):

```bash
venv/bin/python experiments/step8_phase_d.py --analyze-only
venv/bin/python -m pytest experiments/test_calibration.py -q
```

Omit `--analyze-only` to repeat the historical unconstrained 765-row fit; the
runner explicitly passes `physical_constraints=False`. The normal fitter now
uses the adopted physical constraints by default (Phase E).
Available measurement archives are reused. Missing archives trigger acquisition
from the original notebook cells, followed by independently designed validation;
the notebook is read, never rewritten. Original experiment fitting took 80.55 s
wall-clock. Re-acquisition required 554.03 s simulated (Steps 6–7), plus 432.26 s
for validation. These are distinct clocks.

The held-out mean tracker errors support this model as a baseline for this seed;
they do not certify physical robot accuracy. Some fitted wrist slopes violate
known physical priors. See [report notes](../../REPORT_NOTES.md) for limitations.
This is the historical Phase D archive; later development is recorded in
[Phase E](../step8_phase_e/README.md). Official Steps 9–10 remain unperformed.
