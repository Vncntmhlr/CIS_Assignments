# Retained calibration and evaluation evidence

These archives are reusable inputs or report-worthy results, not setup logs.
Load NPZ files with `allow_pickle=False`. Reproduction commands are in
[experiments/README.md](../experiments/README.md).

- [step8_phase_d](step8_phase_d/README.md): the only acquisition datasets
  (765 training readings / 69 configurations; 396 held-out readings / 36 distinct
  configurations), acquisition provenance, original unconstrained parameters,
  numerical summaries and two signed-residual figures.
- [step8_bound_diagnostic](step8_bound_diagnostic/README.md): original constrained
  candidate, sampling covariance and physical-bound/noise-coupling comparison.
  Preserved as an independent historical reference for adoption.
- [step8_phase_e](step8_phase_e/README.md): adopted constrained parameters,
  physical report, cold/warm fit diagnostics and 78-case provided-IK audit.
- [ik_robustness_diagnostic](ik_robustness_diagnostic/README.md): readable failure
  cases, compressed iteration traces, ablation summaries and proposed changes
  that have not been applied to the TA inverse.

The small repeated parameter arrays in the candidate/adoption files preserve
historical comparison and hash provenance. They are deliberately retained; no
raw uncompressed trace or disposable intermediate fit remains. Historical JSON
hashes describe the source at acquisition/analysis time. The IK regression
normalizes only the approved helper-import relocation before comparing its
historical source hash; all numerical archives remain byte-for-byte unchanged.

Step 9 uses the frozen constrained fit, with report-only access to deterministic
physical poses. All 729 fixed-grid configurations pass both accuracy targets for
seed 1; no motion, sampling or fitting occurs. Its outputs reuse this folder:

- [step9_forward_summary.json](step9_forward_summary.json): complete metrics,
  target counts, worst configurations, timing and saved-input hashes.
- [step9_forward_errors.npz](step9_forward_errors.npz): ordered 729x6 encoders
  and nominal/improved errors; each error row is `(angle_rad, position_mm)`.
- [step9_forward_errors.png](step9_forward_errors.png): error distributions
  versus the assignment targets. These artifacts must not be used for fitting.

Step 10 uses the same frozen fit on the original 40-target subset:

- [step10_inverse_summary.json](step10_inverse_summary.json): physical error
  statistics, all nominal/improved commands and seeds, exceptions, returned
  model residuals, limit status and separate replay/testing times.
- [step10_inverse_errors.npz](step10_inverse_errors.npz): 80 alternating
  nominal/improved rows; errors are `(angle_rad, position_mm)`, with NaNs for
  unavailable values if an attempt fails. Load with `allow_pickle=False`.

All 40 improved attempts pass for this seed/noise sequence; the previously
documented edge-case IK limitations remain. No additional calibration data
are acquired and no model is changed. Historical Step 8 summaries describe their
original measurement/analysis scope; the Step 9 oracle is a separate evaluation.
