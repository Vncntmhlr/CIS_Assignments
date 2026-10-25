# Adopted constrained model and Phase E evidence

The normal `ImprovedKinematics.fit(data)` now enforces the known joint-distortion
magnitude and derivative bounds. Existing layout, fixed home corrections,
base-roll gauge and method signatures are preserved. The historical
[unconstrained results](../step8_phase_d/README.md) and
[diagnostic candidate](../step8_bound_diagnostic/README.md) remain available.

- `fit.npz`: adopted normal cold-start parameters, integrated warm-start
  parameters, and the reference candidate; load with `allow_pickle=False`.
- `parameters.json`: unit axes, angle matrix, effective offsets/scales,
  base/tool transforms, distortion bounds and gauge/convention metadata.
- `summary.json`: fitting diagnostics, candidate reproduction, tracker errors,
  all 78 provided-IK round trips, public-control retries and archive hashes.

In the [documented environment](../../SETUP.md), from `assignment2/`:

```bash
venv/bin/python experiments/step8_phase_e.py --verify-fit
venv/bin/python experiments/step8_phase_e.py --audit-only
venv/bin/python -m pytest -q
```

The first command verifies a saved-data warm fit without overwriting any archives. `--audit-only`
reuses these integrated fits and only refreshes reports/IK checks. The optional
marker-refinement test still fails with `NotImplementedError`: its TODO was
intentionally left unimplemented for the exact special calibration body. The
complete command above includes it; this pre-existing failure is not hidden. Three new expected failures
document limitations in the unchanged provided IK; all original assertions are
preserved. See [report notes](../../REPORT_NOTES.md) for the full audit counts.

The production fitter accepts `physical_constraints=False` explicitly for
historical unconstrained experiments. Common diagnostics/options remain available;
SLSQP uses `ftol`, while `xtol/gtol` apply to the legacy TRF mode. Caller-supplied
physical-coordinate box bounds and characteristic parameter scales are supported.
The default fit takes about 64.17 s from zero; warm-start verification 25.71 s.
After helper relocation, the saved-data verification converged in 2 iterations
and 28.40 s at cost `1102.385058090643`; all physical bounds passed. Maximum
prediction difference from the adopted model was `1.47e-6 mm / 1.23e-8 rad`.
No additional simulated time or hidden truth is used.

The provided `inv_kins()` is unchanged. Increasing its existing `max_iter` to 200
and setting `tol=1e-12` improves the tested near-pole/boundary cases, but calibrated
encoder maps are nonperiodic and the solver's final angle wrapping still causes
residuals near a wrist limit. Default and stricter-control results are kept
separate. These are model FK/IK consistency checks, not official robot accuracy.

`joint_offset` means total fitted correction at encoder zero; `joint_scale`
means its local derivative. These do not separately recover the constant/linear
terms of an unknown sinusoidal distortion. Base/axis/tool components depend on
the selected gauge; inter-axis angles and joint maps are invariant.

No TA notebook, original assertion, simulation code or provided inverse solver
was edited. Steps 9–10 have not been started.
