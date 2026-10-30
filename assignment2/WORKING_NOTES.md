# Assignment 2 — Working Notes

Interactive notes for Ahmed's work on `dev/ahmed`. Capture requested points by topic; these notes are source material for report writing and validation, not evidence that experiments ran.

## Workflow Decisions
- Use CIS Working Notes to capture explanations, equations, ideas, decisions and open questions here.
- Use CIS Report Notes for explanatory report sections, with Development inactive.
- Use CIS Report Stats to read these notes on every invocation, apply Development to requested validation work, and write Validation Approach, Final Results, and Conclusion in REPORT_NOTES.md.
- Use Assignment 1's linked final report as a quality and structure reference; keep Assignment 2 evidence separate.
- Restrict all repository writes to `dev/ahmed`. Preserve other branches and existing report content.

## Concepts and Equations

### Calibration pipeline — notation and mathematical model
**Status: conceptual framework/proposed implementation from our discussion, not measured results.** Supporting source: [kinematics.py](src/student/kinematics.py), [robot.py](src/robot.py), and [workflow.py](src/student/workflow.py). Earlier report-oriented explanations remain in [REPORT_NOTES.md](REPORT_NOTES.md).

Use one convention throughout:
- \( {}^AF_B=[R,p] \) maps coordinates from frame B into frame A; \(p\) is expressed in A.
- Frames: workspace {0}, robot base {rob}, tracker {trk}, carriage {c}, wrist input {w0}, wrist output {w}, tool {ee}.
- \(F_X={}^\mathrm{trk}F_\mathrm{rob}\); \(F_c={}^\mathrm{rob}F_c\), \(F_{cw}={}^cF_{w0}\), \(F_w={} ^{w0}F_w\), \(F_{we}={}^wF_{ee}\).
- Joint index \(i=1,\ldots,6\); observation index \(j\); polynomial power \(m\).
- \(\theta_{i,\mathrm{cmd}}\): command/encoder coordinate; \(\theta_{i,\mathrm{est}}=\theta_{i,\mathrm{cmd}}\): nominal joint-coordinate estimate.
- \(\theta_{i,\mathrm{meas}}\): inferred joint coordinate when recoverable from tracker-observed tool motion; the tracker directly measures a tool pose, not a joint angle.
- \(\widehat{\Delta\theta}_i\): modeled measured-minus-estimated correction; \(\theta_{i,\mathrm{cal}}=\theta_{i,\mathrm{est}}+\widehat{\Delta\theta}_i\).
- Joints 1–3: mm; joints 4–6: radians. Hats on axes/transforms indicate calibrated estimates.

**1. Joint coordinates: correct how far/how much the joint moves.** With fixed positive normalization scales \(s_i\), stored offsets \(o_i=\texttt{self.joint_offsets[i-1]}\), polynomial degree \(n=\texttt{n_poly}\), and coefficients \(c_{i,m}\):

```math
u_i=\theta_{i,\mathrm{cmd}}/s_i,\qquad
\widehat{\Delta\theta}_i=o_i+\sum_{m=1}^{n}c_{i,m}u_i^m,\qquad
\theta_{i,\mathrm{cal}}=\theta_{i,\mathrm{cmd}}+\widehat{\Delta\theta}_i.
```
For n_poly = 2, the variable distortion is \(c_{i,1}u_i+c_{i,2}u_i^2\). The power loop adds one polynomial term to all six joints at each iteration. Coefficients stay fixed during prediction; their contribution changes with the command. These are six separate 1D correction functions, fitted jointly with geometry; polynomial fitting is not necessarily interpolation through samples.

**2. Axis directions: correct the direction of motion/rotation.** For each nominal unit axis \(a_i\in\{d_1,d_2,d_3,n_4,n_5,n_6\}\), choose two orthonormal vectors \(b_{i,1},b_{i,2}\) perpendicular to it:

```math
\phi_i=t_{i,1}b_{i,1}+t_{i,2}b_{i,2},\qquad
\hat a_i=\exp([\phi_i]_\times)a_i,\quad i=2,\ldots,6.
```
Here \([\phi_i]_\times\) is the skew matrix of the tilt rotation vector. This is a perpendicular **basis**, not bias. A single axis needs two tilt parameters because rotation about itself does not change its direction. Joint 1's direction stays fixed. A linear joint still has one motion coordinate; its rail direction has two calibration parameters.

**3. Compose the calibrated geometry.** \(R(a,\theta)=\exp([a]_\times\theta)\):

```math
F_{c,\mathrm{cal}}(\theta_{1,\mathrm{cmd}},\theta_{2,\mathrm{cmd}},\theta_{3,\mathrm{cmd}})
=\left[I,\sum_{i=1}^{3}\theta_{i,\mathrm{cal}}\hat d_i\right],
\qquad
F_{w,\mathrm{cal}}(\theta_{4,\mathrm{cmd}},\theta_{5,\mathrm{cmd}},\theta_{6,\mathrm{cmd}})
=\left[R(\hat n_4,\theta_{4,\mathrm{cal}})
R(\hat n_5,\theta_{5,\mathrm{cal}})
R(\hat n_6,\theta_{6,\mathrm{cal}}),0\right].
```
The fixed tool mounting has its own six-parameter SE(3) correction \(\xi_{we}\), independent of joint distortion:

```math
\hat F_{we}=F_{we}\operatorname{Exp}_{SE(3)}(\xi_{we}),\qquad
\hat F_X=F_{X,\mathrm{init}}\operatorname{Exp}_{SE(3)}(\xi_X).
```
Each transform correction has three rotational and three translational degrees of freedom. The translational twist coordinates are converted by the SE(3) exponential; they are not generally the resulting translation column directly.

```math
F_{ee,\mathrm{cal}}^\mathrm{rob}(\boldsymbol\theta_{\mathrm{cmd}})
=F_{c,\mathrm{cal}}F_{cw}F_{w,\mathrm{cal}}\hat F_{we},
\qquad
F_{ee,\mathrm{pred}}^\mathrm{trk}(\boldsymbol\theta_{\mathrm{cmd}})
=\hat F_XF_{ee,\mathrm{cal}}^\mathrm{rob}(\boldsymbol\theta_{\mathrm{cmd}}).
```
Nominal Fcw and Fwe translations are (500,0,0) mm and (100,0,0) mm. Joint distortion affects the final tool pose through Fc/Fw; mounting correction applies to Fwe itself.

### Parameter-to-code map
The proposed flat vector has length \(22+6n\), shape (34,) for n_poly = 2. Home offsets are stored separately.

| Parameter slice | Meaning |
|---|---|
| params[0:6] | Base correction xi_X; used by base_frame(), not inside fwd_kins() |
| params[6:10] | Two tilts each for carriage axes d2 and d3 |
| params[10:16] | Two tilts each for wrist axes n4, n5, n6 |
| params[16:22] | Tool mounting correction xi_we |
| params[22:].reshape(6, n_poly) | One row of distortion coefficients per joint |

For n_poly = 2, joint i uses params[22 + 2*(i-1):22 + 2*i]. Zero extra parameters mean zero additional corrections; nonzero stored home offsets still apply.

## Proposed Methods and Hypotheses

### Algorithmic calibration pipeline
1. **Place frames and markers.** Construct workspace Frames; initialize \(F_{X,\mathrm{approx}}=({}^0F_\mathrm{trk})^{-1}{}^0F_\mathrm{rob}\). Check nominal marker visibility over candidate motions.
2. **Measure home offsets.** Repeated physical homing provides encoder readings at zero physical joint coordinates; estimate the correction with the appropriate opposite sign. Keep the actual home reference distinct from an assumed zero command.
3. **Plan six individual-axis sweeps.** Hold other joints at the reference; sample each joint's range and return to reference. Check joint limits and all-marker visibility. Distinct positions constrain variation; repeated re-command/read cycles reduce tracker and repeatability noise.
4. **Collect aligned observations.** Store each six-coordinate command in data.thetas[j] and its tracker tool Frame in data.frames[j]. Skip missing observations without misaligning the pairs.
5. **Evaluate fwd_kins(theta, params).** Select self.params if params is None; evaluate the six joint polynomials, add home offsets, rotate the five corrected axes, form Fc/Fw, correct Fwe, and compose the robot-base-relative pose. Do not add parameter slices directly to joint coordinates: coefficients must be evaluated and tilts must rotate axes.
6. **Fit pose residuals.** predict_in_tracker() applies the base transform. Compare against each observed Frame using \(E_j=F_{ee,j,\mathrm{meas}}^{-1}F_{ee,j,\mathrm{pred}}\); use its rotation-vector and translation residuals with explicit rad/mm weighting. Nonlinear least squares adjusts the parameter vector, not the original commands.
7. **Validate later.** Check nominal agreement with zero corrections and zero offsets, fit synthetic known models, inspect residual-Jacobian singular values, and evaluate unseen joint combinations. Report position, orientation and time separately; no such runs are claimed here.

### Accuracy-informed sampling proposal
For adjacent samples a,b, let \(h_i=b-a\) and \(\lambda=(\theta_{i,\mathrm{cmd}}-a)/h_i\). With exact endpoint corrections and slope bound \(\omega_{\mathrm{sys},i}\), linear interpolation obeys:

```math
|\Delta\theta_i-\widehat{\Delta\theta}_i|
\le2\omega_{\mathrm{sys},i}h_i\lambda(1-\lambda)
\le\omega_{\mathrm{sys},i}h_i/2,\qquad
h_{i,\max}=2\varepsilon_i/\omega_{\mathrm{sys},i}.
```
Choose \(N_i=\max(\texttt{samples_per_joint},\lceil(\theta_{i,\max}-\theta_{i,\min})/h_{i,\max}\rceil+1)\). The discussed wrist allowance \(\varepsilon_i=0.5\,\texttt{desired_accuracy}/3\) is a tunable allocation, not a guaranteed total RMS budget. Use a separate position allowance in mm for joints 1–3. Minimum counts and ceiling rounding can hide small changes in required counts. Check retained gaps after visibility filtering.

This bound applies to linear interpolation, not automatically to the proposed polynomial fit. Noise, geometric uncertainty and wrist-to-tool lever-arm effects need separate consideration.

## Open Questions
- Confirm the final correction representation: per-joint polynomial fitting versus piecewise linear interpolation; validate spacing against the chosen representation.
- Fix normalization scales, polynomial ordering and perpendicular bases consistently across prediction, fitting and parameter reporting.
- Home offsets measure the correction at the actual home encoder value. If that value is nonzero, anchor the polynomial at that reference (or adjust its intercept consistently) rather than assuming the home correction is exactly the value at command zero.
- Determine identifiable parameter combinations and coordinate conventions before interpreting individual fitted base/axis/tool parameters.
- Choose repetition counts and visibility margins, and verify achieved error/time on held-out configurations.

## Verified Evidence
No experiments or validation results were generated when creating this file. Add evidence links and provenance when verified.
