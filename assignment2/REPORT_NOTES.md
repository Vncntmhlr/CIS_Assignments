# Assignment 2 — Report Notes

Report-oriented synthesis of the calibration discussion through 2026-10-30. These notes synthesize [WORKING_NOTES.md](WORKING_NOTES.md) and the current discussion, distinguishing proposed methods from verified repository implementation. No experimental accuracy or timing results are established here.

## Scope and Report Structure
- Official brief: [2026_assignment_2.pdf](2026_assignment_2.pdf).
- Quality and structure reference: [Assignment 1 report](../assignment1/notebooks/CIS_I_Assignment1_Report_Final.pdf).
- Target approximately 20 pages, avoiding unnecessary length and meeting all official requirements.
- Active collaboration branch: `dev/ahmed`. Preserve course support code and keep Assignment 2 work in this directory.

## Design Decisions

### Proposed calibration approach
Use a parameter-based kinematic calibration: collect commanded joint vectors paired with tracker-observed tool poses, estimate joint-coordinate corrections and geometric corrections, and compare the resulting pose predictions with independent observations. Joint corrections alone do not account for axis-direction errors or tool-mounting errors.

Plan individual-axis sweeps with the other joints held at a reference vector; return to that reference between sweeps. Diverse wrist rotations are needed to constrain wrist axes and the tool offset. A reference command of six zeros is nominal home, not necessarily the encoder vector returned by physical homing.

### Workspace and visibility
`Frame(R, p)` describes orientation and origin position relative to a parent frame. The translation is already expressed in that parent frame.

An illustrative scene discussed was a robot at the workspace origin with identity orientation and a tracker at (900, -1500, 300) mm with rotation Rx(-pi/2). The tracker's local +z direction then points along workspace +y. This is an example placement, not an experimentally validated scene.

For each candidate joint vector, predict all calibration-marker positions in the workspace:

```math
{}^0F_{ee}(\boldsymbol\theta)
={}^0F_{\mathrm{rob}}F_{ee}(\boldsymbol\theta),
\qquad
{}^0p_k={}^0F_{ee}a_k.
```

Require joint limits and tracker visibility of every marker. Nominal visibility is a planning approximation; allow margin for geometric errors.

## Mathematical Derivations and Key Equations

### Frames, coordinates, and units
A transform $`{}^AF_B=[R,p]`$ maps coordinates from frame B to frame A:

```math
{}^Ap=R\,{}^Bp+p,\qquad
[R_a,p_a][R_b,p_b]=[R_aR_b,p_a+R_ap_b].
```


Frames: workspace {0}, tracker {trk}, robot base {rob}, carriage {c}, wrist input {w0}, wrist output {w}, and tool {ee}. Define:

```math
F_X={}^\mathrm{trk}F_\mathrm{rob},\qquad
F_{ee}(\theta_1,\ldots,\theta_6)
=F_c(\theta_1,\theta_2,\theta_3)F_{cw}
 F_w(\theta_4,\theta_5,\theta_6)F_{we}.
```

Joints 1–3 are linear coordinates in mm; joints 4–6 are angles in radians.

```math
F_c=\left[I,\sum_{i=1}^3\theta_i d_i\right],\quad
F_w=[R(n_4,\theta_4)R(n_5,\theta_5)R(n_6,\theta_6),0].
```

For the nominal identity rotation in Fcw:

```math
R_{ee}=R_wR_{we},\qquad
p_{ee}=\sum_{i=1}^3\theta_i d_i+p_{cw}+R_wp_{we}.
```

Nominal offsets discussed: $`p_{cw}=(500,0,0)^T`$ mm and $`p_{we}=(100,0,0)^T`$ mm.

Tracker observations and predictions must be compared in the same frame:

```math
F_{ee,j,\mathrm{pred}}^\mathrm{trk}
=F_XF_{ee}(\boldsymbol\theta_{\mathrm{cmd},j}).
```

Scene placements give the initial relation:

```math
F_{X,\mathrm{approx}}
=({}^0F_\mathrm{trk})^{-1}{}^0F_\mathrm{rob}.
```

A per-observation initialization is $`F_{X,j}\approx F_{ee,j,\mathrm{meas}}[F_{ee,\mathrm{est}}(\boldsymbol\theta_j)]^{-1}`$.
Rotation-aware averaging is required when consolidating rigid transforms.

### Joint-coordinate correction convention
- $`\theta_{i,\mathrm{cmd}}`$: command/encoder coordinate.
- $`\theta_{i,\mathrm{est}}`$: uncalibrated model's joint-coordinate estimate; initially equal to the command.
- $`\theta_{i,\mathrm{meas}}`$: joint coordinate inferred from tracker-observed tool motion, when such inference is available.
- $`\Delta\theta_i=\theta_{i,\mathrm{meas}}-\theta_{i,\mathrm{est}}`$.
- $`\theta_{i,\mathrm{cal}}=\theta_{i,\mathrm{est}}+ \widehat{\Delta\theta}_i(\theta_{i,\mathrm{cmd}})`$.

The tracker observes a tool pose, not joint angles directly. Isolated sweeps can support geometric joint-coordinate inference; a joint model can also be fitted directly against tool-pose residuals without constructing measured joint angles for every observation. Updating the prediction does not alter the original robot command.

### Calibrated coordinates, axes, and transform chain
**Proposed model; not a claim of implemented or validated behavior.** Use i for joint index, j for observation index, and m for polynomial power. With stored offsets o_i, fixed positive normalization scales s_i, and degree n = n_poly:
```math
u_i=\theta_{i,\mathrm{cmd}}/s_i,\qquad
\widehat{\Delta\theta}_i=o_i+\sum_{m=1}^{n}c_{i,m}u_i^m,\qquad
\theta_{i,\mathrm{cal}}=\theta_{i,\mathrm{cmd}}+\widehat{\Delta\theta}_i.
```

Offsets plus distortion correct how far a joint moves or how much it rotates; axis tilts correct its direction. For each nominal axis a_i (d_i for linear joints, n_i for wrist joints), choose two fixed orthonormal vectors b_i,1 and b_i,2 perpendicular to it:
```math
\phi_i=t_{i,1}b_{i,1}+t_{i,2}b_{i,2},\qquad
\hat a_i=\exp([\phi_i]_\times)a_i,\quad i=2,\ldots,6,\qquad
\hat d_1=d_1.
```

The brackets denote the skew matrix of a rotation vector. Rotation about an axis itself leaves its direction unchanged, so only two tilt parameters are needed. A nominal z-axis can use x and y as its perpendicular basis. An entire tool frame needs three orientation parameters because rotation about z rotates its x/y axes.

With R(a,theta) denoting rotation about a unit axis:
```math
F_{c,\mathrm{cal}}(\theta_{1,\mathrm{cmd}},\theta_{2,\mathrm{cmd}},\theta_{3,\mathrm{cmd}})
=\left[I,\sum_{i=1}^3\theta_{i,\mathrm{cal}}\hat d_i\right],
```
```math
F_{w,\mathrm{cal}}(\theta_{4,\mathrm{cmd}},\theta_{5,\mathrm{cmd}},\theta_{6,\mathrm{cmd}})
=\left[R(\hat n_4,\theta_{4,\mathrm{cal}})
R(\hat n_5,\theta_{5,\mathrm{cal}})
R(\hat n_6,\theta_{6,\mathrm{cal}}),0\right],
```
```math
F_{ee,\mathrm{cal}}^\mathrm{rob}(\boldsymbol\theta_{\mathrm{cmd}})
=F_{c,\mathrm{cal}}F_{cw}F_{w,\mathrm{cal}}\hat F_{we},
\qquad
F_{ee,\mathrm{pred}}^\mathrm{trk}(\boldsymbol\theta_{\mathrm{cmd}})
=\hat F_XF_{ee,\mathrm{cal}}^\mathrm{rob}(\boldsymbol\theta_{\mathrm{cmd}}).
```

Commands remain unchanged. Joint distortion affects the tool pose through Fc/Fw; the separate fixed mounting correction applies to Fwe.

### Pose residual convention and fitting objective
**Decision from the latest discussion:** use prediction-inverse times observation, both in tracker coordinates:
```math
\Delta F_j=
\left(F_{ee,j,\mathrm{pred}}^\mathrm{trk}\right)^{-1}
F_{ee,j,\mathrm{meas}}^\mathrm{trk}
=[R_{\mathrm{err},j},p_{\mathrm{err},j}],
\qquad
F_{ee,j,\mathrm{meas}}^\mathrm{trk}
=F_{ee,j,\mathrm{pred}}^\mathrm{trk}\Delta F_j.
```

Let alpha_j be the three-component rotation vector log_SO(3)(R_err,j), and epsilon_j = p_err,j be the three-component translation expressed in the predicted tool frame. This uses a rotation log plus a translation column, not the full SE(3) logarithm:
```math
r_j=
\begin{bmatrix}
\alpha_j/(10^{-3}\ \mathrm{rad})\\
\epsilon_j/(0.2\ \mathrm{mm})
\end{bmatrix},\qquad
r(p)=\begin{bmatrix}r_1\\ \vdots\\ r_N\end{bmatrix},\qquad
\hat p=\underset{p}{\mathrm{argmin}}\ \|r(p)\|^2.
```

Here p is the flat correction-parameter vector, not a transform translation, and N counts observed poses. The denominators balance rad/mm using assignment targets, not assumed noise standard deviations.

Observation-inverse times prediction is the inverse error and is also valid, but the latest residual snippet uses the convention above. Reversing a transform generally rotates as well as negates its translation. The freshly read working notes still show the older order in pipeline step 6; this paragraph supersedes that convention without altering the working-notes input.

### Linear interpolation and sample spacing
For adjacent commanded coordinates a and b, let $`h_i=b-a`$ and
$`\lambda=(\theta_{i,\mathrm{cmd}}-a)/h_i`$:

```math
\widehat{\Delta\theta}_i
=(1-\lambda)\Delta\theta_i(a)+\lambda\Delta\theta_i(b).
```

The horizontal graph axis is commanded coordinate; the vertical axis is measured-minus-estimated correction. Both are in mm for a linear joint and radians for a wrist joint.

If $`|d\Delta\theta_i/d\theta_i|\le\omega_{\mathrm{sys},i}`$, then, for exact endpoint corrections,

```math
|\Delta\theta_i-\widehat{\Delta\theta}_i|
\le2\omega_{\mathrm{sys},i}h_i\lambda(1-\lambda)
\le\frac{\omega_{\mathrm{sys},i}h_i}{2}.
```

An allocated per-joint interpolation allowance $`\varepsilon_i`$ gives:

```math
h_{i,\max}=\frac{2\varepsilon_i}{\omega_{\mathrm{sys},i}},\qquad
N_i=\left\lceil
\frac{\theta_{i,\max}-\theta_{i,\min}}{h_{i,\max}}
\right\rceil+1.
```

A pointwise bound also bounds RMS, but a target RMS by itself does not provide a derivative bound. Axis-tilt magnitude and correction-slope bound are distinct quantities.

### Proposed accuracy budget
For wrist spacing, the discussion proposed $`\varepsilon_i=0.5\,\texttt{desired\_accuracy}/3`$, allocating half the orientation target conservatively across three wrist correction functions. This is a tunable planning allocation, not an assignment requirement or a guarantee of total pose accuracy. It assumes the remaining errors can fit the remaining budget.

Linear joints require a separate position allowance in mm. The analogous proposed allocation is one half of a position target divided among three linear joints. Wrist errors also affect tool position through the tool lever arm; the two budgets alone do not establish a full position-error guarantee.

## Algorithm Implementation

### Parameter vector discussed
The suggested layout in [kinematics.py](src/student/kinematics.py) is:
| Slice | Meaning | Count |
|---|---|---:|
| params[0:6] | Base-transform correction | 6 |
| params[6:16] | Axis tilts for d2, d3, n4, n5, n6 | 10 |
| params[16:22] | Fixed tool-transform correction | 6 |
| params[22:] | Per-joint polynomial coefficients | 6 * n_poly |

The proposed initializer returns a 1D floating array of length $`22+6n_\mathrm{poly}`$. For degree two it has 34 entries, shape (34,), not a square matrix. Zero values mean zero additional correction about the initial geometry; separately stored measured home offsets remain applied.

Each axis direction has two independent tilt parameters perpendicular to its nominal direction. Rotation about the direction itself does not change that direction. Joint 1's direction is fixed by the assignment model. A linear joint still has one motion coordinate; its two calibration parameters describe the rail direction, not two additional motions.

The tool correction is a fixed mounting transform:

```math
F_{we,\mathrm{cal}}=F_{we}\mathrm{Exp}(\xi_{we}).
```

Its six parameters describe three rotational and three translational degrees of freedom. They are distinct from wrist-joint coordinate corrections and axis tilts. An entire frame needs three orientation parameters because rotation about one frame axis changes the other two axes. For an SE(3) exponential, the translational twist coordinates and the resulting translation column are related through the exponential map, rather than always being identical.

The suggested polynomial model is:

```math
\Delta\theta_i(\theta_i)=\mathrm{offset}_i+
\sum_{m=1}^{n_\mathrm{poly}}c_{i,m}(\theta_i/\mathrm{scale}_i)^m.
```

Each correction depends on one joint coordinate. Jointly optimizing all coefficients and geometric parameters is a multivariable optimization, but does not make these correction functions multidimensional. Polynomial least squares is curve fitting and need not pass through each sample. The linear-interpolation spacing bound does not automatically bound a polynomial fit's approximation error.

### Pose planning and collection discussed
- Use N_i evenly spaced coordinates for each sweep; hold the other coordinates at the reference.
- Keep `samples_per_joint` as a minimum via $`N_i=\max(\texttt{samples\_per\_joint},N_{i,\mathrm{required}})`$.
- This counts distinct planned positions along a joint, not repeated observations at each position.
- Reducing the allocated error reduces the maximum spacing, but integer rounding and the minimum count can hide changes in the final count.
- Apply the orientation allowance only to wrist joints (Python indices 3–5); separately size linear joints (indices 0–2) from a position allowance.
- Visibility rejection can reduce accepted counts and create gaps larger than the planned spacing. Inspect actual retained gaps and coverage.
- Store each commanded six-coordinate vector with its corresponding observed Frame; keep skipped or missing observations aligned.

### Repeated observations
Holding the robot still and rereading reduces random tracker noise. Re-commanding the same vector before reading also resamples the simulator's positioning variation. Under independent, zero-mean noise, averaging N observations reduces standard deviation from sigma to sigma/sqrt(N). It does not eliminate systematic bias. Average positions arithmetically and orientations using a rotation-aware method.

### Forward-model and estimation sequence
**Proposed implementation, synthesized from working notes and chat:**
1. Place robot/tracker frames and markers; check nominal visibility.
2. Repeat physical homing to estimate offsets; retain the actual home encoder reference.
3. Plan visible individual-axis sweeps with sufficient travel and wrist rotations; collect aligned command/observed-Frame pairs.
4. In fwd_kins(theta, params), select supplied parameters or self.params. Reshape params[22:] into six coefficient rows, evaluate the polynomials, and add separately stored home offsets.
5. Use params[6:10] for d2/d3 tilts and params[10:16] for n4/n5/n6 tilts. Rotate directions rather than adding tilt slices to theta. Form Fc and Fw using corrected coordinates and axes.
6. Correct Fwe using params[16:22]; compose Fc * Fcw * Fw * corrected_Fwe. Return a robot-base-relative pose. base_frame() separately uses params[0:6]; predict_in_tracker() applies it.
7. Stack residuals and optimize correction parameters while commands and observations remain fixed; inspect identifiability and later evaluate unseen configurations.

For n_poly = 2, joints 1–6 use distortion slices [22:24], [24:26], [26:28], [28:30], [30:32], and [32:34]. The power loop adds a linear, quadratic, etc. term to all six joints simultaneously. Use identical scales, basis choices and coefficient ordering in prediction and reporting.

### Residual Jacobian and local identifiability
**Explanation of the user-supplied central-difference/SVD routine; execution not verified.** Each pose supplies three rotation and three position residual entries. With N poses and P parameters:
```math
J=\frac{\partial r}{\partial p}\in\mathbb R^{6N\times P},\qquad
J_{:,k}\approx\frac{r(p+h_ke_k)-r(p-h_ke_k)}{2h_k},\qquad
h_k=10^{-6}\max(1,|p_k|).
```

Here e_k selects parameter k. Each column changes only that parameter and reevaluates the residuals against unchanged observations. This is sensitivity analysis, not another measurement or a parameter fit.

For J = U Sigma V^T, the routine uses tolerance tau = 10^-6 * sigma_max, rank = count(sigma > tau), and nullity = P - rank. Large singular values indicate sensitive parameter combinations; small values indicate weak sensitivity. Rows Vt[rank:] give poorly determined combinations in parameter-vector order, not necessarily individual unobservable parameters. A full right-singular basis when 6N < P retains nullspace directions missing from a reduced decomposition.

Example dimensions only: 10 poses and 34 parameters yield shape (60,34). A hypothetical rank of 30 implies four weak/null combinations; this is not a measured result. Rank is local and depends on units, scaling, finite-difference accuracy and threshold. Full rank alone does not guarantee reliable estimation.

### Verified implementation status
On 2026-10-25, the branch's modular [kinematics.py](src/student/kinematics.py) and [workflow.py](src/student/workflow.py) still contained TODO/NotImplementedError bodies for the principal student calibration functions, including fit and the pose planner. The snippets discussed in chat are proposals or user-supplied working code; they have not been established as implementations in those modules. This check does not establish the implementation status of separate draft notebooks. A fresh read on 2026-10-30 still showed TODO bodies for fwd_kins, residuals, fit and jacobian_rank_report in the branch's kinematics.py. The residual and Jacobian snippets discussed in chat are not verified as repository implementations. No source code was modified as part of this notes update.

## Experimental Settings and Quantitative Results
No calibration run, seed, measured RMS error, or simulated-time result was verified in this discussion. The scene coordinates and error-budget fractions above are illustrative/planned settings, not experimental findings.

## Algorithmic Validation
Planned evidence to collect:
1. Zero-additional-correction forward kinematics agrees with the expected nominal model, accounting for any separately supplied home offsets.
2. Synthetic known-parameter data tests whether fitting recovers observable corrections and predicts unseen joint combinations.
3. Forward/inverse consistency and held-out position/orientation errors across representative configurations and seeds.
4. Residual-Jacobian singular values identify unobservable or weakly determined parameter combinations.

Report position errors in mm and orientation errors in radians separately. Record noise assumptions, tolerances, sampling coverage, and simulated time. Use hidden simulator truth only for debugging/validation, not calibration inputs. Focus final-report discussion on approximately 2–4 high-value tests while meeting all official test requirements.

## Meaningful Limitations
- Axis sweeps excite parameters but do not by themselves prove identifiability; inspect the fitting Jacobian and fix coordinate conventions/gauge freedoms.
- Repetition improves random precision, whereas distinct poses improve coverage; neither removes systematic tracker bias.
- Joint-interpolation bounds assume bounded correction slope and exact endpoint values. Noisy inferred corrections require an additional uncertainty budget.
- More samples cannot alone guarantee the target RMS: geometry estimation, model mismatch, marker layout, tracker systematic error, and pose visibility also matter.
- Sampling bounds must match the correction representation actually implemented. A per-joint model cannot represent arbitrary dependence on all six joint coordinates.

- Home-offset anchoring remains unresolved: physical homing anchors the correction at the actual home encoder coordinate, which need not be zero. Center the polynomial or adjust its intercept consistently before treating the offsets as exact anchors.
