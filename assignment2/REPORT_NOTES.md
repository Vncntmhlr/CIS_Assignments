# Assignment 2 — Report Notes

Report-oriented synthesis of the calibration discussion through 2026-10-25. These notes distinguish proposed methods from verified repository implementation. No experimental accuracy or timing results are established here.

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
A transform ${}^AF_B=[R,p]$ maps coordinates from frame B to frame A:

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

Nominal offsets discussed: $p_{cw}=(500,0,0)^T$ mm and $p_{we}=(100,0,0)^T$ mm.

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

A per-observation initialization is $F_{X,j}\approx F_{ee,j,\mathrm{meas}}[F_{ee,\mathrm{est}}(\boldsymbol\theta_j)]^{-1}$.
Rotation-aware averaging is required when consolidating rigid transforms.

### Joint-coordinate correction convention
- $\theta_{i,\mathrm{cmd}}$: command/encoder coordinate.
- $\theta_{i,\mathrm{est}}$: uncalibrated model's joint-coordinate estimate; initially equal to the command.
- $\theta_{i,\mathrm{meas}}$: joint coordinate inferred from tracker-observed tool motion, when such inference is available.
- $\Delta\theta_i=\theta_{i,\mathrm{meas}}-\theta_{i,\mathrm{est}}$.
- $\theta_{i,\mathrm{cal}}=\theta_{i,\mathrm{est}}+ \widehat{\Delta\theta}_i(\theta_{i,\mathrm{cmd}})$.

The tracker observes a tool pose, not joint angles directly. Isolated sweeps can support geometric joint-coordinate inference; a joint model can also be fitted directly against tool-pose residuals without constructing measured joint angles for every observation. Updating the prediction does not alter the original robot command.

### Linear interpolation and sample spacing
For adjacent commanded coordinates a and b, let $h_i=b-a$ and
$\lambda=(\theta_{i,\mathrm{cmd}}-a)/h_i$:

```math
\widehat{\Delta\theta}_i
=(1-\lambda)\Delta\theta_i(a)+\lambda\Delta\theta_i(b).
```

The horizontal graph axis is commanded coordinate; the vertical axis is measured-minus-estimated correction. Both are in mm for a linear joint and radians for a wrist joint.

If $|d\Delta\theta_i/d\theta_i|\le\omega_{\mathrm{sys},i}$, then, for exact endpoint corrections,

```math
|\Delta\theta_i-\widehat{\Delta\theta}_i|
\le2\omega_{\mathrm{sys},i}h_i\lambda(1-\lambda)
\le\frac{\omega_{\mathrm{sys},i}h_i}{2}.
```

An allocated per-joint interpolation allowance $\varepsilon_i$ gives:

```math
h_{i,\max}=\frac{2\varepsilon_i}{\omega_{\mathrm{sys},i}},\qquad
N_i=\left\lceil
\frac{\theta_{i,\max}-\theta_{i,\min}}{h_{i,\max}}
\right\rceil+1.
```

A pointwise bound also bounds RMS, but a target RMS by itself does not provide a derivative bound. Axis-tilt magnitude and correction-slope bound are distinct quantities.

### Proposed accuracy budget
For wrist spacing, the discussion proposed $\varepsilon_i=0.5\,\texttt{desired_accuracy}/3$, allocating half the orientation target conservatively across three wrist correction functions. This is a tunable planning allocation, not an assignment requirement or a guarantee of total pose accuracy. It assumes the remaining errors can fit the remaining budget.

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

The proposed initializer returns a 1D floating array of length $22+6n_\mathrm{poly}$. For degree two it has 34 entries, shape (34,), not a square matrix. Zero values mean zero additional correction about the initial geometry; separately stored measured home offsets remain applied.

Each axis direction has two independent tilt parameters perpendicular to its nominal direction. Rotation about the direction itself does not change that direction. Joint 1's direction is fixed by the assignment model. A linear joint still has one motion coordinate; its two calibration parameters describe the rail direction, not two additional motions.

The tool correction is a fixed mounting transform:

```math
F_{we,\mathrm{cal}}=F_{we}\operatorname{Exp}(\xi_{we}).
```

Its six parameters describe three rotational and three translational degrees of freedom. They are distinct from wrist-joint coordinate corrections and axis tilts. An entire frame needs three orientation parameters because rotation about one frame axis changes the other two axes. For an SE(3) exponential, the translational twist coordinates and the resulting translation column are related through the exponential map, rather than always being identical.

The suggested polynomial model is:

```math
\Delta\theta_i(\theta_i)=\mathrm{offset}_i+
\sum_{j=1}^{n_\mathrm{poly}}c_{i,j}(\theta_i/\mathrm{scale}_i)^j.
```

Each correction depends on one joint coordinate. Jointly optimizing all coefficients and geometric parameters is a multivariable optimization, but does not make these correction functions multidimensional. Polynomial least squares is curve fitting and need not pass through each sample. The linear-interpolation spacing bound does not automatically bound a polynomial fit's approximation error.

### Pose planning and collection discussed
- Use N_i evenly spaced coordinates for each sweep; hold the other coordinates at the reference.
- Keep `samples_per_joint` as a minimum via $N_i=\max(\texttt{samples_per_joint},N_{i,\mathrm{required}})$.
- This counts distinct planned positions along a joint, not repeated observations at each position.
- Reducing the allocated error reduces the maximum spacing, but integer rounding and the minimum count can hide changes in the final count.
- Apply the orientation allowance only to wrist joints (Python indices 3–5); separately size linear joints (indices 0–2) from a position allowance.
- Visibility rejection can reduce accepted counts and create gaps larger than the planned spacing. Inspect actual retained gaps and coverage.
- Store each commanded six-coordinate vector with its corresponding observed Frame; keep skipped or missing observations aligned.

### Repeated observations
Holding the robot still and rereading reduces random tracker noise. Re-commanding the same vector before reading also resamples the simulator's positioning variation. Under independent, zero-mean noise, averaging N observations reduces standard deviation from sigma to sigma/sqrt(N). It does not eliminate systematic bias. Average positions arithmetically and orientations using a rotation-aware method.

### Verified implementation status
On 2026-10-25, the branch's modular [kinematics.py](src/student/kinematics.py) and [workflow.py](src/student/workflow.py) still contained TODO/NotImplementedError bodies for the principal student calibration functions, including fit and the pose planner. The snippets discussed in chat are proposals or user-supplied working code; they have not been established as implementations in those modules. This check does not establish the implementation status of separate draft notebooks. No source code was modified as part of this notes update.

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
