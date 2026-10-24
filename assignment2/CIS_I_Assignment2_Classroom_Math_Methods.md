# CIS I — Classroom Mathematical Methods for Assignment 2

**Purpose:** A source-grounded, implementation-oriented reference for Codex / Claude Code choosing mathematical methods for **2026 Assignment 2: Cartesian-robot kinematic calibration**. Prefer methods described in the CIS I lectures, exercises, and Quiz 3 TA notes. This is **not** a completed solution or a claim that an algorithm has been validated on the hidden grader.

**Suggested local path:** `assignment2/CLASSROOM_MATH_METHODS.md` (separate from the evidence-based `assignment2/REPORT_NOTES.md`).  
**Reviewed inputs:** `2026_assignment_2.pdf`; `lecture2_frames.pdf`; `lecture3_rigid3d3dcalculations.pdf`; `lecture3_02_localizers.pdf`; `lecture4_calibration.pdf`; `lecture4_mathreview_leastsquaresreview.pdf`; `lecture4_mathreview_interpolationreview.pdf`; `lecture5_segmentation_-_2023.pdf`; `cis_i_2026_exercise_1.pdf`; `cis_i_2026_exercise_2.pdf`; `quiz 3.pdf` (handwritten TA review).  
**Repo consulted:** [CIS_Assignments](https://github.com/Vncntmhlr/CIS_Assignments), including `AGENTS.md`, `assignment2/README.md`, `SETUP.md`, `REPORT_NOTES.md` and starter `src/student/` layout. Repository branches and local edits can differ; inspect the active working tree before implementation.

**Labels:** **[COURSE]** directly taught / prescribed by a cited class source; **[DERIVED]** algebraic or standard numerical consequence of the course setup; **[ENGINEERING]** recommendation for selecting/testing an implementation, **not** a stated course requirement. Respect the official assignment and TA APIs over this reference.

---

## 0. Read this first: assignment constraints and method selection

### 0.1 The actual task is NOT image-to-patient registration

The task is **calibrate a six-joint Cartesian robot** so the improved forward/inverse kinematic functions predict physical end-effector poses accurately throughout the workspace. Optical marker tracking supplies measurements; registration estimates frames; calibration estimates fixed parameters and smoothly varying joint errors; interpolation helps model the latter. [COURSE: Assignment 2, pp. 1–7]

**Assignment targets** (evaluate with the *assignment's* transform/error ordering):

- **Forward pose:** `||alpha_FwdK|| <= 1e-3 rad`, `||epsilon_FwdK|| <= 0.2 mm`.
- **Inverse-kinematics result:** `||alpha_InvK|| <= 1e-3 rad`, `||epsilon_InvK|| <= 0.2 mm`.
- **Simulated execution time:** maximum **600 s**; time increases when moving robot and sampling tracker. Single tracker sample: `Delta t_samp = 0.03 s`; robot motion also has settling and rate/acceleration constraints.
- One **special calibration marker body** may be attached to the end effector. Its marker positions are *exact* by design, its marker-body frame coincides with the end-effector frame, and its marker positions must satisfy `||a_k|| <= 75 mm`. The special body's definition cannot be recreated after creation.
- Assignment **explicitly recommends Method 1** (identify correct kinematic parameters and joint corrections); **Method 2** (dense map-and-interpolate) is allowed but more likely to exceed the time limit; **Method 3** may combine them. [COURSE: Assignment 2, pp. 5–7]

**Prioritized decision tree:**

| Task observed in code | Prefer | Fallback / extension | Avoid |
|---|---|---|---|
| Recover a rigid pose from corresponding 3-D marker locations | Centered rigid least squares, **SVD / Arun** (Lecture 3, slide 17) | Linearized SE(3) correction (Lecture 3, slides 15–16); weighted LS when noise differs | Unconstrained affine fit as final rotation; reflected `R` |
| Estimate correction to a nearly correct frame | Small-angle **6-D LS update** `[ -sk(q_k) | I ]` | Iterate with exact `Rot(alpha)` increments | Treating `I+sk(alpha)` as an exact rotation |
| Obtain robust frame alignment from multiple robot/tracker poses | Compose transforms consistently; exploit relative pose changes; LS for fixed unknowns | Lecture 4 **AX=XB** calibration when the frame graph fits | Assuming world-to-tracker and world-to-robot are separately identifiable without a reference |
| Identify errors of linear joints 1–3 | Home + excite **one axis at a time**; fit offsets and axis directions via LS | Piecewise-linear fit of remaining 1-D joint distortion | 6-D dense grid before a parametric model |
| Identify wrist axes / angular offsets 4–6 | Excite each revolute joint separately; fit rotation axes/angles; use relative rotations | Small-angle iterative orientation refinement | Subtracting rotation matrices; Euler-angle component averaging |
| Correct smooth encoder/systematic joint errors | **1-D piecewise-linear interpolation**, with spacing chosen from error budget and derivative bounds | Bernstein / B-splines only with proven benefit | High-degree global polynomial without conditioning/extrapolation checks |
| Correct workspace-dependent residual field | Separable / local 2-D or 3-D interpolation of **residuals** | Targeted sparse residual map | Full high-dimensional table without time/memory/error analysis |
| Select sample count/poses | Geometry, observability, noise vs systematic budget, predicted motion time | Pilot study + validation | Unlimited averaging or uniformly sweeping every joint combination |

**Implementation rules** [ENGINEERING, consistent with repo `AGENTS.md`]: inspect active branch and `AGENTS.md`; preserve TA starter, simulator, tests and public APIs; change only authorized student/TODO code; do not commit/push unless asked; document *important validated* design equations, test results and limitations in `assignment2/REPORT_NOTES.md`. **Do not** label this mathematics note as measured experimental evidence.

### 0.2 State variables and non-idealities from the assignment

[COURSE: Assignment 2, pp. 2–5]

- `F*_tracker = F_tracker DeltaF_tracker` (**right** error), but the assignment **sets camera jiggle to zero** for this assignment: `sigma_Ta=sigma_Tp=0`. This does **not** mean the tracker's individual marker measurements are noiseless.
- User-defined marker `a_{m,k}* = a_{m,k} + Delta a_{m,k}`, with bounded fixed manufacturing/placement error; the tracker returns marker coordinates with **random Gaussian** error (`sigma_b = 0.1 mm` typical) plus **spatially varying systematic** error `epsilon_sys^trk(b)`, bounded in size and gradient (`delta_sys^trk <= 0.1 mm`, `omega_sys^trk <= 7.5e-4` in the supplied typical configuration).
- Robot base true pose uses a **left** perturbation: `F*_rob = DeltaF_rob F_rob`. Do not confuse with the tracker convention.
- Nominal end-effector kinematics (write frame composition in the same order as the official TA `FwdKins`):

  `F_ee(theta) = F_c(theta1,theta2,theta3) F_cw F_w(theta4,theta5,theta6) F_we`

  `p_c = theta1 d1 + theta2 d2 + theta3 d3` (nominal `d1,d2,d3 = x,y,z`);  
  `R_w = Rot(n4,theta4) Rot(n5,theta5) Rot(n6,theta6)` (nominal axes `x,y,z`);  
  `F_cw = Frame(I,[500,0,0]^T mm)` and `F_we = Frame(I,[100,0,0]^T mm)`.
- Robot's **actual joints** have random and systematic joint-coordinate errors: `theta*_k = theta_k + Delta theta_{k,rnd} + Delta theta_{k,sys}(theta_k)`. Actual axes are perturbed; `F_we*` has a small additional unknown transform. **HomeRobot** sets physical `theta*_k=0` and returns the *encoder* readings, which gives valuable offset information. The first three and final three joints are kinematically decoupled in the nominal structure.
- Positional error budgets depend on **lever arms**: a small orientation error `alpha` at a distance `L` contributes approximately `||delta p|| <= L ||alpha||`. Thus meeting `0.2 mm` can demand tighter angular calibration than a seemingly small angle suggests.

---

## 1. Lecture 2: coordinate-frame algebra / rotations / first-order pose errors

**Sources:** `lecture2_frames.pdf` (frames, rotation axis/angle, small rotations, kinematic chains, error/sensitivity and covariance sections); `cis_i_2026_exercise_1.pdf`, `cis_i_2026_exercise_2.pdf`.

### 1.1 Frame conventions (MUST check before algebra)

Use explicit type `T_A_B` to mean **maps coordinates in frame B into frame A**:

`p_A = R_A_B p_B + t_A_B`; homogeneous `T_A_B = [[R,t],[0,1]]`.

- **Compose:** `T_A_C = T_A_B T_B_C`. Matrix multiplication is **not commutative**.
- **Invert:** `T_A_B^-1 = [R^T, -R^T t]`.
- **Apply transform to point:** `[R,t] p = Rp+t`, *not* `Rp+tp`.
- **Rotation:** `R^TR=I`, `det R=+1`; inverse `R^-1=R^T`.
- **Cross product / skew:** `sk(a)b = a × b = -sk(b)a` and `sk(a)^T=-sk(a)`.
- **Rotate skew coordinates:** `R sk(a) R^T = sk(Ra)`; equivalently `R^T sk(a)R=sk(R^Ta)`.
- **Rotation composition:** `Rot(a,phi)R = R Rot(R^Ta,phi)` and `R Rot(b,psi)=Rot(Rb,psi)R` (axes must be expressed in the appropriate frame).

**Audit rule:** a product `T_A_B T_C_D` is invalid unless `B=C` (in this convention). Check both **which point** and **which coordinate basis** each translation belongs to.

### 1.2 SO(3) perturbations and exponential update

- Axis/angle: `Rot(n,theta)`, `||n||=1`. Rotation vector `alpha=theta*n` (3×1).
- Small rotation: `DeltaR ~= I+sk(alpha)`; inverse **first-order** `DeltaR^-1 ~= I-sk(alpha)`.
- Products of small errors (e.g. `sk(alpha) epsilon`, `sk(alpha1)sk(alpha2)`) are second order and dropped in a *first-order* derivation.
- **IMPORTANT:** `[I+sk(alpha)]` is a linearization, usually **not orthogonal**. After solving for `alpha`, update with exact `Rot(alpha)` (Rodrigues / rotation exponential), as emphasized in Lecture 3 slides 14, 16 and probabilistic correction slides 37–38.
- Relative orientation discrepancy: `R_err = R_pred^T R_obs` (if using right errors), or `R_obs R_pred^T` (left errors), followed by axis-angle / log extraction. Check the requested transform definition before deciding the order.

### 1.3 Left and right perturbations (Exercise 1/2)

`T* = DeltaT_L T = T DeltaT_R`. They describe the same physical pose but **different 6-D vectors**. For `[alpha;epsilon]` with rotation then translation:

`eta_L ~= Ad_T eta_R` with

```text
Ad_T = [ R       0 ]      (6 x 6)
       [ sk(t)R  R ]
Ad_T^-1 = [ R^T          0 ]
          [ -R^T sk(t)  R^T ]
```

The `Ad` relation comes from `DeltaT_L = T DeltaT_R T^-1`; do not replace left error by the same-valued right error.

**Right-error composition example** (Exercise 1 / Lecture 2 last slides): if `T3=T1 T2`, and `T_i*=T_i DeltaT_i`, then

`DeltaT3 = T2^-1 DeltaT1 T2 DeltaT2`,

`alpha3 ~= R2^T alpha1 + alpha2`,

`epsilon3 ~= -R2^T sk(t2) alpha1 + R2^T epsilon1 + epsilon2`.

Thus `eta3 ~= A1 eta1 + eta2` with

```text
A1 = [ R2^T                 0    ]
     [ -R2^T sk(t2)         R2^T ]
```

This is **not** interchangeable with the all-left convention. [COURSE/DERIVED]

### 1.4 Propagate pose error to a tracked point (Exercise 2 Q1–2)

For a pointer/tool point `b` in the local tool frame and true pose `T*=T DeltaT` (right perturbation), first-order:

`delta p_world ~= [ -R sk(b) | R ] [alpha;epsilon] + R delta b`.

Let `M=[-R sk(b) | R]`. Assuming tool-tip error independent of frame error:

`Cov(delta p_world) ~= M C_eta M^T + R C_b R^T`.

For a **left** pose error (`T*=DeltaT T`), use `q=T b` and `M=[-sk(q) | I]`, plus `R delta b`. **Do not mix these Jacobians.**

### 1.5 Covariance propagation / Gaussian algebra

For `x ~ N(mu,C)` and fixed `A,c`, `y=Ax+c`:

`E[y]=A mu+c`; `Cov(y)=A C A^T`. With independent inputs `eta3=A1 eta1+A2 eta2`:

`C3=A1 C1 A1^T + A2 C2 A2^T`.

If correlated, **include both cross terms** `A1 C12 A2^T + A2 C21 A1^T`. Full uncertainty is not zero merely because a measurement has zero **mean**. A constant `R` has `Cov(R)=0`, but generally `Cov(R epsilon)=R Cov(epsilon)R^T != 0`.

A covariance matrix is symmetric, positive semidefinite, with dimension equal to the random vector (6×6 for pose error). For weighting, use `r^T C^-1 r` (Mahalanobis squared distance) when `C` is positive definite; avoid treating angular radians and positional millimeters as directly comparable without scaling/weighting.

---

## 2. Lecture 3: localizers, point registration, rotation estimation

**Sources:** `lecture3_02_localizers.pdf` (optical tracking and marker visibility), `lecture3_rigid3d3dcalculations.pdf` (slides 3–17, 18–29, 32–38), Lecture 2 three-point pairs.

### 2.1 What data are we fitting?

Given `a_k` in **marker-body** frame and measured `b_k` in **tracker-camera** frame, find `T_tracker_marker=[R,t]` such that

`b_k ~= R a_k + t` by minimizing `sum_k ||R a_k+t-b_k||^2`, subject to `R in SO(3)`.

The identical math also applies to other frame-registration pairs. Marker tracking is *not* interpolation. Camera pose is determined from marker observations **only if sufficient markers are visible with nondegenerate geometry**.

### 2.2 Preferred rigid 3-D/3-D solution: centering + SVD (Arun, slide 17)

1. `a_bar=mean(a_k)`, `b_bar=mean(b_k)`.
2. `a'_k=a_k-a_bar`, `b'_k=b_k-b_bar`; translation eliminated.
3. `H=sum_k (a'_k b'_k^T)` (course's specific **A-to-B** convention).
4. Compute `H = U diag(s) V^T` via SVD; slide 17 gives `R=V U^T` and says verify `det(R)=1`.
5. **[ENGINEERING]** To enforce a proper rotation if `det(VU^T)<0`, use `D=diag(1,1,det(VU^T))`, `R=V D U^T` (the lecture flags the negative-determinant failure but does not spell out this implementation fix).
6. `t=b_bar-R a_bar`; form `Frame(R,t)`; compute residuals in the measured coordinate frame.

Why it works: centering isolates `R`; SVD finds orthogonal directions best aligned under the sum-of-squared Euclidean residuals. Always verify `R^T R≈I`, `det R≈1`, held-out residuals, marker ordering, handedness and units. **Do not** change `H` orientation without also changing the `U,V` reconstruction.

**Observability:** ideally ≥3 **noncollinear**, corresponded 3-D markers; plan a spread-out asymmetric noncoplanar layout for robustness and marker disambiguation. Technically three noncollinear points are sufficient for ideal rigid pose, but a 3-D spread can improve robustness/occlusion. Check centered data singular values / condition number; symmetric, collinear or nearly coincident markers are poor choices. The special calibration body's `75 mm` radius constraint still applies.

### 2.3 Alternative: 6-D iterative small-error registration (slides 15–16)

Given current `T_i` and predictions `q_k=T_i a_k`, measured `b_k`, define residual `r_k=b_k-q_k`. For a **left** correction `T_new=DeltaT T_i`:

`r_k ~= alpha × q_k + epsilon = [ -sk(q_k) | I ] eta`.

Stack `A` (3K×6) and `r` (3K); solve

`eta_hat = argmin_eta ||A eta-r||^2` by numerical least squares/SVD; use **exact** `Rot(alpha_hat)` and `epsilon_hat` to update; recompute; stop on residual and update tolerances. Rank ≈6 required for unique full 6-D update. [COURSE: Lecture 3 15–16; Exercise 2 Q9–12]

**With marker-location error (Exercise 2 Q9):** `r_k ~= A_k eta + R delta a_k`. Consequently, for independent pose and marker-geometry error, `C_b,k ~= A_k C_eta A_k^T+R C_a,k R^T`. If exact marker geometry is assumed, `C_a,k=0`. If a marker has a fixed manufacturing offset across repeated observations, its error is **shared**, not fresh independent noise each sample; do not claim simple `1/N` variance reduction for that term.

### 2.4 LS, pseudo-inverse, weighted LS (Math Review / Exercise 2 Q11–14)

For `A x ≈ b`, minimize `||A x-b||²`. The class presents `x=(A^T A)^-1 A^T b` for full column rank but explicitly recommends **orthogonal/standard-library algorithms**.

- `x=A^+ b` using **SVD pseudo-inverse** (or QR-based least squares) is preferred numerically; do **not** form `(A^T A)^-1` as routine code. It squares the condition number and fails for rank deficiency. [ENGINEERING consequence of Math Review slides 1–10]
- If measurements have known covariance `Sigma`, minimize `(Ax-b)^T Sigma^-1 (Ax-b)`; `x=(A^T Sigma^-1 A)^-1 A^T Sigma^-1 b` when full rank. **Implement with stable solves / whitening**, not literal inverse. [DERIVED from Lecture 2 Gaussian/least-squares material]
- For independent noise covariance `Sigma`, ordinary LS covariance `Cov(x_hat)=A^+ Sigma (A^+)^T`; with `Sigma=sigma² I` and full rank, `Cov(x_hat)=sigma²(A^T A)^-1`. [COURSE: Exercise 2 Q11–14]
- An isotropic `sigma² I` assumption is a **special case**; correlated noise requires cross-covariance blocks. Measurements from the same fluctuating tracker/frame may share systematic errors.

### 2.5 Other course rotation estimators (valid alternatives, not first preference)

- **Rotation-only iterative method:** on *centered* pairs, solve small `DeltaR` via `DeltaR a ≈ b`, update `R_{i+1}=R_i DeltaR` (Lecture 3 slides 12–14).
- **Quaternion/Horn method:** solve a 4×4 eigenvalue problem for a unit quaternion and convert to `R` (slides 18–25). Alternative formulation stacks `M q≈0` and uses the smallest-singular-value right singular vector of `M` subject to `||q||=1` (slides 26–29).
- **Line-pair registration:** construct line directions and line-associated points, estimate alignment using corresponding lines (slides 32–35). Useful only if observation API supplies genuine line geometry; not a replacement for direct known point markers.
- **Similarity transform with scale:** `b≈s R a+t` (slides 30–31). **Do not add an estimated scale** when the robot and tracker already share correct metric units (mm), unless evidence demands it.

---

## 3. Lecture 4 Calibration: parameter estimation, hand–eye relations, robot axes

**Sources:** `lecture4_calibration.pdf` slides 4–6, 7–13 (pointer), 15–33 (robot/parameter models), 34–39 (tracker-to-robot and AX=XB family), 54 onward (workspace mapping). Assignment 2 pp. 3–7.

### 3.1 Prefer a parametric physical model before a dense error map

**[COURSE]** Calibration compares measured behavior with an ideal/reference model and estimates parameters minimizing **residual errors**. The two principal classroom strategies are (i) **parameter estimation** and (ii) **map the space and interpolate residuals**. The **assignment explicitly prefers parameter calibration** to save samples and elapsed time.

Recommended parameter groups:

1. **Global frame alignment:** constant robot-base-to-tracker (or its identifiable composite) transform; obtain from suitable pose correspondences.
2. **Joint home offsets:** compare `HomeRobot()` physical zero with reported joint encoder values.
3. **Cartesian axes 1–3:** fit actual displacement directions, perpendicularity error and per-axis scale/offset/distortion.
4. **Wrist axes 4–6:** estimate relative rotation axes, angle offsets/scales and appropriate wrist/tool constant transform.
5. **Tool/end-effector fixed transform:** if distinguishable by measurements, account for the small `F_we*` correction.
6. **Residual systematic errors:** interpolate a *small number of measured 1-D correction samples* only where parametric fits leave meaningful structure.

**Observability warning [ENGINEERING]:** Not all constant transforms/offsets can be uniquely separated from one another. E.g. with no independent world anchor, `T_tracker_world*T_world_robot` is observable as a composite, but its factors are not independently determined. Likewise, certain global pose/tool offsets can be absorbed into each other unless trajectories excite distinct joint DOFs. Test Jacobian rank and use home/independent geometry to fix gauges.

### 3.2 Linear stages 1–3: direct least squares on position changes

When holding wrist angles fixed and exciting Cartesian axes, an **initial model** for observed end-effector marker origin can be:

`q(theta) ≈ q0 + D (theta_lin-theta0)`, `D=[d1*,d2*,d3*]` transformed into the measured frame.

**[DERIVED from Assignment 2 model + lecture's LS methods]** Fit an intercept and three direction columns using many measured states; regress on *observed* controller states, not commanded states blindly. Axis misalignment appears in columns of `D`; systematic encoder distortion appears as structured residual versus each joint's encoder value. Use independent excitation, return-home checks, symmetric offsets when feasible, then 1-D correction tables. Because the wrist/tool introduces a lever arm, interpret tracker point displacements only after holding/measuring orientation or modeling that lever arm.

For axis `k`, a finite-difference preliminary direction estimate is `d_k ≈ (q(theta+h e_k)-q(theta-h e_k))/(2h)` if both poses are feasible; fit over multiple samples for noise reduction. This is a **recommended identification design**, not a directly specified formula from the assignment.

### 3.3 Wrist joints 4–6: relative rotation measurements

Hold other joints fixed while varying one wrist angle. Use the relative rotations from measured poses, estimate a consistent physical axis and angle scaling/offset; account for conjugation by any constant pre/post transforms. For example `R_a^-1 R_b` represents the motion in the *first pose's coordinates*; changing multiplication order changes the expressed axis. Use the course's `Rot(n,theta)` axis-angle tools, and solve angle/axis fits by LS on rotation vectors for small relative errors, or a nonlinear rotation fit if excursions are large. **Do not apply small-angle approximation to large commanded wrist rotations**—only to their small *corrections*. [COURSE foundation: Lecture 2 rotation composition; Lecture 3 iteration; Assignment 2 kinematics]

### 3.4 Tracker–robot / hand–eye calibration (Lecture 4 slides 34–39)

Multiple poses can lead to an equation involving unknown **constant transforms**, conventionally written `A X = X B`. Form paired **relative motions** so static world/reference frames cancel. Solve the rotation constraint first, then the translation constraint, and validate using independent poses. The lecture discusses this class of frame-calibration problems; specific `A,B` definitions **must be derived from Assignment 2's actual frame graph** and exact marker API. Do not paste a generic hand–eye solver without a verified frame diagram. The robot-mounted special marker has a known rigid correspondence to the end-effector frame, often making a direct pose-fitting method simpler than a full hand–eye solve.

### 3.5 Pointer / pivot calibration: relevant principle, *not* Assignment 2's primary goal

Class pivot calibration (Lecture 4 slides 7–13): fixed physical tip while rotating a tracked tool, with for observation `k`

`R_k b_tip + t_k = b_pivot`.

Stack `[R_k | -I] [b_tip; b_pivot] ≈ -t_k` and solve by LS, with diverse orientations and rank checks. This estimates **tool-tip-to-marker offset**, distinct from calibrating **robot kinematics**. Use it only if an unknown pointer/tool-tip offset actually exists in the active task; in Assignment 2 the special calibration marker frame is designed to coincide with the end-effector frame.

---

## 4. Interpolation: the course's preferred residual-modeling toolkit

**Sources:** `lecture4_mathreview_interpolationreview.pdf` slides 2–18 and 21–33; `quiz 3.pdf` TA review pp. 1–4; `lecture4_calibration.pdf` slide 54 and two-plane calibration 74–78; `lecture5_segmentation_-_2023.pdf` slides 109/112 (related application). **Use primarily for systematic joint correction or residual maps in Assignment 2.**

### 4.1 1-D interpolation and what is being interpolated

Known residuals `r(x0),r(x1)` at known joint encoder positions; for query `x` in `[x0,x1]`, `lambda=(x-x0)/(x1-x0)`:

`r_hat(x)=(1-lambda)r(x0)+lambda r(x1)`.

`x` is the **input joint coordinate**; `r(x)` is a **correction value** (mm for linear stage, rad for rotation stage), *not necessarily a point coordinate*. The same formula works componentwise for vector residuals; beware interpolating rotations directly as matrices. If a joint distortion function is sampled, use `theta*_k ≈ theta_k + r_hat_k(theta_k)` in the corrected forward model, subject to the verified sign convention. To invert, solve this 1-D relation for the encoder command within joint limits (monotonic interpolation / scalar root finding); do not assume simply subtracting `r_hat` evaluated at the desired physical angle is exact.

**Interpolation ≠ regression:** interpolation reproduces the sample residuals exactly, including noise; fitting a smooth function with LS can reduce random-noise overfitting. Average/repeat measurements or fit a simple parametric trend before storing interpolation knots when practical.

### 4.2 Derivative-bound interpolation error and spacing (Quiz 3 TA problems 1 & 3)

For exact endpoint samples, true smooth scalar function `f`, and derivative bound `|f'(x)|<=L` on a cell of length `h`:

`e_det(x)=f(x)-[(1-lambda)f(x0)+lambda f(x1)]`.

Because weights sum to 1, rewrite `e_det=(1-lambda)[f(x)-f(x0)]+lambda[f(x)-f(x1)]`. Mean Value Theorem + triangle inequality yields

`|e_det(x)| <= 2 L h lambda(1-lambda) <= L h/2`.

**[COURSE: quiz 3.pdf pp. 1–3]** Hence, to allocate `tau_interp` maximum deterministic interpolation error to this joint: choose `h <= 2 tau_interp / L`. This is a **sufficient first-derivative bound**, not a numerical guarantee for a different function without a justified `L`; with system error derivative bound `omega_sys`, set `L=omega_sys` only for the specific correction function to which that bound applies.

**Do not conflate** random noise and deterministic interpolation bias: zero-mean noise does **not** imply zero bias. If fixed-value endpoint noise remains, interpolation passes it through.

### 4.3 2-D bilinear, 3-D trilinear and N-linear product weights

For a rectangular cell with normalized `(u,v)`:

```text
w00=(1-u)(1-v)   w10=u(1-v)
w01=(1-u)v       w11=uv
r_hat(u,v)=w00*r00+w10*r10+w01*r01+w11*r11
sum(w)=1
```

For a 3-D cell, corners `i=(i1,i2,i3)` with each `i_r in {0,1}`, define `b_0(t)=1-t`, `b_1(t)=t`, and weight `w_i = product_r b_{i_r}(theta_r)`; `r_hat=sum_{i in {0,1}^3}w_i r_i`. In `n` dimensions there are `2^n` corners and **the identical product formula**: `w_i=product_{r=1}^n theta_r^{i_r}(1-theta_r)^{1-i_r}`. [COURSE: Math Review slides 9–12; TA Quiz 3 page 4]

**Efficiency warning:** `2^n` corner growth makes dense interpolation across all six robot joints unattractive; exploit Cartesian/wrist decoupling, **1-D joint residuals**, and at most low-dimensional sparse residual maps when physically justified.

### 4.4 Error bounds for 2-D / n-D multilinear cells [DERIVED from TA's 1-D bound]

With coordinatewise first-derivative bounds `|partial f / partial x_r| <= L_r`, cell lengths `h_r`, normalized `theta_r`:

`|e_det| <= sum_r 2 L_r h_r theta_r(1-theta_r) <= (1/2) sum_r L_r h_r`.

For a 2-D cell, this is `2 L_x h_x u(1-u)+2 L_y h_y v(1-v)`; at its center, its upper bound equals `(L_x h_x+L_y h_y)/2`. The sum comes from the fact that the multilinear weights sum to one and their marginal weights in any one direction reduce to `1-theta_r`, `theta_r`.

**This is a bound on interpolating the scalar function under the stated derivative assumptions**—it is not automatically a bound on the final robot EE pose. Combine per-joint correction errors via the FK Jacobian and account for tracker/model/tool errors separately.

### 4.5 Random measurement noise and interpolation covariance (Quiz 3 problem 2)

Corner samples: `Y_i=f_i+N_i`, `E[N]=0`, covariance `C_N`. At fixed query, let `w=[w_i]` and `F_hat=w^T Y`:

`E_err = f(query)-F_hat = e_det - w^T N`,

`E[E_err]=e_det`, `Var(E_err)=w^T C_N w`.

If independent and equal variance `sigma²`, `Var(E_err)=sigma² sum_i w_i²`.

For N-linear interpolation, this factors as `sigma² product_r[(1-theta_r)²+theta_r²]`; at the cell center it is `sigma²/2^n`, reflecting reduced **random** variance, **not** systematic interpolation bias. With correlated observations retain cross terms in `w^T C_N w`. For vector-valued residuals, use a block weight matrix `W` and `Cov=W C W^T`. [COURSE 1-D and N-linear premises; DERIVED product/covariance extension]

### 4.6 When NOT to use a simple interpolation formula

- Query outside sampled range: extrapolation errors may dominate; clamp only when physically justified and validate boundary behavior.
- Highly nonlinear/multi-valued joint mapping: verify monotonicity before inversion; use more local samples or physics-based model.
- Noisy calibration knots: fit/average first, and separate variance from systematic bias.
- Missing/occluded corners: cannot simply run full bilinear/trilinear interpolation; choose a supported local alternative or collect data.
- Orientation residuals: interpolate in a consistent rotation-vector/tangent-space convention; recompose with `Rot(alpha)`, not naive entry-wise averaging of rotation matrices.
- High-degree polynomial fitting: the Math Review teaches Bernstein/Bézier/B-splines (slides 21–33) and Lecture 4 illustrates them for image dewarping; they are optional higher-complexity alternatives, **not automatically superior** to bounded 1-D piecewise-linear correction for this assignment.

---

## 5. Proposed Assignment 2 calibration workflow using the course toolkit

**[ENGINEERING] This is a method-selection blueprint, not a claim about current Step 3 completion. Adapt to what is actually implemented locally, and use only permitted TA APIs.**

1. **Geometry / visibility (Step 1–2):** choose tracker pose and one well-spread, identifiable special EE marker layout (`||a_k||<=75 mm`). Simulate representative robot joint poses and check **camera viewing-volume visibility**, marker count, and registration geometry. A nominal robot position that looks reasonable in workspace coordinates may place the tool marker **behind the tracker**; check actual transformed points, not scene appearance alone.
2. **Reference frames (Step 3):** from observed exact EE marker body, estimate `T_tracker_EE` using SVD / registration and recover any **identifiable** constant tracker↔robot relation using several poses. Check conventions with an explicit transform-chain diagram and test a simple commanded pose. Calibration body orientation correspondence must be verified against TA API semantics.
3. **Home and joint-direction data:** invoke homing; measure repeatability; move joints 1–3 separately across feasible range while holding other joints constant; fit `D`, intercept and each 1-D systematic correction. Allocate sample poses according to motion time and expected residual gradient.
4. **Wrist calibration:** excite wrist joints 4–6 separately in safe, observable orientations; estimate axes, angle offsets/scales and constant wrist/tool offsets. Avoid confusing rotation-axis changes from frame conjugation with actual axis misalignment.
5. **Construct `ImprovedFwdKins`:** use corrected stage displacements, calibrated axes, correct constant transforms, and interpolated joint residuals with **exact** rotation matrices. Keep mm/rad units and joint limits explicit.
6. **Construct `ImprovedInvKins`:** exploit nominal Cartesian/wrist decoupling; invert corrected joint mappings (possibly scalar root finding or a bounded iterative correction) and verify `ImprovedFwdKins(ImprovedInvKins(F))≈F` plus physical simulated FK error where allowed.
7. **Targeted residual refinement:** if cross-axis / workspace systematic residuals remain significant, add minimal low-dimensional map/interpolation **after** diagnosing them; compare accuracy benefit against added simulation time and overfitting risk.
8. **Validation:** independent randomized feasible poses across volume and wrist range; evaluate orientation error from `log(R_ref^T R_pred)` and the assignment's prescribed translation error convention. Record max, median, high percentile, worst-case pose, pass/fail and simulated elapsed time. Separate in-sample accuracy from held-out accuracy; test multiple noise seeds without relying on inaccessible ground truth at evaluation time.

### Useful experiment design constraints

- The special EE marker's known exact geometry is a strong calibration reference; **generic marker bodies have unknown fixed geometry perturbations** and may need refinement before being treated as anchors.
- Repeated sampling reduces zero-mean tracking noise as roughly `1/sqrt(N)` **only when samples are independent**; it cannot remove spatial systematic distortion or fixed marker offsets. If temporal/spatial correlations exist, effective sample count is smaller.
- Joint travel/speed/acceleration/settling times matter; estimate simulated motion cost before collecting a grid. The 10-minute limit applies to the grader's program, not merely CPU execution time.
- Design for information: vary the independent axis you need to estimate; use distinct orientations to identify wrist/tool effects; check stacked Jacobian rank and singular spectrum.

---

## 6. Numerical implementation checklist / invariants

1. **Frame sanity:** verify transformation composition on known points; `T*T^-1≈I`, `R^TR≈I`, `det(R)≈+1`; use a named frame convention everywhere.
2. **Skew sanity:** `sk(a)b = cross(a,b)`; `R sk(a) R^T = sk(Ra)` for proper `R`.
3. **Rigid registration:** synthetic known `R,t` + noncollinear points → recover with noise-free tolerance; reject/flag degenerate geometry; the reflected solution is not an SO(3) rotation.
4. **Linearized Jacobians:** finite-difference test `[ -sk(Ta) | I ]` **only for left** perturbations; test `[ -R sk(a) | R ]` **only for right** perturbations. Decrease perturbation magnitude and verify error behaves as second order.
5. **LS / covariance:** check matrix dimensions and rank, residuals and singular values. Prefer `numpy.linalg.lstsq`/SVD/QR; use `solve` on well-conditioned normal equations only when justified; covariance should be symmetric PSD.
6. **Interpolation:** exactly reproduce grid corner values; midpoint linear weights=0.5; 2-D center weights=0.25; all multilinear weights nonnegative inside cell and sum to 1. Check continuity at adjacent knot boundaries.
7. **Random vs systematic:** known injected independent Gaussian noise should match predicted variance over repeated *test* samples; varying bias field should not disappear merely by averaging.
8. **Corrected FK and IK:** check home, each axis separately, limit boundaries, random combined poses, internal FK↔IK roundtrip and comparisons against the simulator's measured/authorized observables.
9. **Evaluation ethics:** never read simulated hidden actual transforms or private simulation state in graded code. Debug-only truth comparisons must be isolated from the exported solution.
10. **Report evidence:** keep a decision/derivation-to-test trail; prioritize ~2–4 high-value algorithmic tests in narrative while implementing/running all required tests; record actual results, units and time—not anticipated outcomes.

### Common math mistakes observed in the class exercises

- Arbitrarily canceling `Q` and `Q^T` across a middle matrix: `Q A Q^T != A` in general; `Q(cI)Q^T=cI` is a special case.
- Confusing transpose (`S^T=-S` for skew) with inverse (`S^-1` need not exist).
- Treating left/right `DeltaF` as the same numerical perturbation.
- Multiplying translation by a point as though `Frame(R,p) a = Ra+pa` (it is `Ra+p`).
- Dropping **first-order** terms instead of only small×small second-order terms.
- Using `R^T sk(a) R = sk(Ra)` (wrong argument; correct `R^T a`).
- Treating a zero-mean Gaussian error as zero **realization** or zero **covariance**.
- Assuming different corner or marker observations are independent when they share a global camera pose bias or physical manufacturing offset.
- Confusing estimation of a marker body's rigid pose (**registration**) with interpolation of spatial/joint correction values (**interpolation**).

---

## 7. Source map / citation guide for code comments and final report

| Source | Highest-value portions for Assignment 2 | Primary reuse |
|---|---|---|
| `2026_assignment_2.pdf` | pp. 2–3 tracker/marker model; pp. 3–5 robot model; pp. 5–7 special EE body, targets, recommended methods/runtime | Binding requirements and constants |
| `lecture2_frames.pdf` | rotation/skew identities; small errors; kinematic chains; tracking error and covariance propagation; final pose-composition covariance slides | Transform calculus, Jacobians, uncertainty |
| `lecture3_02_localizers.pdf` | optical tracker hardware, marker-body localization, optical visibility | What measurements mean physically |
| `lecture3_rigid3d3dcalculations.pdf` | slides 3–4 centering; 12–16 iterative; **17 SVD/Arun**; 18–29 quaternion; 34–38 probabilistic registration | Marker pose solver and refinement |
| `lecture4_mathreview_leastsquaresreview.pdf` | slides 1–10: LS, orthogonal transforms, Householder, SVD | Stable linear systems and conditioning |
| `lecture4_calibration.pdf` | slides 5–6 strategies; 7–13 pivot; 15–33 robot model calibration; **34–39 tracker–robot**; **54 residual map**; 74–78 two-plane bilinear example | Workflow, parameter-vs-map choice |
| `lecture4_mathreview_interpolationreview.pdf` | slides 2–8 linear and uncertain interpolation; **9–12 bilinear/N-linear**; 13–18 barycentric; 21–33 polynomial/B-splines | Joint correction interpolation and error budget |
| `cis_i_2026_exercise_1.pdf` | four left/right perturbation combinations and covariance | Convention-sensitive 6-D error propagation |
| `cis_i_2026_exercise_2.pdf` | **Q9–14** marker Jacobians, pose update, covariance, pseudo-inverse; Q17–19 repeated observations | Marker tracking / LS uncertainty |
| `quiz 3.pdf` (TA notes) | p. 2 deterministic first-derivative error bound; pp. 2–3 correlated Gaussian noise and interval spacing; p. 4 bilinear/N-linear weights | Interpolation correctness and sample spacing |
| `lecture5_segmentation_-_2023.pdf` | slides 109 and 112, linear interpolation on voxel edges | Illustrative application only (not core Assignment 2) |

**Attribution note:** When adapting course equations or using AI-generated implementation, follow the assignment/course disclosure and report requirements; explain the actual implementation choices with source filename + slide/page numbers. This file distinguishes **course-prescribed** formulas from additional **derived** formulas and **engineering** recommendations; do not present the latter as direct TA instructions.

---

## 8. Short instruction to a coding agent (paste into a task prompt)

> Read `assignment2/2026_assignment_2.pdf`, repository `AGENTS.md`, active `assignment2/src/student/` code and this `CLASSROOM_MATH_METHODS.md` before choosing an algorithm. Prefer the professor's **parameter-estimation approach**: SVD rigid marker registration, small-pose iterative LS if needed, independently excited Cartesian/wrist axis calibration, and bounded **1-D interpolation of systematic joint residuals**. Use low-dimensional workspace interpolation only if held-out evidence justifies it. Respect all frame conventions, mm/rad units, 600-second simulated-time budget and 0.2-mm/0.001-rad criteria. Derive the measurement equation and rank/observability conditions before coding; do not leak ground truth or modify non-student TA code. Preserve the current Step 3 progress, document decisions/tests in `REPORT_NOTES.md`, and do not claim success without measured validation.
