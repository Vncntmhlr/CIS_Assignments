# CIS I — Assignment 3 Development Instructions

This `AGENTS.md` governs **only `assignment3/`**, the Fall 2026 CIS I assignment **Surface Registration with a Tracked Pointer**. Work on the current assignment only. Do not apply historical implementation, report-version, or teammate-ownership rules from other assignments.

## 1. Sources, scope, and initial setup

- Student project: https://github.com/Vncntmhlr/CIS_Assignments
- Official TA/course repository: https://github.com/LCSR-CIIS/CIS_I_Course/tree/main/assignments/assignment3
- Read the **local** Assignment 3 handout, TA `README.md`, `notebooks/assignment3_template.ipynb`, relevant `src/student/` stubs and `tests/test_student.py` before deciding what to implement. Use the official upstream repository only to obtain the missing Assignment 3 starter or verify an actual discrepancy. Do not routinely pull, re-clone, or overwrite working files.
- Read `assignment3/CIS_I_Assignment3_Classroom_Math_Methods.md` before choosing mathematical methods. It is a source-grounded **selection guide**, not evidence that a method has been implemented or tested. Its labels `[COURSE]`, `[DERIVED]`, and `[ENGINEERING]` must not be conflated.
- The local `assignment3/` directory may initially be empty. When asked to set up Assignment 3, bring in the official TA starter **without replacing existing user or teammate changes**. Preserve upstream provenance and keep implementation, experiments, results, and documentation inside `assignment3/`.
- Follow the TA's documented Python dependencies and working-directory conventions. Keep environment setup notes in `assignment3/SETUP.md` only when useful.

## 2. Work step by step

- Implement **only the step or specific functions requested by the user**. Do not silently complete later steps or replace agreed methods.
- For each step: inspect the actual starter API and tests; state its goal, inputs/outputs, frame convention, the selected classroom method, alternatives/trade-offs, and key assumptions; implement; run focused tests; report results and remaining limitations.
- If a step depends on unfinished modules, use explicitly labeled **synthetic test data or mocks** for isolated verification. Never report mock or planned outcomes as actual simulator results.
- Preserve the notebook as a teaching and validation entry point. The grader invokes `workflow.design_scene()` and `workflow.run_registration(session, mesh, boxes)` **headlessly**; those functions must not depend on notebook globals or a particular practice-session seed.
- Choose simple, course-supported approaches as the baseline. Add complexity only when measured accuracy, robustness, conditioning, or runtime motivates it. Explain any deviation from the mathematical reference.

## 3. Protect TA starter code and interfaces

- Implement student TODOs in `src/student/closest_point.py`, `src/student/icp.py`, `src/student/uncertainty.py`, and `src/student/workflow.py`; fill student-designated notebook TODO cells and explicitly student-TODO test cases when appropriate.
- Treat `src/session.py`, `src/mesh.py`, `src/student/report.py`, original non-TODO tests, provided data, and other TA infrastructure as read-only by default. Do not change the simulator or its APIs.
- Do not rewrite the notebook's original Markdown, equations, example code, evaluation cells, non-TODO code, or layout without explicit permission. Prefer a separate working copy or an `experiments/` script for additional investigations.
- Keep public function signatures, `Session` methods, expected `Frame`/`vct3`/NumPy types, array shapes, `SampleSet`, `IcpResult`, and `RegistrationReport` compatible with the official starter and grader.
- Additional helper code may be written inside authorized student implementations. Put extra offline experiments, benchmarks, and supplementary tests in separate `assignment3/experiments/` files where useful; never shadow TA packages.
- If a necessary fix appears to require changing a protected file or API, explain the specific blocker and ask permission before editing it.
- Inspect the Git working tree and diffs before and after changes. Preserve other people's work. **Do not commit, push, alter branches/remotes, or change GitHub settings unless explicitly requested.**

## 4. Mathematical defaults and frame invariants

- Prefer the classroom methods in `CIS_I_Assignment3_Classroom_Math_Methods.md`: exact closest point on a triangle (plane projection plus face/edge/vertex cases); batched/vectorized exact mesh search; spatially informative and visible patches; Arun/SVD rigid point registration; robust patch-based coarse alignment; point-to-point ICP with **implemented** MAD-based outlier rejection; left-CT Jacobian-based covariance and first-order TRE propagation.
- Optional alternatives (point-to-plane refinement, robust weighted LS, search trees, Monte Carlo uncertainty) require a concrete reason and comparative testing. Do not introduce unexplained advanced methods merely to make the solution look sophisticated.
- **`F_reg` maps Reference → CT:** `c_CT = F_reg * d_ref`. A measured tip is `d_ref = F_ref.inv() * F_ptr * p_tip`. Use explicit names for source and destination frames; check multiplication order, dimensions, millimeters versus radians, and valid rotation matrices (`R.T @ R ≈ I`, `det(R) ≈ +1`).
- The Assignment 3 handout's pointer-placement expression uses an inconsistent/inverse registration convention. Use the official GitHub `Reference → CT` API convention; explicitly note any handout/starter discrepancies rather than silently mixing conventions.
- Patch IDs identify **approximate surface regions**, not exact CT point correspondences. Never give the estimator the simulator's hidden selected points or true transform.
- `reject="mad"` is only the starter's default **parameter value**; the MAD logic is student code, not a provided solver.
- Distinguish point-to-surface residuals/FRE, **predicted** TRE, and **practice-server actual** TRE. Low residual alone does not establish accurate registration. The handout mentions ~1 mm while the TA README/notebook specifies ≤2 mm; retain both as a source discrepancy until clarified.
- The grader has a **10-minute wall-clock limit**; track wall time separately from simulated time (`place_pointer`: 1 simulated second; `sample_marker_bodies`: 0.03 s).
- For uncertainty, state the left/right perturbation convention, error model, Jacobian, and covariance assumptions. Check rank, numerical conditioning, symmetry, and positive semidefiniteness. Do not confuse per-vertex RMS predictions with the mean or percentile of an actual random TRE distribution.
- The provided `IcpResult` lacks the original `d_ref` array; for Step 7, agree on an accurate and API-compatible way to obtain or approximate transformed points before implementing the covariance calculation.

## 5. Tests, simulator access, and numerical evidence

- Run `python -m pytest tests -q` from `assignment3/` as a baseline, then run focused tests after each completed component. Early `NotImplementedError` failures are expected; do not misreport them as regressions or success.
- Cover face/edge/vertex triangle cases, mesh-search exactness, known synthetic rigid transforms, coarse alignment, ICP convergence, injected outliers, missing tracker observations, and covariance rank/PSD where relevant.
- Benchmark closest-point search and ICP using real mesh sizes; do not assume an algorithm is fast merely because it is vectorized.
- Once the full pipeline works, use **multiple independent practice seeds** to compare actual TRE, predicted TRE, residual statistics, outlier detection, visibility, convergence, simulated time, and wall-clock time. Record test settings and numeric results.
- `session.evaluate_tre()` and `session.true_outliers()` are **practice-validation only**. Never call them inside `run_registration()`, during a graded run, or as hidden inputs to the estimator. Use only the documented `Session` interface in the graded workflow.
- Keep the TA's original assertions and evaluation rules unchanged. Complete student-designated TODO tests or add separate tests instead of weakening existing ones.
- Clearly distinguish **PLANNED**, **VERIFIED**, **FAILED**, and **NOT RUN**. Never invent measurements, claim hidden-grader success, or extrapolate a single passing seed into a guarantee.

## 6. Continuous, report-ready documentation

- Maintain **`assignment3/REPORT_NOTES.md` throughout development**. Create it at the first substantive implementation milestone if missing. Update it after each important design choice, functioning algorithm, substantial change, or meaningful experiment—not after every tiny edit.
- Keep `CIS_I_Assignment3_Classroom_Math_Methods.md` as the general mathematical reference; `REPORT_NOTES.md` must describe **methods actually chosen and used, what the code currently does, and what has really been measured**. Replace or label superseded decisions rather than leaving contradictory descriptions.
- For every report-worthy milestone, capture concisely:
  1. Notebook step, function(s), source file(s), and contributor if known.
  2. Goal, input/output data, coordinate frames, and numerical units.
  3. Actual mathematical method, key derivation/equations, assumptions, and justification; relevant rejected alternatives when consequential.
  4. **Verified** course slide numbers and handout/README citations for major formulas; mark lecture-derived adaptations rather than attributing them verbatim to the professor.
  5. Short pseudocode or algorithm/data-flow description, including non-obvious edge cases.
  6. Reproducible validation: tests run, seeds, configurations, numeric results, tables/plots, timing, and paths to relevant figures/results.
  7. Limitations, observed failures, unresolved checks, and changes needed before integration or reporting.
- Organize notes so the eventual report can cover: scene/device design and specifications; mathematical approach; algorithmic approach and call hierarchy; sampling plan; registration and rejection methods; uncertainty/TRE calculations; verification and achieved performance; discussion, limitations, and team contributions.
- **Write for the CIS I TA, not for a specialist research audience.** Prefer terms and notation used in lectures: `rigid registration`, `Arun/SVD`, `least squares`, `closest point`, `ICP`, `surface normal`, `outlier rejection`, `Jacobian`, `covariance`, `FRE`, `TRE`. Define variables and abbreviations, explain what each equation physically does, and introduce unfamiliar technical language only when needed, with a plain-English definition.
- Avoid name-dropping methods not used in the final code; do not write speculative results or overcomplicate the math section. Favor a few correct, well-explained derivations over excessive terminology. Keep report prose clear and direct.
- Keep routine dependency installations, environment details, import troubleshooting, and verbose test logs out of `REPORT_NOTES.md`; put reusable instructions in `SETUP.md`, and link to concise reproducible evidence instead.
- If two teammates work in parallel, agree on ownership and function interfaces first. Preserve each person's code, and document **verified** individual technical contributions without assuming who did what.

## 7. End-of-step handoff

After each requested implementation step, provide a concise handoff stating: files/functions changed; selected mathematical method and reason; relevant tests passed/failed/not run with evidence; numerical and performance results; remaining risks or blockers; and the exact `REPORT_NOTES.md` sections updated. Review the diff before concluding. Do not advance to the next step unless requested.
