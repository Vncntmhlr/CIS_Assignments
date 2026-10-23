# Assignment 2 — Report Notes

Report preparation notes; update after significant milestones and consolidate by topic. Record established decisions and evidence, not interaction logs. Only the starter setup has been validated; no calibration results exist yet.

## Scope and Report Structure
- Official brief: [2026_assignment_2.pdf](2026_assignment_2.pdf).
- Quality and structure reference: [Assignment 1 report](../assignment1/notebooks/CIS_I_Assignment1_Report_Final.pdf).
- Target approximately 20 pages, with length guided by useful content and official requirements.

## Major Design Decisions and Algorithmic Approaches
- Baseline: [official TA starter](https://github.com/LCSR-CIIS/CIS_I_Course/tree/0723d0b6793c3288457b34acfaf2cd872914207e/assignments/assignment2), commit `0723d0b6793c3288457b34acfaf2cd872914207e`; starter modules and notebook are unmodified.
- Reuse the matching TA Assignment 1 support snapshot under [vendor/course/](vendor/course/), isolated from the completed Assignment 1. The saved version lacks APIs needed by Assignment 2; see [provenance](vendor/PROVENANCE.md) and [environment setup](SETUP.md).
- Scene design, calibration model, pose plan, and estimation algorithms remain undecided. Student implementation belongs in [src/student/](src/student/).

## Mathematical Derivations and Key Equations
<!-- Define coordinate frames, units, notation, assumptions, and equations needed to explain the method. -->

## Experimental Settings and Quantitative Results
- Setup baseline: Python 3.13.11; [tested dependency pins](requirements-lock.txt). No calibration experiments, accuracy measurements, or simulated-time comparisons have been performed.
<!-- Record experiment seeds, important parameters, metrics with units, and links to figures and tables when available. -->

## Validation Evidence and High-Value Tests
- [Provided simulation suite](tests/test_simulation.py): **23 passed**; [validation summary](VALIDATION.md).
- Candidate report examples: `test_fwd_kins_home` checks identity rotation and home translation `[600, 0, 0]` mm; `test_inv_kins_round_trip_random` checks 100 seeded joint vectors with `atol=1e-9`; `test_inv_kins_near_wrist_singularity` checks pose reconstruction with `atol=1e-6`. These test the supplied nominal simulator, not a student calibration model.
- [Provided student suite](tests/test_student.py): **11 failures from unimplemented stubs, 3 TODO skips**; [validation summary](VALIDATION.md). This is an expected setup baseline, not successful student validation.
- Initial notebook imports pass in a fresh Assignment 2 Jupyter kernel; see [validation summary](VALIDATION.md). All official testing requirements still apply; keep the final report discussion to approximately 2–4 high-value tests, especially kinematics tests.

## Meaningful Limitations
- Setup checks establish import compatibility and simulator operation only. The scene and student algorithms remain unimplemented, so no claim about calibrated accuracy, identifiability, generalization across seeds, or calibration/test time is supported.
