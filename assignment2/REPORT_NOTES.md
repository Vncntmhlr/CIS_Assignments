# Assignment 2 — Report Notes

Maintain concise, topic-organized material for the final report. Update after significant algorithm or experiment milestones; preserve links to relevant code, equations, figures, tables, and test results. Current results concern nominal scene design; calibrated accuracy has not been evaluated.

## Scope and Report Structure
- Official brief: [2026_assignment_2.pdf](2026_assignment_2.pdf).
- Quality and structure reference: [Assignment 1 report](../assignment1/notebooks/CIS_I_Assignment1_Report_Final.pdf).
- Target approximately 20 pages, avoiding unnecessary length and meeting all official requirements.

## Design Decisions
- Tracker frame coincides with the workspace frame and looks along `+z`. The robot base has identity orientation and translation `(-800, 0, 1300)` mm, placing the center of its wrist-center travel box at `(0, 0, 1600)` mm in the camera volume. The earlier positive-x, negative-z placement put the robot behind the camera.
- Retain the four-marker tetrahedron `(s,s,s)`, `(s,-s,-s)`, `(-s,s,-s)`, `(-s,-s,s)`, with `s=37.5` mm. Each marker is `sqrt(3)*s = 64.952` mm from the EE origin, within the 75 mm limit.
- Design parameters and visibility sweep: [notebook, Steps 1–2.1](notebooks/assignment2_template.ipynb). Nominal geometry visualization: [function](src/student/visualization.py), [figure](figures/workspace_design.png). The function uses no hidden truth and does not move the robot or spend simulated time.

## Mathematical Derivations and Key Equations
- In tracker coordinates, the nominal wrist center spans `x,y in [-300,300]` and `z in [1300,1900]` mm. This includes the nominal 500 mm base-to-wrist offset.
- The wrist-to-EE translation has length 100 mm. For any wrist rotation, every marker is within `rho = 100 + 64.952 = 164.952` mm of the wrist center by the triangle inequality. Hence a conservative marker envelope is `x,y in [-464.952,464.952]`, `z in [1135.048,2064.952]` mm. This is a bound, not the exact reachable volume.
- Camera half-width is `h(z) = 500 + (z-1000)*500/1200` mm for `1000 <= z <= 2200`. At the envelope's nearest depth, `h=556.270` mm exceeds its maximum lateral magnitude by approximately 91.3 mm. Near/far depth margins are approximately 135.0 mm. Thus the entire nominal envelope is inside the frustum, including wrist rotations between sampled poses.

## Algorithm Implementation
<!-- Explain implemented calibration, registration, forward/inverse kinematics, and estimation methods; link to relevant code in src/student/. -->

## Experimental Settings and Quantitative Results
<!-- Record experiment seeds, pose/sample settings, accuracy and simulated-time metrics with units, and links to figures and tables. -->

## Algorithmic Validation
- Scene geometry: the notebook's 81 corner/edge poses all keep every marker visible. A separate sweep of 3 values for each linear joint and 5 independently chosen values across each wrist joint's limits also passes: `3^3 * 5^3 = 3375/3375` nominal poses. No random robot realization or hidden truth was used to choose the placement.
<!-- Add validation of implemented calibration/kinematics algorithms as they become available. Focus the final discussion on approximately 2–4 high-value tests while meeting all official requirements. -->

## Meaningful Limitations
- Coverage bounds and plots use nominal kinematics and placement. They do not establish accuracy or coverage under the simulator's unknown base, axis, tool, and joint errors, nor do they establish a feasible calibration-time budget. The 3375-pose sweep is a geometry check, not a calibration acquisition plan.
