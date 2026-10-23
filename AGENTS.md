# CIS_Assignments

JHU Computer Integrated Surgery I, Fall 2026; one repository for assignments 1–4.

## Sources and organization
- Project: https://github.com/Vncntmhlr/CIS_Assignments
- Official TA/course repository: https://github.com/LCSR-CIIS/CIS_I_Course/tree/main
- Prefer local assignment briefs, READMEs, notebooks, and source files. Fetch upstream resources only when relevant to the task (e.g. a missing reference or requested comparison/update); do not routinely browse, clone, or pull upstream.
- `assignment1/`: pivot calibration. Student work is in `functions/` and `notebooks/assignment1_final.ipynb`; the report is `notebooks/CIS_I_Assignment1_Report_Final.pdf`. `archive_notebooks/` contains historical experiments, not the primary entry point.
- `assignment1/src/` (including `simulation/`), `data_types/`, and `uncertainty_networks/` contain course support code. Preserve these unless explicitly asked to modify them.
- `assignment2/`: current work; brief: `2026_assignment_2.pdf`. `assignment3/` and `assignment4/` are placeholders. Keep each assignment's code and documentation in its own directory.
- `.local_archive/` holds ignored local backups, including a course checkout; consult only as relevant reference, not as the active implementation.

## Development
- Follow the relevant assignment README and brief. Assignment 1 uses Python 3.9+ and `assignment1/requirements.txt`; use an isolated environment.
- Follow existing Python/NumPy conventions and naming. Preserve coordinate-frame meanings, units, array shapes, and numerical assumptions; explain non-obvious mathematics.
- Preserve notebook relative imports: execute Assignment 1's final notebook with `assignment1/notebooks/` as the working directory.
- Validate changes with focused numerical checks and relevant notebooks; for notebook changes, restart the kernel and run all cells when feasible. Account for simulation randomness and report checks run or limitations.
- Avoid unrelated notebook output churn; do not add environments, caches, or generated artifacts unless required for the task.

## Collaboration
- Inspect the working tree before edits, preserve user changes, and keep changes scoped to the request. Do not change other assignments or course simulation code incidentally.
- Explain material assumptions, mathematical changes, and validation results. Ask when requirements are ambiguous enough to affect correctness.
- Do not commit, push, or change GitHub settings unless the user requests it. Keep shared instructions here; `CLAUDE.md` imports this file.

## Report-Oriented Documentation
- Maintain `REPORT_NOTES.md` inside each assignment directory for report-worthy material only: major design decisions, mathematical derivations and key equations, algorithmic approaches, important experimental settings and quantitative results, validation evidence, and meaningful limitations.
- Keep environment/setup checks, dependency/import results, and setup test summaries out of report notes. Put necessary runtime instructions in `SETUP.md`; retain algorithmic validation relevant to the final report.
- Update notes after significant milestones, not every interaction or minor code change. Organize by topic, consolidate superseded material, and avoid repetitive activity logs. Preserve links to relevant code, figures, tables, and test results; distinguish measured evidence from planned work.
- Target approximately 20 pages for the final report, following the general quality and structure of Assignment 1's report while avoiding unnecessary length; official assignment requirements take precedence.
- Focus the report's unit-test discussion on approximately 2–4 high-value tests, especially kinematics tests. Still implement and run all officially required tests; the discussion limit does not reduce testing requirements.
