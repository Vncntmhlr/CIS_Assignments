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

## Branch isolation
- All writes for these custom skills and this collaboration are restricted to `dev/ahmed` in `Vncntmhlr/CIS_Assignments`. Always specify the branch explicitly. Do not write to main, dev/vincent, any other branch, or repository settings; do not merge or force-push. Skill invocations do not authorize expanding this scope.

## Task profiles
- **CIS Working Notes (cis-working-notes):** Development is inactive. Capture requested discussion points, ideas, equations, decisions and open questions in `assignment2/WORKING_NOTES.md`; preserve other content and distinguish proposals from verified evidence. Do not change report notes or execute experiments for capture requests.
- Select the profile from the explicitly invoked skill or requested task. Common Sources and organization, Collaboration, and Report-Oriented Documentation instructions apply to all profiles.
- **CIS Report Notes (cis-report-notes):** capture working notes and draft explanatory report material. The Development section below is inactive for this profile; do not run tests, change implementations, or execute notebooks merely to write notes. Read relevant source as needed to verify explanations. Read assignment2/WORKING_NOTES.md when present and assignment2/REPORT_NOTES.md before updating. Preserve Validation Approach, Final Results, and Conclusion unless the user explicitly requests an evidence-based edit there.
- **CIS Report Stats (cis-report-stats):** apply Development for validation and quantitative analysis. On every invocation, first read `assignment2/WORKING_NOTES.md` and `assignment2/REPORT_NOTES.md`, then relevant code and saved evidence. Use working notes to identify hypotheses and questions, not as proof of measured results. Document the validation method under Validation Approach, verified measurements under Final Results, and evidence-supported interpretation and limitations under Conclusion in assignment2/REPORT_NOTES.md. Distinguish proposed validation from executed tests; a heading called Final Results does not imply unfinished work is complete.
- Use Assignment 1's linked final report as a quality/structure reference for both profiles, not as Assignment 2 evidence. Keep all active outputs in Assignment 2 unless another assignment is explicitly requested.
- A profile changes task scope, not permissions or scientific standards. Preserve source protections and existing user work. A stats invocation is not permission to modify unrelated algorithms or protected course code. For ordinary implementation work outside either skill, Development remains active.

## Development

Applies to CIS Report Stats and ordinary implementation/validation work; inactive for CIS Report Notes and CIS Working Notes.
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
