---
name: cis-report-stats
description: Read AGENTS.md, WORKING_NOTES.md and REPORT_NOTES.md on dev/ahmed; apply Development to requested validation and quantitative analysis; write Validation Approach, Final Results and Conclusion. Use for validation planning, saved-result statistics, test execution and evidence-based conclusions.
---

# CIS Report Stats

## Branch and context

Use only `Vncntmhlr/CIS_Assignments`, branch `dev/ahmed` (exact lowercase spelling). Always supply this ref/branch explicitly. Never write to main, dev/vincent, another branch, or change repository settings. Do not merge, create branches, or force-push. A request to target another branch conflicts with this skill's scope: explain the restriction instead of silently switching.

Discover the GitHub connector tools. Read fresh root `AGENTS.md`, applicable nested instructions, `assignment2/WORKING_NOTES.md`, and `assignment2/REPORT_NOTES.md` from dev/ahmed before work. Do not substitute memories for current files. If a required file cannot be read, report the problem; do not pretend it was read. Only the working-notes capture skill may create WORKING_NOTES.md if confirmed absent.

Apply the selected task profile plus common source, collaboration, and report rules. Higher-priority instructions and explicit user constraints govern. AGENTS.md is guidance, not an executable agent. Use Assignment 1's linked final report as a quality/structure reference when useful, never as evidence of Assignment 2 results. Do not edit Assignment 1.

## Validation workflow

Read `assignment2/WORKING_NOTES.md` on EVERY invocation before planning validation, analyzing data, or drafting report sections. Extract relevant hypotheses, decisions, assumptions and open questions. Treat captured notes as context, not proof of execution or correctness. Verify claims against current code and recorded outputs.

Apply Development. Read the active assignment brief, README, implementation and tests as needed. Use the active assignment's environment and working directory; Assignment 1 runtime guidance is not automatically applicable to Assignment 2. For a plan request, describe pending tests without executing them. For saved-result analysis, inspect provenance and calculate appropriate statistics without unnecessary reruns. Execute focused relevant checks when requested or necessary for authorized validation work and tools permit. Do not edit protected code, weaken tests, or change algorithms to obtain favorable outcomes; report failures and scope fixes separately.

Record provenance (branch/commit, data, commands, seeds, sample counts, units and tolerances). Separate training from held-out evaluation, deterministic physical accuracy from noisy tracker error and numerical consistency, and simulated time from wall-clock time. Count missing/failed attempts explicitly; state statistical denominators. Never fabricate results when execution is unavailable. A finite sample does not establish continuous-workspace guarantees.

## Report destinations

Write only the requested sections of `assignment2/REPORT_NOTES.md`, defaulting to these exact level-two headings:
- `## Validation Approach`: objectives, methods, metrics, thresholds, sampling, assumptions and test links. Label planned versus executed tests.
- `## Final Results`: verified statistics and failures with evidence links. Label preliminary/incomplete coverage; write pending if no verified results exist.
- `## Conclusion`: interpretation against goals, limitations and remaining work supported by those results.

Preserve other sections and working notes. Consolidate relevant existing validation/results material when adding the requested headings, preserving provenance and avoiding contradictions. Focus written test discussion on 2–4 high-value checks without reducing official testing requirements. A Final Results heading does not certify completion.

## Save and verify

An explicit request to capture or update notes authorizes a documentation commit to dev/ahmed. Load-only, explanation-only, or preview-only requests do not authorize writes. Preserve all unrelated content and honor named-section requests. Do not infer permission for code changes from a notes request.

Read the current file and blob SHA before updating through GitHub with an explicit branch. Review the diff; on concurrent changes refetch and reconcile without losing other work. Read back the saved file, then return its GitHub link and summarize additions. If saving fails, report that nothing was saved and show the draft. Never claim unseen tests passed or that future chats/background processes will automatically run this skill.
