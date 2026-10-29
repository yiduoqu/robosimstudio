---
name: build-a-feature
description: Implement a scoped RoboSimStudio feature or fix autonomously through code exploration, a concrete plan, implementation, and validation. Honor explicit audit-only or plan-only requests.
metadata:
  source: "https://github.com/ericchen321/HAG4R/blob/b8e54b0938f8b17860f9cfd8bed73606dea86353/.agents/skills/build-a-feature/SKILL.md"
---

# Build a feature

Carry the requested feature through Explore -> Plan -> Implement -> Validate.
An implementation request authorizes that complete sequence: do not stop for
routine approval after exploration, planning, an implementation revision, or a
failed check. A user who explicitly requests only an audit or plan has authorized
only that scope. In a read-only audit, return the findings and plan in chat
without creating files or running mutating commands.

Read the applicable `AGENTS.md` files and [RSS workflow](references/rss-workflow.md)
before making implementation decisions. Existing user decisions and permissions
take precedence over this workflow. Reuse relevant plans and verified findings
from the current conversation rather than restarting exploration.

## Roles and delegation

Separate exploration, planning, implementation, and validation responsibilities.
For substantial changes, use subagents for these roles when the host supports
delegation. An implementor should not be the sole validator of its own work.
For a small change, the main agent may explore, plan, and implement, then obtain
an independent review when it adds meaningful confidence.

Use the configured model unless the user or project instructions specify another;
do not depend on historical model aliases. Keep delegation within available
capacity, allow one owner to edit a file at a time, and stop or release workers
when their phase finishes. Use the host's actual agent lifecycle operations; where
workers cannot be closed, keep their number bounded and reuse them. Pass concise
requirements, evidence, paths, and acceptance criteria between roles. If delegation
is unavailable, perform the phases sequentially and disclose that review was not
independent; do not wait for an unavailable tool.

## 1. Explore

Trace the relevant entry points, data flow, implementation examples, configurations,
and checks. Identify the narrow extension points and any mismatches between the
requested behavior and existing backend capabilities. Verify important facts
against the current source, especially library APIs and experimental physics.

Record the concrete behavior requested, session decisions, relevant files,
dependencies, risks, and any assumptions that affect acceptance. Distinguish
observed behavior from code-based inference. Exploration itself is read-only.

## 2. Plan

For nontrivial implementation work, write
`.agents/plans/PLAN_<feature-name>.md` in the repository root. For a small change,
a concise in-chat plan is enough. If the user requests a plan without permission
to write files, return it in chat instead.

Make the plan executable by another agent. Include:

- The desired behavior and explicit scope, incorporating the latest user steering.
- Existing code to reuse, files to add or change, and public interface invariants.
- Dependency or environment changes, with verified versions or source commits.
- Ordered implementation steps and concrete acceptance criteria.
- Validation commands or scenes, required evidence, and known runtime constraints.
- Material assumptions, compatibility decisions, and deferred work where relevant.

Resolve ordinary engineering choices from context. Explain significant decisions
in a progress update and proceed; the plan is not a new approval checkpoint.

## 3. Implement

Implement the plan with focused changes, preserving unrelated user work. Update
affected configuration, examples, documentation, and meaningful tests as needed.
Use a feature branch before making commits when project instructions require one.

If inspection or execution disproves an assumption, revise the plan, record why,
and continue within the authorized goal. Do not silently weaken acceptance
criteria, remove a requested capability, or substitute a different object model
to obtain a passing result. Escalate only when a revision would change a material
user decision or require authority the task does not provide.

## 4. Validate and repair

First compare the actual changes with the plan and its acceptance criteria. Then
run the relevant checks from the project instructions, including execution of the
affected behavior when it matters. For simulation work, inspect recorded states,
diagnostics, and rendered behavior as appropriate; a clean import or attractive
frame alone does not establish physical correctness.

Have the validator identify failures and send concrete evidence to the implementor.
Fix issues, rerun the affected checks, and continue autonomously until the feature
meets its criteria. Broaden testing only when changes, failures, or unresolved
risks justify it. Do not modify unrelated code merely to silence existing failures.

Separate environment failures, simulation failures, and incomplete media or reports.
An unavailable GPU does not establish that the physics is wrong, and a failed
video encoder does not invalidate recorded numerical results. Repair the failing
component and retain valid evidence. Mark any still-required check as unverified.

Avoid blind retry loops. When attempts stop producing new evidence, change the
diagnostic approach or narrow the reproduction. Ask the user only for genuinely
missing information, access, or a material decision that cannot be inferred;
report the exact blocker and work already completed. Do not use an arbitrary
number of failed attempts as a reason to hand routine debugging back to the user.

## Finish

Update the saved plan with implementation status, important deviations, validation
evidence, and remaining limitations. Report what changed, why, what was checked,
and anything that remains unverified. A feature is complete only when its required
behavior and checks are satisfied; otherwise report partial completion accurately.

Commit, push, publish, or merge only when requested or already authorized. Do not
start unrelated features after completing the requested one. An explicit request
to stop after planning or another phase remains a stopping boundary.
