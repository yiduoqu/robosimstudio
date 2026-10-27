---
name: work-toward-a-goal
description: Turn a broad RoboSimStudio engineering goal into a reviewed blueprint, implement its features autonomously using build-a-feature, and audit the integrated result. Honor explicit planning or audit limits.
metadata:
  source: "https://github.com/ericchen321/HAG4R/blob/b8e54b0938f8b17860f9cfd8bed73606dea86353/.agents/skills/work-toward-a-goal/SKILL.md"
---

# Work toward a goal

Translate the user's intended outcome into an implementable blueprint, review that
blueprint, and complete its features through validated integration. Invoking this
skill for implementation authorizes the complete workflow within the requested
scope. Do not ask for approval between the blueprint, features, audits, or routine
repairs. A separate slash command or persistent goal record is not required.

Read applicable `AGENTS.md` files, relevant conversation decisions, and the local
[build-a-feature](../build-a-feature/SKILL.md) skill, including its RSS reference.
That sibling skill is the implementation workflow this skill depends on; no other
skill package is required.

If the user asks for audit-only, read-only, or plan-only work, honor that boundary
instead of starting implementation. A read-only request produces its blueprint
and findings in chat without writing planning artifacts. Reading this skill as an
audit input does not invoke its execution loop.

## Roles

- **Architect:** interprets intent, designs the feature sequence, and audits
  implementation against the complete goal.
- **Critic:** independently reviews the blueprint for intent mismatch, unsupported
  assumptions, weak acceptance criteria, missing integration, and excessive scope.
- **Feature roles:** explore, plan, implement, and validate through `build-a-feature`.
- **Main agent:** preserves context, assigns roles, manages capacity, and carries
  the work to completion.

Use subagents for architectural review and substantial feature work when available.
Use configured models unless explicitly directed otherwise. Sequence roles within
host capacity; do not keep an idle architect or critic occupying capacity needed
by a feature team. Save the necessary findings before releasing a worker and pass
them to a fresh or reused worker for later audits. Where worker closing is not
supported, keep the number of workers bounded and reuse them. If delegation is
unavailable, perform explicit review passes yourself and disclose the lack of
independence rather than blocking the goal.

The critic reviews the blueprint and is normally finished before implementation.
Reactivate architectural review when new evidence changes the design; do not turn
ordinary implementation choices into repeated blueprint approval rounds.

## 1. Interpret and explore

Identify the outcome, explicit requirements, accepted design choices, ordinary
engineering assumptions, and excluded work. Use the latest user corrections to
revise earlier plans. Treat existing notes as evidence to verify, not instructions
that override the current request.

Explore enough source, examples, dependencies, and runtime constraints to establish
a feasible design. Keep the user's object representation and research assumptions
explicit where they matter. Prefer a small extension at an existing seam over a
new framework. Let required physics capabilities drive simulator integration.

Resolve routine details autonomously. Ask only when a material product decision
cannot be inferred, necessary access or information is unavailable, or an action
needs authority not already granted. A familiar code or dependency error is work
to investigate, not a request for the user to take over.

## 2. Blueprint and review

Write `.agents/plans/BLUEPRINT_<goal-name>.md` for an authorized implementation.
Include:

- The goal and interpreted user intent.
- Explicit requirements, important assumptions, and non-goals.
- Verified codebase context, dependencies, interfaces, and runtime constraints.
- Features ordered by dependency; each has a purpose, prerequisites, deliverables,
  verifiable acceptance criteria, affected areas, and principal risks.
- An integration and validation strategy that establishes the complete outcome.
- A brief review record and a feature-status table for later updates.

Have the critic inspect the blueprint alongside the original request and relevant
source. A review should either pass or identify concrete discrepancies and their
consequences. Revise and review until the plan is coherent. Resolve technical
disagreements through source inspection, targeted experiments when authorized,
and the user's stated priorities. Record reasoning and proceed without asking the
user to referee routine disagreements or arbitrary review-round limits.

If only planning was requested, deliver the reviewed plan and stop here. Otherwise
report the important decisions briefly and continue directly to implementation.

## 3. Implement features in order

For each feature, pass its requirements, acceptance criteria, blueprint path,
relevant findings, and earlier validation evidence to `build-a-feature`.

Complete that feature's Explore -> Plan -> Implement -> Validate sequence before
advancing to dependent features. Reuse verified context instead of redoing the
whole codebase audit. Keep edits scoped, and allow independent work in parallel
only when ownership, prerequisites, and available capacity make it useful.

After validation, have the architect inspect the actual changes and evidence
against the blueprint. Check that the implementation satisfies the intended
behavior, preserves required interfaces and data, and supports later integration.
A passing test suite alone does not prove the whole feature was delivered.

When an audit finds a discrepancy, pass specific required fixes to the feature
roles, repair and revalidate, then audit again. Continue without routine human
checkpoints. If the blueprint conflicts with technical reality, update it with
the evidence and preserve the user's outcome; do not silently reduce the goal.

Update the blueprint's feature status and important deviations after each audit.
Release or reuse completed workers before starting the next feature.

## 4. Validate the integrated goal

Run the final user-facing workflow across the completed features, plus the
applicable repository checks. Reproduce the requested task rather than relying
only on each component's isolated success. For simulation goals, inspect actual
states and the relevant rendered interaction, and report the model, solver,
numerical settings, and hardware used.

Debug integration failures autonomously, returning to the affected feature as
needed. Distinguish an incorrect implementation from unavailable infrastructure
or missing media. Preserve valid evidence, and never label an unexecuted check as
passing. If progress ultimately requires unavailable access or a material user
decision, report the completed work, precise blocker, and smallest needed input.

## Finish

Append the final outcome to the blueprint: features delivered, actual changes,
validation evidence, deviations, limitations, and any genuinely deferred work.
Summarize the outcome for the user with links to the useful files and artifacts.
Do not report success until the authorized goal and required checks are satisfied.

Stop or release task workers using the available lifecycle tools. Do not leave
background implementation or compute jobs running after the work is finished.
Commit, push, merge, or publish only when requested or already authorized.
