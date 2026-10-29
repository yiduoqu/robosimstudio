---
name: squeeze-a-bug
description: Diagnose and fix difficult RoboSimStudio bugs through recorded, falsifiable hypotheses, controlled experiments, implementation through build-a-feature, and independent evaluation of runtime evidence. Honor explicit diagnosis-only or read-only scope.
metadata:
  source: "https://github.com/ericchen321/HAG4R/blob/b8e54b0938f8b17860f9cfd8bed73606dea86353/.agents/skills/squeeze-a-bug/SKILL.md"
---

# Squeeze a bug

Carry a difficult bug from reproducible failure to a verified fix through a
hypothesis -> experiment -> implementation -> evidence loop. An invocation to fix
a bug authorizes routine investigation, instrumentation, edits and validation
within that task; proceed without approval checkpoints between rounds. A request
to port, inspect or audit this skill does not invoke debugging. Honor explicit
read-only, diagnosis-only, planning and stopping boundaries.
Diagnosis-only permits investigation, diagnostic runs and their artifacts within
the requested scope, but excludes repair edits. Read-only excludes writes and
experimental execution; return the analysis and proposed tests in chat.

Read applicable `AGENTS.md` files and the local
[build-a-feature](../build-a-feature/SKILL.md) skill, including its
[RSS workflow](../build-a-feature/references/rss-workflow.md). Use that sibling
workflow for implementation, with its plan and checks scaled to the experiment.
No slash command, persistent goal record or additional skill package is required.

## Roles and independence

- **Hypothesizer:** proposes causal explanations, defines distinguishing evidence
  before changes, and evaluates the actual results against those predictions.
- **Implementor:** implements the scoped experiment through `build-a-feature`.
- **Validator:** reproduces the failure and collects evidence, checking the
  original user-facing behavior as well as the experiment's mechanics. Use a
  different worker from the implementor when claiming independent validation.
- **Main agent:** preserves scope, maintains the report, coordinates roles and
  decides what the next experiment must resolve.

For substantial debugging, use subagents to separate hypothesis evaluation from
implementation when available. Use configured models, bounded host capacity and
the lifecycle tools that actually exist. Keep the hypothesizer idle during
implementation, then return the measured evidence for evaluation. Reuse workers
with concise handoffs rather than requiring a fresh team after every round. One
owner edits each file at a time; keep generated experiments isolated.

The main agent may implement a small experiment, or perform the roles sequentially
when delegation is unavailable. Do not claim independent validation in that case.
An implementor's own passing check alone does not establish the root cause.

## 1. Establish the failure and preserve context

Recover the expected behavior, observed failure, reproduction command, relevant
source, logs, recordings, prior attempts and accepted design choices from the
conversation and workspace. Read existing plans, bug reports and relevant branch
notes when present. Resolve routine missing details from source; ask only for
information or authority that cannot be obtained within the authorized scope.

Run the smallest useful baseline when execution is authorized. Record whether the
failure reproduced, its frequency if intermittent, and the metric that identifies
it. Distinguish a code defect from an unsupported model, controller limitation or
incorrect success predicate; do not assume which one it is before testing.

For RSS, trace the relevant path through scene specifications, entity assembly,
Newton model data, solver/backend routing, controls and hooks, state readback and
scoring or rendering. Record actual simulator versions and source revisions,
asset/configuration identity, random seeds, device, timestep and relevant solver
settings. Inspect units, coordinate frames and index mappings where they matter.
Use a verified environment and allocated compute nodes for GPU runs, following
the RSS reference; infrastructure failures are separate from physics failures.

Create `.agents/reports/BUG_FIX_<slug>.md` for authorized debugging and keep a
compact context summary there. Give every worker the current summary, relevant
source and the report path, including previous refuted and untested ideas. For a
read-only audit, keep the findings in chat instead of writing artifacts.

## 2. Record a hypothesis and a distinguishing experiment

Before each new implementation round, record H0, H1, and so on with:

- **Statement and rationale:** a precise causal claim naming the relevant code,
  data flow, timing condition or violated invariant, consistent with the evidence.
- **Predictions:** observations that would support the claim, observations that
  would contradict it, and plausible alternative explanations to distinguish.
- **Experiment:** the smallest instrumentation or intervention that separates
  those explanations. Specify the reproduction, baseline/control, variables held
  fixed and measurements. Prefer one causal change; justify necessary bundles.
- **Feature:** a short name, implementation goal, affected scope, acceptance
  criteria and expected observable effect, passed to `build-a-feature`.
- **Confirmation checklist:** observable, unambiguous conditions for the causal
  claim, separately from the conditions for the user's bug to be fixed.

Use quantitative gates appropriate to the reported failure; do not choose new
thresholds after observing results simply to pass. A runtime exception, failed
grasp or bad frame is a symptom, not a root-cause statement. If prior experiments
changed several variables together, record that confound and narrow the next test.
Describe earlier trials as reconstructed history, never as hypotheses recorded
before those trials occurred.

For simulation, preserve the user's object representation, labels and source
material estimates. Changes to solver settings, contact geometry, controller or
asset must be identified as distinct interventions. An attachment, prescribed
object motion, softened material or changed score cannot silently substitute for
the interaction or outcome the user requested. A diagnostic simplification may
isolate a cause, but the fix must also pass the original scenario.

## 3. Implement and collect evidence

Pass the feature, current hypothesis, baseline, checklist, report and relevant
context to `build-a-feature`. Implement only what is needed to test the claim.
Keep investigation changes reviewable and preserve unrelated workspace edits.
Decide explicitly which instrumentation or correction should remain after testing;
selectively revert only your own obsolete experiment changes.

The validator must establish that the intervention actually took effect and that
the intended execution path ran, then reproduce the original failure scenario.
Save commands, effective configuration, exit status, relevant logs and numerical
results under `outputs/debug/<slug>/<run>/` or an existing task run directory.
Preserve earlier evidence instead of overwriting it with later attempts. Capture
states, time series or media when needed to explain the failure; pass focused
summaries and artifact paths between agents, not unbounded terminal dumps.

For a manipulation failure, distinguish detected contact records from active
contact forces and retained grasp, and test lift, transport, release and stable
placement as applicable. For deformables, inspect finite states and mesh integrity
where relevant. Rendered motion complements numerical evidence; an attractive
frame, successful import or animated robot alone does not prove task success.
Separate failed encoding, missing hardware and incomplete runs from a failed solve.

## 4. Evaluate, refine and continue

Have the hypothesizer evaluate each checklist condition using the actual runtime
evidence and relevant source. Mark missing measurements as unverified and label
inference explicitly. Record one verdict:

- **CONFIRMED:** the predicted causal mechanism is supported by the distinguishing
  experiment, within the hypothesis's exact scope. Finding a real defect does not
  establish that it caused the reported failure without the predicted causal
  evidence. Confirmation does not by itself mean the user's bug is fixed.
- **PARTIAL:** some predictions hold but the explanation or fix is incomplete;
  state exactly what remains unexplained.
- **REFUTED:** a valid experiment contradicts the prediction; record what it rules
  out and what alternative it favors.
- **INCONCLUSIVE:** the test did not execute, the intervention was ineffective,
  evidence is missing, or confounding prevents a causal conclusion.

Track the fix separately as **unresolved**, **fixed** or **unverified**. A bug
persisting after a patch does not alone refute its suspected cause; verify that
the patch tested the proposed mechanism. A single passing run does not establish
a fix for an intermittent failure; repeat matched cases or seeds enough to assess
the original failure mode and state the coverage and remaining uncertainty.

If unresolved, refine the hypothesis or design a better measurement and continue
autonomously. Reuse confirmed facts and explain what changed between rounds. If
experiments stop distinguishing causes, narrow the reproduction, improve
instrumentation or seek an independent review before more parameter changes.
Do not repeat blind retries or turn a fixed count of failed hypotheses into a
mandatory human checkpoint. Ask only for genuinely unavailable access, information
or a material decision; report the precise blocker and supporting evidence.

## 5. Verify the fix and finish the report

Before reporting fixed, run the original user-facing scenario on the final
implementation, satisfy its acceptance conditions and causal checklist, and
complete relevant regression checks from `AGENTS.md`. Confirm that the final
code, assets and configuration match the recorded run. Keep any changed success
predicate honest and justified by the user's intended behavior.

Maintain this compact report structure throughout:

1. **Context and baseline:** requested outcome, observed failure, reproduction,
   environment/source/assets, prior attempts, constraints and acceptance gates.
2. **Each Hn:** claim, rationale, predictions, experiment/control, feature and
   changed files/configuration, checklist results, evidence paths, verdict,
   uncertainties and next step. Record the hypothesis before its implementation.
3. **Final status:** supported cause, implemented fix, original-scenario and
   regression results, actual coverage, remaining limitations and retained or
   removed instrumentation. If unresolved, say so and describe the current blocker.

Give the user concise updates at hypothesis and evidence milestones rather than
requiring them to supervise each round. Link the final report and useful artifacts.
Complete only when the authorized outcome and required checks are satisfied;
otherwise describe the partial result accurately. Stop task workers and compute
jobs you started when finished or explicitly paused; preserve their useful evidence.
Commit, push, merge or publish only when requested or already authorized.
