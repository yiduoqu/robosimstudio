# BLUEPRINT — heterogeneous scenes (Clawhauser on a rod, Clawhauser into a bottle)

**Oracle**: `.agents/plans/ORACLE_heterogeneous_scenes.md` holds the approved plan
verbatim; it is the authority for scope, rules and deliverables. This blueprint
turns it into an ordered, verifiable feature list. Deviations are recorded in the
*Deviation log* below, with the evidence that forced them.

## Goal and interpreted intent

Simulate the existing labelled heterogeneous Clawhauser FEM (parts 0 tail, 1 body,
2 loop) in two new scenes with the current RSS/Newton pipeline — hung on a
horizontal rod through its loop, and stuffed into a glass cylinder too small for
it — and report what the simulation can and cannot do, with videos. Reaching the
scenes needs two small physics/actuation fixes the previous branch already
qualified: a coarse common-translation corrector (the stiff FEM cannot balance its
own weight at 12 iterations) and a force-bounded claw (the current claw prescribes
poses, so contact force is unbounded and unmeasurable). Both are re-implemented
concise and checked for parity against the branch's own code on the same scenes.

## Requirements

1. Two runnable examples: `examples/10_heterogeneous/clawhauser_on_rod.py`, `clawhauser_stuff_into_bottle.py`, same style and CLI as `clawhauser_pick_place.py`, rendering a 30 fps video plus `result.json` / diagnostics.
2. Corrector parity (§7 of the oracle): same hook point, terms, budget and refusal, minus the documented omissions.
3. Claw parity: same PD law, gains, clamp order, pad geometry, mass formula, velocity semantics.
4. `collide=False` must actually drop particle collision.
5. Quality instrument: substep-latched per-part min J, inversions, nonfinite, penetration.
6. One new test module; the existing examples keep working unchanged.
7. Videos + a results README section with the observed limits.

## Assumptions

- Newton pinned at `third-party/newton` `454fc3e` (`hanson/dino-twist`); the branch's `vbd_coarse.py` is written against that commit, so its Newton calls port directly.
- The branch's qualified setting is a 20 mm contact query gap with a 5 mm per-substep correction budget; 0.25 mm budgets fail.
- Clawhauser mass ≈ 1.227 N in this scene; the branch's resting control reports ≈1.238 N support.
- The branch's numbers come from its own runs; our parity A/B reproduces them under our harness rather than trusting the report.

## Non-goals

Everything in oracle §6 under *Not written or ported*: `parallel_gripper`, `cord_grasp`/`cord_policy`, `trajectory`, contact recording/determinism, `check_gripper`, reset/benchmark machinery, material overrides, multi-world/marker/reset work, the rewritten pick-and-place example, the two long branch documents. Also out of scope: real-robot evidence, material fitting, and any change to the source mesh/materials.

## Verified context

| Item | Fact |
|---|---|
| Checkout | `robosimstudio/`, branch `hanson/dev` @ `6d60d7b`; mission branch work happens here, commits local (no push) |
| Reference branch | `origin/yiduo/archive-clawhauser-backend-20261027` @ `1a89afe`, one commit ahead of `hanson/dev`; reference implementation extracted to `.scratch/ref-1a89afe/` |
| Object | one face-connected labelled FEM volume, 1,840 verts / 6,850 tets; part 0 tail, 1 body 110×107×73 mm, 2 loop 45×63×11 mm, hole axis +z, inradius ≈15 mm |
| Materials | E 0.5/5.0 MPa, ν 0.45/0.49, ρ 300/1100 kg/m³, per-tet |
| Existing example | `clawhauser_pick_place.py`, contact-only, kinematic claw, VBD 30 Hz / 20 substeps / 12 iterations, `soft_contact_margin` 2 mm, `contact_ke` 1e5, `contact_kd` 25 |
| Correctors in Newton | VBD has no whole-object translation block; the branch adds one as `SolverVBDCoarse` |
| Branch diagnosis | 192.88 N residual on the translation mode at 12 iterations (29.41 N at 512) — the object cannot support 1.227 N |
| Cluster | Arrhenius `ehpc-dev-2026d08-170`, `gpu` partition; `$WORK/newton-perf/{newton@454fc3e,venv_newton,harness}`; RSS staging + smoke scripts written under `hpc/` |

## Features (dependency order)

### W1 — coarse common-translation corrector (blocks all scene work)
- **Purpose**: make the stiff FEM able to balance its own weight and external load.
- **Deliverable**: `robosimstudio_materials/coarse_translation.py`, opt-in via dotted backend path, wired through the existing material-backend seam only.
- **Prerequisites**: none.
- **Acceptance**: resting control on the real volume — support ≈ weight (≈1.238 N vs 1.227 N), zero inverted substeps, no energy gain; budget pair 20 mm/5 mm passes and 0.25 mm fails; plain VBD at the same substeps/iterations is the negative control; A/B against the branch's `vbd_coarse.py` on one scene agrees on COM path, support load, min J and inversions within a stated tolerance.
- **Plan file**: `PLAN_coarse_translation.md` (includes W3 + W5, which ride with it).
- **Risk**: the concise port may not reach the branch's numbers; mitigation = control + A/B before scene work; fallback to the reviewed `vbd_coarse.py` only with the user's say-so.

### W2 — force-bounded claw (blocks grasp-dependent scene work)
- **Purpose**: contact force becomes a bounded, measurable quantity; kills the overcompression the branch traced to prescribed poses.
- **Deliverable**: `examples/10_heterogeneous/gripper.py` gains `mode="dynamic"` with clamped PD wrench on the existing finger bodies, plus `pad_half_width` and `ledge` fields.
- **Prerequisites**: none (independent of W1).
- **Acceptance**: `mode="kinematic"` default unchanged and the existing example regresses clean on the HPC; dynamic mode applies ≤5 N / ≤0.03 N·m with gravity compensation inside the same budget; A/B against the branch gripper on one scene with identical waypoints agrees on applied wrench, peak force/torque, finger poses and object outcome.
- **Plan file**: `PLAN_gripper_dynamic.md` (includes W4).

### W3 — `collide=False` also clears particle collision
- **Deliverable**: one line in `robosimstudio/scene/entity.py`, byte-identical to the branch.
- **Acceptance**: a `collide=False` decor slab generates no particle contacts (covered in W5's module).

### W4 — mesh quality instrument
- **Deliverable**: `examples/10_heterogeneous/mesh_quality.py` (~60 lines): substep-latched per-part min J, inversion/nonfinite counts, penetration measure.
- **Acceptance**: same definitions as oracle §7; A/B against the branch module on one run.

### W5 — one test module
- **Deliverable**: `tests/test_coarse_translation.py` — resting balance, budget clamp, self-contact/DAT refusal, and the `collide=False` two-case check.
- **Acceptance**: passes in the fast tier (HPC); nothing else added.

### Scene A — `clawhauser_on_rod.py`
- **Deliverable**: scene per oracle §9 — slab, post, horizontal rod through the loop; narrow-pad dynamic grasp at the loop apex; waypoint path settle → close → lift → thread → seat → hold → open → withdraw; video + results.
- **Prerequisites**: W1, W2, W3, W4.
- **Acceptance**: oracle §9 gates (rod threaded; ≥20 mm body clearance through the hold; zero finger contact after release; min J > 0; penetration ≤0.5 mm; peak force ≤5 N, torque ≤0.03 N·m; support load vs 1.227 N; loop stretch; seated depth) plus the kinematic and plain-VBD controls for contrast.
- **Commit** before Scene B starts.

### Scene B — `clawhauser_stuff_into_bottle.py`
- **Deliverable**: scene per oracle §10 — 24-panel faceted glass cylinder, inner Ø60 mm, decor non-colliding; bounded pads pinch the body, transport over the mouth, descend, hold, open, withdraw; stop reason recorded.
- **Prerequisites**: W1, W2, W3, W4; shares the harness Scene A builds.
- **Acceptance**: oracle §10 (max insertion depth, inside fraction, wedged after release?, jaw effort vs deformation, penetration ≤0.5 mm, no inverted/nonfinite substeps); knob sweeps reported with measured effects.

### Docs and reporting
- `examples/10_heterogeneous/README.md` section: settings, run recipe, observed limits.
- `ROBOSIMSTUDIO_PLAN.md` updated with status; final report linking videos and result files.

## Integration and validation strategy

- Corrector and claw are validated in isolation (controls + A/B parity) before any scene consumes them; the untouched kinematic path and `clawhauser_pick_place.py` guard against regressions.
- Every scene run is a submitted HPC job; artifacts are pulled back and looked at (video stills, diagnostics), not inferred from exit codes.
- The complete outcome is validated by: both scene videos showing the intended interaction, their `result.json` gates passing, and the limits section reporting measured numbers with the plain-VBD control as the counterfactual.
- **Independence**: only web-research subagents exist in this environment, so no independent implementer/validator reviews the port. Parity A/B against the branch's own implementations is the substitute evidence; the report says so.

## Review record

- Oracle reviewed and approved by the user (mission approval, 2026-10-27). Blueprint derived from it and self-reviewed against oracle §§5–13 on the same day: feature order follows dependencies; every oracle acceptance item maps to a feature; no oracle deliverable is missing; no unrequested work (parallel gripper, schemes) added.

## Feature-status table

| Feature | Status | Evidence |
|---|---|---|
| W1 corrector | not started | — |
| W2 dynamic claw | not started | — |
| W3 collide fix | not started | — |
| W4 quality instrument | not started | — |
| W5 test module | not started | — |
| Scene A | not started | — |
| Scene B | not started | — |
| Docs/report | not started | — |

## Deviation log

(none yet)
