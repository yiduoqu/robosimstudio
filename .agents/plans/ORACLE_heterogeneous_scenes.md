# ORACLE — heterogeneous scenes mission (approved plan, authoritative)

This file is the **oracle**: the user-approved plan for this mission, reproduced verbatim.
It is the single source of truth for scope, ground rules, deliverables and acceptance.
`BLUEPRINT_heterogeneous_scenes.md`, `PLAN_*.md` and every progress report refer to it.

**Anti-drift rule.** Work follows the oracle. If reality forces a change — a measured
result contradicts an assumption, a step proves impossible, or scope has to move —
the deviation is written into the blueprint's *Deviation log* **before** the work
continues, with the evidence that forced it. Unrecorded drift is a defect.

---

# Final plan: a small, parity-checked physics + claw fix, then the two scenes

## 0. Compute policy (HPC-first)

Anything that takes longer than ~2 minutes runs on Arrhenius, not locally. In practice:

- **HPC by default**: scene runs and smokes beyond a few steps, the corrector's control/parity scenarios, the plain-VBD negative control, throughput measurements, sweeps, the fast test tier, and anything that triggers Warp kernel compilation. Submitted as separate jobs so independent runs overlap (`srun` for interactive iteration, `sbatch` for longer/parallel work), polled with `squeue` and logs — never blocking on a run.
- **Local (WSL) only for the edit loop**: `ruff`, imports/syntax, git, and micro-smokes under ~2 minutes with rendering off. If a local command might exceed that, submit it instead.
- **Sync in one hop**: `hpc/sync_rss.sh` (already written) pushes the checkout; `hpc/rss_env.sh`, `hpc/rss_smoke.sh`, `hpc/run_rss_scene.sbatch` are staged. Only small artifacts (`result.json`, `diagnostics.json`, `quality_substeps.json`, `overview.mp4`) come back.
- Warp's kernel cache lives in the shared `$HOME/.cache/warp`, so compiled kernels are reused across GPU nodes; first-run compilation cost is recorded once and kept out of the runtime numbers.

## 1. Core goal (unchanged)

- **RoboSimStudio + heterogeneous objects (part graphs)**: represent heterogeneous objects, optimise the simulation so they can be simulated reliably.
- **Heterogeneous objects help the controller plan and simulate, then apply zero-shot to the real task.**
- Tied to `Heterogeneous_manipulation.md` (RSS 2027). Real-robot evidence out of scope.

## 2. Ground rules (unchanged, verbatim)

- Use the existing Clawhauser implementation and the current changes in the `hanson/dev` branch as a reference for your modifications.
- Keep pipeline changes to a minimum, the physics should work already, just try to get the new scenes working with the current pipeline.
- LOOK at the output results. Do not write random probes to analyze the scene numerically unless you need to. Have frequent intermediate outputs that you can look at to understand, it will also help me review your progress.
- parallelize work and research, do not block on long running simulations.
- keep code changes clean, concise, and match the existing style. Avoid ai fluff in ALL of your responses (THIS IS IMPORTANT). Be direct and to the point.
- Keep changes small and incremental, add frequent commits with clear messages. DO NOT PUSH. DO NOT add your signature to the commits.
- Commit Scene A before starting Scene B.

## 3. How to develop (unchanged, verbatim)

- RUN CODE VIA HPC ON ARRHENIUS: take a look at the HPC cluster to set up the environment and run simulations. OFFLOAD AS MUCH AS POSSIBLE TO THE HPC.
- `ssh -i ~/.ssh/id_EFP -o IdentitiesOnly=yes -o CertificateFile=~/.ssh/id_efp-login.hpc.arrhenius.naiss.se-cert.pub junhan@efp-login.hpc.arrhenius.naiss.se`
- If you have to run something locally, run via WSL in an interactive session. Make sure to use the same environment as the HPC cluster to avoid discrepancies.
- Use the `build-a-feature` and `work-toward-a-goal` skills; follow the pattern of `.agents/plans/PLAN_*.md`.

## 4. Skills-guided process

- **`work-toward-a-goal`**: write `.agents/plans/BLUEPRINT_heterogeneous_scenes.md` at kickoff — goal, requirements, verified context, features in dependency order with acceptance criteria, integration/validation strategy, review record, feature-status table kept current.
- **`build-a-feature`** for W1 (corrector), W2 (gripper), W4 (quality): `.agents/plans/PLAN_<feature>.md` with scope, invariants, dependency constraints, ordered steps, acceptance criteria, validation commands — updated with status, deviations, evidence.
- **Review**: only web-research subagents are available, so independent evidence is the **parity A/B against the branch's own implementations** plus the control scenarios; the report states that no independent implementer/validator reviewed the port.
- Gates before progressing: W1 validated before any scene work; the baseline example regression (unchanged kinematic path) before Scene A.

## 5. The two real problems, stated small

1. **Support under load.** At 12 iterations the whole-object translation mode of this stiff connected FEM has a 192.88 N residual (29.41 N at 512) — the object cannot balance its own 1.227 N weight against a local contact. Fix = 3-DOF common-translation solve inside the iteration, same objective, source FEM/materials untouched.
2. **Claw authority.** The claw prescribes body poses, so force is unbounded and "maximum force" is meaningless (the branch's H1 traced overcompression to this). Fix = clamped PD wrench on dynamic finger bodies, same pad geometry and trajectory tables.

## 6. What we write (line budget) and what we do not

| # | File | Change | Est. |
|---|---|---|---|
| W1 | `robosimstudio_materials/coarse_translation.py` (new) | translation-only, single-world corrector: hooks `_solve_particle_iteration`; inertia + contact blocks; 3×3 solve; position-only apply; cumulative per-substep budget; deterministic reductions; DAT/self-contact refusal | ~250 |
| W2 | `examples/10_heterogeneous/gripper.py` (edit) | dynamic mode: finger bodies with mass/inertia (not kinematic) driven by a clamped PD wrench (≤ 5 N, ≤ 0.03 N·m, gravity compensation) from the existing pose/velocity tables; `pad_half_width`, `ledge` fields; `mode="kinematic"` stays default | ~80 added |
| W3 | `robosimstudio/scene/entity.py` (edit) | `collide=False` also clears `COLLIDE_PARTICLES` (byte-identical to the branch) | 1 line |
| W4 | `examples/10_heterogeneous/mesh_quality.py` (new) | substep-latched per-part min J, inversion and nonfinite counts; contact-penetration measure | ~60 |
| W5 | `tests/test_coarse_translation.py` (one file, core only) | resting balance, budget clamp, refusal; plus a two-case check that `collide=False` decor generates no particle contacts. Nothing else | ~80 |
| W6 | `clawhauser_on_rod.py`, `clawhauser_stuff_into_bottle.py` (new) | the two scenes | ~450 |

**Not written or ported:** `parallel_gripper.py`, `cord_grasp.py`, `cord_policy.py`, `trajectory.py`, `contact_recording.py`, `contact_determinism.py`, `check_gripper.py`, `reset_pick_place.py`, `benchmark_pick_place.py`, `quality.py`, `material_overrides`/effective damping, multi-world/marker/reset machinery, the proxy FREE-joint fix, the rewritten `clawhauser_pick_place.py`, the two long branch documents.

## 7. Parity requirements

| Component | Must match | Deliberate omissions (documented in the file) | Evidence |
|---|---|---|---|
| W1 | hook point after the base particle solve; inertia block from VBD's inertial target and masses; contact block through Newton's own force/Hessian functions; 3×3 solve; position-only apply; cumulative budget ≤ ¼ of contact gap; deterministic reductions; DAT/self-contact refusal; fixed enablement | object rotation, rigid FREE-assembly block, multi-world loops, `notify_model_changed` revalidation, diagnostic ledgers | resting control (≈1.238 N support vs 1.227 N weight, zero inverted substeps, no energy gain); budget table control (20 mm/5 mm passes, 0.25 mm fails); A/B on one scene with both solvers and identical inputs: COM path, support load, min J, inversions within a stated tolerance |
| W2 | PD law/gains, clamp order (gravity compensation inside the same budget), pad geometry (`hx=0.003`, `hy=pad_half_width`, `hz=0.006`, ledge −0.0068/1.2 mm), mass/inertia formula, velocity semantics | per-world buffers/helpers, reset machinery, ledgers our diagnostics do not read | A/B on one scene with identical waypoints: applied wrench, peak force/torque, finger poses, object outcome |
| W3 | the exact 1-line flag change | — | identical hunk; checked in the single test module |
| W4 | J = det/rest_det per tet, latched min per part, inversion/nonfinite counts per substep | per-world arrays, unused contact-depth helpers | same run compared against the branch module |

## 8. Validate the corrector before scene work (HPC jobs)

- Resting-object control on the real Clawhauser volume: support balance, no inversions, no energy gain.
- Budget pair comparison (20 mm / 5 mm vs 2 mm / 0.25 mm).
- Plain VBD at the same substeps/iterations as the negative control, so the report states what the corrector buys.
- Cost: control steps/s at the qualified settings, separated from diagnostics/rendering.
- The A/B parity run of §7. All of these are submitted jobs, run in parallel where independent.

## 9. Scene A — `clawhauser_on_rod.py`

- **Statics**: base slab, vertical post (r ≈ 10 mm), horizontal rod (r ≈ 6 mm) cantilevered from it, aligned with the loop's hole axis; rod height set from the mesh so the hanging body keeps ≥ 20 mm table clearance.
- **Pose**: plush standing, loop up, hole axis horizontal (rotation about x by 90°), resting on the table from the mesh's own min z.
- **Grasp**: narrow pads (`pad_half_width ≈ 0.006`, `ledge=False`) on the loop's apex cord, hole free; existing apex-patch selection reused; dynamic mode, bounded force.
- **Motion**: our own waypoint path — settle, close, lift so the hole clears the rod, slide the rod's free end through the hole, seat, hold, open, withdraw.
- **Gates**: rod threaded; loop supported through a hold window with body min z ≥ table + 20 mm; zero finger contact after release; no inverted/nonfinite tet-substeps; min J > 0; penetration ≤ 0.5 mm; peak force ≤ 5 N, torque ≤ 0.03 N·m; report support load vs 1.227 N weight, loop stretch, seated depth.
- **Controls**: kinematic claw for contrast; plain VBD for the physics claim.
- **Commit** before Scene B.

## 10. Scene B — `clawhauser_stuff_into_bottle.py`

- **Container**: thin-walled faceted glass cylinder, open top — 24 wall panels (`rss.slab`, 4 mm) + bottom; inner diameter 60 mm (sweep 50/70 later), depth ≈ 90 mm; decor `collide=False`.
- **Motion**: plush flat as the baseline; bounded pads pinch the body, lift, transport over the mouth, descend slowly and hold, then open and withdraw; stop reason recorded (`rim-blocks-pads`, `object-jammed`, `solver-artifact`).
- **Gates/report**: max insertion depth below the rim; inside fraction; whether it stays wedged after release; applied jaw effort (≤ 5 N) vs achieved deformation; penetration ≤ 0.5 mm; no inverted/nonfinite tet-substeps. "Maximum force" answered as bounded effort vs depth.
- **Knobs**: container diameter, jaw cap, friction — each reported with its measured effect.

## 11. Physics limits to report

1. Support under load on a 6 mm rod at the qualified budget (and vs rod radius / friction).
2. Insertion under bounded effort: depth vs jaw cap vs penetration; what stops it.
3. Cost per scene at the qualified settings, separated from diagnostics/rendering.
4. Limits stated from observation: penalty contact vs hard force limit, one global soft friction, and exactly what the corrector does and does not fix (plain-VBD control as evidence).

## 12. Workflow

1. Blueprint + W1 plan files; W1 + W3 + W5; validate W1 on the HPC (controls, budget pair, plain-VBD control, A/B parity); commit.
2. W2 + W4; baseline `clawhauser_pick_place.py` regression on the HPC with the untouched kinematic default; commit.
3. Scene A: statics/pose, settle stills, then the trajectory; micro-smokes locally only if under ~2 minutes, everything else via `hpc/rss_smoke.sh` / `hpc/run_rss_scene.sbatch`; pull artifacts, **watch the video**, iterate; commit.
4. Scene B the same way; commit.
5. Sweeps (rod radius/friction; container diameter/jaw cap; budget pair) as parallel HPC jobs.
6. README section (settings, run recipe, observed limits), `uvx ruff@0.16.1`, the single new test module, fast tier on the HPC.
7. Update `ROBOSIMSTUDIO_PLAN.md` and the blueprint's feature-status table with status, evidence, deviations, limitations; report with links to videos and result files.

## 13. Risks and open choices

- **W1 risk**: the concise corrector may not reach the branch's numbers quickly. Mitigations: control + A/B parity before scene work; bounded scope; documented fallback to the branch's reviewed `vbd_coarse.py` only with your say-so.
- **Open choice**: claw default mode — `kinematic` (existing examples unchanged, recommended) vs `dynamic`.
- **Open choice**: coarse budget pair to start from — 20 mm / 5 mm (recommended) vs 2 mm / 0.25 mm conservative.
- **Open choice**: land the work on `hanson/dev` directly or on a short-lived branch.
