# PLAN — coarse common-translation corrector (W1, with W3 and W5)

Oracle: `.agents/plans/ORACLE_heterogeneous_scenes.md` §6 (W1/W3/W5), §7 (parity),
§8 (validation). Reference implementation: `.scratch/ref-1a89afe/vbd_coarse.py`
(branch `origin/yiduo/archive-clawhauser-backend-20261027` @ `1a89afe`), written
against `third-party/newton` @ `454fc3e`.

## Scope

A translation-only, single-world corrector that runs inside VBD's particle
iteration and lets the whole object translate against its contacts. Opt-in via
the material backend, so no existing example changes behaviour. Plus the one-line
`collide=False` fix (W3) and one new test module (W5).

## Why it is needed (branch measurement, to be reproduced)

VBD's particle solve cannot move a stiff *connected* FEM along its common mode;
at 12 iterations the whole-object translation residual is 192.88 N (29.41 N at
512) against a 1.227 N weight, so the object sinks or squashes instead of being
supported. The corrector solves that 3-DOF mode directly.

## Invariants

- Hook: after the base particle solve within `_solve_particle_iteration`, once per iteration per substep.
- Blocks: inertia from VBD's own inertial target and per-particle masses; contacts evaluated through Newton's own force/Hessian functions on the existing contact buffers.
- Solve: 3×3 SPD solve per world; position-only apply (no velocity injection).
- Budget: cumulative per substep, ≤ ¼ of the contact gap; state reset each substep.
- Determinism: deterministic reductions on by default.
- Refuses to act when DAT or self-contact is enabled (the formulation does not hold), and when a contact involves a body-attached particle it cannot represent.
- Omissions vs the branch, stated in the file: object rotation, rigid FREE-assembly block, multi-world loops, model-change revalidation, diagnostic ledgers.

## Ordered steps

1. Port kernel + block + apply from the reference, translation-only, single-world. Keep the reference's math and naming so the diff reads as a trim, not a rewrite.
2. Wire it as a material backend through the existing dotted-path seam (`robosimstudio_materials/coarse_translation.py`), exposing the qualified knobs (query gap, correction budget).
3. W3: `collide=False` also clears `COLLIDE_PARTICLES` in `robosimstudio/scene/entity.py`.
4. W5: `tests/test_coarse_translation.py` — resting balance, budget clamp, refusal, and the `collide=False` two-case check.
5. HPC validation per oracle §8: resting control; budget pair (20 mm/5 mm vs 0.25 mm); plain-VBD negative control; cost; A/B parity against the branch implementation on one scene.
6. Commit.

## Acceptance criteria (oracle §7, §8)

- Resting control: support ≈ weight (≈1.238 N vs 1.227 N), zero inverted substeps, no energy gain.
- 20 mm/5 mm passes the control; 0.25 mm budget fails it (as the branch reports).
- Plain VBD at identical substeps/iterations fails the same control.
- A/B parity: COM path, support load, min J and inversions agree within the stated tolerance.
- `uvx ruff@0.16.1` clean; fast tier green on the HPC (W5 included).

## Validation commands

- Local: `uvx ruff@0.16.1 check` / `format --check`, import/syntax only.
- HPC: control/parity scenarios and the fast tier via `hpc/run_rss_scene.sbatch` / `hpc/rss_smoke.sh`.

## Status

- [ ] 1 kernels/blocks ported
- [ ] 2 backend seam wired
- [ ] 3 W3 collide fix
- [ ] 4 W5 test module
- [ ] 5 HPC validation + A/B parity
- [ ] 6 commit

## Evidence

(pending)
