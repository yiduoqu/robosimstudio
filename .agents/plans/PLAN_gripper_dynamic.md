# PLAN — force-bounded claw (W2, with W4)

Oracle: `.agents/plans/ORACLE_heterogeneous_scenes.md` §6 (W2/W4), §7 (parity).
Reference: `git show 1a89afe:examples/10_heterogeneous/gripper.py`, plus
`quality.py` for the definitions W4 has to match.

## Scope

Give the existing task-local claw a `mode="dynamic"` in which the finger bodies
are dynamic and driven by a clamped PD wrench, so contact force is bounded and
reportable. Keep the existing kinematic path as the default. Add `pad_half_width`
and `ledge` so the narrow cord pinch the branch qualified is expressible. W4 adds
the substep quality instrument the scene gates need.

## Why it is needed

The claw currently writes `body_q`/`body_qd` for its fingers — effectively
infinite authority. Overcompression, jaw effort and "maximum force" then have no
physical meaning, which is exactly the defect the branch traced (H1). A clamped
PD wrench on dynamic fingers makes the jaw effort a number the scene can report
and bound.

## Invariants (oracle §7)

- Same PD law and gains as the reference; clamp order: force clamp first, gravity compensation inside the same budget.
- Same pad geometry formula (`hx=0.003`, `hy=pad_half_width`, `hz=0.006`, ledge −0.0068 / 1.2 mm) and mass/inertia formula.
- Same trajectory sampling and velocity semantics as the existing gripper.
- `mode="kinematic"` remains the default and the existing example is untouched behaviourally.
- Omissions vs the branch, stated in the file: per-world buffers/helpers, reset machinery, ledgers our diagnostics do not read.

## Ordered steps

1. Add the dynamic mode to `examples/10_heterogeneous/gripper.py` (~80 lines added; existing tables reused).
2. Add `pad_half_width` and `ledge` fields.
3. W4: `examples/10_heterogeneous/mesh_quality.py` — per-part min J, inversion/nonfinite counts latched every substep, contact penetration.
4. HPC: baseline `clawhauser_pick_place.py` regression with the untouched kinematic default; then the claw A/B against the branch gripper on one scene with identical waypoints.
5. Commit (with W4).

## Acceptance criteria

- Existing kinematic example regresses clean: same gates as its recorded baseline.
- Dynamic mode reports peak wrench ≤ the configured caps; jaw effort trace recorded.
- A/B vs the branch gripper: applied wrench, peak force/torque, finger poses and object outcome agree within the stated tolerance.
- W4 numbers match the branch module's definitions on the same run.

## Validation commands

- Local: `uvx ruff@0.16.1 check` / `format --check`; baseline scene build only if under ~2 minutes.
- HPC: baseline regression, claw A/B, and the scene runs that consume both.

## Status

- [ ] 1 dynamic mode
- [ ] 2 pad fields
- [ ] 3 W4 instrument
- [ ] 4 baseline regression + claw A/B on HPC
- [ ] 5 commit

## Evidence

(pending)
