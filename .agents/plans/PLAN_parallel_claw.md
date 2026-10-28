# PLAN — parallel claw, body-first cup press

**Oracle**: `.agents/plans/ORACLE_heterogeneous_scenes.md` (unchanged). This plan
is the approved rebuild that replaces the PD-finger claw in both scenes.

## Scope

1. Replace the freely rotating PD fingers with the archive's parallel mechanism:
   one level carrier, two prismatic jaw posts, two rectangular pad inserts.
2. Remove the pad ledges; grip is friction only, using the pad-particle contact
   friction the pipeline already has.
3. Cup scene: place the toy by the loop, release, land the open pads on the
   body's top, press the body into a 90 mm bore under a 50 N carrier budget,
   hold, open and withdraw.
4. Rod scene: same claw, same trajectory and gates, must stay green.
5. Keep three final runs and their videos per scene; delete the sweep/test
   outputs.

## Force budgets

| Path | Limit | Owner |
|---|---|---|
| Closing the pads | 5 N per pad, 0.03 N·m | `ParallelGripperSpec.force_limit` |
| Pressing with the carrier | 50 N, 0.08 N·m | `ParallelGripperSpec.push_force_limit` |

The jaws never exceed the closing clamp; the press is applied to the carrier and
is the only path that may reach 50 N. `ParallelGripper` owns both peaks.

## Mechanism

- `ParallelGripperSpec`: `center`, `half_gap`, `closed_half_gap`, `waypoints`,
  `force_limit`, `push_force_limit`, pad and carrier dimensions.
- Carrier: 120 x 30 x 18 mm free body, mass 0.08 kg, level guide on x/y/z
  translation only, gravity compensated, PD at 900 N/m / 140 N·s/m,
  `push_force_limit` clamp.
- Fingers: 10 x 40 x 10 mm posts + 5 x 40 x 16 mm pad inserts, mass 0.02 kg,
  slide on the carrier through native prismatic joints along +/-x, so the pads
  cannot rotate; PD at 900 N/m / 140 N·s/m, `force_limit` clamp.
- Closing aims a fixed interference past the loop's measured width; the clamp
  is the squeeze.
- The carrier's pose is the guide's pose, so the pads stay level through the
  press; the body sees a flat face, not a rotating finger.

## Cup flow

`pick` loop -> `carry` over the bore -> `descend` tail first -> guarded stall at
the rim -> release to 80 mm half-gap while the toy settles -> open pads descend
until a pad-particle contact says they are on the body's top -> close at 60 mm
half-gap -> `press` under the 50 N carrier budget until the body seats or the
press stalls -> hold 0.5 s -> rise -> open.

`press_m` is capped at 40 mm; the force clamp, not the path, bounds the press.
The depth rate (`0.5 mm/s` over 0.2 s) ends the press on a stall. `result.json`
reports `press_m`, `press_travel_m`, `press_contact_frames`,
`peak_push_force_n`, `max_insertion_depth_m`, `max_inside_fraction`, the local
inversion and the stop reason.

## Acceptance

- Rod: three runs with `rod_threaded`, rod support >= 90% of the toy's weight
  over the settled window, 0 inverted tets, `success: true`.
- Cup: three runs with both pad-particle contacts on the body for at least
  0.1 s, `peak_push_force_n` above the 5 N closing clamp, visible insertion and
  deformation, `success: true`; local inversion is reported, not hidden.
- `ruff` clean; the fast test tier unchanged (no library edit); no HPC job
  longer than the 90 minute limit.
- Final videos: `outputs/f1..f3/overview.mp4` (rod) and
  `outputs/bottle/f1-dyn..f3-dyn/overview.mp4` (cup), pulled locally.

## Not in scope

New scenes, bore sweeps in the final, gate changes, library/physics edits,
real-robot evidence, output retention beyond the final runs.
