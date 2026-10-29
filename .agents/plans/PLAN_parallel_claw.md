# PLAN — parallel claw, body-first cup press

**Oracle**: `.agents/plans/ORACLE_heterogeneous_scenes.md` (unchanged). This plan
is the approved rebuild that replaces the PD-finger claw in both scenes.

## Scope

1. Replace the freely rotating PD fingers with the archive's parallel mechanism:
   one level carrier, two prismatic jaw posts, two small pad inserts.
2. Remove the pad ledges; grip is friction only. Raise the cup's global soft
   friction to 2.0 because the 12 mm pads slip on the loop at 0.9.
3. Cup scene: place the toy by the loop, release, let it settle under gravity,
   land the open pads on the body's top, press the body under a 50 N carrier
   budget, hold, open and withdraw.
4. Rod scene: same claw, same trajectory and gates, must stay green.
5. Keep three final runs and their videos per scene; delete the sweep/test
   outputs.

## Force budgets

| Path | Limit | Owner |
|---|---|---|
| Closing the pads | 5 N per pad, 0.03 N·m | `ParallelGripperSpec.force_limit` |
| Pressing with the carrier | 50 N, 2 N·m | `ParallelGripperSpec.push_force_limit` |

The jaws never exceed the closing clamp; the press is applied to the carrier and
is the only path that may reach 50 N. `ParallelGripper` owns both peaks.

## Mechanism

- `ParallelGripperSpec`: `center`, `half_gap`, `closed_half_gap`, `waypoints`,
  `force_limit`, `push_force_limit`, pad and carrier dimensions.
- Carrier: 120 x 30 x 18 mm free body, mass 0.2 kg, level guide on translation
  only, gravity compensated, PD at 1500 N/m / 42 N·s/m, `push_force_limit`.
- Fingers: 10 x 12 x 30 mm posts + 6 x 12 x 12 mm pad inserts on the inner lower
  face, mass 0.02 kg each. The pads are a small patch on the finger's inner end
  and never extend past its 12 mm width or 30 mm height. They slide on the
  carrier through native prismatic joints along +/-x, so they cannot rotate.
- Closing aims a fixed interference past the loop's measured width; the clamp
  is the squeeze. The pad face and friction are the only grip aid (no ledge).

## Self-contact

Both scenes run with the toy's self-contact on: 2 mm radius/margin, topological
filter 2, rest-shape exclusion 2 mm (the plush's own surface is within the
margin at rest), 32/64 vertex/edge buffers. The Newton solver's default 0.2 m
margin makes every particle pair touch and explodes the toy. The coarse
translation block accepts self-contact: its pair forces are internal to the
object, and the common-translation solve reads only rigid-soft contacts. A
`tests/test_coarse_translation.py` case pins that acceptance. Both scenes print
and record the solver's self-contact state.

## Cup flow

`pick` loop -> `carry` over the bore -> `descend` tail first -> guarded stall at
the rim -> open the pads to 80 mm and hold still for 0.5 s while gravity settles
the toy -> approach the body's top at 30 mm/s with the pads at that gap ->
`press` under the 50 N carrier budget for up to 40 mm -> hold 0.5 s -> rise ->
open. The two 12 mm patches land on the body's shoulders; the pads do not close
on the body.

The toy is never driven down by the pads before the press; the pads wait through
`SETTLE_TIME`, then approach. The first contact with a **body** particle (part 1,
not the loop) is the press handover; a run with no body contact is refused. The
press is bounded by the 50 N carrier clamp, not by the 40 mm path.

`result.json` reports `press_m`, `press_travel_m`, `press_contact_frames`,
`max_body_contacts`, `peak_push_force_n`, `max_insertion_depth_m`,
`max_inside_fraction`, the local inversion, the self-contact state and the stop
reason. Local tet inversion at the 100 mm rim is reported, not hidden.

## Acceptance

- Rod: three runs with `rod_threaded`, rod support >= 90% of the toy's weight
  over the settled window, 0 inverted tets, `success: true`.
- Cup: three runs with both pad-particle **body** contacts for at least 0.1 s,
  `peak_push_force_n` above the 5 N closing clamp, visible insertion and
  deformation, `success: true`; local inversion is reported, not hidden.
- `ruff` clean; the fast test tier unchanged plus the self-contact acceptance
  case; no attachments, springs or hard-wired toy joints.
- Final videos: `outputs/f1..f3/overview.mp4` (rod) and
  `outputs/bottle/f1-dyn..f3-dyn/overview.mp4` (cup), pulled locally with their
  `result.json`.

## Plan deviations (measured)

| Deviation | Evidence |
|---|---|
| Bore 90 -> 100 mm | 90 mm press expelled the toy off the bench (`object-dropped`, 49.7 mm, 2.1% inside, push peak 5.26 N); the 100 mm b9 run is the user's realistic reference (`object-jammed`, 65.1 mm, 32.4% inside) |
| Pads 40 -> 12 mm | user's claw sketch: a small patch on the finger's inner end, not a face wider than the finger |
| Cup `mu` 0.9 -> 2.0 | the 12 mm pads lose the loop during the carry at 0.9 (`finger_contacts` 5 -> 0 at 8 s, toy on the bench) |
| Self-contact on (was off) | user requirement; 2 mm margin + 2 mm rest exclusion is stable (`min J 0.985`, 0 inverted at rest); the coarse block now accepts it |
| Pad hold, then approach | user requirement: gravity settles the toy; the pads do not chase it (a 60 mm/s chase hit the loop and tipped the toy) |
| Press 40 mm at 10 mm/s | the force clamp is the bound; the path only says how far the press is allowed to ask |

## Not in scope

New scenes, gate changes, library/physics edits beyond the self-contact
acceptance, real-robot evidence, output retention beyond the final runs.
