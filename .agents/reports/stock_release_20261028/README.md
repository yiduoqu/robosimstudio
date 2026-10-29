# Historical stock-Newton release evidence

These reports and recordings preserve the stock-Newton standalone-gripper
investigation completed on 2026-10-28. They are historical evidence, not tests
of Hanson's parallel claw or its solver configuration. The integration branch
uses `hanson/dev` implementation commit `7476ea2777c8de80b58f89a92e583503e350979c`
without importing any local contact, controller, solver or material changes.

| Recording | Observed result | Evidence |
| --- | --- | --- |
| Opening both jaws without tipping | Ring remains on the right finger; no airborne release | [Video](runs/opening_only_retained/overview.mp4), [result](runs/opening_only_retained/result.json), [states](runs/opening_only_retained/states.npz) |
| Airborne release, final repeat | Release at 8.5667 s before right-jaw withdrawal; no recontact; zero recorded inversions before release; peak 3 after release | [Video](runs/airborne_drop/overview.mp4), [result](runs/airborne_drop/result.json), [states](runs/airborne_drop/states.npz) |
| 200 mm lowered release, final repeat | Table contact at 9 s, release at 10.7333 s; final body drift 0.0435 mm; target error 159.5 mm; peak 3 post-release inversions | [Video](runs/lowered_release/overview.mp4), [result](runs/lowered_release/result.json), [states](runs/lowered_release/states.npz) |

The two final recordings pass their respective release-motion predicates; their
overall physics success flags remain false. These are not accepted accurate
pick-and-place results. Checks sample 30 Hz frames, rather than every solver
substep. Candidate contact counts do not measure support load.

## Reports

- [Lowering and collision investigation](reports/BUG_FIX_clawhauser_release_only.md)
- [Airborne release investigation](reports/BUG_FIX_clawhauser_airborne_release.md)
- [Final release-order and lowering investigation](reports/BUG_FIX_clawhauser_drop_then_place.md)

Reports are preserved verbatim. Their absolute workspace links and reproduction
instructions refer to the original local stock configuration. The portable
recordings listed above include diagnostics, manifests and available validation
files. Source snapshots and older trials remain in the original workspace;
they are not part of this evidence-only addition. Do not apply the stock
gripper's independently tipping-finger trajectory to Hanson's parallel claw.

`checksums.json` records SHA-256 hashes of the copied reports and run files.
The saved states contain particle and rigid-body poses; they do not contain
all velocities or solver history needed for an exact dynamic restart.
