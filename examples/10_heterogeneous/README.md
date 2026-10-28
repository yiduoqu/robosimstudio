# Clawhauser loop pick and place

`clawhauser_pick_place.py` builds an RSS scene with one labelled tetrahedral
Clawhauser, a bench, and two scripted kinematic fingers. The fingers pinch its
loop by Newton contact, lift it, move it 180 mm across the table, lower it,
release it, and withdraw. There are no particle-body attachments or scripted
object transforms. The mesh remains one face-connected, part-labelled FEM
volume throughout the run; tail, body, and loop are not separate objects.

The source estimate is used directly: parts 0 and 1 have E=0.5 MPa, ν=0.45,
ρ=300 kg/m³; part 2 has E=5 MPa, ν=0.49, ρ=1100 kg/m³. The builder maps each
tetrahedron to its source part and gathers density-weighted mass at vertices.
`damping_time` and the contact penalty are solver settings. Newton VBD currently
exposes one global soft friction coefficient; this scene uses the loop's 0.9
estimate during loop grasping and keeps all per-part friction values in the
source JSON for later contact work.

Run from an allocated CUDA compute node with the sibling Newton checkout at
commit `d960ec3b8be23d7acac01c0ba10b8d04dca8b549` (1.8.0.dev0). In the
current cluster workspace:

```bash
rss_root="$PWD"
source ../newton/outputs/stage1_setup/activate.sh
export PYTHONPATH="$rss_root:$rss_root/../newton"
python "$rss_root/examples/10_heterogeneous/clawhauser_pick_place.py" \
  --out "$rss_root/outputs/clawhauser/run" --render-every 3
```

On the GH200's Linux ARM environment, `uv sync` of the entire RSS project is
currently blocked by an unrelated PyRoKi dependency chain: `yourdfpy` requests
`trimesh[easy]`, whose `embreex` wheel is unavailable for Linux ARM. The command
above uses the validated Newton environment and the RSS source checkout. It
needs `imageio-ffmpeg` there for video output.

The output contains state arrays, per-frame diagnostics, a manifest, and a
result summary. The `overview.mp4` video is written when rendering is enabled.
The example runs at 30 Hz with 20 Newton substeps and 12 VBD iterations per
substep. Its success check requires bilateral loop contact during transit,
clearance over the table, placement near the target, no finger contact after
release, a supported and stable final body, and no inverted tetrahedra in the
settled interval. It also reports the peak transient inversion count and
requires it to stay below 0.2% of the source elements. Inspect the recorded
positions, contacts, and video as well as the result summary: this is an
initial contact-manipulation scene, not a claim that every intermediate FEM
element remains inversion-free.

## The rod and the container: two scenes for a force-bounded claw

`clawhauser_on_rod.py` and `clawhauser_stuff_into_bottle.py` put the same
labelled FEM volume through grasps whose force is bounded by the claw rather
than by the script: the fingers carry their own mass and are driven by a clamped
PD wrench (5 N, 0.03 N·m), so `peak_force_n` and `peak_torque_nm` are
measurements, not consequences of prescribed motion. Both derive their geometry
from the mesh -- the loop's hole centre and inradius by an inscribed-circle fixed
point over the loop's vertices, the loop's width in the closing direction from
that same plane, the tail's centre and extent from its own part labels -- and
both print the geometry the motion was built on.

| Scene | What it asks | What it measured |
|---|---|---|
| `clawhauser_on_rod.py` | pinch the loop's sides, lift, thread the rod's free end through the hole, seat the toy on the rod, open, withdraw | the rod carries the toy: 1.205-1.213 N of its 1.227 N weight with the jaws open, 28.6-28.9 mm of bench clearance, peak jaw force 4.47 N of the 5 N cap, peak torque 0.015 N·m, min tet J 0.82, no inverted tetrahedra, three identical runs |
| `clawhauser_stuff_into_bottle.py` | pinch the loop, carry the toy over a 90 mm bore, lower it tail first, release, drop the pads onto the body, press, open, withdraw | 79.6 and 77.3 mm of insertion in two of three repeats, 42% of the volume inside, wedged after the jaws open, 12.8-18.2 mm of bench clearance, peak jaw force at the 5 N cap, min tet J 0.10-0.59, no inverted tetrahedra; the third repeat crushes the body at the rim and is refused |

Each scene takes the same controls and knobs, all of them reported with their
measured effect:

```bash
python examples/10_heterogeneous/clawhauser_on_rod.py  --out OUT --render-every 3            # or: --claw kinematic, --no-coarse, --rod-radius R, --mu MU
python examples/10_heterogeneous/clawhauser_stuff_into_bottle.py --out OUT --render-every 3 # or: --claw kinematic, --no-coarse, --bore R, --cap N, --mu MU
```

Both use the qualified pair of the coarse common-translation solve: detection
gap 20 mm and a 5 mm per-substep correction budget, with `SolverVBDCoarse` on
the soft, rigid and wildcard backends, 12 VBD iterations, 20 substeps at 30 Hz,
`soft_contact_margin` 20 mm, `contact_ke` 1e5, `contact_kd` 25, self-contact off.
The scenes' own outputs are `diagnostics.json` (per frame: contact counts and
loads, penetration, per-part minimum J and inversions, claw wrench, finger
poses), `result.json` (the gates and the measurements they rest on) and
`overview.mp4`.

### Knobs, with their measured effect

| Scene | Knob | Value | Outcome |
|---|---|---|---|
| rod | `--rod-radius` | 4 mm | threads; the rod carries 1.124 N of the toy's 1.227 N (91.6%) |
| rod | `--rod-radius` | 6 mm (default) | threads; 1.205 N (98.2%) |
| rod | `--rod-radius` | 10 mm | threads; 1.218 N (99.3%) |
| bottle | `--bore` | 50 mm | fails: the 47 mm tail fouls the bore, 6.1% of the volume inside, mesh crushed at the jam |
| bottle | `--bore` | 60 mm | 65.7 mm deep, 33.6% inside, wedged; the press slides off the loop |
| bottle | `--bore` | 66 / 72 / 80 mm | 67.8 / 69.6 / 74.7 mm deep, 34.9 / 35.9 / 39.7% inside, wedged |
| bottle | `--bore` | 90 mm (default) | 79.6 / 77.3 mm deep in two of three repeats, 41.5-42.8% inside, wedged; one repeat crushes the body at the rim |
| bottle | `--cap` | 2.5 / 5 / 10 / 15 / 25 N | at 60 mm with the pinch press: 65.7 / 65.7 / 66.4 / 53.9 (crushed) / 65.8 mm -- past 5 N the clamp is not the limiter, the loop slides out of the 12 mm pads |
| both | `--mu` | 0.3 | fails: the pinch cannot hold. Scene A never seats the toy (rod support 0.000 N); Scene B never gets it off the bench (depth 0.0) |
| both | `--mu` | 0.9 (default) | as tabulated above |

The distance between the two friction rows is the honest cost of Newton VBD
exposing a single global soft-contact coefficient: at 0.3 the loop slides out of
the pads before either scene's task starts.

### Observed limits

1. **The corrector is not optional.** Stock VBD at the same substeps and
   iterations mangles this volume within 0.1 s of it standing on the bench:
   minimum J −2.72 with 1489 inverted tet-substeps and 1.1 mm of penetration,
   the same numbers in both scenes, while the qualified pair holds min J ≥ 0.48
   and 4.9e-5 m of penetration. Both scenes report the plain-VBD control
   (`--no-coarse`) rather than asserting this.
2. **A flat pad cannot hold a round cord; the 2.4 mm ledge is load-bearing.**
   Flat prisms were measured at 8 mm squeeze and mu 2.0: the loop rides 87-95 mm
   down the grip during the carry, the toy falls into the cup, and the fingers
   end above the body with nothing to press. With the ledge, two of three 90 mm
   repeats carry, release and press cleanly. Scene A never needs the ledge
   because the rod takes the toy's weight within 2 s of the load coming on.
3. **The claws' footprint bounds the task.** Pads clamping the 110 mm body sit
   104 mm apart -- wider than any of these bores -- so the body is pressed, not
   gripped: after the stall the jaws release and their bottoms push its
   shoulders. A grip on the tail (46.9 by 36.1 mm, and the part that goes in
   first) hangs the body *above* the pinch, where two flat faces cannot resist
   the moment: the toy tipped 60 degrees and left the jaws by t = 3.6 s. Both
   are recorded as deviations in `.agents/plans/BLUEPRINT_heterogeneous_scenes.md`.

4. **The claw has the archived carrier.** A 120 x 30 x 18 mm kinematic block
   follows the fingers' midpoint 45 mm above them, as in
   `yiduo/archive-clawhauser-backend-20261027`'s `parallel_gripper.py`. It is a
   body, not a third drive axis; the pads stay the 6 x 12 x 12 mm prisms.

5. **The body press is not deterministic.** Two of three 90 mm repeats are
   clean; the third crushes the body at the rim (min J -0.95, 2842 inverted
   tet-substeps) and is refused, the same failure mode as the earlier b7 run.
   The clean runs are 77.3-79.6 mm deep with 41.5-42.8% of the volume inside.
4. **The clamp is applied in float32**, so a run that sits on it reports the
   cap's own representation: `peak_force_n` 5.0000005 against a float64 5.0. The
   gates allow one part in a million for that, and nothing more.

## AIRBOT Play arm variant

`airbot_play_pick_place.py` imports the six-joint AIRBOT Play arm and parallel
finger visual model from DISCOVERSE. Its `assets/airbot_play/SOURCE.md` records
the upstream revision, MIT license and local gripper changes. RSS controls the
articulated arm with its kinematic backend and couples finger contact one way
into the same VBD Clawhauser volume. Its visual shell, six joint axes and
fingers come from the AIRBOT model; this mode does not validate AIRBOT motors
or two-way force response. The task-specific fingertip pads and shelves are
visible and documented in `SOURCE.md`.

Run it on the same CUDA node and Python environment as above:

```bash
python "$rss_root/examples/10_heterogeneous/airbot_play_pick_place.py" \
  --out "$rss_root/outputs/clawhauser/airbot_play" --render-every 1
```

The default 390 steps record 13 seconds at 30 fps. `diagnostics.json` reports
arm pose, joint positions, per-side loop contacts and tetrahedral integrity at
each frame; `result.json` scores actual lift, transfer, release and settling.
Shorten `--steps` for a smoke run. The baseline script above remains the
verified contact-grasp reference while the AIRBOT arm variant is tuned. The
current full AIRBOT recording under `outputs/clawhauser/airbot_play_final/`
shows the arm reaching, closing and moving, but `result.json` reports
`success: false`: the loop slips off the fingertips during lift and remains at
the source zone.
