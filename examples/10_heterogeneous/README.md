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

## The rod and the container: two scenes for a parallel claw

`clawhauser_on_rod.py` and `clawhauser_stuff_into_bottle.py` build the same
parallel claw: a level 120 x 30 x 18 mm carrier, two 10 x 12 x 42 mm finger
posts on native prismatic slides, and 6 x 12 x 12 mm pad inserts on the posts'
inner faces. Each post runs from the pad's bottom face to the carrier's
underside, so the pad sits flush in the post and the finger is one column. The pads cannot rotate, and there is no ledge: the pad face and its
friction are the only grip aids. The jaws close under a 5 N clamp; the carrier
is the press path under a separate 200 N budget, so `peak_force_n` is the jaw
peak and `peak_push_force_n` the press peak. The press runs the same path at the
same rate as the carry and differs only in stiffness (50 kN/m against 1.5), so
the force is `ke * lag` and the budget is reachable inside 4 mm: measured
145-200 N at the 200 N budget and exactly 100.0 N at 100 N, where the 1.5 kN/m
carry spring capped the same command at 63 N. The toy's self-contact is on (2 mm
margin, 6 mm rest-shape exclusion), and the coarse common-translation solve
stays the pipeline's weight fix. Both scenes print and record the self-contact
state, and both refuse particle-body attachments.

The rod and the container, their geometry derived from the mesh (loop hole
centre and inradius, loop width in the closing direction, tail centre and
extent), and the measured outcomes:

| Scene | What it asks | What it measured |
|---|---|---|
| `clawhauser_on_rod.py` | pinch the loop's sides, lift, thread the rod's free end through the hole, seat the toy on the rod, open, withdraw | 3 of 3: seated at frame 150, rod carries 1.212-1.220 N of the 1.227 N toy with the jaws open, 26.2-26.4 mm of bench clearance, peak jaw 4.76-4.78 / 4.68-4.70 N under the 5 N clamp, 4.7e-5 m penetration, 0 inverted tets |
| `clawhauser_stuff_into_bottle.py` | pinch the loop, carry the toy over a 100 mm bore, lower it tail first, release, let gravity settle it, land the pads on the body's top, press the body 85 mm under the 200 N budget, hold, open while lifting | 3 of 3 released and wedged: 85.3 mm of insertion, 53.0-57.8% of the volume inside, 363-371 frames of body contact, 145.4-200.0 N push peak, 0.60-0.99 mm into the container's shell, 0-1 inverted tets; the judge refuses all three on its 0.5 mm shell gate, which is what 200 N does to this toy. At 100 N the same path lands 48.1-59.0% inside with 0.25-0.60 mm and 1 of 3 releases. Before the press spring was stiffened the same command measured only 63.0 N and passed 3 of 3 at 46.1-48.2%, 0.12-0.43 mm |

Run either scene the same way as the baseline, with `--render-every 3` for the
video. Both run at the pipeline's default `mu`; the cup's carry held at 0.9 in
every 200 N run.

```bash
python examples/10_heterogeneous/clawhauser_on_rod.py  --out OUT --render-every 3
python examples/10_heterogeneous/clawhauser_stuff_into_bottle.py --out OUT --render-every 3
```

Controls and knobs, each reported with its measured effect:

- `--no-coarse`: stock VBD, no common-translation solve. It cannot balance the
  volume's weight; the toy is on the bench from the first frames.
- `--bore`: the container's inner radius [m]. 50 mm (100 mm across) with the
  110 mm body; 90 mm was measured to expel the toy off the bench, 110/120 mm
  inverted its rim tets. 100 mm is the default.
- `--mu`: the pipeline's single global soft friction coefficient. 0.9 for the
  rod; 2.0 for the cup's loop grip.
- `--cap` / `--push-cap`: the jaw closing clamp and the carrier push budget [N].
  The jaws never exceed 5 N; the press measures 145-200 N at the 200 N default
  and 100.0 N at 100 N, over the same 85 mm path.
- `--rod-radius`: 6 mm; the rod carries the toy at 1.16-1.19 N.

The cup flow: close on the loop, carry over the 100 mm bore, descend tail first,
stop on the guarded stall, open to 68 mm and hold still for 0.5 s while gravity
seats the toy, approach the body's top at 30 mm/s, then press with the carrier
85 mm below the pose where the pads first touch the body -- never past 6 mm
above the container's floor -- hold 0.5 s, and open while lifting clear. The
press is bounded by distance rather than by the floor: the claw's pose rate is
half the scene's control rate, so a descent to the floor is 180 mm and does not
finish inside 24 s. 45 mm measured 3.8-4.8 N on the body and 96 mm buried the
claw hard enough that it never left (s1, D14). The first contact with a **body** particle (part
1, not the loop) is the press handover; a run with no body contact is refused.
The 68 mm opening puts the finger post's outer face at 47 mm in the 50 mm bore;
an 80 mm opening reached 53 mm and was refused on `rim-blocks-pads` after 88
frames of wall contact. The pads land on the body's shoulders at 34 mm each
side, clear of the loop.

`result.json` records `press_travel_m`, `press_contact_frames`,
`max_body_contacts`, `peak_push_force_n`, `max_insertion_depth_m`,
`max_inside_fraction`, `local_inversion`, `self_contact` and `stop_reason`.
`max_container_penetration_m` is the gate: the toy must not sink into the cup's
shell. `max_pad_penetration_m` is how far the pads indent the plush, which is
the press's deformation rather than a fault.
The pad-container test is by body and by penetration: `max_finger_container_contacts`
counts a pad-to-container contact only when the witness points overlap, while
`max_finger_container_near` counts the margin contacts and
`max_finger_other_contacts` counts pad contacts with anything else (the toy's own
proxy, not the cup). `finger_container_detail` gives the pad body, gap and world
point for the first container contacts. `diagnostics.json` records the per-frame
contact counts (including `body_contacts` and `finger_container_contacts`),
per-part minimum J and inversions, penetration, jaw and carrier wrenches, and the
pad positions.

### Observed limits

1. **The corrector is not optional.** Stock VBD at the same substeps and
   iterations mangles this volume within 0.1 s of it standing on the bench;
   the qualified pair holds min J ≥ 0.48 and 4.9e-5 m of penetration. Both
   scenes report the plain-VBD control (`--no-coarse`).
2. **The 12 mm pads need friction, not ledges.** Every 200 N cup run and all
   three rod runs carry at the pipeline's default `mu = 0.9`; an earlier claw
   that lost the loop on the way to the cup needed 2.0.
3. **The claw's footprint bounds the cup.** The 34 mm pad half-gap puts the
   post 3 mm from the 50 mm bore wall; at 40 mm the post touched. The body is
   110 mm wide in a 100 mm bore, so the press deforms its lower rim; local tet
   inversion is the measured deformation and is reported, not gated away.
4. **The press is the carrier's, not the jaws'.** The jaws stay under 5 N; the
   carrier's 200 N budget is the press, and at that cap the 85 mm path, not the
   clamp, ends it (peak 60.5-63.0 N). At 50 N the clamp bound the press: the
   carrier lagged its path, the body did not travel with the pads, and 1 of 3
   runs left the pads inside the plush with `released: false`.
5. **The clamp is applied in float32**, so a run that sits on it reports the
   cap's own representation: 5.0000005 against a float64 5.0. The gates allow
   one part in a million for that, and nothing more.
6. **Self-contact is millimetre-scale.** Newton's solver default is a 0.2 m
   margin; at that value every particle pair touches and the toy explodes on
   frame 1. The scenes pin 2 mm margin, 2 mm radius, topological filter 2 and
   6 mm rest-shape exclusion; a 2 mm rest exclusion left the 6 mm grasp squeeze
   folding the 11 mm loop into itself.

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
