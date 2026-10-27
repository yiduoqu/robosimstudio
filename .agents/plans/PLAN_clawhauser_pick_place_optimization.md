# Clawhauser: a reliable, efficient simulation backend for policy rollouts

## Status, objective and stopping boundary

**Scoped turn stopped after three trials, within its 60-minute budget.** No
complete cycle passed. All three passed existing physics checks. In trial 3,
alignment and averaged table support were eligible at timeout, but the body
never held the required uninterrupted 0.4 s stability window; speed was
29.2 mm/s against the 20 mm/s threshold. Release stayed blocked. Evidence and
the smallest proposed next step are in the report's final standalone outcome.
Proposed only: a fixed-command settling hold before the unchanged release gate.
No further trial, arm work or broader optimisation is authorised in this turn.

**Current user scope supersedes the broader sequence below:** one complete
standalone-gripper Clawhauser pick/place, preserving the pickup-qualified 40/24
profile, mesh/materials/force limits and all existing success/physics checks.
AIRBOT, material tuning, batching, throughput, policy/reset generalisation and
visual polish are deferred. Only small placement/trajectory fixes are allowed.
Timebox: 27 October 2026, 10:14:57–11:14:57 UTC, at most three full-cycle trials.
The already-running H49 placement-budget trial counts as trial 1. On a full pass,
save one video/results and stop; on budget exhaustion, preserve evidence and stop
with the exact failure and smallest next step. No new solver/infrastructure work.

Timebox ledger: trials 1/H49 and 2/H50 both passed physics checks but refused
release because averaged table support remained below the existing 90% weight
threshold. Trial 2 established body support and reached 0.996 N against the
1.104 N eligibility threshold. Final trial 3/H51 changes only placement load
admittance gain to 0.020 m/(N s), retaining the existing 10 mm/s bound and all
other settings/gates. It started 10:53:42 UTC. No fourth trial is authorised.

Revised 27 October 2026 after reading the complete `Newton Simulations` chat
(thread `01a16438-a4ea-7f83-98ab-25f9be4b30bf`) and inspecting the sibling Newton
checkout, accepted recordings, effective materials and rendering code.
**Approved; implementation active.** The previous allocation (4483730) was
cancelled at the pause. Execution resumed on allocation 4484050 after approval.
Implementation evidence is tracked in the matching bug report.

The objective is an RSS scene/physics backend that is reliable and fast enough
for downstream policy experiments and planning. The two pick-and-place videos
are inspection artifacts for that backend. A convincing movie alone is not an
acceptance criterion. The sibling Newton project's goal was a tuned visual demo;
its successful choices are hypotheses and reusable implementation patterns, not
production defaults or evidence of sufficient rollout speed.

Preserve the agreed object representation: one connected, part-labelled
Clawhauser tetrahedral mesh, with per-element materials and shared interfaces.
Keep all source files and inferred estimates intact. Explicit material tuning is
now authorised, with one selected effective profile stored separately. Do not
introduce rigid replacements, task grasp attachments, prescribed toy motion,
material variants, a policy-learning system or a general calibration framework.

Work on pure pickup/transport/table placement first, then AIRBOT. Keep tests to
core behaviour. Reuse RSS scene, control, reset and replication interfaces; extend
only the missing pieces. Native RSS machinery is useful where it serves this
purpose, not a reason to accept incorrect physics or unnecessary overhead.

## What the Newton work actually establishes

The inspected files live in `../newton/outputs/task_videos/` and
`../newton/outputs/visual_upgrade/`, not just the tracked example directory.
Both use the same Newton source commit as RSS's current environment:
`d960ec3b8be23d7acac01c0ba10b8d04dca8b549` (1.8.0.dev0).

SHA-256 comparisons confirmed that the two projects have identical Clawhauser
mesh, source NPZ, inference JSON and visual-binding NPZ. There are 1,840 vertices,
6,850 tets, labels 0/1/2 (tail/body/loop), mass about 0.12507 kg, weight 1.227 N.
The earlier visual improvement did not require a different Clawhauser physics mesh.

| Setting | Current RSS pure demo | Earlier Newton Clawhauser |
| --- | --- | --- |
| Loop E / nu | 5 MPa / 0.49 | 10 MPa / 0.45 |
| Plush E / nu | 0.5 MPa / 0.45 | 0.25 MPa / 0.45 |
| Density, plush / loop | 300 / 1100 kg/m3 | Same |
| Absolute damping, plush / loop | 250 / 2500 Pa s | 125 / 5000 Pa s |
| Steps per 30 Hz action / VBD iterations | 20 / 12 | 40 / 24 |
| Soft-contact stiffness | 100,000 N/m | 30,000 N/m |
| Full-surface / self-contact | Off / off | Off / off |
| Settling and alignment | Starts directly | 1 s settling; XY grasp refitted to settled rim |
| Grasp | Contact only | 12 compliant particle-body attachments during pickup |
| Additional velocity drag | None | Mass-proportional rate 6/s in placement run |
| Execution | Eager; host diagnostics each frame | CUDA graph over physics substeps |

The earlier profile simultaneously changed stiffness, compressibility, damping,
resolution, grasp mechanics and motion. Its improvement cannot be attributed to
any single row. In particular, E times damping_time is the damping coefficient:
changing E at constant damping_time changes two properties.

Existing Newton diagnostics have zero inversions in the 360 recorded frames
(minimum J about 0.115), but the preceding 30-frame warmup has up to 3 inversions
and minimum J about -0.462. Existing RSS diagnostics have up to 8 inversions and
114 of 390 frames with at least one (minimum J about -1.063). These are frame
samples, not verification of every physics substep. Hiding a settling period
from the video does not make it an acceptable reset for the policy backend.
The accepted Newton states/model/material files still match their delivery hashes.

The earlier contact-supported hanging pilot is useful evidence that this mesh
can carry its own weight on a stick with no grasp attachments. The no-stick
control falls. It does not establish contact-only pickup or table placement.

The detailed Franka arms in the Blender videos were IK visual replays added
onto saved floating-gripper simulations. Their sampled geometry checks do not
establish arm contact response, actuator limits or continuous nonpenetration.
Blender, textures, denoising and body subdivision explain visual improvements;
they do not improve physical validity or rollout throughput. Defer this work.

The tracked USB-hub example offers a separate relevant pattern: finite-effort
MuJoCo arm drives coupled to VBD through SolverCoupledProxy with contact feedback.
Its payload is rigid housing plus a soft cable, so reuse the coupling/control
pattern if useful, not its object representation or numerical force settings.
RSS already exposes a proxy coupler; avoid creating a second orchestration layer.
That example's 80 iterations and proxy iterations are not cheap defaults either.

## Hypotheses and limitations at approval

- Prescribed squeezing/lowering can drive unnatural deformation. Neither RSS
  demo currently enforces a maximum manipulator force: pure fingers overwrite
  body poses; AIRBOT's kinematic backend ignores contact response. XML effort
  limits do not constrain that backend. VBD also lists joint_effort_limit as
  unsupported at this revision. Penalty stiffness is not a force cap.
- Body inversions around release and large local loop stretches are distinct
  symptoms. Wrong materials, poor convergence, excessive squeeze, trajectory
  compression and missed thin-feature contacts remain competing explanations.
- AIRBOT changes targets at 30 Hz without interpolating the physics-substep
  motion. Several visible links lack collision shapes. Detection alone cannot
  stop a prescribed arm/table intersection. The existing one-way coupler already
  preserves old/new body poses for friction; do not patch a historical bug blindly.
- Full-surface contact could improve thin shelf/loop coverage, but has a cost.
  Self-contact could prevent crossings but is not justified by bending alone.
  Newton's legacy self-contact margin default is 0.2 m: set explicit geometry-scale
  values if enabling it. DAT does not make kinematic commands penetration-free.
- RSS supports replication and CUDA graphs in general. This specific pure demo's
  ScriptedGripper has single-world substep hooks and no world_aware declaration;
  the compiler explicitly refuses such entities in replicated builds. Do not
  claim batch support merely because Scene.build accepts num_worlds.
- Every-frame host copies, JSON diagnostics and rendering can dominate an eager
  demo. RSS also queues readback in its step path. Measure the actual policy path,
  not just solver kernels, and do not assume run(count) eliminates readback costs.

## Revised execution sequence after approval

### 1. Establish headless cost and failures before increasing fidelity

Use the current asset and saved evidence. Add only the diagnostics needed for
load, penetration, per-part deformation and task scoring. Capture physics-substep
minimum J/inversion counts with device reductions; read full states only in
selected diagnostic runs. Report settling/reset failures separately from rollout.

Measure cold setup/compilation, warmed headless stepping, action/observation
latency, reset cost and memory on an allocated GH200. Separate three timings:
solver/physics stepping, the RSS action-observation path, and optional diagnostic
recording/rendering. Synchronise correctly at timing boundaries so enqueue time
cannot masquerade as GPU execution time. The baseline's invalid physics is a
cost reference, not an acceptable fast solution.

Compare eager execution against actual successful CUDA capture with the same
physics and commands. Verify which path ran and whether dynamic action updates
and reset still work. Reuse the existing RSS executor rather than wrapping it in
another simulation loop. Do not change materials or solver resolution to make
this initial throughput comparison look better.

### 2. Make the simple manipulator reusable by policies

Separate the time-scripted example driver from the physics. Expose bounded jaw
and carriage targets through reusable device-side control buffers; the demo
trajectory writes those same controls. Alternative action sequences must alter
the interaction, without rebuilding a scene or prescribing object motion.
Keep control cadence independent of physics substeps and evaluate smooth desired
motion at the substep times, including correct angular velocity.

Use the existing dynamic rigid/control route with explicit finite drives and
contact feedback where required. Start from the existing MuJoCo/VBD and proxy
patterns rather than inventing a force controller stack. A simpler route is
acceptable only if its applied limits and contact response are demonstrated.
One short obstacle-stall check is sufficient for the force-limit plumbing.
Report actuator effort, actual contact load and impact overshoot separately.
A capped finger slide limits squeeze only; an all-direction force-limit claim
also requires bounded carriage/lift/orientation actuation. A pose stop guard
must not be presented as a hard instantaneous force cap.

Implement complete reset of controls, entity ticks, velocities, solver contact
history and object state using RSS lifecycle hooks. A validated settled state
may be cached and restored for the same configuration; it must be generated
through physics, not by changing rest geometry or pinning the object. Charge its
preparation and restoration to the appropriate setup/reset timings. Different
initial conditions must not reuse an unrelated settled state.

### 3. Fix pure-demo physics with a small set of informative comparisons

First use unchanged materials to isolate motion/contact failures: settle before
closure, align the grasp to the observed rim, avoid excessive closure, and lower
onto actual table support before opening/withdrawing. Preserve the 50 mm transit
clearance requirement. Measure whether the release compression and inversions
disappear. Reuse the earlier substep-consistent motion and alignment ideas, not
its task attachments or hidden warmup failures.

Use 40 substeps/24 iterations as a diagnostic reference for the old 20/12 setup,
not the selected backend setting. If resolution materially changes the result,
separate timestep and iteration effects only as needed. Keep a finer check to
identify numerical softness, then find the cheapest configuration that satisfies
physical and task checks. Include runtime with every comparison.

Test the earlier loop values (10 MPa, nu 0.45) as a targeted candidate against
the source values. Change E and nu separately when making causal claims; hold
absolute damping and density fixed. The full earlier material profile is a
useful named comparison, not an automatic replacement. Test its 5000 Pa s loop
or softer plush only if the measured response warrants it. Keep inferred source
arrays separate from effective arrays and validate the built coefficients.
No broad stiffness grid, uncertainty framework or material fitting to a movie.
The earlier artificial swing drag stays off initially; introduce damping only
for a measured response problem, not to hide instability or slow motion.

Enable full-surface contact only for demonstrated missed surface/edge contacts,
and self-contact only for actual nonadjacent crossings. Inspect real collider
geometry and filtering before changing detection margins. If enabled, verify
edge/face records using contact indices and barycentric data, not particle-only
counters. Report buffer saturation. Keep each extra contact feature's quality
benefit and throughput cost visible.

### 4. Verify core rollout/reset/batch behaviour and choose the operating point

Core checks only:

1. Actual contact-only pickup, sustained transport, release and stable placement;
   finite states, per-part deformation, no inversions during policy rollout and
   no hidden reset corruption. Two alternative commands must cause appropriately
   different behaviour (for example, open jaws do not execute the same pickup).
2. Reset and replay of the same action sequence reproduce the outcome within a
   declared numerical tolerance, without rebuilding the scene or stale contacts.
   Repeat one modest grasp/initial-pose perturbation without material retuning.
3. Two independent worlds receive different actions; no cross-world contacts,
   control leakage, label corruption or reset interference. Make hooks genuinely
   world-aware using WorldMap; do not bypass the compiler guard with a flag.

Benchmark accepted configurations at 1, 8 and 32 worlds, increasing further only
if useful and within memory. Use RSS replication and GPU reductions; verify
per-world diagnostics rather than reading world 0 for every world. Report any
specific unsupported operation instead of claiming general batch readiness.
Measure physics-only and action/observation-inclusive rates with cameras off.
If future policies require images, add their rendering cost separately.

Record aggregate simulated seconds/second, control steps/second, rollouts/second
for a declared horizon, per-world latency, reset cost and peak memory. Include
batch size, device, capture mode, mesh size, substeps and solver iterations.
For planning, report decision time for N candidate rollouts as approximately
N times horizon divided by measured aggregate simulated-seconds/second. Use
physics-only throughput plus separately measured reset/control/observation costs,
or end-to-end throughput plus only costs excluded from that measurement; never
count the same overhead twice. The brief's thousands-of-rollouts aspiration
is a sizing question, not a proven capability or a fixed latency budget.
Do not call the backend fast enough until the intended policy/planning budget
is explicit and the corresponding measured rate meets it.

Select the lowest-cost configuration passing the quality gates. A fine reference
run may validate it but is not the policy backend default. If none meets both
quality and throughput needs, report the measured bottleneck and next focused
change; do not make the toy rigid or change its representation silently.

### 5. Transfer the accepted setup to AIRBOT

Keep the accepted object/material settings and scoring fixed. Use actual imported
pad geometry, interpolate commands at physics substeps and correct the grasp
path. Audit per-link collider coverage, flags and relevant table/self/object
pairs; add only missing physically justified collision shapes.

Replace unlimited kinematic penetration with collision-aware bounded control or
a contact-responsive backend. Reuse RSS proxy coupling and the Newton USB-hub
finite-drive pattern where this improves reaction fidelity, and measure its cost
against the simple scene. Do not copy the Franka visual-replay shortcut. Check
actual AIRBOT effort enforcement in the active backend, not XML metadata alone.
Do not retune the toy to hide an arm-only control or collision defect.

Run the same contact-only task, reset and small-batch checks, then measure AIRBOT
throughput separately. Include arm solve/IK and control updates in the relevant
end-to-end timing; do not infer its speed from the floating-gripper benchmark.

### 6. Deliver evidence useful for downstream work

Provide a reusable headless example with action inputs, part-labelled state
outputs, reset and task success/score, backed by RSS interfaces. Keep the scripted
pick-and-place as a smoke example of that interface. Do not implement a trainer,
planner, Gym compatibility layer or large benchmark system for this fix.

Save one selected effective material/solver/control configuration, exact source
versions, the small performance/quality comparison table, known limitations and
reproduction commands. Save states and short matched 30 fps overview/loop-closeup
videos for inspection; camera appearance never determines physical acceptance.
Rendering is optional and outside the timed rollout. Blender and asset polish
are deferred unless separately requested.

## Quality gates and focused implementation scope

Keep the original plan's physical requirements: no inversion or degenerate tets
at instrumented substeps; no unexplained surface crossing or contact overflow;
proposed geometric penetration target <=0.5 mm and <=10% local bearing thickness;
contact-only transfer clearance >=50 mm; final body XY error <=20 mm; genuine
support/release; <=5 mm drift in the final second. Confirm geometry-based
measurement tolerances before selecting a candidate, not after observing results.
Keep numerical convergence checks focused on the accepted setup: compare
phase-integrated impulses, loads averaged over fixed physical time windows,
per-part strain and opening/sag, targeting <5% change under final refinement.
Report raw peaks separately. Do not claim measured real-material calibration.

For a dedicated known-load opening check, the earlier proposed engineering
criteria were >=50% relaxed opening width under toy weight and recovery within
5% after 2 s unload, in a tracked loop-local frame. Use such a bench only if the
rollout comparison leaves the material mechanism unresolved; these are not
measured properties, and no standalone calibration suite is required now.

Expected edits: the heterogeneous adapter's explicit effective material override;
the example's control/diagnostic/reset/world indexing; minimal scene/config helpers;
AIRBOT actuation/collision setup; a small focused core check and benchmark entry
point; README and the bug report. Change RSS core only for a demonstrated missing
capability or defect. No Newton source edits are currently proposed.

Use the existing squeeze-a-bug/build-a-feature workflow, with hypotheses recorded
before experiments and independent review of meaningful physics conclusions.
Run relevant core checks and applicable repository lint/regressions when code is
changed; do not build an exhaustive test matrix. The user approved resuming implementation with this revised plan.

## Main inspected evidence

- Chat: `Newton Simulations`, full available thread read, including the explicit
  demo objectives, compliant attachments, later visual-only arms and rendering.
- Newton: `outputs/task_videos/demo.py:147-169,230-260,371-450`;
  `outputs/task_videos/final/clawhauser/{config,materials,diagnostics,warmup_diagnostics}.json`;
  `outputs/task_videos/final/physics_validation.json`; `outputs/task_videos/support_stiff/`;
  `outputs/visual_upgrade/final/{README.txt,validation.json,geometry_validation.json,manifest.json}`;
  `outputs/visual_upgrade/{prepare,render}.py`; original and final review sheets.
- Newton finite-drive reference: `newton/examples/multiphysics/usb_hub_arm.py:41-96`;
  `example_usb_hub_pickup.py:79-132`; `docs/tutorials/usb_hub_pickup.rst`.
- RSS: `examples/10_heterogeneous/{clawhauser_pick_place,gripper,airbot_play_pick_place}.py`;
  `robosimstudio_materials/heterogeneous.py`; `core/{pipeline,schedule,worlds}.py`;
  `scene/compiler.py:1121-1161`; `scene/physics/{kinematic,couplers}.py`;
  `sim/sim.py:743-810`; `examples/07_policies/README.md`.
- Original project brief and rough task attachment reread for context; latest
  user decisions control scope (Clawhauser, one labelled mesh, no material
  variants now, material optimisation allowed, backend performance primary).


## Execution update: convergence and thin contacts

Confirmed an RSS core collision-flag defect: `collide=False` previously left
particle collisions enabled. Corrected with a focused red/green test.
Dynamic finite-wrench controls, reusable target buffers, graph execution,
all-world reset/replay and independent world commands are implemented. Initial
2-world core check passed before coarse acceleration; new subclass core check
is being rerun after adapting its profile guard to RSS build notifications.
Fast suite passed 2,423 tests (26 skipped); repository lint/format passed.

A fixed-inertia VBD trace isolated slow whole-object translation convergence.
The 12-iteration source-material case could balance at about 190 N of support
against a 1.23 N toy. An opt-in RSS material-side `SolverVBDCoarse` subclass now
adds a 3D common translation block inside the same objective/iteration, with
source FEM/materials unchanged and no task forces/attachments. Its restricted
formulation received independent review; the exact Newton dependency and
collision-coverage limitations are explicit. The production frozen-step trace
matches the isolated prototype. Original-path 3 s pickup now has physically
scaled support, 241 mm clearance and zero inverted substeps. Full-task success
and refinement convergence remain unverified.

The observed-geometry path subsequently loses the ring. Independent geometry
analysis found both real ledge-edge escape and triangle/ledge crossings missed
by particle-only contacts. A tighter finite-drive target alone did not fix it.
The next controlled comparison enables full-surface detection alone; its first
kernel compilation is unusually expensive and is recorded separately from warm
runtime. AIRBOT remains after acceptance of pure manipulation. Later force/lag evidence
favors unified native VBD for both the robot and the FEM, with bounded joint
efforts and complete arm collider/filter coverage; the proxy route is retained
only as historical comparison, not the current preferred implementation.

### Current findings after the controlled comparisons

The 20 mm query / 5 mm coarse correction budget passes an isolated unactuated
floor impact with zero inversions and no excess mechanical energy. Query-only
and smaller-budget controls fail. This is a restricted result, not whole-scene
acceptance. Native deterministic collision module settings plus deterministic
VBD gather now give exact short full/partial-reset replay with full-surface
contacts; neighboring world state and captured graphs are preserved.

A measured single-cord pinch replaces whole-hoop compression. Continued closure
increases normal load but does not retain the object. Increasing orientation gain
reduces measured pad-normal deflection but still loses it. A tenfold smaller
friction smoothing speed also fails at 20/12. Source materials remain unchanged;
none of these trials is an accepted full task. Those early 40/24 traces predate the corrected coarse update and are historical
evidence. Subsequent H27–H38 results below supersede their solver/control
conclusions. AIRBOT implementation remains after pure-task acceptance.

### Current execution status: corrected ordering and angular convergence

H27 fixed a Warp deterministic-atomic ordering defect in the coarse application:
current positions now use the same explicitly computed value as displacement
history. The CUDA regression passes. H28 raises physical slide stiffness to
100 kN/m and passes mechanical alignment gates, but the original task still
loses the ring. Both complete 450-frame trials fail; neither demo is accepted.
Source materials remain unchanged.

H29 isolates a large unbalanced object torque even at 128 local iterations.
H30's finite object translation/rotation probe reduces that torque by 71% while
preserving tet shape and energy within float32 rounding. Implement the minimal
optional rotational block, independently validate the loaded state and negative
controls, then test pickup before spending another full placement run. The
same block must support AIRBOT's fixed base. Large general solvers are deferred.

The first bare headless timing is only 0.683 control steps/s on GH200 at 20
substeps/12 iterations. Profile collision, native local iterations and coarse
reductions before optimizing. Rich diagnostics/readback do not explain this
cost. Correctness and speed are both unresolved; the timing workload holds
initial targets and does not establish grasp success or policy observations.


### Current checkpoint: contact-based grasp and supported placement

The pure scene uses one native VBD solver for the mechanical gripper and the
source FEM. Source tetrahedra, labels, estimates and densities remain unchanged.
Neither demo is accepted yet.

H39 fixed the command/measurement handoff that skipped placement. Actual release
then exposed tail-first support: the body was still airborne and tipped away
from the target after opening. H41's body-support gate correctly refused release,
but its load regulation was incorrectly conditional on body support. Continued
lowering against the grounded tail overloaded and inverted the body. The whole
COM remained outside the tail footprint, so releasing at one object's weight
would also be unsafe. H46/H47 now adds global load relief, a guarded hold, then
one slow conditional grasp-position arc to lay the body down before alignment
and the existing support/release gate. Object motion remains contact-driven.

The 40-substep/24-iteration condition is not robust: one repeat inverted a body
element during closure. Increasing iterations to 48 kept all elements positive
but the ring escaped the finite pads. Geometry confirmed real edge escape. H44
now tests freezing the fitted grasp frame after sustained actual bilateral
contact: continued tracking moved the command substantially after securing the
ring. All physical collision geometry and actuator limits remain active. The
bar-contact ablation H43 is prepared but deferred in favor of this earlier
control issue. H44 completed clean pickup and transport with 48 iterations;
H45 also passed at 24 iterations. Both retained bilateral contact, positive
elements at every instrumented substep, bounded actuation and no carrier load.
These are pickup results; the complete placement remains unresolved.

H42 fixed replicated kinematic markers being mistaken for dynamic mechanism
parts, and canonical reset pose caching. Seven two-world native VBD core checks
pass under ordered eager execution, including masked reset, immediate neighbor
preservation, the neighbor's next step, selected-world replay and actuator caps.
Fast unordered execution captures and steps; its replay variation is recorded
separately and is not an exact deterministic replay claim.

After a clean complete pure task: numerical refinement, measured fast 1/8/32-world
throughput and 30 fps rendering of saved states. Then update AIRBOT to finite
efforts, native two-way response and complete collision geometry using the same
object. A read-only collider audit is saved under outputs/debug. The current
GH200 allocation is4491214 on n486; prior allocations are stopped.

Provisional 40/24 held-target cost is now measured: CUDA-captured physics gives
2.56/10.40/29.13 aggregate control steps/s for 1/8/32 worlds, with about
0.206/1.533/6.020 GB current Warp pool use. Identical-target public updates and
sim.step give 2.56/10.34/28.90. This is neither a full manipulation workload nor
a complete per-world policy observation benchmark. Rendering/readback are not
the main bottleneck. After placement passes, compare cheaper timestep/iteration
profiles against physical validity and a finer reference before selecting the
backend default. Do not equate successful batching with sufficient policy speed.

H48's complete 40 s run has zero inverted/nonfinite elements across all 48,000
physics substeps and passes actuation, mechanism, penetration and carrier checks.
The contact guard and laydown arc reach body support. Full placement still fails:
the 26 s placement deadline leaves only 1.83 s for final alignment and weight
transfer. H49 changes only that budget to 36 s, with a 50 s maximum run and an
exit after the existing release/withdrawal/settle interval completes. Materials,
speeds, load gain, arc limit and all acceptance gates remain unchanged. The
per-pad tracking guard was corrected to use the finger midpoint, since bounded
jaw closure deliberately yields to cord width; raw pad errors remain diagnostic.
