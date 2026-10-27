# Clawhauser backend investigation

Status: implementation active after approval of the backend-focused revision.
Source baseline RSS 6d60d7b; Newton d960ec3 / 1.8.0.dev0; GH200 allocation
4484050. Same 1840 vertices / 6850 labelled fused tets, 0.12507 kg object.
No object attachments or prescribed particle motion permitted in task rollouts.
Only core task/control/reset/batch checks; rendering optional, outside timing.

## H0: eager execution dominates current rollout cost

Prediction: identical physics and targets under a successfully captured graph
run materially faster, without changing final particle positions beyond floating
point repeatability. Control: frozen copies of original pure demo and gripper in
outputs/debug/clawhauser_pick_place_quality/baseline_source/. Compare 390 steps
eager/captured, cameras and readback disabled only for physics timing; preserve
final states to check actual solve. This speed is not acceptance of invalid physics.
Experiment: baseline_speed.py in that directory. Verdict pending.

## H1: prescribed gripper authority drives overcompression

Prediction: dynamically integrated fingers with capped force/torque stall against
an obstruction rather than overwriting body transforms; same toy/materials show
smaller loaded strain and release compression. Alternative: underconverged FEM,
wrong material, missed thin-feature contacts. Feature: bounded free-body VBD servo,
with device controls and world-aware indexing. Do not cap reported contact forces;
servo actuation and impact reactions are distinct. Test stall then actual full task.
Verdict pending.

## H2: material response and numerical conditioning contribute to loop distortion

Prediction: E-only, nu-only and damping-only interventions change load response
separately once H1/contact motion is controlled. Source damping held fixed for E
comparisons. Earlier Newton 10 MPa/.45 loop profile is a candidate, not ground truth;
its attachment-assisted pickup is not a contact-only reference. Compare source and
candidate at 20/12 versus a 40/24 diagnostic reference if needed; select cost and
physical validity together. Verdict pending.

## H3: contact coverage and arm target timing cause AIRBOT failures

Prediction: demonstrated geometry crossings improve with corrected collision
coverage and substep motion; detected overlap without response remains under
unlimited kinematic targets. First complete pure scene. Then preserve its material
settings while checking AIRBOT pad path/colliders/actuation. Verdict pending.

## Acceptance and evidence

Approved plan .agents/plans/PLAN_clawhauser_pick_place_optimization.md. Track full
pickup/transport/release/stable placement and frame/substep integrity separately
from old success flag. Measure headless physics and actual action/observation path,
reset, memory and batch throughput. Maintain raw source materials and one selected
effective profile. No claim of real material identification or policy-budget
sufficiency from a video. Generated logs/states under outputs/debug/.

### H0 measured / H1 first result

Frozen 390-step baseline: eager95.28s versus captured66.11s (physics only),
0.136 vs0.197 simulatedseconds/wallsecond; final particle arrays exactly equal.
Capture improves1.44x here, does not alone make planning fast. H0 PARTIAL: real
cost reduction but numerical work remains dominant. Material CPU tests2/2pass.

Dynamic finite-authority source-material test120frames: zero inverted tet-substeps,
5N/0.03Nm applied caps respected, but no retained pickup. Pads tilt substantially
under ring/table reactions; source-material body stretch remains high. This proves
new controller path executes, not that task is fixed. H1 PARTIAL/unresolved.
Next distinguishing test changes angular servo stiffness/torque authority only
(1Nm/rad,0.0008Nms/rad,0.1Nmcap), same5Nlinear cap, mesh, materials,contacts,path,
resolution. Prediction: pad attitude preserved and capture/retention improves;
if stillmisses,inspect settledgraspheight/coverage rather than blind forceincreases.

Attitude-only test failspickuptoo; no inversions. Independentreview confirmsworld
angularrate/frame convention; identifies budgetallocation: closureFx~4.94N then
liftallocates Fz~4.99N andleaves Fx~0.3N. Requestedclosedhalf-gap10mm versus~20mm
blockedjawposition consumesauthorityratherthanestablishingstablegrasp. Nexttest
sameattitudegains/5N cap changesclosedhalf-gaponlyto23mm (restgrasppatchhalfspan
~21.6mm plus3mmpadhalfwidth gives~24.6mm nominalfirstcontact). Prediction: modest
indentation yieldscontinuousnormalforceandretainedliftwithoutcrushingloop.

Relaxedgap23mm stillfailspickup; rightfinger remains~.762m against~4.3N downward
reaction despite5N upwardactuation whilelooppatchandbody staynearrestingheights.
This refutesgap/attitudetuningasalonesufficientfix. Controllednexttest keeps
source material/contact/control/path atrelaxedgap and increasesresolution20/12
->40/24 to testunderconvergedcoupledFEM response; separatesnumericalworkfrommaterial
beforeanyEchange. One bundledresolutionreference isdiagnostic, not a claim about
timestepvsiterationcauses and not a production speedchoice. Need correctgraspand
forcepathsfirst; physicalfirstfailure remaining.

Backendalternativeaudit: XPBDconsideredforparallelconstraint/momentumcomparison,
butnotselected. Same model arrayswouldNOTpreserveconstitutive/contactbehaviour:
currentXPBDusesCdev=tr(F^TF)-3squared andCvol=J-1constraints,nottheVBDstableNH
stress; infinitesimalsimpleshearhasnoquadraticenergyinthatconstraint. ItsCONTACT_F
reportsrigidcontacts onlyandfullsurfaceunsupported. Avoidpresentingspeedfromthis
solverasan equivalentphysicalreplacement. VBD convergence/forcebalance remains
thenextissue; noXPBDrunwasmade.

### H4 recorded before correction: noncolliding decorations still collide with FEM

Dumpcontacts_0060 confirms source_zone(shape1) applies78.61Nup onparticles despite
collide=False; table applies109.07N; rightledge4.262N. Sourcezonecontactplane
z=.75125 matchesvisualslab, notmaintable. RSSapply_display onlyclearsCOLLIDE_SHAPES,
leavingCOLLIDE_PARTICLES. Prediction: clearbothflags removesallzonecontactrecords
withsamephysicalsetup. Feature:onecoreflagfixandonefocusedtest(visible/colliding
independent). Corrected90ftrialwilltestimpactseparately; cannotclaimallpickupfixed
because tableforcealonealreadyunphysical. Red/greenCPUtest planned.

Forcebalance nowmeasured: at2.03s netcontactontoy~191.94Nz versusweight1.227N,
nearzeromassweightedmomentum. Staticforcebalanceinvalidatesaplausiblescreenshot
or submillimetrepointdepth; refinement/contactconditioning remainsarealissue.
XPBDnotusedbecausecontact/constitutivelaws differ. Stalltestpassed5N appliedcaps,
~58mmtargeterroragainsttable, conservativebottom.749999m. Two-worldcorecheckpassed
oppositecommands,resetreplay,allworldreset andgripper-onlytargetedreset; partial
object/solverresetnotyetimplementedorclaimed.

H4flagfixcoretestred2subcases,greenallpass; correctedcontrolledcasehasnozone
contactrecords, butpickupstillfailsandstaticforceimbalancepersists~190N.
CONFIRMEDcollisionbugfixed, NOTfullphysicalfix. Nexttestsamecorrectedconfiguration
changesallparticipatingshapeke ANDsoftke100k→1ktoisolatecontactconditioning,
measuretableforcebalance/penetration andretainedlift. Thisisadiagnosticcontact
lawchange,notsilentmaterialtuningoracceptedproductionchoice. Applied5Nlimits
andtorque.1unchanged; timestep20/12unchanged.

Runtime modelaudit verifies all1840ACTIVE,physicalmass.1250655,consistentinvmass,
8colorscover1840. Freshactualtetgradientkernel netelasticforce~3e-6N, whereas
contactnet~198N; residualmedian.038N,p95.58N; predictedlocalNewtonsteps1–3µm.
LowercontactK1kdoesnotresolve(207N,6.6mmfloorpenetration),rejectasproduction.
RecordH5: incompleteVBDlocalrelaxation creates repeatablequasi-staticfalseforce
balance. Distinguishingexperimentstep_trace.py prolongsONEphysicalstepwithfixed
inertiatargetto512iterations (doesnotadvancetime512steps), startingexistingtoy
state,zero rates,fingersparkedclear. RecordCOM,actualinertiaforce andcontacts at
iteration1/2/4/8/12/24/48/96/192/384/512. Ifresidualshrinks underfixedtargetconfirms
underconvergence; ifnot,traceactualscatter/updatepath. NoEchange/nootherbackend.


### H5 outcome and H6: coarse translation convergence

The fixed-inertia experiment supports H5: at iteration 12 the fresh whole-object
residual is 192.88 N vertically; at iteration 512 it is 29.41 N. Contact force
falls from 194.10 to 46.49 N while the inertial restoring force rises from 1.224
to 17.079 N. This demonstrates slow relaxation at fixed physical time; it does
not establish sufficient convergence even at 512 iterations. Evidence:
`outputs/debug/clawhauser_pick_place_quality/step_trace.{py,json,log}`.

H6, recorded before implementation: local vertex solves poorly propagate the
whole-object translation mode through the stiff connected FEM. A coarse 3D
translation block within the same iteration can reduce that residual cheaply,
without changing the asset, constitutive law, gravity, contacts or actuator caps.
Prototype only, opt-in, in outputs first. Compare the same frozen state and
inertial target at 12 iterations with the block against 12 and 512 without it.
Then test the original dynamic pickup. Measure residual, J, motion and runtime;
a lower residual alone is not a fixed task. Independent formulation review
accepts the overlapping block conditionally: use fresh residuals, the complete
fused object per world, exact contact block Hessians including barycentric cross
terms, unchanged step-start anchors, consistent displacement bookkeeping and
explicit collision-coverage limits. Reject unsupported pins/attachments/DAT.
This is numerical preconditioning, with no desired object pose or task trajectory.
Production integration is conditional on measured benefit and review.


H6 partial supported: controlled original-path 90-frame trial with the block
has resting support 1.238 N versus 1.227 N weight, pickup clearance 241 mm at
3 s, and no inverted tet-substeps. Source E/nu and 5 N caps unchanged; worst
loop principal stretch about 1.32. Recording wall time 17.67 s. This addresses
the false static force balance, not complete manipulation or strain convergence.
The observed-geometry path picks up but loses the ring near the lift hold,
then impact creates inversions. This path is rejected; no placement attempted.

H7 recorded before next trial: particle-only contact misses support on the thin
ledge as the deforming rim slides, causing the observed loss. The 2.4 mm shelf
is comparable to/below mesh spacing; full-surface contact is a targeted detection
comparison. Hold mesh/material/control/path/coarse block/resolution fixed and
enable full_surface_contact only. Compare retained active forces and ring
support, clearance, crossings, inversions and cost over the same 9.2 s pickup.
If loss persists despite surface records, inspect geometry/friction and velocity
rather than attributing it to missing contacts. No claim confirmed yet.


Independent geometry review refined H7: final left escape is across the inner x
edge, not the y end. The right bearing patch reaches its y end earlier. Small
surface crossings coexist with true geometric escape. H7 remains unconfirmed
as the dominant cause pending its controlled run.

H8 before the next separate comparison: the fixed commanded gap stops applying
inward force once the hanging ring narrows/rotates; lost squeeze lets its bearing
patch escape the ledge. Test 16 mm commanded half-gap against 19.55 mm with the
same 5 N capped dynamic drives, source materials, observed path and point-only
collision. This is not prescribed object/jaw compression. Measure actual gap,
bilateral loads, ring strain and retained lift. Production coarse subclass
matched the frozen-step prototype to numerical roundoff before this trial.


H8 result: insufficient. Tighter 16 mm target loses the ring earlier, around
5 s; zero inverted substeps but no transport. Source profile and 5 N limits
remain unchanged. Actual contact records become unloaded before escape;
smaller nominal gap alone does not establish a stable pinch. Evidence:
`observed_pick_tight/`. No further jaw/force tuning until H7 is evaluated.

Production coarse fixed-step trace matches the prototype to rounding error;
12-iteration residual 31.03854 N versus 31.03858 N. Its first updated 2-world
core attempt was blocked by an overly broad notification rejection during RSS
build. This is an adapter integration defect, not a physics failure; revise the
guard to inspect unchanged supported particle-profile values on broad startup
notifications while retaining restrictions on actual unsupported mutations.


Full-surface first trial has not reached physics: native generic face-contact
NVRTC compilation remains CPU-bound after >12 min (1.1 MB generated CUDA).
This is setup cost, not evidence of failed collisions. Before another try,
select Warp's existing module compiler option optimization_level=1 (release,
backward disabled for this nondifferentiable test), leaving contact equations,
iterations and geometry unchanged. Public Warp option maps to fast NVRTC
compilation on installed CUDA 13.4. Record this compiler setting explicitly;
validate its numerical outcome and warm cost independently. No new collision
algorithm or global Newton dispatch mutation is introduced.


H9 before material comparison: source loop nu=0.49 gives a bulk/volumetric
coefficient several times larger than nu=0.45 and worsens local conditioning.
Compare nu=0.45 alone with E=5 MPa, density and absolute damping unchanged,
using full-surface detection and the same bounded observed-geometry controller.
This is the originally planned compressibility candidate, saved as an explicit
effective override; the raw estimates remain immutable. Observe settled rim
motion, retention, deformation/J and runtime, then refine any successful setting.
No claim of physical calibration or that nu alone caused the original failure.


H7 result: full-surface contacts execute, recorded plane penetration <=5.3 um
and zero inverted substeps, but no retained grasp. This is not proof all surface
crossings are eliminated. H9 nu-only candidate also fails and adds transient
body inversions; reject it, keep raw source values for the next control test.

H10 before control correction: the approach uses only upper-patch width and
brushes a wider lower-loop vertex (605) with the right ledge at 2.3 s. By close
start the patch is 16 mm above the planned pad centre and its two sides differ
18 mm in height (35 degree tilt), growing to 47 degrees. Force authority is
unused because the grasp misses. Feature: compute safe open approach width from
all label-2 particles plus actual ledge inward reach/detection clearance. Keep
closure target, material, solver and rest of path unchanged. Prediction: remove
premature brush and induced rim tilt; inspect pre-close geometry and bilateral
loaded grasp before claiming the complete task. Reobserve/orient only if this
minimal envelope correction remains insufficient.


H10 partial: whole-loop approach clearance produces an initial bilateral pickup,
but the hoop still escapes near 5.3 s. Subsequent impact/ejection causes
inversions and excessive travel; this run is rejected, including its failed-grasp
behaviour. Request an energy/numerics audit; do not hide this by showing a good
pickup frame. No further material candidate is selected.

H11 before a physically different grasp policy: pinching the opposite sides of
an entire hoop makes retention depend on its projected width while it rotates.
Instead pinch one upper-cord section, as a parallel jaw normally would. Keep
one identical mesh and source material profile. Use measured source apex IDs,
a fitted current tangent and horizontal closing direction, narrow 12 mm flat
fingertips without ledges (explicit manipulator geometry change), and the same
5 N finite actuator budget. Smaller positive solid-pad inertias require matching
lower angular gains (0.1 Nm/rad, 0.0003 Nms/rad, 0.03 Nm cap); this is a declared
controller/geometry bundle, not a causal claim about a single scalar.
Frozen whole-boundary triangle/OBB checks choose a clear approach, followed by
fresh frame/width acquisition immediately before closing. Source apex width
is about 7-8 mm, rather than squeezing the full 42 mm hoop. Placement preserves
jaw orientation, aligns observed body XY to the target, lowers to support and
opens normally. Measure retained loaded pinch, complete task, strains/J, failure
behaviour and cost before selection. No object attachments or pose commands.


H11 static trial: bounded safe target raise resolves the initial 1.4 mm table
clearance issue, but one-time reacquisition still couples a 16 mm horizontal
realignment and ~10 mm lowering with closure. One pad jams above the cord;
no retained pickup. This run has zero inverted/nonfinite substeps and zero
finger/table penetration; it does not pass the task. Evidence cord_grasp_raised.
Before the next trial, change only the command policy to track the observed
cord during hover, descent, closure and hold at 30 Hz with smooth 600 Hz target
segments. Freeze that measured frame before lift; do not track/command object
motion during transport. Same narrow flat geometry, 5 N caps and source FEM.
Expect less stale alignment and actual bilateral pinch. Geometry checks remain
preflight estimates; contacts, loads and task outcome decide acceptance.


Initial-settle energy audit refutes an assumed energy problem there: rest NH
energy ~1e-12 J; first-second elastic energy 0.002771 J is below measured COM
potential loss 0.005216 J. The 2 mm initial bottom clearance is not a COM drop
limit because the object reorients. Do not add rotation acceleration from that
unsupported concern. Partial-world reset passes owned-state/neighbor/replay
checks with graph retained; selected coarse diagnostic rows now respect masks.

Controlled unactuated floor impact does fail energetically at 20/12: freefall
is almost rigid and dissipative, but first contact raises total mechanical plus
contact energy from -0.00417 J to +6.86792 J, before inversions. Later peaks
reach >10 J. Zero non-floor contact candidates and zero overflow isolate the
impact. One finer 40/24 comparison is underway, same source law/mesh.

H12 before changing impact settings: the diagnostic coarse budget (0.25 mm per
substep) cannot supply the common-translation correction required by 3.85 m/s
impact (~6.4 mm per substep), while the 2 mm detection gap may miss first contact
before the predictor crosses the floor. A restricted budget can leave impact
response to unconverged local vertex deformation. Compare query gap 20 mm with
old budget, then a consistent 5 mm budget <= gap/4 at 20/12 if needed. Source
materials, contact stiffness/damping and applied gravity remain identical.
Measure pre/post energy, fresh contact residual/impulse, budget hits and J.
This changes query coverage and an allowed numerical correction, not physical
contact radius, shape thickness or an object target trajectory. No general CCD
claim follows from a larger detection gap.


H12 controlled outcome: increasing query distance alone to 20 mm fails with
+6.665 J excess physical mechanical energy and 252 inverted tet-substeps.
With the same query and 5 mm translation budget, no excess mechanical or
contact-inclusive energy, zero inverted tet-substeps, minimum J 0.6532 over
600 substeps. Budget saturation falls from 34 to 2 substeps. CONFIRMED that
the overly restrictive correction budget contributes to impact failure in this
controlled drop; full task and refinement remain required. Fine 40/24 with old
settings reduces but does not remove inversions (52 inverted tet-substeps).
Next run changes only query/budget in the prior cord_feedback task before H13.

H13 before a retention intervention: the tracked cord approach establishes
bilateral contact, but freezing the aperture does not retain a loaded pinch.
In `cord_feedback`, at 3.6 s the commanded inner gap is 6.760 mm, actual
projected gap 8.519 mm and apex width 6.756 mm. By 4.333 s the actual gap
widens to 11.247 mm while the apex projection increases to 9.310 mm: the
failure is not explained by cord narrowing. Pad rotations reach 6-12 degrees;
the right servo reaches its 5 N vector cap with 4.34-4.56 N used vertically
and 2.40-2.04 N inward. The apex slides below the pad midpoint (7.75 mm below
at 5.0 s versus 6 mm pad half-height), and both contacts disappear by 5.233 s.
The force diagnostic is on the finger; its negative is force on the object.
Frame fit changes stay below 1.9 degrees per control interval, without a sign
jump. Evidence: `cord_feedback/retention_geometry_audit.json` and saved states.

Test only continued finite-authority closure: optionally blend the target
half-gap from its frozen value toward 3.2 mm over the first 0.4 s of lift,
using the existing quintic interpolation, then hold that request through
transport. This leaves a positive requested 0.4 mm solid-pad gap. Preserve
all carriage midpoint/orientation samples, source FEM/mesh/materials, narrow
flat pads, 5 N force and 0.03 Nm torque caps. The actual jaws remain dynamic
and may stall against the cord; a cap does not guarantee safe deformation.
Default `None` preserves the earlier frozen-aperture policy. Record the exact
request, interpolation window and measured actual gap, normal loads, local
cord slip, retained transport clearance, per-part strain/J and task outcome.
A failed loaded-contact gate remains a possible later policy improvement;
this experiment does not add one. Use the same validated backend impact
settings for the paired baseline and candidate, once those settings are ready.


Independent energy accounting details for H12 (all source material values held):
The isolated experiment disables particle collision on the bench and pads,
retains the existing static floor, starts stress-free with zero particle
velocity, and records exact NH energy, actual particle kinetic energy,
mass-weighted gravitational potential change and quadratic normal contact
penalty at every physical substep. There are no non-floor candidates or
contact overflows in any profile. Initial NH energy is about 1e-12 J.

| Substeps / iterations | Query gap | Translation budget | Maximum physical mechanical energy above initial | Maximum including contact penalty | Minimum J | Inverted tet-substeps |
| --- | --- | --- | --- | --- | --- | --- |
| 20 / 12 | 2 mm | 0.25 mm | +9.00025 J | +10.45932 J | -2.73798 | 1514 |
| 40 / 24 | 2 mm | 0.25 mm | <= 0 | +1.13661 J | -0.49734 | 52 |
| 20 / 12 | 20 mm | 0.25 mm | +6.66489 J | +9.59014 J | -3.19995 | 252 |
| 20 / 12 | 20 mm | 5 mm | <= 0 | <= 0 | 0.65316 | 0 |

Physical mechanical energy means NH strain energy + actual KE + gravity
potential change. Contact penalty is tracked separately because its adaptive
numerical stiffness is not a measured conservative object material. Thus the
fine 40/24 run demonstrates inversion failure, but not physical mechanical
energy injection. The 20/12 failures exceed their available gravity budget
even when contact penalty is excluded. Under the same 20 mm query, raising
the numerical translation budget isolates a necessary contribution of the old
cap: cap-hit substeps decrease from 34 to 2, and peak elastic energy decreases
from 7.56563 to 0.18638 J. At initial impact the accepted profile's common
correction uses the full 5 mm allowance for two substeps, then stops without
inversion. Source E, nu, density, damping and contact constitutive settings
are unchanged; this is not a material fit.

Evidence is under outputs/debug/clawhauser_pick_place_quality:
floor_drop_comparison.json, the four floor_drop_* result/energies/states
artifacts and floor_drop_energy.py. Larger query coverage and a successful
single floor impact do not prove general CCD or robustness to every contact
configuration. A matched final-profile refinement must retain the 20 mm query
and 5 mm budget while changing only time resolution/iterations; reducing the
budget with dt would confound numerical convergence with the cap constraint.

Command-clearance replay before H13 execution: the stable-backend baseline
stopped at frame 82 (2.733 s), command sample 12/20, with 499.800645 um
oriented-pad clearance against the 500 um requirement. Segment endpoints
are 499.974822 and 500.022375 um: this is a 0.199355 um interior
rotation/projection excursion plus float32 rounding, not observed physical
penetration. Plan endpoints with a 20 um numerical geometry reserve, still
validate every sample against 500 um, and retain the 2 mm maximum target
raise. Record the reserve explicitly and use it for both baseline/candidate.
A larger excursion will still reject the command instead of lowering the
clearance requirement. Evidence: `cord_clearance_replay.json`.


Full-surface replay isolation: masked reset restores both physical buffers,
controls and selected diagnostics exactly, preserves the neighbor exactly and
retains CUDA capture. Nevertheless full-surface 12-step replay fails the
existing 1e-5 tolerance with the default native contact scatter, and still
fails with public SolverVBD RUN_TO_RUN alone. Two identical FULL resets remove
the partial-reset helper from the experiment: initial state/controls are
bit-identical; substeps 1-21 have exact candidate multisets but differing record
order and list heads; q first differs at substep 22 by 1.19209e-7 m while the
candidate multiset is still identical. Candidate geometry diverges only after
that state difference. Native gather consumes increasing contact node ID,
not list insertion order, so unstable adjacency head order is harmless once
the contact buffer order is fixed.

For this primitive-only, nondifferentiable pure scene, enabling Warp's native
RUN_TO_RUN on the actual particle collision kernel module and generic SDF
edge/face kernel modules canonicalizes their consumed int32 counter slots.
The otherwise identical two-full-reset trace then has exact q and qd equality
at all 120 physical substeps and exact contact-record order throughout; list
heads can still differ without affecting the result. No contact sorting
adapter, changed tolerance or modified collision equations are needed.
Evidence: fullsurface_substep_trace.{json,npz,log} and
fullsurface_substep_trace_native_collision_deterministic.{json,npz,log} in
the debug evidence directory. This proof applies to the current primitive
passes, not mesh target contact allocation (which uses consumed int64 slots)
or differentiable backward. The corresponding partial-reset verification with
these same native module settings is the next gate.


The final partial-reset gate with native deterministic particle/edge/face
collision allocation plus SolverVBD RUN_TO_RUN passes all 17 stages. Replayed
world-0 state/controls are bit-identical for all 12 control steps while world 1
continues; the neighbor's two physical buffers, controls, playback and coarse
diagnostics remain exact. Selected rest state and diagnostics restore exactly,
custom playback cancels, and the same captured graph remains active with
12 additional replays and zero eager steps. Evidence:
partial_reset_fullsurface_deterministic_collision_deterministic.{json,log}.


H14 before orientation-gain intervention: loaded flat pads rotate several
degrees as the cord slips below their 6 mm bearing half-height; the repeated
baseline loses the grasp without reaching the 5 N force cap. A finite but
compliant orientation drive may permit rolling escape. Compare angular Ke
0.3 versus 0.1 Nm/rad, holding Kd=0.0003, torque cap=0.03 Nm, force cap=5 N,
source FEM and repeatable gap20mm/budget5mm setup fixed. Retain the optional
H13 aperture request to distinguish this gain alone from its prior run. For
minimum pad inertia 0.75e-6 kg m2 at dt=1/600 s, Ke*dt^2+2*Kd*dt
=1.833e-6 < 4*Imin=3e-6, satisfying the isolated explicit PD stability bound.
This is not a contact-coupled stability proof. Expect reduced normal-axis
rotation and slip; sustained lift/transport and substep mesh integrity decide
acceptance. Initial states and all parameters except this gain stay fixed.


H15 before friction-regularization intervention: native VBD defaults to
friction_epsilon=0.01 m/s (solver_vbd.py:382). Its rigid-soft force law uses
eps_u=epsilon*dt and a smoothed Coulomb factor s*(2-s) below epsilon, with
s=slip_speed/epsilon. Thus supporting a 1.2 N load at total normal4 N and
mu0.9 can require roughly1.8 mm/s sustained slip, enough to exhaust a6 mm
pad bearing height during lift, even below the Coulomb limit. This could
compound material-frame rotation/slip. Test epsilon=0.001 m/s alone on the
H14 profile: same source mu0.9, E, nu, damping, geometry, gains and caps.
This is a global numerical smoothing change affecting bench and fingers, not
a new material estimate or per-part friction implementation. Expected creep
scale is ten times smaller; near-stick Hessian also grows, so convergence and
substep J must be checked rather than presume improvement. At600Hz the
smoothing displacement is1.67 micrometres, still above float32 position
resolution in this scene. Measure loaded slip, retention, strain/J and cost.

H15 pair-friction correction: source soft friction remains 0.9 and the pad
shape friction remains 2.0. Newton's `_average_contact_material` combines
these by geometric mean, so the actual rigid-soft pair coefficient is
sqrt(0.9*2.0)=1.34164 (`rigid_vbd_kernels.py:1068-1086,5076-5087`). For a
1.227 N load and total normal load 4 N, the regularized equilibrium example
has s=1-sqrt(1-1.227/(1.34164*4)) about 0.122. At epsilon=0.01 m/s this means
about 1.22 mm/s, or 2.92 mm over 2.4 s; epsilon=0.001 predicts about 0.292 mm
for the same idealized loads. These illustrate the known force law, not a
measurement of this dynamic grasp's per-contact slip. Saved 30 Hz geometry
can estimate material motion relative to moving pads, but lacks the exact
substep body history and barycentric anchors used by the contact law. The
intervention changes only global numerical epsilon; no raw source or pad
friction coefficient is changed.


Contactless numerical isolation from exact q+qd at repeatable-preload frame
148: this saved state has no contact candidates and positive J. Only the first
20 ms of the paired freeflight runs is used, retaining >10 mm bench clearance
in both runs. This is not the earlier contactless-inversion reproduction: the
preload trajectory reaches the bench during the following saved frame. At
20 ms, source 20/12 has Eelastic 0.017034 J, body max stretch 2.569 and fresh
rotational residual 0.001147 Nm; source 40/24 has 0.009815 J, 2.324 and
0.004810 Nm. Both lose mechanical energy and retain positive J, but their q
fields differ by up to 1.973 mm. Angular momentum change is not monotone under
this paired time/iteration change, so it is not proof of rotational convergence.
Actual best-fit whole-object rotation KE is 0.000284/0.000406 J; most KE is
COM translation (~0.0395 J). Evidence: contactless_frame148_comparison.json
and both contactless_cord_repeatable_preload_frame148_* q+qd/energy artifacts.

A subsequent frozen single-step experiment holds dt=1/600 s, initial q+qd,
inertial target and metric-damping anchor fixed. Local VBD iteration counts
1/12/24/64/128 lower the same total incremental objective from
0.0207463/0.0207027/0.0206572/0.0205151/0.0203128 J while fresh net rotational
residual grows from 0.000106/0.001321/0.002587/0.006483/0.011844 Nm. Net force
remains about 0.009 N, with no inversion. This demonstrates that ordinary local
descent can relax stiff strain modes while introducing an unresolved global
angular error; 128 iterations is not a converged rotational reference. Both
NH elasticity and the actual metric-damping incremental objective are exactly
invariant under common current rigid rotation, so in this contactless state
the fresh torque residual is computed directly from
sum((q-COM) cross m*(inertia-q)/dt^2). This does not establish that a rotational
preconditioner would resolve internal deformation or the older inversion.
No rotational solver feature was added. Evidence: rotation_step_trace.json/log.


H16 numerical reference before further tuning: after H13/H14/H15 all fail
retention, inspect coupled rigid-soft convergence rather than continue blind
controller changes. Run same H15 materials/forces/gains/path/epsilon at40
substeps and24iterations, using Newton1.8 native particle tile solve (its
default CUDA mode, currently disabled in this example). Tetindices/materials
are explicitly passed by the native tiled kernel; this changes the elasticity
work distribution, not its model. This diagnostic bundle is a candidate
reference, not a causal claim that dt, iterations or tiling alone fixes it.
Measure retained pickup/transport, all-substep J, force behavior and wallcost.
If quality improves, separate necessary numerical settings and verify tile
against equivalent scalar solve only as needed before choosing a default.
No additional material or manipulator geometry change is included.

H17 before a mechanical-gripper comparison: two independently floating pad
bodies admit relative height and orientation changes that a physical parallel
jaw mechanism constrains. H13 preload produced differential tilt up to about
21/18 degrees, and H14 reduced typical normal-axis deflection but still lost
the cord. This motivates testing the missing linkage rather than treating
independent body pose servos as a complete parallel gripper. It does not prove
that linkage alone fixes retention or numerical convergence.

Implement one experimental example entity: native MuJoCo floating carrier
with two opposed prismatic finger joints, retaining the same 6 x 12 x 12 mm
flat pads, source object/materials and two-finger target API. An overhead bar
collider represents the carrier and includes it in proxy coupling. Carrier
mass is 0.2 kg, fingers 0.05 kg each; explicit solid-box inertias and a 45 mm
bar COM offset are declared engineering proxy choices. Carrier world PD has
finite 10 N / 0.1 Nm norm caps, including only the assembly's own gravity
force/moment compensation. Each native slide has position/velocity PD and a
5 N joint-level cap on total P+D effort. No finger world-pose overwrites,
no object constraints and no unbounded finger world forces are introduced.

The two-pose adapter rejects unequal rotations, nonparallel separation and
out-of-range apertures. It preserves reusable substep playback buffers and
world selection. Reset restores the mechanism's generalized coordinates,
rates, body states, controls and target bookkeeping; this is not a claim of
partial reset for the coupled object/solver. Report carrier applied wrench
separately from actual native joint-actuator observables. RSS integration must
retain the two internal proxy slides while omitting the unconstrained FREE
root from native proxy joint lists; native unsupported-joint guards remain.
First check exact builder/control indices on CPU, syntax/lint and source
observable mapping, then an allocated build/stall check once the main driver
route is ready. Compare retained contact-only pickup and all-substep integrity
before selecting this model. The integration and allocated GPU rollout are now
authorized within the active goal. CPU builder/parallel-path checks and an
independent read-only review passed. The first GPU build routes the mechanism
to native MuJoCo and the object to VBDCoarse; measured native actuator output
obeys the combined +/-5 N drive limits. A separate physical obstacle-stall
check remains to be completed; an unblocked motor cap observation is not one.

Before the first task run, add explicit coupling checks: authoritative versus
soft-proxy pad-corner error must stay <=0.5 mm, and active carrier-bar normal
load during transport must stay <=0.02 N. The bar must not become undeclared
object support. Record actual joint motor effort availability and both carrier
wrench caps. First mechanical trial uses source materials, 20 substeps / 12
VBD iterations, full-surface contacts, deterministic native allocation, tile
solve, 20 mm detection gap, 5 mm coarse correction budget and 1 mm/s friction
epsilon. These are inherited from the preceding diagnosed trials; the new
mechanism/coupling route is a deliberate model comparison, not an isolated
single-gain causal experiment. Trial: cord_mechanical (264 frames).

H17 first task failed at frame 99. During free approach frame 46, proxy pad
corner error was 1.50 mm. At frames 94--98, source/proxy divergence grew to
46.89 mm, with one sampled pad normal load 953.5 N, followed by nonfinite
native gripper poses and object inversion. Native jaw effort remained capped
at +/-5 N. The policy's subsequent table-safe geometry rejection is a symptom,
not the first failure. Saved states and diagnostics preserve all 99 frames.
No pickup or material-profile acceptance is claimed.

H18 before changing proxy drive handling: native lagged coupling initializes
the destination from source begin pose and source end velocity, which already
contains the source jaw-motor impulse. Mirrored joint controls then cause VBD
to apply a second joint PD drive, and VBD does not support joint_effort_limit.
This can create artificial proxy motion/feedback even with no soft contact.
Prediction: zero only the destination view's joint_target_ke/kd, retaining
the structural prismatic constraints, limits and unchanged source MuJoCo
drives, should reduce free-motion divergence. If the grasp still loses
stability, do not infer that this single correction solves coupling.
Use explicit opt-in configuration, view-local replacement arrays and the
native JOINT_DOF_PROPERTIES notification before capture. Verify the source
coefficient arrays remain unchanged. Repeat the same task; compare free-hover
corner errors, first divergence, native effort availability/caps and contact
loads. This is a diagnostic correction, not a claim that general coupled
reset or contact convergence is established.

H18 result: failed at frame 95 (versus 99). Frame 46 free-hover error remained
1.500 mm, contradicting the prediction that duplicate drives explain it.
Loaded instability also persisted (frame 94 corner errors 9.11/10.91 mm,
pad normal loads 161/304 N). Disabling the destination motor removes an
unsupported effort path but does not solve the dominant problem. Two native
concerns remain unisolated: explicit lagged stiff contact transfer, and
origin-displacement pose correction combined with COM-referenced velocities
for offset-COM bodies. No claim of a general native coupling fix is made.

H19 before a unified solve: test the same three bodies, two structural slides,
mass/inertia, pad geometry, controls and source object together in VBD. This
removes the explicit MuJoCo/VBD transfer boundary. Native VBD supports external
joint_f and applies its parent/child reaction forces; use a capped +/-5 N jaw
PD command through that channel, with native joint drive gains zero. Compute
jaw position/rate from current maximal body poses and COM velocities because
VBD does not refresh generalized q/qd. Carrier world PD retains 10 N / 0.1 Nm
limits. Label jaw readings as applied external joint effort, not MuJoCo actuator
observables. No state overwrite or grasp attachment is introduced.

This is a solver-route comparison, not isolation of either native concern.
Prediction: it avoids cross-solver divergence while preserving finite actuation
and allows contact-only pickup. Measure actual prismatic constraint residuals:
maximum lateral displacement <=0.5 mm and relative orientation <=1 degree,
with slide travel-limit error <=0.1 mm (preserving positive pad clearance),
in addition to the existing no-inversion, penetration and task gates. Structural
VBD joints are compliant; zero proxy mismatch alone cannot certify the physical
mechanism. First run the same pickup/transport phases before full placement.

H19 result: finite through 77 frames and no frame-sampled inversions, but the
geometry guard then rejected a 2.084 mm required target raise (>2 mm bound).
The gripper contacted the ring before closure (frame 67) and the carrier bar
also contacted it (frame 74). It did not perform a successful pickup. Before
contact, the structural lateral error reached 0.695 mm, exceeding the gate.
The saved pose trajectory shows severe common yaw lag: by end of hover the
target is near 90 degrees but the real linked jaws are near 20 degrees. The
approach therefore enters geometry with the wrong pad orientation and later
presses the cord down; changing material estimates would not correct this.

H20 before adjusting joint conditioning: native VBD defaults structural
angular stiffness to 100,000 Nm/rad. Body-local iteration is poorly conditioned
for rotating this three-body assembly together at that stiffness (the carrier's
inertial angular coefficient is only about 100 Nm/rad at 1/600 s). Test 100
Nm/rad, keeping structural linear stiffness 100,000 N/m, all masses, controller
gains/caps, object materials, path and 12 iterations unchanged. This is an
explicit gripper-joint conditioning choice, not a change to the object.
With rigid_compliant_alm=False and default joint_is_hard=1, the native legacy
solver updates the constraint multiplier as lambda += k*(C-alpha*C0); force
is k*C+lambda. Therefore this is a capped angular penalty scale, not a pure
torsional spring or a guarantee of any steady deflection. The earlier static
0.1/100 estimate does not establish physical compliance. Judge the measured
constraint residuals against the unchanged gates.
Prediction: common yaw follows commands much more accurately, precontact
lateral error decreases, and the open approach ceases to push the cord down.
Run 110 frames first to test approach/closure/hold, then extend if meaningful.
The predeclared structural-error and task gates remain unchanged.

H20 result: at frame 61 free-motion lateral error improved from 0.695 mm to
0.0093 mm and the desired common yaw is followed. This supports angular
conditioning as one issue. However the whole-run precontact lateral peak is
3.53 mm at frame 40, so the mechanism gate still fails. The 99-frame trial
then ends at the command-clearance guard (0.498389 mm versus 0.5 mm), after
loaded contact spikes (finger normal 81.1 N; carrier load 4.32 N). All sampled
and instrumented tet-substep J remain positive. No successful grasp claimed.

H21 before changing the remaining rigid conditioning: lower only structural
linear penalty from 100,000 to 10,000 N/m, retaining angular 100, the native
hard multiplier updates, 12 iterations, same controller/path and object.
Finger/carrier linear inertial coefficients m/h^2 are about 18,000/72,000
N/m; the default penalty dominates these local blocks. Prediction: precontact
lateral error drops below 0.5 mm while orientation remains within 1 degree,
without changing the success predicate. This is another numerical penalty
comparison, not a material fit. Repeat 110 frames, keeping any remaining
loaded-control or clearance failure separate from the free-motion result.

H21 result: linear penalty reduction did not change the dominant precontact
error (3.530 mm peak). It ended after 100 frames on command clearance
0.495399 mm. Finger normal-load maximum was 6.85 N, but the carrier contacted
the object up to 4.84 N, so grasp quality still fails. All instrumented tets
remain finite and positive. This refutes penalty magnitude alone as the
explanation of the remaining free-motion lateral error.

H22 before switching the rigid constraint formulation: the example explicitly
forces deprecated rigid_compliant_alm=False, whose default stabilization
alpha=0.95 retains most of the initial-step residual. Newton 1.8 defaults to
compliant ALM with alpha=0 and separate numerical penalty conditioning.
Test the native recommended formulation, changing only this boolean from
H21. Keep the same finite structural coefficients 10,000 N/m and 100 Nm/rad,
all actuation/source materials/path/resolution/gates. Prediction: measured
constraint residuals follow the authored finite stiffness rather than the
legacy stabilization trajectory; approach alignment remains within the
predeclared limits and the bar ceases to push the ring. Run the 110-frame
approach/closure diagnostic first. Do not attribute a grasp fix to this mode
unless loaded and later complete-task evidence supports it.

Independent source review refined the H22 mechanism before its result: legacy
non-rod C0 stores the full prior-world anchor separation, including a valid
prismatic axial displacement. The later force applies the current rotating
transverse projector to C-alpha*C0. Even perfectly aligned rotating jaws have
P_new*C=0 but generally P_new*C0 !=0. This produces a spurious transverse
target proportional to aperture and angular step, largely independent of
the penalty magnitude, matching H21. Setting only joint alpha=0 on legacy is
a possible narrower diagnostic. The current compliant-ALM trial removes it
via alpha=0 but also changes other rigid-row semantics, so do not call that
comparison a fully isolated proof. Structural linear damping stays at zero;
its analogous prior/current separation term must not be added casually.

H22 result: free approach is now within the lateral gate, but loaded error
reaches 0.692 mm with the finite 10,000 N/m structural stiffness. Finger
normal-load maximum is 4.82 N, no tet inversions/nonfinite values; frame 98
still ends at the 2 mm target-raise guard after the carrier contacts the
object (5.68 N). Thus correcting the constraint formulation is useful but
does not establish grasp retention. The remaining approach/closure controller
keeps refitting and moving the entire carriage as its own contact deforms the
cord; that can push the reference frame downward into the object/table.

H23 before changing closure feedback: freeze the measured cord center,
orientation and width at the start of closure, after the tracked open approach.
Close only the two joints, hold that carriage goal through grasp establishment,
then execute the same lift/transport commands. This is a normal controller
choice, not an object attachment or pose overwrite. Keep H22 physics, material,
caps and gates unchanged. Prediction: it stops chasing contact-induced cord
deformation, avoids the downward carriage/bar collision and retains bilateral
contact during the subsequent lift. Record the exact frozen observation/time.
Run the full 264-frame pickup/transport window; if lateral compliance alone
remains above gate, address that separately rather than relax its threshold.

H23 full result: FAIL, 264 frames, source identity unchanged during run.
Instrumented tet-substep J remains positive (tail/body/loop minima
0.916/0.209/0.770), actuator caps valid, but no pickup: body lift -2.88 mm,
transport clearance 0.591 mm, target error 240 mm. Loaded joint lateral
error reaches 1.64 mm. Pads descend from about 0.7976 m at frame 73 to
0.7620 m at frame 108 despite a roughly 10 N upward carriage drive, then
remain near the table throughout the requested lift. The overhead bar contact
is observed during this sinking, but is not yet established as its cause.
Do not change bar geometry or object materials before distinguishing force
balance from geometry. Stored frozen-closure observation rows are cached
reference data, not fresh measurements of the deformed cord.

H24 before investigating loaded collective-mode convergence: keep the same
H23 model/controls/source, but disable only gripper-to-particle collisions
on the two pads and carrier before compilation in an isolated output harness.
Keep rigid/table collisions. Verify every substep has zero soft candidates
and forces for exactly these three shapes. If the free assembly follows the
same lift while the loaded one stalls, the interaction solve is implicated;
if it still stalls, investigate the mechanism/drive itself. RSS runtime FK
overwriting was checked and ruled out: its maximal-coordinate mirror runs
inverse kinematics (body poses to q), and sync only synchronizes the device.

If free motion succeeds, freeze one loaded inertial step near frame 90,
requiring zero body/body contact records, and trace fresh object, three-body
assembly and combined common-translation residuals as iterations increase.
Object FEM and assembly joint forces cancel within their respective common
translation sums. Jaw/object contacts cancel in the combined sum. Use native
current contact force/Hessian functions, not stale iteration force snapshots.
Small friction epsilon makes tangent contact Hessians large relative to the
inertial blocks; the object-only translation correction cannot directly solve
a common mode requiring the rigid assembly to move as well. This is a
conditioning hypothesis to measure before adding another solver feature.

H24 no-contact result: all 3,600 instrumented substeps have exactly zero
candidates/forces on the three gripper shapes. The frozen reference center,
quaternion and width match H23 bit for bit. The assembly follows the complete
350 mm lift (pad Z 1.147515 m versus loaded 0.757 m); mechanism error/cap gates
pass. This confirms loaded interaction, rather than an inability of the free
mechanism/drive to follow the lift, is required for the observed sinking.
Hook-recorded position and public frame position differ by about 59 micrometres
because of sampling phase; do not use that hook's momentum as an impulse proof.

H25 before adding solver machinery: restore only native friction_epsilon
0.01 m/s from the inherited experimental 0.001 m/s. The smaller value raises
near-stick tangential Hessians roughly tenfold; at a loaded pinch they can
dominate the separate object and gripper inertial blocks. Prediction: the
native default reduces artificial loaded motion lag at 12 iterations, but
may allow measurable physical regularization creep. With 5 N per jaw and
effective mu about 1.34, a 1.23 N payload predicts roughly 0.47 mm/s creep
at the default, not the earlier 2 N-per-jaw estimate. Test actual retained
pickup/transport and all unchanged structural/mesh/contact gates. Same H23
geometry/source/controllers/ALM/12 iterations. No new solver feature yet.

H25 result: FAIL. Gripper motion improves but retention fails: maximum transit
body lift 48.5 mm, clearance 0.671 mm, target error 312 mm; bilateral grasp
false. Maximum lateral error 1.048 mm, source-body minimum J -0.268 with
704 inverted-tet substeps. Actuator caps remain valid. This rules out simply
restoring the native default as a sufficient fix. Global epsilon also changes
floor settling and therefore the initial measured grasp pose; this trial is
not a matched loaded-state proof of the proposed conditioning mechanism.
Proceed with the prepared single-step convergence audit on the exact H23
89-frame prefix, holding inertial targets/history fixed during that step.

H24 instrumentation clarification: the no-soft recorder retained a private tick
through warm-up/reset. Its 3,600 rows cover 40 warm-up plus 3,560 episode
substeps, omitting the last 40 episode substeps. All RECORDED gripper soft
candidates/forces are exactly zero. The before-build collision flags and public
180-frame geometry/cap evidence remain valid; do not claim complete episode
substep instrumentation for that particular diagnostic.

H26 before implementation: the exact H23 prefix (particle positions/velocities
and all three body poses bit-identical at frame89) followed by one frozen
loaded substep confirms slow collective translation convergence. Fresh combined
Z residual falls 6.877 N at12 iterations, 4.816 at24, 2.990 at48, and0.977 at128;
contact Hessian eigenvalues are about3–5 MN/m. Coarse cumulative path is only
71.8 micrometres at128 against the5 mm budget. All9 external rigid/table
queries have zero native force throughout. Inertial targets, previous poses,
controls and applied efforts are fixed during the audit. Evidence:
outputs/debug/clawhauser_pick_place_quality/loaded_step_trace_retry2/trace.json.
Thus simply24/48 iterations is not a convincing low-cost resolution for this
loaded state; neither an active table collision nor exhausted coarse budget
explains the residual.

Feature: opt-in independent translations for the complete fused object and its
FREE connected gripper assembly. Assemble one6D inertial/contact block using
the same native force and PSD Hessian, retaining exact negative off-diagonal
contact coupling. Use native ldlt6_solve with numerical residual/descent guards.
Do not add forces, targets, attachments, FEM changes or actuator reevaluation.
Keep timestep anchors and inertial targets fixed; apply only physical output
translations with common scalar budget limiting. Require one positive-mass
free assembly per world, no fixed/world/foreign joints or unsupported dynamic
colliders. Active unsupported external rigid contact falls back to the existing
object-only block; speculative zero-force queries alone do not disable it.

Predictions/checklist: on the same loaded state the fresh combined residual
should drop much faster than native12/128 without exhausting budget or changing
controls. Shape and internal mechanism relative transforms are unchanged by
this translation correction. With zero actual object–gripper contact Hessian,
the two blocks decouple exactly and no carrier effort reaches the object.
Disabled option preserves the existing object-only path. Validate the small
linear solve and actual frozen-step objective/residual independently, then run
the original pickup/transport/release scenario under all unchanged gates.
Failure of that original scenario remains an unresolved task even if numerical
conditioning improves. This restricted free-assembly correction does not yet
apply to a fixed-base AIRBOT arm.

Acceptance instrumentation strengthened without changing physics: ring load is
now apportioned by the contact barycentric weight on ring-labelled vertices,
rather than crediting an entire face/edge record merely because it contains a
ring vertex. Final support requires an active upward static reaction and no
carrier assistance during settling. Cached closure-reference rows identify their
actual observation time. Recorded-plane depth and independent analytic gripper
box/table depth are now latched at every physics substep (still not proof of
missing-contact coverage or continuous CCD). Frame states also retain body
velocities. The two new device depth kernels compile and match known0.5mm plane
and1mm box/table penetrations on allocated ARM CPU. Independent review confirms
the post-solve global/local state selection; enabled GPU validation is pending
the H26 scenario. These diagnostic/scoring changes cannot make an earlier
failed pickup pass.

H26 comparison, not task acceptance: same124-array checkpoint restores native
baseline12 positions/allthreebodyposes exactly. Coupled12 reduces the combined
residual norm6.908→2.323N, but separate residuals worsen and iteration2 has a
276N transient. Open-jaw cross Hessian is exactlyzero and object positions
match the no-carriage-transfer baseline bit for bit. No budget/fallback binds.
Do NOT run/accept a full rollout from this evidence alone.

H27 before correction: independent before/after inspection finds the existing
object-only `_apply_block` uses `displacement[p] += delta`, then reads
`displacement[p]` to update q in the same kernel. Warp lowers array augmented
addition to an atomic operation. Under RUN_TO_RUN reduction lowering that
update may be deferred to kernel end, so q reads the old displacement while
the rigid6D side moves immediately. This predicts the exactly identical pre/post
object residual observed in every baseline iteration, and the6D transient.
The observer was verified not to overwrite the coarse diagnostics.

Distinguishing test: isolated GPU application with zero reference/displacement
and a known nonzero shift under the actual module's deterministic settings.
Before the fix, the prediction is updated displacement but unchanged q. Replace
only this one-writer augmented update with a local updated value, explicit array
assignment, and q computed from that local value. Expect simultaneous matching
q/displacement changes and unchanged tet shape. Then rerun the saved loaded
checkpoint with corrected3D and6D paths independently; compare fresh force
balance/budgets with fixed inertial/control histories. A successful small test
alone does not resolve the original manipulation task.

H27 CONFIRMED: isolatedGPU red has displacement1mm butq0; the explicit-local
fix givesq=displacement=1mm. Saved loaded-step immediate application changes
q by the requested1.95017µm within44.7nm world-float rounding; its inertia-force
change agrees within16.5µN and elastic energy changes only0.0034%. Focused CUDA
regression `tests.test_vbd_coarse` passes. Independent retained-kernel audit found
no other same-kernel augmented-array-write/read hazard in the grippers/coarse
solver. Older deterministic coarse trials contained this implementation error;
their failures are not final validation of the corrected physics/materials.

On the same124-array loaded checkpoint, corrected3D12/24/48/128 combined Z
residuals are4.901/3.583/1.931/0.282N. Corrected6D12 givescombinednorm0.618N
(objectZ0.560,gripperZ-0.235), no fallback or budget exhaustion, and removes
the276N transient (largest sampledcombinednorm3.879N). OpenjawcrossH remains
exactlyzero, with bit-identical object positions under the10N carriage control.
Evidence: h27_loaded_compare/, deterministic_apply_{red,green}.json,
coarse_apply_trace{,_green}.json. The matched checkpoint validates this local
mechanism, not a physically corrected full-prefix rollout.

Current full candidate: cord_mechanical_vbd_coupled_translation_config.json;
390frames headless, corrected3Dapplication+6Dfree-assemblyblock, unchangedsource
material estimates/caps/strictgates. Main contact settings now configure actual
native NXN/primitive/GJK modules and all existing compacted/specialized soft
modules before launch, covering batch dispatch choices rather than inferring
coverage from one-world runs. Proxy-owned pipelines are not covered/validated.
Last-substep block diagnostics are identified explicitly as snapshots.
Full task status remains unverified until that run completes and is reviewed.

Regression suite before the H27 one-line correction:2424tests pass,26skip,
169.757s (fast_tests_h26.log). The focused CUDA regression covers H27; broader
required checks will be finalized on the selected implementation.

Full-rollout setup corrections: first attempt stopped before physics because
Warp RUN_TO_RUN does not support a consumed-return atomic_max inside native
primitive-kernel SDF scale checking. Removed that unsupported rigid module
setting. Added the small public CollisionSpec.deterministic flag (defaultFalse),
forwarded through main/channel/one-way constructors to native contact sorting.
Focused propagation/manifest tests pass; fullfast2426tests pass/26skip in148.501s.
Soft producer allocation settings remain separate. Compact fallback emitters
now size per-thread deterministic records from actual compatible pair counts
and the native capped grid; dynamic loop trips exceed Warp's static automatic
record estimate at large batches. Runtime replay coverage is still pending.

A390-frame retry was deliberately terminated near frame31: the current observed
placement path needs450frames to include its full14.8s horizon. Evidence/logs
are preserved; this is an interrupted run, not a physics verdict. Current full
run uses450frames and writes to cord_mechanical_vbd_coupled_translation_full/.
It has reached closure with positive J, loaded bilateral ring forces, no bar
load, and last-substep6D corrections accepted without fallback. Await complete
lift/transport/release/settling and the every-substep latches before acceptance.

H28 before intervention: corrected full rollout now lifts the physical object
(clearance49.8mm byframe136), but loaded jaw transverse error reaches0.867mm,
exceeding the fixed0.5mm gate. NativeALM uses the authored finite10,000N/m
structural stiffness; observed5–9N local contact loads are consistent with
millimetre compliance at that value. This is separate from actuator force caps
and from object FEM material. Old legacy-alpha stabilization previously
confounded a stiffness-only trial; current nativeALM avoids that defect.

Test only structural linear stiffness10,000→100,000N/m, preserving angular100,
source materials, controls, timesteps, correction and all gates. Prediction:
loaded transverse errors shrink below0.5mm without losing retention or mesh
integrity; if not, separate local solve convergence from physical compliance.
Run an isolated450-frame candidate while the unchanged H27 full cycle continues
for retention/release evidence. Neither concurrent run is a throughput benchmark.

H28 provisional result atframe166: jaw errors shrink below0.1mm during the
loaded121 sample, but ring contact is again lost by166. Thus physical slide
compliance explains that alignment gate, not the whole retention failure.
Await its final every-substep/penetration evidence; no acceptance claimed.

H29 before diagnostic: distinguish rotational under-convergence from physical
rolling/slip of the small grasp patch, rather than changing material/friction
or force caps blindly. In H27 loaded121–151 the nominal vertical friction
capacity far exceeds weight, but opposite fingertip vertical forces create a
large contact couple; body/loop stretch rises before loss. Servo torque is not
saturated, so the0.1Nm cap alone is not an established cause. Common closing-axis
tilt does not prove carrier tilt because unequal joint errors also affect it.

Use the first completed corrected rollout's actual positive-J unsupported state
near121 (its jaw errors are still within0.5mm), with q/qd/bodyq/bodyqd saved.
Initialize fresh native history and a new fixed physical substep, holding the
actually logged last-substep carriage wrench and jaw efforts. This deliberately
avoids claiming exact warm-start/prefix reproduction. Compare12/48/128 native
iterations with identical state/history/inertia/control and fresh contact data.
Measure net object force/torque residual, angular momentum and contact patch
slip; verify elastic+metric-damping torques cancel under common objectrotation.
An iteration-sensitive torque residual implicates numerical convergence; a
small residual with material points rolling/sliding out implicates grasp/contact
mechanics. No new rotation solver or material change is authorized by this
hypothesis alone; collect the distinguishing evidence first.

H29 result: native fixed-step torque residualnorm0.153/0.173/0.127/0.103Nm
at12/24/48/128iterations; at128 inertial torqueX0.00671 versus contactX-0.10859Nm.
Elastic and objective metric-damping torque sums are below1e-7Nm, so there is
no missing internal torque balancing that residual. No support/rigid forces,
nonfinite values/inversions, cap violation, or coarse fallback/budget hit.
This is a genuine angular-stationarity problem on the tested new cold-history
step, comparable to the whole payload gravity lever. It does not establish
all causes of H28's later failure. Patch fragility is also observed: rightjaw
has onlyone active ring record, atlocal tangential6.28mm versus padhalf6mm
(within0.7mm particle radius); left41% active records nearpadedge. Do not
attribute retention to friction alone while angular response is unconverged.

H30 before prototype: a common object rigid-rotation mode can move a stiff FEM
volume coherently without changing its elastic/metric-damping energy, while
vertex-local iterations struggle to do so. On the same frozen12-iteration
state, form a native-force/GN-Hessian6D objecttranslation/rotation solve about
its current mass COM, with angular coordinates scaled by RMSradius. Apply one
small finite whole-volume rotation+translation derived only from this residual;
keep all inertial, contact, previous-pose and rigid-body anchors fixed. Cap max
particle motion0.25mm and rotation0.1degree with onecommon scale. Test step
scales1/.5/.25 from identical savedstate if needed. Measure fresh torque/force
residual and positiveJ/elastic+damping invariance. This is an output-only
numerical prototype, not a delivery grasp attachment or scripted object path.
Only if supported should an opt-in solver mode be implemented; prefer native
LDLT6 and overlapping existing translation/objectSE3 blocks over a newgeneral
large linear solver. AIRBOT's fixed-base case needs objectmode portability.

H27/H28 full450-frame outcomes are both FAIL, code_changed_during_run=False.
H27: ringlost, transient bodyclearance neverretained; finalXYerror874mm,
bodyminJ-1.045/5043inverted-tet-substeps, maxsubstepplane1.024mm.
H28: mechanicalgatesPASS (maxlateral0.212mm, maxangle0.079deg, validcaps),
but ringlost; finalXYerror224mm, bodyminJ-0.954/4031inverted-tet-substeps,
maxsubstepplane0.516mm. No gripper-box/table penetration, nonfinite tet values,
or contactoverflow in either run. Tail andloop J remainpositive; inversions
occur in thebody after the failed lift/fall. Stiffer physicalslides are retained
as a supported alignment correction, not a grasp fix. H28actual121 savedstate
is nowavailable for the same cold-step angular/prototype comparison.

H30 SUPPORTED on H28actual121: onefinite objectSE3 update rotates0.015209degree
and translates(-1.91,+2.12,-14.47)µm, maxparticlemotion31.34µm; fresh torque
norm0.160876→0.046009Nm(71.4%reduction), force0.616→0.370N. Half/quarter
steps give0.09478/0.12293Nm. No budgetbind/inversion; minJ0.809439→0.809487,
maxprincipalstretchchange5.74e-5, elastic∆E2.17e-7J(0.0023%), metricdamping
∆E1.53e-8J, consistent float32world-coordinate roundoff. Inertia objective
17.59→5.316µJ; normal spring181.385→181.255µJ. Fullfrictionobjective is not
claimed. Anchors/rigids/controls unchanged; baseline state restored afterward.
Evidenceh30_stiff_object_se3/se3_probe.json. This supports implementing the
restricted rotational block; it does not establish a grasp fix.

Implementation feature: opt-in coarse_object_rotation for the same complete
fused positive-mass FEM volume. NativeLDLT6 uses currentmassCOM/RMS-scaled
angularcoordinates and nativeinertia/contactGNblocks; no FEM stiffness in the
rotationblock because energy is invariant. With FREEassemblytranslations,
alternate ONE fresh objectSE3 or existingcoupledtranslationblock per native
iteration; with fixed-base bodies use conditionalobjectSE3 eachiteration.
Finitewhole-object rotation preserves labels/sharedinterfaces. Explicitq/disp
writes; nohistory/inertia/velocity/actuationchanges. Sharepoint-motion/relative
budgets and a0.1degreepercorrectioncap with a common scale; record accepted
rotation/applications/budget/fallback diagnostics. Independent algebrareview
supports this minimum overlapping-block approach;9D/12Dgeneral solves deferred.
Validate loaded-step force/torque, zero-contact no-actuationtransfer and shape
invariance, then rerun complete originaltask and refinement/costchecks. Until
then both taskdemos remain unaccepted.

H31 before profiling: bare headless cost is dominated by physics, not rich
frame diagnostics/readback. The held-initial-target benchmark measures
1.4644s/control interval (20substeps,12iterations) with graph replay and no
rendering, contact-force recording or readback. Public observation costs about
the same. This is a short cost reference, not loaded-task performance.
Warp pool5.86GB is unexpectedly large for1840vertices/6850tets. Hypothesis:
deterministic contact/coarse reductions or oversized contact capacity dominate
runtime/memory. First collect timed native collision/localVBD/coarse sections
and actual contact capacity versus usage. Any optimization must preserve full
surface coverage and overflow checks; nondeterministic modes are diagnostic
comparisons until numerical/task consequences are measured. No speed claim yet.

H31 first measured comparison supports deterministic coarse reduction overhead:
only the pinned coarse module switches floating atomics to NOT_GUARANTEED;
native VBD and collision settings/hashes remain the H28 values. Eight graph
replays take7.5709s versus11.7151s (1.547x). Warp pool falls5.861GB→0.581GB,
about90.1%. Thus this explains most memory but only35.4% of replay time; it
cannot alone establish sufficient throughput. Full contact capacity54110 is
exactly12880point+24738edge+16492face candidates, not a mystery allocation.
Final active count1413, states finite. H31task success/repeatability remain
untested. Import-shadowing and a misplaced builder conditional were caught
before physics; corrected and recorded in pinned diagnostic identity.
Next control switches nativeVBD deterministic reductions off as well while
preserving deterministic collision producers/order. No production perf edit yet.

H30 production validation PASS: default-disabled native12 q/body matches the
frozen probe bitwise. One enabled correction gives torque0.160876→0.057133Nm,
maxmotion31.40µm/rotation0.01521degree; GPUq differs from float64prototype by
at most59.6nm (oneULP), with near-stick friction sensitive to rounding. Bodies,
queries, inertia/history remain bitwise fixed; no inversion/fallback. Open-jaw
+10Ncarrier moves assembly97.6µm but leaves objectq bitwiseunchanged. Both
focused CUDA tests pass, including finite shape/history/world isolation.
Production validation source27d4d37e...; artifactsproduction_object_rotation_validation/.
This is a local solve result; whole-task alternating stability is unverified.

H31 second measured ablation: nativeVBD and coarse unordered reductions,
collision producers/order retained, take0.735145s/8steps (10.882controlsteps/s,
15.936xH28); pool115,315,255B. NativecontactgatherFalse differs from ordered
path, so this is not a claim of physical equivalence. Finitefinalstate and
matchingfullsurfacecapacity54110 only; active1446vs1413. Before further
fullruns expose native/coarse determinism independently of collision ordering,
with prior defaults preserved. Feature scope: one optional coarse_deterministic
backendparameter and example solver_deterministic override, no changedforces,
geometry/material/control/timesteps. Validate actualtask plus meaningful replay
tolerance; exactbitwise reset was never a userrequirement. ConservativeH30
profile remains the controlled rotation-only comparison to H28.

H30 full-task attempt INCONCLUSIVE for retention: both ordered and fast264
qualifications stop at1s/30savedframes, before any grippercontact, because the
frozen whole-boundary SAT approach guard rejects all10 sampled apertures.
No droppedgrasp/actuatorforce conclusion applies. All sampled/substep tets
remain finitepositive; nooverflow. FastbodyminJ0.6966 versusordered0.8303;
maxplane12.46µmvs24.08µm. Ringcentroiddiff1.91mm, bodycentroiddiff0.39mm,
so reductionmode changes settledgeometry and must be refined after feasibility.
Sourcehashesmatchbothruns. One later explicit False override now passes native
NOT_GUARANTEED rather than relying on global wp.config default.

H32 before geometry probe: the sparse approach aperture list may miss a narrow
collision-clear window after the improved angular solve changes resting pose.
At halfgap22.18mm the orderedrun hits only2triangles, while27.18mm hits15;
there may be free space between. Use savedactual30framestate, identical pads,
sourcefaceboundary, 1mmclearance+0.7mmparticleradius expansion andtableclearance
unchanged. Scan physicalaperture interval densely; if no free candidate exists,
distinguish real obstruction from a missed interval before changing trajectory
or geometry. No production change/relaxedguard authorized by this claim alone.

H32 CONFIRMED: same orderedframe30state has a collision-clear aperture interval
19.63575–22.04464mm, width2.40889mm. Original samples18.18074/22.18074mm
straddle it. Midpoint20.84019mm passesexactoriginalSAT[0,0], tableclearance
36.3569mm (required0.52mm). Dense25µmcheck has96freepoints; independent
analyticintervalauditmatchessparseSATcounts. No yaw/geometry/material change
or weaker margin needed. Implement feature: keep existing sparsefastpath,
then try adjacentmidpoints and bounded0.5mmrefinementofthesameaperturerange.
Everycandidate facesidenticalfullboundarysweptSAT/tablechecks; stillrefuseif
nonepass. Validate savedordered+faststates, then rerunpickup. No newplanner
framework; metadataalreadyrecordsallattempts/chosenaperture.

H32scope narrowed before rerun: fastframe30 has NO feasible aperture at the
same orientation under25µm/exactintervalaudit; minimum54right-pad overlaps.
Therefore refinementfixes the orderedstate searchmiss and must keep rejecting
faststate. Fast fittedcordtangent is steep (tangentZ−0.891/upZ0.453), so this
is a distinct approachorientation issue, not proved falsecollisiondetection.
Rootimplemented sparsepath→adjacentmidpoints→0.5mmgrid over the same range;
alloriginalSAT/tablechecks retained. Independent savedstate recheck pending.

H33 before output-only policy probe: square pad orientation about the fixed
horizontal closingnormal is a physical grasp-command degree of freedom. A
small roll may clear the other ring strand in the fastsettledpose while
covering the selected cord. Keep center/closingnormal/width, rebuild actual
sectionbounds in each proposedframe, enforce existing≥50%tangent/vertical
coverage, tableclearance and fullsweptSAT. Test small±5/10/20degree candidates
first. Noobjectmotion/geometry/material edits, no lowerclearance. A supported
candidate would justify a small bounded observed-grasp roll choice; failure
would instead favor resolution/graspstrategy reevaluation. No implementation
before the probe result.

H33 REFUTED within testedrolls: no feasible aperture in9.08968–60mm for0,
±5,±10,±20degree orhorizontal-up+63.04degree. ExactintervalSAT confirms
no missednarrowwindow; sectioncoverage100%/table≥32.82mm. Do not implement
pad-roll search. H32conservative264rerun is active with originalphysics and
only supportedsearchrefinement; source frozen.

H34 before output-only probe: fastsettledring is steep, and hardcoded world-Z
approach may cross the upperloop before reaching its opening. Use the same
observedframe/pads/center/apertures, but sweep along positive localpadZ (the
normal to closingaxis/cordtangent). ItspositiveworldZ component allows a
hover above the object; choose length meeting samehighestpoint clearance.
Retain fullboundarySAT and endpointtablechecks. This tests a physical side
approach, not a force/material/asset change. Only implement if feasible.

H34 SUPPORTED: same faststate frame/width has broad9.534–27.381mm feasible
window usinglocalpadZapproach; secondoriginalsample11.08968mm passesSAT[0,0].
Direction(−0.88616,−0.09608,+0.45331), travel164.73mm, endpoint/straight-path
minimumtableclearance32.821mm. No roll/material/objectchange. Implement opt-in
normal_approach=False, sharedoffsethelperinstaticcheckANDtrackedhover/descent;
retain defaults and physicsgates. Rootmainwiresconfig; workerowns2policyfiles.

H30/H32 conservative run now FAIL: transient lift at121 hasminJ−0.002738,
ringlostby136; no laterframe canrecover the zero-inverted-substeps gate.
Deliberately interrupted this failed264-frame qualification to preserve raw
states/forces/quality rather than spending the remaininghorizon. Localtorque
correction is supported but not sufficient for stabletask. Atloaded121 only3/6
SE3applications occur and cumulative relativepoint-motion budget hits0.25mm;
rotation_budget_scale becomes0. Controller andmaterialremainunchanged.

H35 before experiment: the inherited0.25mmrelativecorrectionbudget can truncate
the new coherent rotation block before angular convergence in loadedsteps.
Expose existing coarse_relative_budget through exampleconfig, default0.25mm
preserved. Test a larger bound≤existing5mmobjectquerybudget; this changes only
numericaladditional-motion safeguard, not any material/force/score. Confirm the
intervention removes saturation and inspect freshloadedresidual/mesh/task, since
removing a safeguard could also worsen transientcontacts. Use pairedfastnormal
pickup firstdefaultbudget then1mm/5mm only as distinguished bymeasurements,
notblindparameter sweep. Conventionaliteration/timesteprefinement remains
necessary if task staysincorrect. No newgenerallinear solver.

H35 SUPPORTED locally: actualpositiveH32frame120 newcold-history fixedstep,
allstate/drives/sourcehashesmatch; iteratesmatchbitwise at1/2 and diverge at3
exactlywhen0.25mmbudgetlimitsSE3. Enlargingrelativeboundto1mm yields
fresh12force3.308294→0.220473N(93.34%↓), torque0.423295→0.052361Nm(87.63%↓).
Defaultbudgetuses250µm/2SE3/latestscale0; 1mmuses534.835µm/all6/latestscale1.
No inversion/fallback, minJ0.27977/0.27974; all12freshstatic+rigidforceszero.
Initialelasticenergy0.770J is alreadyloaded, so this does notexplainitsorigin
orprovefulltaskfix. Frame106probe failedunsupported-screen atiteration1
(newstaticforce), correctlyexcluded, notsilentlyaltered. Evidenceh35_comparison.json.

H34production saved-state/cachedpolicy check passes all79commands throughlift
freeze, identicaltablecriteria/continuouscommandendpoints. Policy nowevaluates
approachnormalonlyduringhover/descent, neverunusedclose/hold. One small root/
worker overlapping edit was resolved beforelint/syntax and finalCPUcheck;
firstfastnormaltrialstartidentity mayprecede lastpolicyedit, so finalaccepted
rollout must rerunonfrozenfinalsource. Paired1mm-budgetpickup startsafterfinal
freeze; compareitagainstsamefastnormaldefaultbudget as diagnostic only. No
throughputclaims from concurrent rich-diagnosticruns.

Pairedfastnormal264outcomes FAIL. Defaultrelativebudget losesbilateralringload
after146, firstframeinversion148, peakclearance128mm. 1mmbudget extendslast
bilateralframe154/firstinversion156 and peakclearance306mm, but still loses
ring beforetransport. BodyminsubstepJ−1.240/−1.587; tail/loop remainpositive.
Thus H35 is a supported localconvergence correction and improvedpartiallift,
not a fullretentionfix. Frame151 at1mm hasbodyminJ0.0826 whileloaded, before
loss; bodystrain is alreadyexcessive. Morebudget alone is not justified.

H36 before controlled refinement: remaininglocaldeformation modes may be
under-converged even after globaltranslation/rotation blocks. Comparefastnormal
1mmbudget12→24nativeiterations ONLY, with same20substeps/material/control/
collisiongeometry. Prediction: better loadedshape/stationarity and stable
retention; ifthis fails or lacks convergentstrain behavior, inspect geometry/
material/backendlimitations rather than increasingbudget blindly. Saveall
substepquality/force states; this is not a newmaterialprofile or solverframework.

H36 PARTIAL:24iterations retainbilateralringload throughall264frames, bodylift
309mm/mintransitclearance205mm; caps/mechanism/penetration PASS. Mesh FAIL:
208bodyinvertedtet-substeps/minJ−0.24248; firstsavedbadframes74–80(beforeany
recordedgripperload), later219–222airborne. At74 allpadresultants/carrierload0,
allobjectforceequalsfloorforce; cannot infereveryinterveningsubstepforcefrom
30Hzsamples. Sourceunchangedduringrun. No taskacceptance despitegoodretention.

Independent source meshqualityaudit rules out severe restslivers/simplefloat
rounding asfirstinversionexplanation: Dmcondition≤12.82overall/8.54body,
minimummeanratio≥0.4056overall/0.431body, altitude≥247µm/300µm. Earlier12iter
1mmbudget firstinvertedtet2438 hascondition5.277/meanratio.6801/altitude1.061mm
at sharedinterface; J−.4028 vastlyexceedshalf-ULPuncertainty.000172. No mesh
or sourcechanges justified bythatcause. Moreinterface/cohortanalysis pending.

H37 before refinement: keep24iterations and halvephysicaldt by20→40substeps
per30Hz action, holdingmaterials/mass/geometry/forcecaps/contactgap/budgets/
controllerphase durationsfixed. IncreasedinertialcurvaturemayimprovelocalFEM
convergence/stability. Test264pickup first; full450release onlyifmeshclean.
Compare actualmatchedphysicalphase loads/strains; optional further refinement
onlyifresultsjustify. Allocation4484050 isnearingexpiry; queuedcontinuation
4490979 uses thesameARMn83environment. No otherproductionedits underway.

H37 PARTIAL:264frames complete, zero inverted/nonfinite tets atall10560substeps;
minJtail/body/loop0.9931/0.2368/0.9542, maxplane68.49µm, box/table0, nooverflow.
But no framehas bilateralringload>0.01N; one-sidedpeak2.323N andcarrierpeak
0.189N at86. Toynevertransported, so not a cleanloadedrefinement proof. Need
actualsavedapex-vs-pad/command audit to distinguish stalefreeze fromcontact
failure before changingcontroller. Fast suite2426PASS/26skip in171.103s after
newcoarseAPI; focusedCUDA2PASSalreadyrecorded. FullRuff756filespass.

Resource continuation: old4484050cancelledafterallrunsended; fixedn83 queued
4490979wasunavailable(plannednode), cancelled. New4491214 requestsanyGH200,
runningn486/6h/8CPU64GB, sameARMenvironment/source/cache. No hardwarechange
claimbeyondnode; futurethroughputbenchmarkmustrecorddeviceandscopeagain.

H38 before controlled policy change: H37miss is explained by a stalefrozen
reference, not missingpadcollision. At72 targetmatchesactualapex; by86 actual
apexmoves+30.75mmtoolvertical/+16.99mmtangent, whilepadsfollowcommandwithin
0.23mm. At91 bothpadsoverlapselectedsection0mmY/Z andactualsurfacegaps18.3mm;
fullboundarySATinflatedby0.7mmalsohaszerooverlap. Carriercontactfirstsample86
occurs afterapexalready+28.8mm at84; no claimaboutunrecordedsubstepforces.
Freeze-on-close at2.4s precedes0.8sclosure+0.4shold, enough forungraspedcord
motion toinvalidateitsframe. Test existing freeze_on_close=False ONLY, same
40substeps/24iterations/1mmbudget/sourceparameters. This tracks actualcord
throughclosure/hold thenfreezesatlift asalreadyimplemented. Prediction:
bilateralringloadestablished with no unnaturalobjectforces, cleanmesh and
retainedpickup. PreviousFalsefailurewaspreH27/H30andnot a currentcontrol.
Evidenceindependent_grasp_geometry.json; no newcontrollercode/framework.

H38 full450 repeat: allphysics/cap/mechanism/penetrationgates PASS, sustained
bilateralringgrasp throughouttransit(minclearance234mm). Zero invertedtets,
bodyminJ.6995/plane68.43µm. TaskFAIL becausePLACEMENTNEVERSTARTED, notbecause
releasephysicswasattemptedandfailed: phases.placement_skipped at8.8s says
'parallel waypoint separation must follow its positive closing axis'. Object
remainsheldthrough450, so release/support/target/driftscorescorrectlyfail.

IndependentH38pickup-only geometryaudit supports genuine one-pad ringbearing
inits16one-sidedsamples (this repeatisbilateral). All64transit snapshots have
no body/tail-owned boundary faces near eitherpad andnocarriercandidates. Exact
leftgap0.616–19.931µm beyond700µmparticleradius, rightcompression4.47–5.91µm;
closestfacesbelongringtetsandring-exclusivevertices. Meanupreaction1.218N
versus1.227Nweight, sampledforce/momentumbalancewithin0.64%. No missedleft
penetration proved at thoseframes. Continuousbilateral is an assumedmethod
proxy, notanexplicituserconstraint; keepitsstrictresultvisible. NOscore edit
hasbeenmade, historicalresults remainimmutable. Fulltaskgate stillunmet for
otherreasons. Genericring-weightattributionincludes87sharedinterfacevertices;
owner-faceaudits/exclusiveverticesneededbeforeanyreusablescorerevision.

H39 before fix: placement passes measuredmaximal-coordinate fingerposes into
anexactparallel COMMAND manifold validator. Finitecompliantjoints correctly
permit smalllateral/orientationerrors (allphysicalgatespass), so thesevalid
measurementsareinvalididealwaypoints. Usecurrentheldcommandposes as the
placementtrajectory start for mechanicalgrippers; preserve measuredobject
geometry/desiredbodyXY forplacementcalculation and allactualstate/qualitygates.
Nobodypose overwrite, constraintloosening, motor/materialchange orteleport.
Prediction: placementno longer skipped; controllercontinuesfromsamecommand
without a targetjump, physicallyopens/lowers/retractsunderexistingcaps. Test
complete450originaltask; no need a mirroringunit test for this smalladapterfix.

H39 interim: same requested configuration and all code identities except the
placement adapter match prior full450, but unordered physics reductions diverge
from frame1 and observed feedback amplifies that difference. New repeat has an
inverted body element at frame106, BEFORE placement runs at264. Do not attribute
that to the later command-start fix or accept recovery as clean mesh integrity.
The run continues solely to test actual placement/release, under unchanged gates.

H40 before experiment: the 40-substep/24-iteration condition is not robust to
small unordered-reduction/observed-feedback differences. Test local convergence
by increasing ONLY iterations24->48 at fixed40substeps, materials, contact law,
controller and coarse budget. This isolates iteration count from timestep-induced
changes in inertia/contact/history. Prediction: closure/transit retain positive
element Jacobians while the same contact-only task executes. Any inversion,
failed ring pickup or skipped placement rejects the candidate; a clean first
run alone does not establish repeatability. If unresolved, inspect that failure
before further changes; timestep refinement remains a distinct later check.
Config cord_mechanical_vbd_normal_tracked_40_48_config.json; no production
physics source changes for this experiment.

H39 completed450: command-start fix CONFIRMED within its narrow scope: place
executes at8.8s, opens, withdraws and releases, allcodehashesunchanged. True
release/staticreaction/drift0.357mm pass. Fulltask remainsFAIL: body465inverted
substeps(minJ−.1477), finalXY121.756mm despite bilateraltransit andcaps/
mechanism/penetrationpass. Not a placementprecision pass.

H41 diagnosis recorded before any placement-policy edit: the planner lowers
until ANY toy point reaches support. That point belongs to the tail; source
body remains airborne. At354 beforeopening bodymean[.65446,−.09525,.85914]
is~4mm fromdesiredXY[.65207,−.09836], bodyminimumZ=.81070 whiletailminimum
=.75070; tableFz=.210N versus1.227Nobjectweight. Afteropeningbodydrops~73mm
andtips: y−.106@366→−.241@378→−.21995settled. XYcorrectionworkedwhileheld;
releasebeforestablebody-supportcausesfinalmiss. Needboundedcontact-only
lowering/settling andobservedXYcorrectionafterweighttransfer, beforeopening.
No new scorethresholds, objectstatewrites,attachments ormaterialchanges.
H40 iteration-only run is underway first; production source remains frozen.

H42 before fix (replicated-backend build): the two-world finite mechanical
core check aborts in `_initialize_rigid_profile` before stepping. Confirmed on
the actual model: replicated source_zone/target_zone markers have positive
mass .0072kg and inv_mass138.8889 but KINEMATIC flags2. The six mechanism
bodies have DYNAMIC flags1 and masses .2/.05/.05kg per world. `dynamic=mass>0`
incorrectly includes kinematic environment markers. Evidence:
`outputs/debug/clawhauser_pick_place_quality/h42_model_body_diagnostic.json`.
Prediction: selecting positive-mass DYNAMIC bodies only restores the intended
one-FREE-assembly/world grouping; kinematic fixtures remain outside the coarse
translation block. Explicit PROXY/unsupported flag rejection, frozen topology,
FREE-tree guards and active-external-contact fallback must remain. Test build
and a bounded two-world action/reset/cap check, not a full pickup rerun.

H40 completed450: zero inverted/nonfinite substeps, minbodyJ.62137, recorded
plane165µm/maxbox-table0/caps/mechanism valid. Genuine pickup reached body
height.985m (~199mm lift) by169, then selectedapex slipped below12mmpads at170
(no inflatedboundary-padSAToverlap; forcescorrectlyzero); no retainedtransport.
This distinguishes true edgeescape from missedcollision. Iteration48 improved
integrity in this sample but is not an accepted task profile. Prelift carrier
bar-body contact reached2.65N and up to19°attitudelag under.1Nmcap, unlike
prior40/24runs with no barload; causality for eventualescape unproven.

H43 proposed before diagnostic: carrierbar-bodycontact duringclosure perturbs
the frame/pinch andseeds eventualedgeescape. Isolate by disabling ONLYbar-soft
collision in an output-only harness, retainingpadcontacts/bar-rigidcontacts,
sourceobject, inertia/actuation,40substeps/48iterations andallgates. This cannot
be an acceptedphysicalfix. Ifbarremovalimprovesretention withoutmeshfailure,
plan actualbarclearance/approachgeometry; ifnot, examinefriction/rotation and
finitepadgeometry before more solverparameter changes. No production collision
removal authorized by this diagnostic; originalphysicalscene must stillpass.
H42 result: CONFIRMED in its build scope. Minimal flag-aware grouping now
accepts two replicated kinematic markers/world while keeping all PROXY/unknown
flags rejected. Both worlds build/capture and step finite; actual mechanical
state and bounded native-VBD effort are available. Original failure log and
model arrays remain preserved. Ruff/format/syntax pass.
The expanded core check separately failed BEFORE masked reset: immediate full
reset matches initial state, but unordered native replay differs in toy-only
state (2steps:56.5um position/1.216mm/s velocity;12steps:11.1mm/.1275m/s).
Mechanism/control arrays are not listed as mismatches. This is not evidence of
masked reset contamination and is not repaired by widening a force tolerance.
No partial-reset next-step pass is claimed; deterministic core-only validation
or an explicitly measured unordered-replay envelope is needed next. Evidence:
`h42_native_two_world_core.json`, `h42_core_stages.json` under debug outputs.
H42 ordered core-only probe:40substeps/48iterations, two worlds, graphFalse,
solver/coarse orderedTrue and collision deterministicTrue. Finite state and
actual effort observations pass; core still fails narrowly ONLY in effort
arrays (44.504uN currentjaw effort,16.875uN peak), with no pose/rate mismatch
at1e-5. No tolerance was widened and no reset pass is claimed. This eager
profile is intentionally NOT a throughput result.
Additional reset/FK diagnostic confirms an independent exact-state mismatch:
fullSim.reset runs FK after entity reset, while partial mechanism reset copies
the builder's authored `initial_body_poses`. World0 jaws differ by−29.8023nm
and−59.6046nm inX (1–2float32ULPs); carrier unchanged, world1 bitwise unchanged.
AtKe1500 those offsets can yield44.7–89.4uN PD differences. A canonical FK
initial-body cache would remove this mismatch without moving neighboring
bodies or relaxing effort tolerances. No gripper source change yet. Evidence:
`h42_ordered_core_{config,result}.json`, `h42_ordered_core.log`, and
`h42_reset_fk_diagnostic.json` in debug outputs.
H42 reset follow-up before fix: canonical FK cache. Compiler `_start_pose`
runs before `_run_ready` for both single and replicated builds. The latter
invokes gripper.on_scene_ready, so the uncoupled main-pipeline state already
contains the canonical FK body transforms. `stepping_model` and WorldMap
spread indices address that replicated main model. Cache ONLY these canonical
transforms into initial_body_poses for uncoupled native VBD; leave authored
initial command poses, motor targets, model/joint/inertia properties unchanged.
Keep historical coupled paths outside this correction. Prediction: immediate
partial body reset becomes bitwise equal to full-reset FK and removes the
micro-newton next-step effort mismatch, without tolerance relaxation or global
FK during a masked reset. Rerun the identical ordered2world40/48core profile.

H43 deferred without running: finer timeline supplies an earlier causal
candidate. By89–91 genuinebilateralgrasp already exists (loads.34/.65, .73/1.16,
.99/1.19N), entireselectedcordsection fits pads at91, orientationlag.38deg/bar0.
Continuedfittedframechasing precedes bar97: command shifts8.9mmX/2.5mmY/−6.9mmZ
androtates9.8deg by96, then73mmY/34deg by108. Disablingbarcollision is a
lessdirectnexttest and wouldrequireoriginalphysicalcollisionrestorationanyway.

H44 before implementation/test: after sustained actualbilateralcontact,
continuingtofollowdeformationoftheheldmaterialframe cancreateunwantedgripper
motion andseededgeescape. Freeze existingobservedframe cache after3consecutive
controlobservations withbothactualringloads>.2N (notcontactrecordcounts), then
completeclosure/hold/frozenlift with unchangedcaps/materials/ALLcolliders.
Mutuallyexclusivewitholdfreeze_on_close at72. Firsttest264pickupONLY with
placement_feedback=False,40substeps/48iterations; logcontacttrigger and saved
geometry/forces. Prediction: nocontinuedpostcontactframechasing, nocarrier
interference, retainedringtransport andpositiveallsubstepJ. Noacceptancefrom
justobservingtrigger; strictphysical/taskchecksremain. H41placementfeedback
willbetestedlaterseparately; itscodeisopt-in soH44isolationpreserved.
H42 canonical reset result: GREEN on the SAME ordered uncaptured2world40/48
profile with2coresteps, no tolerance changes (1e-5). All7core checks pass:
full reset, replay, distinct world actions, masked toy+mechanism reset,
immediate bitwise neighbor preservation, neighbor next-step counterfactual,
and selected-world next step matching fresh full reset. Actual VBD jaw effort
availability/caps and carriage wrench caps pass. This confirms the canonical
FK reset-cache correction removes the narrow effort mismatch; no global FK
runs during masked reset, and authored command targets remain unchanged.
Evidence `h42_canonical_ordered_core_result.json` / `.log`. Scope is bounded
native-VBD reset/control plumbing under ordered eager execution, NOT pickup
acceptance, fast unordered repeatability, or throughput. Both production files
parallel_gripper.py and vbd_coarse.py are frozen; allocation handed back to root.

H44 completed264pickup: sustainedbilateraltransit/allsubstepintegrity/caps/
mechanism/penetration/carriercheckspass. Clearance163.3mm/lift244.1mm, minJ
[tail.9791,body.67575,loop.97488], no inverted/nonfinite substeps. Triggerframe90
afteractualbilateralloads88–90, noobjectstatewrites/allphysicalcollidersactive,
codechangedFalse. Firstclean48iterationpickup withcontactfreeze; fullplacement
stilluntested. BaroffdiagnosticH43remainsdeferred/unrun. Exactcausalrepeatability
underunorderedphysicsisnotclaimedfromthisonesample.

H45 beforetest: old24iterationfailureoccurredaftercontinuedpostcontactframe
chasing; nowtestONLY48->24iterations withH44contactfreeze retained, placement
feedbackFalse, allphysicalsettingsunchanged. This tests whetherimprovedcontrol
avoidstheclosureimpulsethatpreviouslyrequired48iterations andcanhalvebackend
cost. Needretainedringtransport, allsubstepJpositive andexistinggates; anyfailed
conditionrejectsthecandidate, noacceptancefromspeedalone. Higheriterationcase
isreference, not a materialchange. H41supportloadwindowfix is separateopt-in
code, inactiveinthispickupcomparison.

H45 completed264pickup with24iterations: all10560substepspositive/noinverted
or nonfinite elements, minbodyJ.65566(vs48reference.67575), bilateraltransit,
clearance197.8mm/lift275.1mm, carrier0/caps/mechanismvalid. Thisprofileisapickup
candidateonly, notfulltaskacceptance. Improvedgraspcontrolallows24iterations
inthiscase; sourceparametersremainraw.

H41 mean-supportgate review before physicaltest: originalinstantFz>=.9W
windowwouldrejectall33windows evenonH39settledstate(mean1.2375Nvs1.2269N
weight, instantaneous.812–2.025N). Correctedmean-loadhistoryindependentlyof
geometry/stabilitymembership; boundedgentleloadtransferandfresh.4sholdafter
commands. CPUcaseusingrealoscillatoryloadsequencepasses, sustainedunder/
overweightadjustsdown/up, isolatedpeakcannotopen. Independentread-onlyreview
findsnomaterialblocker; operationalposture/loadgateisnotformalstabilityproof.
FirstphysicalH41full900run enablesONLYplacement_feedback onH45profile24.
Allsource/collision/cap/integrity/taskgatesunchanged. Needactualsupportbefore
release, nofall/tiptargetmiss, realjawwithdrawal, finalXY<=20mm/drift<=5mm and
allsubstepintegrity. Timeout/floor/carrier/qualityfailuremustholdandFAIL.

H41 physical900 outcome: FAIL. Pickup/transitpassedagain, codeunchanged. First
savedinversion573 triggeredfailed_hold; carrier0 throughout. Tail-onlysupport
started441. BodybearingforcewasZEROforall310placementdecisions andbodymin
neverbelow.77746m, so releasegatecorrectlyrefused. Howeverbody_bearingFalse
bypassedgentleloadregulation: meanfloor>1.1weightfrom457, commandscontinued
120.5mmZdescentuntil572, reaching5.01Nmean(~4weights) andlargecarrierattitude
lag112deg. Firstbadbodytet4958 has4body-loopinterfacevertices/J−.947; later
ringlossandbodyimpactfollowedfailed_hold. Thisisaglobaloverloadguardblindspot,
notbarcollisionormissingcontact. Independentreviewpreviouslymissedthatbranch.

Tailposturewasnotfreestanding: massCOMoutsideALL132conservativesupporthulls
441–572 (minimum11.31mmgap;45727.19mm; even2mmplaneallowancenoinsidecases).
Meanweightcontactalonecannotjustifyrelease. Guard-onlywouldavoidcrushingbut
centroidfeedbackcanstabilizetheuprightcolumnwithoutlayingitdown.

H46/H47 stagedplacementfeature BEFOREimplementation: separateglobalcontact
loadrelief fromfinalbody-releaseeligibility. Afterfirstsustainedfloorweight
contact, hold/relievefor~1s withALL-contactmeanloadregulation independentof
bodylabel/posture; immediatehighloadpausesgrossmotion. Thenifbodyunsupported
guideexistingheldgraspSIDEWAYSinthedirectionfoot->wholemassCOMononebounded
~50degpivotarc; keepcommandquaternion/aperture/caps/allcollidersunchanged.
Thearcisaservercommand, NOTprescribedobjectmotion. DisablebodycentroidXY
regulationduringarc; stoponactualbodybearing, thenresumeobservedbodyXY
alignment andunchangedstablebody/load/releasegate. Useactualstaticcontact
centre/COMforarcgeometry, actual/commandclearanceguards andpauseontracking
lag/load. A50deg arcfromthe457geometryrequests~150mmlateral/~90mmlowering,
wellabovefloor, givingbodyachancetorestwithoutaxialtailcompression.

Bundlejustification: globalguardisrequiredtosafelytestanyplaceapproach;
measuredunsupportedCOMprovesthatguardalonecannotestablishrelease. ONErun
containsanexplicitguarded-holdstagebeforeconditionalarc, soeachmechanismcan
beevaluatedwithoutanotherfullpickup-onlyrerun. Recordstage/forcelimit/window/
tracking/postureobservables separately. Rawmaterials/forces/taskgatesunchanged;
maxsupport26splus3sreleaseafter8.8sstartfits1200frames/40s. Needpositive
ALLsubstepJ, boundedcontactload/servoerrorbeforebodylanding, actualstablebody
supportbeforeopen, finalXY<=20mm/drift<=5mm. Failedguard/timeoutstaysheldFAIL.

H45 provisional backend cost (GH2004491214; held initial targets, no cameras or
contact recording, same40/24physics, no task-success claim): completed1/8/32
worlds,4warm+20timedsteps per path. Actual CUDA capture/replay, no eager fallback;
all observed final states finite. Physics aggregate controlsteps/s2.561/10.402/
29.134; public identical-target update+sim.step path2.561/10.343/28.904. Batch
latency.390/.769/1.098s; Warp currentpool.206/1.533/6.020GB. Aggregate simulated
seconds/wallsecond.085/.347/.971: parallelism helps, but this does not establish
sufficient policy rollout speed. Built-in toy summary observesworld0, no embodiment
proprioception; complete per-world policy observation is NOT benchmarked. Evidence
`h45_fast_40_24_cost.json` / `.log`, exit0. No effort/integrity/fulltask acceptance
from this timing workload; existing focused core checks cover reset/effort plumbing.
After clean placement, test cheaper temporal/iteration settings against physical
quality before selecting defaults; raw materials unchanged.

H46/H47 frozen-code review before physicaltest: independent reviewer confirmed
lost-floor recovery deadlock. In pivot_arc, no_current_floor_support pauses
nominalangle as intended, but `negative admittance and any pause` also suppressed
the only bounded downward reacquisition motion. Correct only that condition:
permit underweight descent when no_current_floor_support is the sole pause;
tracking/overload and existing oriented-floor/apex guards still veto descent.
Nominalangle stays paused, no object-state writes. Gain.001 and upper-band-edge
relaxation may be slow; that is a physical-run performance risk, not a safety
blocker or reason to loosen support thresholds before evidence.
H46/H47 pre-run checks PASS: old stage/continuity/guard/release CPU cases plus
lost-floor nominal-angle pause / downward reacquisition / tracking-veto cases.
Independent review finds no further necessary blocker after this correction.
Production frozen policySHA0c9a9ca3a7b1936e8a10e26632e81e985f3a7ad803b826a9d71b920e19966d0b,
mainSHA4a5eb099d07a61769b13c13f824e0281b184d1aa94ab6c89ee4d22e475ac8280.
Original physical1200 run launched with unchanged40/24placeconfig/allcolliders/
rawmaterials/caps under4491214, run `cord_mechanical_vbd_contact_freeze_40_24_guarded_arc_full`.
No physics acceptance claimed from command-only checks. No source changes until
run completion; later verdict must separate guarded-hold, nominalarc and relief.

H46/H47 firstphysicalrun intentionally INTERRUPTED at447completedframes
(exit130) to avoid spending the full40s horizon on an unrelated approach stall.
Finally artifacts (states, diagnostics, phases, bothsubstepqualityJSONs) preserved;
NO manifest/result was generated after KeyboardInterrupt. SourceidentitySHA
matches pre-run frozen files. Mesh remains positive all17880substeps, minJ
[.93750,.52991,.97846], no nonfinite/inversions. Pickup/transport retained.
The guardedhold/arc mechanism is INCONCLUSIVE: never reached table contact.
Placement184observations mostlyapproach_paused/tracking_lag, floor0 throughout.

H48 BEFOREimplementation: per-pad positional tracking error includes expected
finite-force jaw compliance, so it incorrectly vetoes carriage translation.
At264 actualhalf-gap7.965mm versus commanded3.2mm; maxpaderror7.142mm but
midpointerror2.540mm and attitude1.27deg. At274 midpoint1.653mm vs6.237mmpad;
at447 midpoint1.418mm vs5.963mmpad. The5mm carriageguard is tripped by holding
a finite-width cord against bounded closure, not by correspondingcarriagelag.
Feature: measure carriage translation at the actual-v-command finger midpoint,
retain rawmaxpaderror as separateaperture/compliance diagnostic; preserve5mm
translation/3degattitude thresholds and allmotor/mechanism/integrity/support
checks. Sameoriginal1200physicalrun after CPUcheck symmetricjawopening isnot
carriagelag, while6mmcommontranslation still pauses; no material/force change.
Prediction: approaches thefloor instead of refusing descent, allowing actual
H46/H47guardedhold/arc test. Placement/taskfix still unresolved.
H48 independent offline decomposition CONFIRMEDmetricconfound:184observations,
182oldtrackingpauses, ZERO midpoint>5mm or attitude>3deg. Maxmidpoint3.3045mm
vs8.0283mmmaxpad; symmetricaxialgaperror4.744–4.797mm. Reconstructedrawpad
error agrees4.5nm. Saved `independent_tracking_decomposition.json` in earlyrun.
CPU oldcases+new symmetricjawcompliance/common6mmtranslationchecksPASS,
Ruff/formatPASS. PolicyfrozenSHA5a5c7ac9d0219a9b499aae9f29c9bc04d3fc16fa5c9600232c554392cfe31366,
mainunchanged. Sameoriginal1200run restarted as
`cord_mechanical_vbd_contact_freeze_40_24_midpoint_arc_full`; no placement
success claim until physicaloutcome. AIRBOTpolicy-seam audit saved separately,
implementation still follows pure-task acceptance.

H48 full1200 COMPLETE exit0/codechangedFalse. All48000substepspositive,
minJ[.91487,.54426,.96978], zeroinverted/nonfinite. Fullsurfacepenetration68.43um,
box/table0, overflowFalse, carrier0, actuator/mechanismgatesPASS; bilateraltransit,
clearance192.8mm/lift267.0mm. H48 confirmed carriageguard correction allows
approach. H46globalguard avoids old axialcrush; H47arc reachesactualbodybearing.
Stages: approach8.8–14.9, guardedhold14.9–15.9, pivotarc15.9–32.9667,
body_align32.9667–34.8, timeoutfailedhold. Arcstopped45.8229deg (bound50),
28pausedobservations (18floorreacquisition/9tracking/2instantload, overlap).
Only1.83salignment before26ssupportdeadline: XYerror56.1mm, meanfloor.4627N
vsweight1.2269N. Healthy alignment/loadtransfer stillinprogress; no release,
finalXY48.5mm =>taskFAIL. Policycorrectlyholds; sourceparametersunchanged.

H49 BEFOREimplementation: current26ssupporttimeout is shorter than this
conservative bounded approach+laydown+alignment sequence. Afterbodylanding,
56mm remainingXY alone needs>3s at16mm/s mean, plusloadtransfer/stability;
only1.83s wasavailable. Minimal intervention ONLYextend supportdeadline26→36s,
keep speeds/gain/angle/forces/materials/allgatesunchanged. Testsameoriginal
physicalpath with--steps1500(max50s). Alsostoploop ONLYafterexistingcomplete
release/.7swithdrawal/1.5ssettle path actuallyfinishes: this removesemptytail
and cannot changepriorphysicalcommands oracceptance. Necessarybundle isrun
budget/termination, notphysicsparameterchanges. Prediction: bodyalignmentand
loadtransfer continuepast34.8 into actualsupportedrelease; if theyinsteadstall,
H49 insufficient, useforce/motionevidence fornextchange. No gain retuning or
supportthresholdrelaxation. PositiveallsubstepJ/fulltaskgatesstillrequired.
H49 CPUcommandcases/Ruff/formatPASS. FrozenpolicySHAa974d8eefe364df83c4984127219336edd097eb9b98e4664b8bac597631be666,
mainSHA9d85282a41895f7510c35ac89f277764deb80aec4ea0f188bacdf89afcabca8c.
Originalphysical1500max run `cord_mechanical_vbd_contact_freeze_40_24_placement_budget_full`
started under4491214; previousH48recordingwall930.85s (richdiagnostics, NOTbare
backendtiming). Productionfilesfrozen until completion; noforces/gainschanged.

USER SCOPE REPLACEMENT 2026-10-27 10:14:57UTC: onlyONE complete standalone
Clawhauser/gripper cycle; no AIRBOT, materialoptimisation, batching/throughput,
policy/reset/generalinfrastructure orvisualpolish. Preservemesh/labels/materials/
forces/existingqualityandsuccessgates. Onlysmallplacement/trajectorychanges.
Deadline11:14:57UTC, maximum3full-cycletrials; currentH49alreadyrunning counts1.
OnfullpasssaveONEvideo/resultsandSTOP; onbudgetexpireSTOPwithprecisefailure
andminimalnextstep. Earlierbroaderplan isdeferred. Noadditionalresolvedpickup
investigations unlessfailureactuallyreproduces. ExistingCPUfastsuite already
inflight beforethisinstruction mayfinish; no newbroadwork initiated.
Trial1=cord_mechanical_vbd_contact_freeze_40_24_placement_budget_full, source/
profilefrozenasabove, 1500maxframes, 36ssupportbudget. Twofulltrialsremain.

TIMEBOX TRIAL1/H49 completed1500/50s, exit0, sourceunchanged. TaskFAILheldtimeout
44.8s, actualreleaseFalse; finalXY10.389mm. All60000substepspositive, minJ
[.92428,.39010,.96834], zero inversions/nonfinite; plane68.49um, box0, nooverflow,
carrier0, actuator/mechanismchecksPASS. Stages approach→14.7667 /guard→15.7667 /
arc→31.7333 /body_align→44.8. Alignmentreceived13.07s and reached9.05mmerror;
171aligned observations from38.9333,5.8667s, but meanfloorload only.10665–.19214N
(weight1.22689N), final.18865N/body-exclusiveFz0. AlignedcommandZ lowered3.004mm
in5.8667s; nooverload/trackingpauseatfailure. Thusremainingfailure isinsufficient
weighttransfer/bodybearing, notXYtimeout alone. Existinggatecorrectlyrefusesopen.

H50/TIMEBOX TRIAL2 BEFOREimplementation: placementloadadmittance gain.001 gives
only~.5mm/s mean descent at this near-weight error; realbody remains justabove
floorwhiletailcarries~15%weight. Test ONLYgain.001→.005 m/(N*s), existingpeak
10mm/s clamp unchanged. Predictedunderweightpeak~5mm/s (~2.5mm/s mean), enough
forseveralmillimetresofremainingbodylanding before same36ssupportdeadline.
Keepallcommands/contact-onlygrasp/rawmaterials/forces/solver/arc/successandquality
gates unchanged; no additionaltimeout increase andnoinfrastructure. Increased
upwardreliefuses samegain/limit; instantaneousoverload/trackingguardsstillveto
negativeadmittance. Sameoriginalmax1500fullcycle; thisis2of3trials. Ifitpasses,
saveONEvideo/results andSTOP; deadline11:14:57UTCstillbinding.
Trial2/H50 launched: `cord_mechanical_vbd_contact_freeze_40_24_load_transfer_full`,
max1500, same40/24config. PolicyfrozenSHAf6a39f04e0863bdccd6e5e0cd13a45bf2d74bbb97726962f335c57fbbff26a67,
mainunchangedSHA9d85282a41895f7510c35ac89f277764deb80aec4ea0f188bacdf89afcabca8c.
ExistingCPUcommandcases andRuff/formatPASS aftergain-onlyedit. AtmostONEfurther
fulltrial remains, absolute deadline11:14:57UTC; noexpandedworkallowed. Earlier
inflightfullfastsuite completed2426PASS/26skip167.064s; fullRuff756filesPASS.
IndependentTrial1reviewconfirms gainintervention matches observed.92–.99mm/s
peak/.5mm/smean transfer. Nuance: body-exclusivecontactwasINTERMITTENT(max.217N
inalignedwindow), notzeroentirewindow; finalsample0 andnoreleaseeligibility.
Fivefoldgainpredicts4.6–4.9mm/speak atsameerror, belowunchanged10mm/sbound.
Noall-substep/force/penetrationfailureinTrial1. AwaitTrial2existingartifacts only.

TIMEBOX TRIAL2/H50 COMPLETE1500/50s, exit0, sourceunchangedFalseflag. TaskFAIL
heldtimeout44.8; finalXY6.219mm/releasedFalse. All60000substepspositive, minJ
[.96447,.40419,.97792], zero inverted/nonfinite; maxplane68.49um/box0, nooverflow,
carrier0, actuator/mechanismchecksPASS. Body_align30.8333–44.8. Alignedfrom37.6,
7.2s/197observations; commandZ lowered11.336mm; meanfloor.22194→.99779N, final
.99579N versusrequiredlowerband1.10420N (weight1.22689N). Actualbodybearingat
final1.15491N, bodymean.78756/min.750698, speed/XY/trackingacceptable. Thusgain
increase materiallyimprovedbodylanding/weighttransfer withoutphysicalfailures,
but itstillapproached eligibilitytoo slowly; releasecorrectlyremainedblocked.
No interpretive drift score isusedwhenreleaseFalse. Wall1175.5s richrecording.

H51/TIMEBOX TRIAL3 (FINAL) BEFOREimplementation: useONLYsamegain.005→.020,
existing10mm/speakclamp unchanged. Measuredalignedloadrise~.776N/11.336mm
(~68N/m) supports a faster response; increasinggain4x reduces unsaturated
response timescale4x, while underweightdescentoftenjustusesexisting10mm/sbound.
Finaldeficittoeligibility.1084N wasstilldrivingonly.542mm/speak attimeout;
newgainrequests2.17mm/speak for that deficit. Keep36sdeadline/arc/forces/raw
materials/physics/alltaskgates unchanged. Existinginstantaneousoverload/tracking
vetoes andupwardrelief stayactive. Sameoriginalmax1500fullcycle. ThisisFINAL
trial3/3; hardstop11:14:57UTC; no furtherretry orscopeexpansion, evenonfailure.
Onfullpasssaveoneexisting-state30fpsvideo/results andSTOP. Ifnotpassedpreserve
exactfailureandminimalproposednextstep only.
Trial3/H51 launched10:53:42UTC, `cord_mechanical_vbd_contact_freeze_40_24_final_transfer_full`,
max1500/same40/24profile. PolicyfrozenSHA4fab9979bad3526b2871b539bf278bc71d284667a995006c062d12c034efea91,
main9d85282a41895f7510c35ac89f277764deb80aec4ea0f188bacdf89afcabca8c. Existing
CPUchecks/Ruff/formatPASS. No fourthtrial; nofileschangeduntilphysicalrunends.
At11:13:50UTC ifstillrunning withoutcompletecycle, interruptownpython tosave
finallyartifacts andfinishcleanup before11:14:57UTC. No hiddencontinuation.


## Standalone timebox outcome: stopped after three trials

Trial3/H51 completed1500frames/50s, exit0, code_changed_during_run=False.
Full task NOT PASSED: placement timed out44.8s; no release, withdrawal or
post-release settling occurred. All60000physics substeps positive, no inverted
or nonfinite tets; minJ tail/body/loop .90601/.42341/.95742. Existing effort,
mechanism, penetration and overflow checks allpass; carriertransportload0.
Maximumall-substepplane depth68.49um, gripper/table0. Contactgrasp/transport
retained; clearance188.2mm. FinalbodyXYerror7.055mm.
At release deadline: XYerror5.093mm and meanfloor1.16153N wereeligible,
bodyfloorcontactpresent, but centroidspeed29.159mm/s exceeded20mm/s. No
uninterrupted.4s quiet support window; corrections and speed fluctuations
prevented release. Longest candidate/no-correction sequence 4 observations.
This scoped work extended placement budget26→36s, exits onlyaftercomplete
release/withdraw/settle, and tested load gains.001/.005/.020 under unchanged
10mm/s bound. No mesh, label, material, forcecap, solver orqualitygatechanges.
Finaltestedgain.020 remains in source; it is NOT an accepted completecycle.
Smallest proposed next step (NOT implemented): insert a fixed-command
settling hold afteralignment andaveragedweighttransfer, then evaluate theSAME
unchanged stability/releasegate; resumeboundedrelief onlyasneeded forunsafe
loads. This directly tests whether continuousmicrocorrections excite residual
motion. No fourthtrial, no AIRBOT/performance/material/reset/infrastructurework.
Evidence: outputs/debug/clawhauser_pick_place_quality/cord_mechanical_vbd_contact_freeze_40_24_final_transfer_full/{result.json,manifest.json,phases.json,states.npz,quality_substeps.json,contact_quality_substeps.json}.
No successvideo generated because nofullpass. Fullfastsuite2426PASS/26skip;
Ruff/formatPASS. Taskstopped within60min and3trialbudget; broaderworkdeferred.
