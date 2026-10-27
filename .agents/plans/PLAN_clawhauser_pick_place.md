# Clawhauser labeled FEM pick and place

## Goal and scope

Build and verify an RSS scene in which a scripted parallel gripper picks the existing Clawhauser mesh by its loop, moves it to a second table location, releases it, and withdraws. The object is one connected tetrahedral volume with the existing part IDs 0 (tail), 1 (body), and 2 (loop/clasp). Every part remains FEM. The current source material estimate is the sole material choice; variants, fitting, and uncertainty ranges are deferred.

## Implementation

1. Bundle the current Clawhauser `.mesh`, per-tet parameter NPZ, source material JSON, and visual binding with the RSS example. Preserve source bytes and record provenance.
2. Add a reusable `HeterogeneousSoftSpec` and `EntityHeterogeneousSoft` in `robosimstudio_materials`. Load source mesh and arrays with strict identity/count checks. Correct inverted input winding in place while retaining tet order and labels. Pass per-tet Lamé arrays to Newton 1.8 and compute nodal mass from incident per-tet densities and volumes. Keep label/part data on the entity for tracking and later variants. No rigid conversion.
3. Stage an RSS arrangement that routes the soft object and gripper to Newton VBD on a GPU. Use the existing RSS scene/compiler/physics pipeline. Add a task-local scripted kinematic gripper with a contact grasp at the loop, table, and source/target markers. Keep any temporary attachment experiment clearly separate from the intended contact grasp.
4. Record state, contacts, mesh quality, phase and placement diagnostics. Render a short scene video or still sequence from the recorded RSS run. Document reproduction and source material limits.

## Physics and dependency

Use the local Newton `1.8.0.dev0` source at commit `d960ec3b8be23d7acac01c0ba10b8d04dca8b549`. RSS currently pins `newton[sim]>=1.7,<1.8`; adjust its dependency for this feature and record the exact source revision used in validation. The existing ARM64 Newton environment runs on an allocated GH200 node. Newton accepts per-tet elastic coefficients, but its mesh builder takes only scalar density, so explicitly replace nodal masses. The VBD contact interface has one global soft friction value; preserve each part's source friction as metadata and report the actual global contact setting.

## Acceptance and checks

- RSS builds and steps the labeled object, two gripper fingers, and a table on Newton 1.8; the object retains 1,840 vertices, 6,850 tetrahedra, and all three part IDs in source order.
- Built per-tet stiffness and density-derived masses agree with the source arrays. No implicit stiffness scale or Poisson clamp.
- A recorded run shows approach, ring grasp, lift, translation to a distinct target, lower, release and retreat. Evaluate success by ring/body position, loss of gripper support after release, table support, finite state, and inversion count. A video or representative frames show the scene.
- Run focused asset/build tests, RSS fast tier and a GPU end-to-end scene run. Diagnose failures and repair them; avoid unrelated test expansion.

## Decision notes

The earlier Newton demo supplies working mesh parsing and gripper motion patterns but changes the source elastic values and uses temporary particle-body attachments. It is reference code, not an accepted result. The source loop is elastomer in the supplied inference, not a separate metal ring. The initial scene is a simple table pick/place, not the prior stick-hanging task.

The first two-frame RSS smoke build reached the physics step, then spent over eight minutes compiling Newton's full-surface contact kernel. The existing Newton handoff records a stopped full-surface compilation experiment and a validated particle-contact support pilot. The first RSS trial therefore uses particle contact; this is a contact-generation choice and does not change mesh or material data. Revisit full-surface contact only if the contact grasp cannot hold with particles.

The first particle-contact smoke run confirmed 6,850 tets and source material arrays but reported zero table contacts: the object's world x was 1.07 m, beyond the bench's 0.90 m edge. The example had treated an intended world x=0.47 m as RSS's work-surface-relative x=0.47 m. Set source and target to -0.13 and +0.05 in the bench frame, respectively, and retest table support.

With corrected bench placement, a 30-frame probe produced 240-245 table/soft contacts, held the mesh near the 0.75 m table top, and had no inversions. The ring settled from its initial 0.788 m center to about 0.755 m while the fingers stayed at their original height, so closing alone did not touch it. Add a 27 mm finger approach before the closing phase, then run the full trajectory.

The full 159-frame contact-only run with the first approach had transient 42 finger contacts but no lift or placement. The object remained at the source and had no inversions. Tighten the closed finger half-gap to 10 mm, center the 12 mm-high pads just above the table, and hold the pinch briefly before lifting. Verify lift before spending another full run.

## Delivered implementation and validation

The bundled asset is one face-connected source mesh with 1,840 vertices, 6,850
tetrahedra, and source part labels 0/1/2. `HeterogeneousSoftSpec` is independent
of the other material packs. Its loader corrects source winding in memory,
checks the connected volume and label/material correspondence, and builds one
Newton FEM object with per-tet Lamé coefficients and density-derived nodal
masses. The RSS bench scene uses a scripted kinematic contact gripper with
small supporting ledges, a source zone and a target zone. No particle-body
attachment is present. The physical object is never moved by script.

The validated Newton 1.8.0.dev0 run at commit
`d960ec3b8be23d7acac01c0ba10b8d04dca8b549` used a GH200 GPU, Newton VBD,
30 Hz control, 20 substeps, and 12 VBD iterations per substep. Its 390 frames
show bilateral loop contact during transit, 102 mm table clearance, 186 mm
body translation, 5.7 mm target-x error, release with zero finger contacts,
and a table-supported body drifting only 1.6 mm over the final settling
window. Eight of the 6,850 tetrahedra inverted transiently at peak (0.12%);
all recovered and the settled interval had zero inversions. The result check
requires zero settled inversions and caps the reported transient fraction at
0.2%. It does not claim an inversion-free trajectory. The tiled renderer
shows the physics surface, but its colors do not clearly distinguish source
parts; labels in the model and recorded data are authoritative.

The final run, state arrays, per-frame diagnostics, result and video are in
`outputs/clawhauser/validated/`. Ruff lint and format checks pass. The RSS
fast tier passed 2,420 tests (26 skipped) in the supplemented Newton environment.
A full RSS `uv sync` on the Linux ARM
compute node is blocked by PyRoKi -> yourdfpy -> `trimesh[easy]` -> `embreex`,
whose current wheel does not support Linux ARM. The documented run uses the
existing Newton environment and RSS source checkout.
