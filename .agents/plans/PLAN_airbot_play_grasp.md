# AIRBOT Play grasp in the Clawhauser RSS scene

## Target

Keep the original labelled tetrahedral Clawhauser asset, source stiffness and density, Newton 1.8 VBD material path, and contact-only grasp. Add an AIRBOT Play articulated arm and parallel jaw hand from DISCOVERSE. Record smooth 30 fps video. The arm should be the moving manipulator, not static scenery.

## Source and scope

- Existing scene: `examples/10_heterogeneous/clawhauser_pick_place.py`; 30 fps baseline output `outputs/clawhauser/smooth_30fps` is already complete and passes its existing numerical check.
- AIRBOT model: DISCOVERSE commit `d67f47c084aba0e0cf422a8725235f8b9238655a`, `models/mjcf/manipulator/new_airbot_play/mjx_airbot_play.xml` and its OBJ meshes, MIT. Vendor the model with attribution and minimal edits: fix mesh path, make both fingers direct position drives, and use collision-only finger adapters only if needed for a reliable contact pickup.
- RSS owns robot import, IK, kinematic articulated simulation and one-way contact coupling; VBD continues to own the deformable. No change to the object representation or material map.

## Steps

1. Vendor the AIRBOT Play MJCF and mesh dependencies with license/provenance. Validate with MuJoCo parser.
2. Build an RSS scene variant with AIRBOT Play, VBD soft-body domain, kinematic articulated domain and one-way coupling. Confirm imported joint names, finger shapes, solver composition and unchanged tet count/materials.
3. Compute reachable waypoints for a ring pickup, arm lift, horizontal transport and release. Drive real arm and finger joints from the task script; retain the scripted-finger baseline as a regression example.
4. Run on a GPU node. Save states, per-frame contact and pose diagnostics, success summary, and 30 fps video. Tune approach/contact only from evidence; do not attach or teleport the object.
5. Validate object translation, lift, bilateral ring contacts, release, table support and settled mesh integrity. Run relevant checks; obtain independent review.

## Acceptance and limits

Baseline smooth video must report 30 fps and preserve its passing numerical result. AIRBOT variant must build and visibly move the articulated arm. A successful pickup requires numerical evidence of arm-controlled contact transport and release; if that remains unsolved, report the AIRBOT stage as partial, with video and diagnostics, rather than claiming completion. Do not change source elastic parameters or substitute a rigid object. No commit or push.

## Implementation and validation status (2026-10-26)

- The baseline 390-frame, 30 fps video is in `outputs/clawhauser/smooth_30fps/overview.mp4`; its original pick/place result remains successful.
- The DISCOVERSE AIRBOT Play model is vendored under `examples/10_heterogeneous/assets/airbot_play/` with its MIT license and provenance. Both finger joints have direct position drives; task-specific pad and shelf contact shapes have matching visible shapes. The six arm joints and original visual meshes are preserved.
- `airbot_play_pick_place.py` imports the arm through RSS, routes it through a kinematic articulated island and one-way coupling to Clawhauser's Newton 1.8 VBD island, verifies the source tetrahedron order, mass and elasticity, plans reachable poses and records per-frame pose/contact/force/inversion diagnostics at 30 fps.
- The final 390-frame, 30 fps visible-adapter trial is `outputs/clawhauser/airbot_play_final/`. It reached the source and target poses but did not pick up the toy: bilateral loop contact appeared during closure, the ring rose only a few millimetres, and it slipped as the hand lifted. Its result reports 0.207 m target error, no sustained object lift, no settled inversions and `success: false`.
- Tested variants: the native AIRBOT pads, a contact shelf, wider/stronger closure, a Cartesian lift path, increased contact friction and a shared VBD arm/soft solver. The shared VBD arm drifted immediately under its existing gains, so the kinematic arm remains the supported option. No particle attachment or prescribed object motion was used.
- Independent review found and we fixed an airborne false positive in the table support predicate, and added visible shapes for the modified contact pads. No commit or push.
- The AIRBOT trial records all 390 rendered frames and per-frame aggregate diagnostics, but omits full particle state arrays while the contact pickup is unresolved. The validated floating-finger baseline retains `states.npz` for full mesh replay.
