# RoboSimStudio integration and validation

Use this reference for repository-specific decisions. Recheck the current source
and applicable `AGENTS.md`; names and behavior here can change.

## Ownership and extension points

- `robosimstudio/` owns scene construction, simulation, entity registration,
  physics composition, state access, and rendering contracts.
- `robosimstudio_materials/` implements deformable kinds through that core API.
  It imports the core rather than the studio or policy packages.
- `robosimstudio_studio/` provides the interactive scene viewer.
- `robosimstudio_real2sim_agent/` provides scene-file reload and reference checks.
- `robosimstudio_bench/` owns benchmark tasks and scoring;
  `robosimstudio_policy_agent/` owns graph policies and robot tools.

`rss.Scene.add()` collects specifications; `Scene.build()` compiles entities into
Newton and assembles the physics pipeline. Custom entities can build directly into
the Newton builder. Inspect `robosimstudio/scene/entity.py`, the registry, existing
material entities, and `examples/04_extending/` before changing core interfaces.
Keep an object's source representation and part labels independent of the entity
that realizes them in a simulator. Use native constructors where they serve the
requested model; extend the implementation where they do not.

## Physics and data integrity

The generic arrangement currently routes the `soft` domain to SuperDex. A request
for Newton deformable simulation therefore needs explicit routing and confirmation
of the actual solver. Check CPU adaptation too: a requested arrangement may be
adapted to different engines depending on the machine.

Choose library versions for the required physics capabilities. Inspect the actual
installed source and relevant prior experiments; do not downgrade or disable a
requested capability merely to preserve an existing dependency pin. Record exact
versions and source commits, and validate RSS compatibility after dependency changes.

Respect the user-selected asset representation. For labelled tetrahedral assets,
preserve element identity and material correspondence through orientation changes,
assembly, reset, rendering, and tracking. Compute nodal mass from incident element
densities and volumes. Keep source material estimates distinct from solver settings
and explicitly requested material overrides. Do not silently change stiffness or
Poisson ratio to make a scene succeed.

Check units and frames: RSS specifications generally use the cell's work-surface
frame, while state readback is in world coordinates. Track the transform explicitly.
For manipulation, distinguish contact forces from grasp attachments, pins, or
prescribed object motion. Validate the interaction model actually requested and
record any diagnostic assistance separately.

## Running and checking work

Use the repository's `uv` environment and current lockfile or a documented,
task-specific dependency override. Reuse suitable existing environments without
modifying unrelated ones. On a cluster, use an allocated compute node for GPU work;
do not mistake a login-node device listing for permission to run CUDA there.
Save generated runs, videos, timing, and diagnostics under `outputs/`.

Follow current `AGENTS.md` check requirements. The canonical entry point for test
tiers is `uv run python scripts/testrun.py --tier <tier>`; CPU and medium tiers need
the extras and hardware described there. Read the current instructions for lint
and formatting versions instead of maintaining a second dependency policy here.
These skill files are documentation changes: validate their structure and workflow
behavior without installing the simulator or running unrelated GPU suites.

For library changes, test the changed behavior and required regression checks.
For an interactive photo/video reconstruction, follow `examples/live/AGENTS.md`
and its live-page workflow. That workflow does not turn every library or offline
simulation feature into a reference-video reconstruction task.

Separate numerical correctness, rendering, and throughput measurements. Report
compilation/setup time separately from warm simulation throughput, and identify
whether rendering and readback are included. Preserve useful numerical evidence
when only media generation fails.
