# SPDX-License-Identifier: Apache-2.0
"""Owned-state reset for the pure Clawhauser scene, including replicated worlds.

Prepare once after scene construction, before CUDA capture. Call ``reset(world)``
between control steps, on the simulation stream. This restores the saved toy
state and the direct-control gripper in one world; global clocks and neighboring
worlds continue. The gripper may be free fingers or the native VBD parallel
mechanism. This is not an AIRBOT, coupled-solver, or general Sim reset API.
"""

from __future__ import annotations

import operator

import numpy as np
import warp as wp
from newton.solvers import SolverVBD

__all__ = ["prepare_reset"]


@wp.kernel
def _restore_particles(
    world: int,
    indices: wp.array2d[int],
    rest_q: wp.array2d[wp.vec3],
    rest_qd: wp.array2d[wp.vec3],
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    force: wp.array[wp.vec3],
):
    local = wp.tid()
    particle = indices[world, local]
    q[particle] = rest_q[world, local]
    qd[particle] = rest_qd[world, local]
    force[particle] = wp.vec3(0.0)


@wp.kernel
def _clear_finite_flag(world: int, flags: wp.array[int]):
    flags[world] = 0


class PickPlaceReset:
    """Cached reset data for one fused toy and its direct-control gripper.

    The cached particle state comes from the entity's ``snapshot_rest`` record,
    including any validated physical settling done before that snapshot. Call
    ``prepare_reset`` again if the rest state, model layout, or entities change.
    The gripper's public reset cancels user trajectory playback when its spec
    has ``scripted=False``. Particle rest data, index maps and solver masks are
    allocated only during preparation.
    """

    def __init__(self, sim, obj, grip):
        pipeline = sim.pipeline
        program = pipeline.program
        if program.coupled or not isinstance(program.solver, SolverVBD):
            raise ValueError("partial Clawhauser reset requires the pure scene's uncoupled VBD solver")
        if getattr(obj.spec, "entity", None) != "heterogeneous_soft":
            raise ValueError("expected the labelled heterogeneous toy entity")
        kind = getattr(grip.spec, "entity", None)
        supported_gripper = kind == "scripted_loop_gripper" or (
            kind == "parallel_loop_gripper" and grip.spec.motor_backend == "vbd"
        )
        if not supported_gripper or grip.spec.scripted:
            raise ValueError("partial reset requires direct-control free fingers or the native VBD parallel gripper")
        if program.collision is None or pipeline.collide_every != 1:
            raise ValueError("partial reset requires fresh collision detection every physics substep")
        if not obj.rest or "particle_q" not in obj.rest or "particle_qd" not in obj.rest:
            raise ValueError("the toy must have a saved rest state before preparing reset")
        if obj.worlds is not grip.worlds:
            raise ValueError("toy and gripper must share the scene's WorldMap")
        worlds = obj.worlds
        count = int(sim.num_worlds)
        indices = (
            worlds.spread_particles(obj.particles)
            if worlds is not None
            else np.asarray([obj.particles], dtype=np.int32)
        )
        if indices.shape != (count, len(obj.particles)) or grip.num_worlds != count:
            raise ValueError("toy/gripper world layout does not match the simulation")
        q = np.asarray(obj.rest["particle_q"], dtype=np.float32)[indices]
        qd = np.asarray(obj.rest["particle_qd"], dtype=np.float32)[indices]
        if not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise ValueError("cannot restore a non-finite toy rest state")
        device = sim.model.device
        self.sim, self.obj, self.grip = sim, obj, grip
        self.indices = wp.array(indices, dtype=int, device=device)
        self.rest_q = wp.array(q, dtype=wp.vec3, device=device)
        self.rest_qd = wp.array(qd, dtype=wp.vec3, device=device)
        self.world_count = count
        self.particle_count = len(obj.particles)
        self.masks = []
        particle_world = sim.model.particle_world.numpy()
        for world in range(count):
            owned_worlds = np.unique(particle_world[indices[world]])
            if len(owned_worlds) != 1:
                raise ValueError("the owned toy particles must belong to exactly one world")
            model_world = int(owned_worlds[0])
            if model_world < 0 and count != 1:
                raise ValueError("replicated reset cannot own global particles")
            if model_world >= 0 and model_world != world:
                raise ValueError("particle ownership does not match the WorldMap")
            # Newton's canonical mask includes an extra slot for global world -1.
            mask = np.zeros(int(sim.model.world_count) + 1, dtype=bool)
            mask[model_world if model_world >= 0 else -1] = True
            self.masks.append(wp.array(mask, dtype=wp.bool, device=device))

    def reset(self, world: int):
        """Restore only this world's toy/gripper; the next step rebuilds contacts.

        This queues device work without synchronizing. Existing contact records
        describe the previous state until the next physics step, so do not read
        them as reset-state contacts. This helper does not reset task counters,
        global clocks, other entities, or hooks owned by a policy.
        """
        world = operator.index(world)
        if not 0 <= world < self.world_count:
            raise IndexError(f"world {world} outside {self.world_count} reset worlds")
        pipeline = self.sim.pipeline
        mask = self.masks[world]
        pipeline.program.reset(pipeline.state_0, flags=0, world_mask=mask)
        pipeline.program.collision.reset_contact_matching(world_mask=mask)
        for state in (pipeline.state_0, pipeline.state_1):
            wp.launch(
                _restore_particles,
                dim=self.particle_count,
                inputs=[
                    world,
                    self.indices,
                    self.rest_q,
                    self.rest_qd,
                    state.particle_q,
                    state.particle_qd,
                    state.particle_f,
                ],
                device=self.sim.model.device,
            )
        self.grip.reset(pipeline, self.sim.episode, world=world)
        if pipeline.readback is not None:
            wp.launch(
                _clear_finite_flag,
                dim=1,
                inputs=[world, pipeline.readback.flags],
                device=self.sim.model.device,
            )
        # The clock deliberately stays unchanged, so invalidate this world's
        # step-keyed particle summary before an immediate reset observation.
        probe = self.obj.probe_for(world)
        if probe is not None:
            probe.step = None
        pipeline.mark_dirty()


def prepare_reset(sim, obj, grip):
    """Cache ``obj.entity``/``grip.entity`` reset data once before capture."""
    return PickPlaceReset(sim, obj, grip)
