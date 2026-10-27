# SPDX-License-Identifier: Apache-2.0
"""Contact diagnostics for the single-volume examples, including proxy coupling."""

from dataclasses import dataclass
from functools import partial

import numpy as np


@dataclass
class ContactRecording:
    sim: object
    solver: object
    model: object
    contacts: object
    observables: object
    island: str | None

    def state(self):
        if self.island is None:
            return self.sim.pipeline.state_0
        return self.sim.pipeline.program.solver.entry_state(self.island)

    def proxy_alignment(self, gripper, authoritative_body_q):
        """Compare actual pad corners, respecting entry-local body indices."""
        if self.island is None:
            return {"proxy_finger_corner_error_m": [0.0, 0.0]}
        from cord_grasp import _rotation

        shapes = [side[0] for side in gripper.shapes_by_side]
        local_bodies = self.model.shape_body.numpy()[shapes]
        if np.any(local_bodies < 0):
            raise RuntimeError("a dynamic pad is missing from the soft proxy view")
        proxy = self.state().body_q.numpy()[local_bodies]
        actual = authoritative_body_q[gripper.bodies]
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        corners = corners * np.array((0.003, gripper.spec.pad_half_width, 0.006))
        errors = []
        for original, mirrored in zip(actual, proxy, strict=True):
            a = corners @ _rotation(original[3:]).T + original[:3]
            b = corners @ _rotation(mirrored[3:]).T + mirrored[:3]
            errors.append(float(np.linalg.norm(a - b, axis=1).max()))
        return {"proxy_finger_corner_error_m": errors}


def prepare_contact_recording(sim, object_name, gripper):
    """Observe the contacts actually consumed by VBD, then let the caller reset.

    A proxy channel detects contacts internally. Its parent contact buffer is
    not a substitute. One public eager substep block discovers the persistent
    buffer passed into the soft solver; no private coupling storage is read.
    This is diagnostic setup, excluded from rollout timing. The caller must
    reset the whole scene before recording its task.
    """
    program = sim.pipeline.program
    solver = program.solver_for(object_name)
    island = None
    model = sim.model
    contacts = sim.pipeline.contacts
    if hasattr(gripper, "motor_observables"):
        motor_solver = program.solver_for(gripper.spec.name)
        motor_solver.step = partial(motor_solver.step, observables=gripper.motor_observables)
    if program.coupled:
        island = program.island_of(object_name).name
        model = program.solver.view(island)
        original_step = solver.step
        found = []

        def discover(state_in, state_out, control, actual_contacts, dt, **kwargs):
            if actual_contacts is None:
                raise RuntimeError("the soft solver received no contact buffer")
            if found and actual_contacts is not found[0]:
                raise RuntimeError("contact recording requires one persistent soft contact buffer")
            if not found:
                found.append(actual_contacts)
            return original_step(state_in, state_out, control, actual_contacts, dt, **kwargs)

        solver.step = discover
        try:
            sim.pipeline.substep_block()
            sim.sync()
        finally:
            solver.step = original_step
        if not found:
            raise RuntimeError("the soft solver did not execute during contact discovery")
        contacts = found[0]
    observables = solver.observables({solver.ObservableKind.CONTACT_F}, contacts=contacts)
    solver.step = partial(solver.step, observables=observables)
    sim.pipeline.invalidate_graph()
    return ContactRecording(sim, solver, model, contacts, observables, island)
