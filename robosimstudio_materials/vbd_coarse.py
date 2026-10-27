# SPDX-License-Identifier: Apache-2.0
"""Optional VBD coarse translation solve for a complete labelled tetrahedral volume.

Private compatibility surface: Newton 1.8.0.dev0, commit d960ec3b8be23d7acac01c0ba10b8d04dca8b549.
This module intentionally reuses that revision's rigid-soft force/Hessian and
contact-eligibility functions, plus VBD's inertial target, particle anchors and
accumulated-displacement storage. Revalidate these contracts before changing
Newton versions. The stock Newton solver and RSS core are not modified.

Use the RSS dotted backend path
``robosimstudio_materials.vbd_coarse.SolverVBDCoarse`` with
``coarse_translation=True`` to opt in. Default behavior is ordinary VBD.
"""

from __future__ import annotations

import numpy as np
import warp as wp
from newton import BodyFlags, JointType, ModelFlags
from newton._src.geometry.soft_contacts_mesh import (
    _make_soft_contact_filter_data,
    _soft_contact_force_enabled,
    _SoftContactFilterData,
)
from newton._src.sim.contacts import contact_surface_separation
from newton._src.solvers.vbd.rigid_vbd_kernels import (
    _eval_body_particle_contact,
    _eval_soft_ef_contact,
    _evaluate_body_body_contact_slot,
    ldlt6_solve,
)
from newton.solvers import SolverVBD

__all__ = ["SolverVBDCoarse"]


@wp.kernel
def _check_contact_capacity(count: wp.array[int], capacity: int, invalid_world: wp.array[int]):
    w = wp.tid()
    # Overflow means omitted contacts may belong to any world, including one
    # with no eligible stored record. Never infer coverage from the prefix.
    if count[0] > capacity:
        invalid_world[w] = 1


@wp.kernel
def _clear_masked_diagnostics(
    mask: wp.array[wp.bool],
    mask_indices: wp.array[int],
    path_length: wp.array[float],
    applications: wp.array[int],
    shift: wp.array[wp.vec3],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    invalid_world: wp.array[int],
):
    w = wp.tid()
    if mask[mask_indices[w]]:
        path_length[w] = 0.0
        applications[w] = 0
        shift[w] = wp.vec3(0.0)
        residual[w] = wp.vec3(0.0)
        hessian[w] = wp.mat33(0.0)
        invalid_world[w] = 0


@wp.kernel
def _inertia_block(
    dt: float,
    q: wp.array[wp.vec3],
    inertia: wp.array[wp.vec3],
    mass: wp.array[float],
    world: wp.array[int],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
):
    p = wp.tid()
    w = world[p]
    stiffness = mass[p] / (dt * dt)
    wp.atomic_add(residual, w, stiffness * (inertia[p] - q[p]))
    wp.atomic_add(hessian, w, stiffness * wp.identity(n=3, dtype=float))


@wp.kernel
def _contact_block(
    dt: float,
    q: wp.array[wp.vec3],
    previous: wp.array[wp.vec3],
    world: wp.array[int],
    radius: wp.array[float],
    indices: wp.array[wp.vec3i],
    count: wp.array[int],
    capacity: int,
    barycentric: wp.array[wp.vec3],
    contact_filter: _SoftContactFilterData,
    ke: wp.array[float],
    kd: wp.array[float],
    mu: wp.array[float],
    friction_epsilon: float,
    use_log_barrier: bool,
    shape_body: wp.array[int],
    body_q: wp.array[wp.transform],
    body_previous: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_com: wp.array[wp.vec3],
    shape: wp.array[int],
    body_pos: wp.array[wp.vec3],
    body_velocity: wp.array[wp.vec3],
    normal: wp.array[wp.vec3],
    margin: wp.array[float],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    invalid_world: wp.array[int],
    rigid_enabled: bool,
    body_groups: wp.array[int],
    rigid_residual: wp.array[wp.vec3],
    rigid_hessian: wp.array[wp.mat33],
    cross_hessian: wp.array[wp.mat33],
    rotation_enabled: bool,
    centers: wp.array[wp.vec3],
    radius_scale: wp.array[float],
    angular_residual: wp.array[wp.vec3],
    angular_hessian: wp.array[wp.mat33],
    angular_linear_hessian: wp.array[wp.mat33],
    rotation_contact_active: wp.array[int],
):
    c = wp.tid()
    if c >= wp.min(count[0], capacity) or not _soft_contact_force_enabled(c, contact_filter):
        return
    corners = indices[c]
    if corners[0] < 0:
        return
    w = world[corners[0]]
    if count[0] > capacity:
        invalid_world[w] = 1
        return
    f = wp.vec3(0.0)
    h = wp.mat33(0.0)
    weight_sum = float(1.0)
    if corners[1] < 0:
        p = corners[0]
        f, h = _eval_body_particle_contact(
            p,
            q[p],
            previous[p],
            c,
            ke[c],
            kd[c],
            mu[c],
            friction_epsilon,
            radius,
            shape_body,
            body_q,
            body_previous,
            body_qd,
            body_com,
            shape,
            body_pos,
            body_velocity,
            normal,
            margin,
            dt,
            use_log_barrier,
        )
    else:
        weights = barycentric[c]
        weight_sum = float(0.0)
        for k in range(3):
            p = corners[k]
            if p >= 0:
                if world[p] != w:
                    invalid_world[w] = 1
                    return
                weight_sum += weights[k]
        f, h, _point = _eval_soft_ef_contact(
            c,
            corners,
            weights,
            q,
            previous,
            radius,
            ke[c],
            kd[c],
            mu[c],
            friction_epsilon,
            shape_body,
            body_q,
            body_previous,
            body_qd,
            body_com,
            shape,
            body_pos,
            body_velocity,
            normal,
            margin,
            dt,
            use_log_barrier,
        )
    # Whole-object translation moves every corner together. Include this
    # contact Hessian ONCE, with (sum bary)^2, not sum of squared barycentrics.
    wp.atomic_add(residual, w, weight_sum * f)
    wp.atomic_add(hessian, w, weight_sum * weight_sum * h)
    if rotation_enabled:
        if radius_scale[w] <= 1.0e-8:
            invalid_world[w] = 1
            return
        lever = q[corners[0]] - centers[w]
        if corners[1] >= 0:
            lever = wp.vec3()
            for k in range(3):
                if corners[k] >= 0:
                    lever += barycentric[c][k] * (q[corners[k]] - centers[w])
        G = wp.skew(lever / radius_scale[w])
        wp.atomic_add(angular_residual, w, G * f)
        wp.atomic_add(angular_hessian, w, G * h * wp.transpose(G))
        wp.atomic_add(angular_linear_hessian, w, weight_sum * G * h)
        if shape_body[shape[c]] >= 0 and wp.trace(wp.transpose(h) * h) > 0.0:
            rotation_contact_active[w] = 1
    if rigid_enabled:
        b = shape_body[shape[c]]
        if b >= 0 and body_groups[b] >= 0:
            if body_groups[b] != w:
                invalid_world[w] = 1
                invalid_world[body_groups[b]] = 1
                return
            wp.atomic_add(rigid_residual, w, -f)
            wp.atomic_add(rigid_hessian, w, h)
            wp.atomic_add(cross_hessian, w, -weight_sum * h)


@wp.kernel
def _solve_block(
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    invalid_world: wp.array[int],
    budget: float,
    max_shift: float,
    relaxation: float,
    path_length: wp.array[float],
    shift: wp.array[wp.vec3],
    applications: wp.array[int],
):
    w = wp.tid()
    shift[w] = wp.vec3(0.0)
    if invalid_world[w] != 0:
        return
    step = relaxation * (wp.inverse(hessian[w]) * residual[w])
    length = wp.length(step)
    remaining = wp.max(0.0, wp.min(max_shift, budget - path_length[w]))
    if length > remaining and length > 0.0:
        step *= remaining / length
    if wp.isfinite(step[0]) and wp.isfinite(step[1]) and wp.isfinite(step[2]):
        shift[w] = step
        path_length[w] += wp.length(step)
        if wp.length(step) > 0.0:
            applications[w] += 1


@wp.kernel
def _apply_block(
    world: wp.array[int],
    shift: wp.array[wp.vec3],
    reference: wp.array[wp.vec3],
    q: wp.array[wp.vec3],
    displacement: wp.array[wp.vec3],
):
    p = wp.tid()
    delta = shift[world[p]]
    # One writer owns each particle. Array += lowers to atomic_add, whose
    # deterministic reduction can run after this kernel's read of displacement.
    # Use the local value for both outputs so this iterate moves immediately.
    updated = displacement[p] + delta
    displacement[p] = updated
    q[p] = reference[p] + updated


@wp.kernel
def _clear_rigid_diagnostics(
    mask: wp.array[wp.bool],
    indices: wp.array[int],
    path: wp.array[float],
    relative: wp.array[float],
    shift: wp.array[wp.vec3],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    cross: wp.array[wp.mat33],
    applications: wp.array[int],
    fallback: wp.array[int],
):
    w = wp.tid()
    if not mask or mask[indices[w]]:
        path[w] = 0.0
        relative[w] = 0.0
        shift[w] = wp.vec3()
        residual[w] = wp.vec3()
        hessian[w] = wp.mat33()
        cross[w] = wp.mat33()
        applications[w] = 0
        fallback[w] = 0


@wp.kernel
def _rigid_inertia_block(
    dt: float,
    groups: wp.array[int],
    q: wp.array[wp.transform],
    inertia: wp.array[wp.transform],
    com: wp.array[wp.vec3],
    mass: wp.array[float],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
):
    b = wp.tid()
    w = groups[b]
    if w < 0:
        return
    stiffness = mass[b] / (dt * dt)
    error = wp.transform_point(inertia[b], com[b]) - wp.transform_point(q[b], com[b])
    wp.atomic_add(residual, w, stiffness * error)
    wp.atomic_add(hessian, w, stiffness * wp.identity(3, float))


@wp.kernel
def _guard_external_rigid_contacts(
    dt: float,
    count: wp.array[int],
    capacity: int,
    groups: wp.array[int],
    shape0: wp.array[int],
    shape1: wp.array[int],
    point0: wp.array[wp.vec3],
    point1: wp.array[wp.vec3],
    surface_velocity: wp.array[wp.vec3],
    offset0: wp.array[wp.vec3],
    offset1: wp.array[wp.vec3],
    normal: wp.array[wp.vec3],
    margin0: wp.array[float],
    margin1: wp.array[float],
    shape_body: wp.array[int],
    q: wp.array[wp.transform],
    previous: wp.array[wp.transform],
    com: wp.array[wp.vec3],
    penalty: wp.array[float],
    rho: wp.array[float],
    ke: wp.array[float],
    kd: wp.array[float],
    mu: wp.array[float],
    tangent_rho: wp.array[float],
    lam: wp.array[wp.vec3],
    C0: wp.array[wp.vec3],
    alpha: float,
    hard: int,
    alm: int,
    epsilon: float,
    fallback: wp.array[int],
):
    c = wp.tid()
    if count[0] > capacity:
        # A truncated record might involve any world.
        for w in range(fallback.shape[0]):
            fallback[w] = 1
        return
    if c >= count[0] or shape0[c] < 0 or shape1[c] < 0:
        return
    b0, b1 = shape_body[shape0[c]], shape_body[shape1[c]]
    w0, w1 = int(-1), int(-1)
    if b0 >= 0:
        w0 = groups[b0]
    if b1 >= 0:
        w1 = groups[b1]
    if w0 == w1 or (w0 < 0 and w1 < 0):
        return
    _b0, _b1, _p0, _p1, f0, _t0, f1 = _evaluate_body_body_contact_slot(
        c,
        dt,
        shape0,
        shape1,
        point0,
        point1,
        surface_velocity,
        offset0,
        offset1,
        normal,
        margin0,
        margin1,
        shape_body,
        q,
        previous,
        com,
        penalty,
        rho,
        ke,
        kd,
        mu,
        tangent_rho,
        lam,
        C0,
        alpha,
        hard,
        alm,
        epsilon,
    )
    # Match native skeleton-point separation: offsets are already represented
    # by the combined margins, so do not subtract them twice. Guard touching
    # contacts too: zero elastic force can still have an active normal Hessian.
    cp0 = wp.transform_point(q[b0], point0[c]) if b0 >= 0 else point0[c]
    cp1 = wp.transform_point(q[b1], point1[c]) if b1 >= 0 else point1[c]
    gap = contact_surface_separation(cp0, cp1, normal[c], margin0[c], margin1[c])
    if (
        not wp.isfinite(gap)
        or gap <= 0.0
        or not wp.isfinite(wp.length(f0))
        or not wp.isfinite(wp.length(f1))
        or wp.dot(f0, f0) > 0.0
        or wp.dot(f1, f1) > 0.0
    ):
        if w0 >= 0:
            wp.atomic_max(fallback, w0, 1)
        if w1 >= 0:
            wp.atomic_max(fallback, w1, 1)


@wp.func
def _finite_vector(v: wp.vec3):
    return wp.isfinite(v[0]) and wp.isfinite(v[1]) and wp.isfinite(v[2])


@wp.func
def _finite_matrix(m: wp.mat33):
    valid = True
    for i in range(3):
        for j in range(3):
            valid = valid and wp.isfinite(m[i, j])
    return valid


@wp.kernel
def _solve_coupled_block(
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    rigid_residual: wp.array[wp.vec3],
    rigid_hessian: wp.array[wp.mat33],
    cross_hessian: wp.array[wp.mat33],
    invalid: wp.array[int],
    fallback: wp.array[int],
    budget: float,
    relative_budget: float,
    path: wp.array[float],
    rigid_path: wp.array[float],
    relative_path: wp.array[float],
    shift: wp.array[wp.vec3],
    rigid_shift: wp.array[wp.vec3],
    applications: wp.array[int],
    rigid_applications: wp.array[int],
):
    w = wp.tid()
    shift[w] = wp.vec3()
    rigid_shift[w] = wp.vec3()
    if invalid[w] != 0:
        return
    ro, rg = residual[w], rigid_residual[w]
    A, D, B = hessian[w], rigid_hessian[w], cross_hessian[w]
    do, dg = wp.vec3(), wp.vec3()
    coupled = fallback[w] == 0
    if coupled:
        if _finite_vector(ro) and _finite_vector(rg) and _finite_matrix(A) and _finite_matrix(D) and _finite_matrix(B):
            do, dg = ldlt6_solve(A, D, B, ro, rg)
            # Positive mass and native PSD contact blocks make the assembled
            # system SPD analytically. Reject nonfinite/poor float32 solves or
            # a direction which fails the quadratic descent check.
            eo = A * do + wp.transpose(B) * dg - ro
            eg = B * do + D * dg - rg
            scale = wp.max(1.0, wp.length(ro) + wp.length(rg))
            descent = wp.dot(do, ro) + wp.dot(dg, rg)
            coupled = (
                _finite_vector(do)
                and _finite_vector(dg)
                and wp.length(eo) + wp.length(eg) <= 0.001 * scale
                and descent >= 0.0
            )
        else:
            coupled = False
        if not coupled:
            fallback[w] = 2
    if not coupled:
        # Existing object-only fallback: the object contact Hessian still
        # includes the fixed current assembly pose, exactly as before H26.
        do = wp.inverse(A) * ro
        dg = wp.vec3()
    if not _finite_vector(do) or not _finite_vector(dg):
        return
    lo, lg = wp.length(do), wp.length(dg)
    factor = float(1.0)
    if lo > 0.0:
        factor = wp.min(factor, wp.max(0.0, budget - path[w]) / lo)
    if coupled and lg > 0.0:
        factor = wp.min(factor, wp.max(0.0, budget - rigid_path[w]) / lg)
    relative = wp.length(do - dg)
    contact_active = wp.trace(wp.transpose(B) * B) > 0.0
    if coupled and contact_active and relative > 0.0:
        factor = wp.min(factor, wp.max(0.0, relative_budget - relative_path[w]) / relative)
    do *= factor
    dg *= factor
    shift[w] = do
    rigid_shift[w] = dg
    path[w] += wp.length(do)
    rigid_path[w] += wp.length(dg)
    if coupled and contact_active:
        relative_path[w] += wp.length(do - dg)
    if wp.length(do) > 0.0:
        applications[w] += 1
    if coupled and (wp.length(do) > 0.0 or wp.length(dg) > 0.0):
        rigid_applications[w] += 1


@wp.kernel
def _apply_rigid_block(groups: wp.array[int], shift: wp.array[wp.vec3], q: wp.array[wp.transform]):
    b = wp.tid()
    w = groups[b]
    if w >= 0:
        pose = q[b]
        q[b] = wp.transform(wp.transform_get_translation(pose) + shift[w], wp.transform_get_rotation(pose))


@wp.kernel
def _object_moments(
    q: wp.array[wp.vec3],
    mass: wp.array[float],
    world: wp.array[int],
    anchor: wp.array[int],
    first: wp.array[wp.vec3],
    second: wp.array[float],
):
    p = wp.tid()
    w = world[p]
    # Reduce offsets rather than absolute world coordinates for float32 COM.
    offset = q[p] - q[anchor[w]]
    wp.atomic_add(first, w, mass[p] * offset)
    wp.atomic_add(second, w, mass[p] * wp.dot(offset, offset))


@wp.kernel
def _finish_object_moments(
    q: wp.array[wp.vec3],
    anchor: wp.array[int],
    mass: wp.array[float],
    first: wp.array[wp.vec3],
    second: wp.array[float],
    center: wp.array[wp.vec3],
    radius: wp.array[float],
    invalid: wp.array[int],
):
    w = wp.tid()
    mean = first[w] / mass[w]
    center[w] = q[anchor[w]] + mean
    variance = second[w] / mass[w] - wp.dot(mean, mean)
    radius[w] = 0.0
    if not wp.isfinite(variance) or variance <= 1.0e-16:
        invalid[w] = 1
    else:
        radius[w] = wp.sqrt(variance)


@wp.kernel
def _se3_inertia_block(
    dt: float,
    q: wp.array[wp.vec3],
    inertia: wp.array[wp.vec3],
    mass: wp.array[float],
    world: wp.array[int],
    center: wp.array[wp.vec3],
    radius: wp.array[float],
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    angular_residual: wp.array[wp.vec3],
    angular_hessian: wp.array[wp.mat33],
    angular_linear_hessian: wp.array[wp.mat33],
    max_radius: wp.array[float],
):
    p = wp.tid()
    w = world[p]
    if radius[w] <= 1.0e-8:
        return
    r = q[p] - center[w]
    G = wp.skew(r / radius[w])
    k = mass[p] / (dt * dt)
    f = k * (inertia[p] - q[p])
    wp.atomic_add(residual, w, f)
    wp.atomic_add(hessian, w, k * wp.identity(3, float))
    wp.atomic_add(angular_residual, w, G * f)
    wp.atomic_add(angular_hessian, w, k * G * wp.transpose(G))
    wp.atomic_add(angular_linear_hessian, w, k * G)
    wp.atomic_max(max_radius, w, wp.length(r))


@wp.kernel
def _clear_rotation_diagnostics(
    mask: wp.array[wp.bool],
    indices: wp.array[int],
    center: wp.array[wp.vec3],
    radius: wp.array[float],
    max_radius: wp.array[float],
    angular_residual: wp.array[wp.vec3],
    angular_hessian: wp.array[wp.mat33],
    angular_linear_hessian: wp.array[wp.mat33],
    rotation: wp.array[wp.vec3],
    applications: wp.array[int],
    path_angle: wp.array[float],
    budget_scale: wp.array[float],
    motion_bound: wp.array[float],
    fallback: wp.array[int],
    contact_active: wp.array[int],
):
    w = wp.tid()
    if not mask or mask[indices[w]]:
        center[w] = wp.vec3()
        radius[w] = 0.0
        max_radius[w] = 0.0
        angular_residual[w] = wp.vec3()
        angular_hessian[w] = wp.mat33()
        angular_linear_hessian[w] = wp.mat33()
        rotation[w] = wp.vec3()
        applications[w] = 0
        path_angle[w] = 0.0
        budget_scale[w] = 0.0
        motion_bound[w] = 0.0
        fallback[w] = 0
        contact_active[w] = 0


@wp.kernel
def _solve_object_se3(
    residual: wp.array[wp.vec3],
    hessian: wp.array[wp.mat33],
    angular_residual: wp.array[wp.vec3],
    angular_hessian: wp.array[wp.mat33],
    angular_linear_hessian: wp.array[wp.mat33],
    radius: wp.array[float],
    max_radius: wp.array[float],
    invalid: wp.array[int],
    contact_active: wp.array[int],
    budget: float,
    relative_budget: float,
    path: wp.array[float],
    relative_path: wp.array[float],
    shift: wp.array[wp.vec3],
    rotation: wp.array[wp.vec3],
    applications: wp.array[int],
    rotation_applications: wp.array[int],
    path_angle: wp.array[float],
    budget_scale: wp.array[float],
    motion_bound: wp.array[float],
    fallback: wp.array[int],
):
    w = wp.tid()
    shift[w] = wp.vec3()
    rotation[w] = wp.vec3()
    budget_scale[w] = 0.0
    motion_bound[w] = 0.0
    fallback[w] = 0
    if invalid[w] != 0 or radius[w] <= 1.0e-8:
        fallback[w] = 1
        return
    rt, ru = residual[w], angular_residual[w]
    A, D, B = hessian[w], angular_hessian[w], angular_linear_hessian[w]
    t, u = wp.vec3(), wp.vec3()
    accepted = False
    if _finite_vector(rt) and _finite_vector(ru) and _finite_matrix(A) and _finite_matrix(D) and _finite_matrix(B):
        t, u = ldlt6_solve(A, D, B, rt, ru)
        et = A * t + wp.transpose(B) * u - rt
        eu = B * t + D * u - ru
        accepted = (
            _finite_vector(t)
            and _finite_vector(u)
            and wp.length(et) + wp.length(eu) <= 0.001 * wp.max(1.0, wp.length(rt) + wp.length(ru))
            and wp.dot(t, rt) + wp.dot(u, ru) >= 0.0
        )
    if not accepted:
        fallback[w] = 2
        t = wp.inverse(A) * rt
        u = wp.vec3()
    if not _finite_vector(t) or not _finite_vector(u):
        return
    omega = u / radius[w]
    angle = wp.length(omega)
    # Conservative bound for every finite-rotation point displacement; shared
    # with translation blocks. One scalar retains the small-solve direction.
    cost = wp.length(t) + max_radius[w] * angle
    factor = float(1.0)
    if angle > 0.0:
        factor = wp.min(factor, 0.0017453292519943296 / angle)  # 0.1 degree
    remaining = wp.min(0.00025, wp.max(0.0, budget - path[w]))
    if cost > 0.0:
        factor = wp.min(factor, remaining / cost)
        if contact_active[w] != 0:
            factor = wp.min(factor, wp.max(0.0, relative_budget - relative_path[w]) / cost)
    t *= factor
    omega *= factor
    used = factor * cost
    shift[w] = t
    rotation[w] = omega
    budget_scale[w] = factor
    motion_bound[w] = used
    # One writer per world; explicit assignments avoid deferred atomic reads.
    path[w] = path[w] + used
    if contact_active[w] != 0:
        relative_path[w] = relative_path[w] + used
    path_angle[w] = path_angle[w] + wp.length(omega)
    if used > 0.0:
        applications[w] = applications[w] + 1
        if accepted:
            rotation_applications[w] = rotation_applications[w] + 1


@wp.kernel
def _apply_object_se3(
    world: wp.array[int],
    center: wp.array[wp.vec3],
    shift: wp.array[wp.vec3],
    rotation: wp.array[wp.vec3],
    reference: wp.array[wp.vec3],
    q: wp.array[wp.vec3],
    displacement: wp.array[wp.vec3],
):
    p = wp.tid()
    w = world[p]
    omega = rotation[w]
    angle = wp.length(omega)
    delta = shift[w]
    if angle > 0.0:
        r = q[p] - center[w]
        # Quaternion Rodrigues coefficients keep the tiny-angle displacement
        # local, avoiding subtraction of two nearly equal world coordinates.
        axis = omega / angle
        sine, cosine = wp.sin(angle), wp.cos(angle)
        delta += sine * wp.cross(axis, r) + (1.0 - cosine) * wp.cross(axis, wp.cross(axis, r))
    updated = displacement[p] + delta
    displacement[p] = updated
    q[p] = reference[p] + updated


class _TranslationBlock:
    """A restricted common translation solve, independent of desired robot poses."""

    def __init__(
        self,
        solver,
        *,
        contact_gap_m,
        substep_budget_m,
        rigid_translation=False,
        relative_budget_m=0.00025,
        object_rotation=False,
    ):
        self.solver = solver
        max_shift_m = substep_budget_m
        relaxation = 1.0
        if solver.rigid_soft_enable_dat or solver.particle_enable_self_contact:
            raise ValueError("coarse translation does not support DAT or self-contact")
        limits = np.asarray((contact_gap_m, substep_budget_m, max_shift_m, relaxation), dtype=float)
        if not np.isfinite(limits).all() or np.any(limits <= 0) or relaxation > 1:
            raise ValueError("positive finite limits and relaxation <= 1 required")
        if substep_budget_m > contact_gap_m / 4 or max_shift_m > substep_budget_m:
            raise ValueError("require max_shift <= substep budget <= one quarter of contact detection gap")
        model = solver.model
        if model.attachment_particle_body_count or model.spring_count:
            raise ValueError("coarse translation refuses attachments and springs")
        mass, inv_mass = model.particle_mass.numpy(), model.particle_inv_mass.numpy()
        flags = model.particle_flags.numpy()
        if not len(mass) or not np.isfinite(mass).all() or np.any(mass <= 0):
            raise ValueError("all particles must have positive finite mass")
        if np.any(flags != 1) or not np.allclose(mass * inv_mass, 1):
            raise ValueError("all particles must be active, unpinned and have consistent inverse mass")
        source_worlds = model.particle_world.numpy()
        worlds = source_worlds.copy()
        if np.all(worlds == -1):
            # Newton's nonreplicated builder stores a single global system as
            # world -1 and may report world_count == 0. This block owns
            # that one global object as its sole reduction block.
            worlds = np.zeros_like(worlds)
            self.world_count = 1
        elif np.any(worlds < 0):
            raise ValueError("mixed global and per-world particles are unsupported")
        else:
            self.world_count = int(model.world_count)
        if not np.array_equal(np.unique(worlds), np.arange(self.world_count)):
            raise ValueError("each world must contain one tetrahedral object")
        tets = model.tet_indices.numpy()
        if not len(tets) or not np.array_equal(np.unique(tets), np.arange(len(mass))):
            raise ValueError("every particle must belong to the tetrahedral volume")
        if np.any(worlds[tets] != worlds[tets[:, :1]]):
            raise ValueError("cross-world tetrahedra are unsupported")
        # Check shared-face connectivity independently in each world.
        parent = list(range(len(tets)))

        def root(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        faces = {}
        for i, tet in enumerate(tets):
            for omit in range(4):
                face = tuple(sorted(np.delete(tet, omit).tolist()))
                if face in faces:
                    parent[root(i)] = root(faces[face])
                else:
                    faces[face] = i
        for w in range(self.world_count):
            if len({root(int(i)) for i in np.flatnonzero(worlds[tets[:, 0]] == w)}) != 1:
                raise ValueError("each world must contain exactly one face-connected volume")
        self._profile_counts = (model.particle_count, model.tet_count, model.world_count)
        self._particle_profile = {
            "particle_mass": mass.copy(),
            "particle_inv_mass": inv_mass.copy(),
            "particle_flags": flags.copy(),
            "particle_world": source_worlds.copy(),
            "tet_indices": tets.copy(),
        }
        self.budget = float(substep_budget_m)
        self.max_shift = float(max_shift_m)
        self.relaxation = float(relaxation)
        self.world = wp.array(worlds, dtype=int, device=model.device)
        mask_indices = np.full(self.world_count, int(model.world_count), dtype=np.int32)
        if not np.all(source_worlds == -1):
            mask_indices = np.arange(self.world_count, dtype=np.int32)
        self.mask_indices = wp.array(mask_indices, dtype=int, device=model.device)
        self.residual = wp.zeros(self.world_count, dtype=wp.vec3, device=model.device)
        # This is the residual BEFORE the latest coarse correction. It is not
        # an independently evaluated post-correction force-balance diagnostic.
        self.hessian = wp.zeros(self.world_count, dtype=wp.mat33, device=model.device)
        self.shift = wp.zeros(self.world_count, dtype=wp.vec3, device=model.device)
        self.path_length = wp.zeros(self.world_count, dtype=float, device=model.device)
        self.applications = wp.zeros(self.world_count, dtype=int, device=model.device)
        self.invalid_world = wp.zeros(self.world_count, dtype=int, device=model.device)
        self.rigid_translation = bool(rigid_translation)
        self.relative_budget = float(relative_budget_m)
        self.rigid_residual = wp.zeros_like(self.residual)
        self.rigid_hessian = wp.zeros_like(self.hessian)
        self.cross_hessian = wp.zeros_like(self.hessian)
        self.rigid_shift = wp.zeros_like(self.shift)
        self.rigid_path_length = wp.zeros_like(self.path_length)
        self.relative_path_length = wp.zeros_like(self.path_length)
        self.rigid_applications = wp.zeros_like(self.applications)
        # 0=coupled, 1=unsupported active external rigid contact/overflow,
        # 2=nonfinite or inaccurate small linear solve. Per latest iteration.
        self.rigid_fallback = wp.zeros_like(self.invalid_world)
        self.body_groups = wp.full(model.body_count, -1, dtype=int, device=model.device)
        self._rigid_profile = {}
        self.object_rotation = bool(object_rotation)
        self._iteration = 0
        self.last_mode = "none"
        if self.object_rotation and (
            not np.isfinite(self.relative_budget) or not 0 < self.relative_budget <= self.budget
        ):
            raise ValueError("coarse relative budget must be positive and no greater than coarse_budget")
        self.total_mass = wp.array(
            np.bincount(worlds, weights=mass, minlength=self.world_count), dtype=float, device=model.device
        )
        self.anchor_particle = wp.array(
            [np.flatnonzero(worlds == w)[0] for w in range(self.world_count)], dtype=int, device=model.device
        )
        self._first_moment = wp.zeros_like(self.residual)
        self._second_moment = wp.zeros_like(self.path_length)
        self.centers = wp.zeros_like(self.residual)
        self.radius = wp.zeros_like(self.path_length)
        self.max_radius = wp.zeros_like(self.path_length)
        self.angular_residual = wp.zeros_like(self.residual)
        self.angular_hessian = wp.zeros_like(self.hessian)
        self.angular_linear_hessian = wp.zeros_like(self.hessian)
        self.rotation_vector = wp.zeros_like(self.residual)
        self.rotation_applications = wp.zeros_like(self.applications)
        self.rotation_path_angle = wp.zeros_like(self.path_length)
        self.rotation_budget_scale = wp.zeros_like(self.path_length)
        self.rotation_motion_bound = wp.zeros_like(self.path_length)
        self.rotation_fallback = wp.zeros_like(self.invalid_world)
        self.rotation_contact_active = wp.zeros_like(self.invalid_world)
        if self.rigid_translation:
            self._initialize_rigid_profile()

    def _prepare_object_rotation(self, state, dt):
        solver, model = self.solver, self.solver.model
        self._first_moment.zero_()
        self._second_moment.zero_()
        self.max_radius.zero_()
        self.angular_residual.zero_()
        self.angular_hessian.zero_()
        self.angular_linear_hessian.zero_()
        self.rotation_contact_active.zero_()
        self.rigid_shift.zero_()  # This block never moves the rigid assembly.
        wp.launch(
            _object_moments,
            dim=model.particle_count,
            inputs=[
                state.particle_q,
                model.particle_mass,
                self.world,
                self.anchor_particle,
                self._first_moment,
                self._second_moment,
            ],
            device=model.device,
        )
        wp.launch(
            _finish_object_moments,
            dim=self.world_count,
            inputs=[
                state.particle_q,
                self.anchor_particle,
                self.total_mass,
                self._first_moment,
                self._second_moment,
                self.centers,
                self.radius,
                self.invalid_world,
            ],
            device=model.device,
        )
        wp.launch(
            _se3_inertia_block,
            dim=model.particle_count,
            inputs=[
                dt,
                state.particle_q,
                solver.inertia,
                model.particle_mass,
                self.world,
                self.centers,
                self.radius,
                self.residual,
                self.hessian,
                self.angular_residual,
                self.angular_hessian,
                self.angular_linear_hessian,
                self.max_radius,
            ],
            device=model.device,
        )

    def _apply_object_rotation(self, state):
        model, solver = self.solver.model, self.solver
        wp.launch(
            _solve_object_se3,
            dim=self.world_count,
            inputs=[
                self.residual,
                self.hessian,
                self.angular_residual,
                self.angular_hessian,
                self.angular_linear_hessian,
                self.radius,
                self.max_radius,
                self.invalid_world,
                self.rotation_contact_active,
                self.budget,
                self.relative_budget,
                self.path_length,
                self.relative_path_length,
                self.shift,
                self.rotation_vector,
                self.applications,
                self.rotation_applications,
                self.rotation_path_angle,
                self.rotation_budget_scale,
                self.rotation_motion_bound,
                self.rotation_fallback,
            ],
            device=model.device,
        )
        wp.launch(
            _apply_object_se3,
            dim=model.particle_count,
            inputs=[
                self.world,
                self.centers,
                self.shift,
                self.rotation_vector,
                solver.pos_prev_collision_detection,
                state.particle_q,
                solver.particle_displacements,
            ],
            device=model.device,
        )

    def _initialize_rigid_profile(self):
        solver, model = self.solver, self.solver.model
        if solver.integrate_with_external_rigid_solver or not solver._integrates_rigid_bodies:
            raise ValueError("coarse rigid translation requires native rigid VBD integration")
        if not solver.rigid_compliant_alm:
            raise ValueError("coarse rigid translation requires native compliant ALM joints")
        if not np.isfinite(self.relative_budget) or not 0 < self.relative_budget <= self.budget:
            raise ValueError("coarse relative budget must be positive and no greater than coarse_budget")
        if model.tendon_count or model.constraint_mimic_count or model.muscle_count or solver._has_joint_mimics:
            raise ValueError("coarse rigid translation refuses tendons, muscles and mimic constraints")
        mass, inverse = model.body_mass.numpy(), model.body_inv_mass.numpy()
        flags, worlds = model.body_flags.numpy(), model.body_world.numpy().copy()
        if np.all(worlds == -1):
            worlds[:] = 0
        # Replication may give static environment fixtures positive mass and
        # inverse mass while retaining KINEMATIC flags. They are not members of
        # the freely translating mechanism. Proxies remain unsupported.
        if np.any((flags & int(BodyFlags.PROXY)) != 0) or np.any(
            (flags != int(BodyFlags.DYNAMIC)) & (flags != int(BodyFlags.KINEMATIC))
        ):
            raise ValueError("coarse rigid translation refuses proxy or unsupported body flags")
        dynamic = (mass > 0) & (flags == int(BodyFlags.DYNAMIC))
        if not dynamic.any() or not np.isfinite(mass).all() or np.any(mass < 0):
            raise ValueError("coarse rigid translation requires positive finite assembly masses")
        if np.any(flags[dynamic] != int(BodyFlags.DYNAMIC)) or not np.allclose(mass[dynamic] * inverse[dynamic], 1):
            raise ValueError("assembly bodies must be dynamic, nonproxy and have consistent inverse masses")
        inertia = model.body_inertia.numpy()[dynamic]
        if (
            not np.isfinite(model.body_com.numpy()[dynamic]).all()
            or not np.isfinite(inertia).all()
            or np.any(np.linalg.eigvalsh(inertia) <= 0)
            or not np.isfinite(model.body_inv_inertia.numpy()[dynamic]).all()
        ):
            raise ValueError("assembly COM/inertias must be finite with positive definite inertia")
        if np.any((worlds[dynamic] < 0) | (worlds[dynamic] >= self.world_count)):
            raise ValueError("assembly world ownership must match particle worlds")
        groups = np.full(model.body_count, -1, dtype=np.int32)
        groups[dynamic] = worlds[dynamic]
        types = model.joint_type.numpy()
        parents, children = model.joint_parent.numpy(), model.joint_child.numpy()
        enabled = model.joint_enabled.numpy()
        supported = {int(JointType.FIXED), int(JointType.PRISMATIC), int(JointType.REVOLUTE)}
        for w in range(self.world_count):
            members = set(np.flatnonzero(groups == w).tolist())
            if not members:
                raise ValueError("each particle world requires one positive-mass FREE assembly")
            roots, links = [], []
            for j, (p, c) in enumerate(zip(parents, children, strict=True)):
                if c not in members and p not in members:
                    continue
                if not enabled[j] or c not in members:
                    raise ValueError("assembly cannot have disabled or foreign child joints")
                if p < 0 and types[j] == int(JointType.FREE):
                    roots.append(int(c))
                elif p in members and types[j] in supported:
                    links.append((int(p), int(c)))
                else:
                    raise ValueError("assembly joints must be one FREE root and supported internal joints")
            if len(roots) != 1 or len(links) != len(members) - 1:
                raise ValueError("assembly must form one FREE rooted tree")
            reached = {roots[0]}
            while True:
                expanded = reached | {c for p, c in links if p in reached}
                if expanded == reached:
                    break
                reached = expanded
            if reached != members:
                raise ValueError("assembly must be connected to its FREE root")
        self.body_groups.assign(groups)
        self._rigid_counts = (
            model.body_count,
            model.joint_count,
            model.shape_count,
            model.tendon_count,
            model.constraint_mimic_count,
            model.muscle_count,
        )
        names = (
            "body_mass",
            "body_inv_mass",
            "body_com",
            "body_inertia",
            "body_inv_inertia",
            "body_flags",
            "body_world",
            "joint_type",
            "joint_parent",
            "joint_child",
            "joint_X_p",
            "joint_X_c",
            "joint_axis",
            "joint_dof_dim",
            "joint_enabled",
            "joint_world",
            "joint_q_start",
            "joint_qd_start",
            "joint_mimic_joint",
            "shape_body",
            "shape_flags",
        )
        self._rigid_profile = {name: getattr(model, name).numpy().copy() for name in names}

    def _apply_coupled_solve(self, state, contacts, dt):
        solver, model = self.solver, self.solver.model
        if contacts is not None and contacts.rigid_contact_max:
            wp.launch(
                _guard_external_rigid_contacts,
                dim=contacts.rigid_contact_max,
                inputs=[
                    dt,
                    contacts.rigid_contact_count,
                    contacts.rigid_contact_max,
                    self.body_groups,
                    contacts.rigid_contact_shape0,
                    contacts.rigid_contact_shape1,
                    contacts.rigid_contact_point0,
                    contacts.rigid_contact_point1,
                    contacts.rigid_contact_surface_velocity,
                    contacts.rigid_contact_offset0,
                    contacts.rigid_contact_offset1,
                    contacts.rigid_contact_normal,
                    contacts.rigid_contact_margin0,
                    contacts.rigid_contact_margin1,
                    model.shape_body,
                    state.body_q,
                    solver.body_q_prev,
                    model.body_com,
                    solver.body_body_contact_penalty_k,
                    solver.body_body_contact_normal_rho,
                    solver.body_body_contact_material_ke,
                    solver.body_body_contact_material_kd,
                    solver.body_body_contact_material_mu,
                    solver.body_body_contact_tangent_rho,
                    solver.body_body_contact_lambda,
                    solver.body_body_contact_C0,
                    solver.rigid_contact_alpha,
                    int(solver.rigid_contact_hard),
                    int(solver.rigid_compliant_alm),
                    solver.friction_epsilon,
                    self.rigid_fallback,
                ],
                device=model.device,
            )
        wp.launch(
            _solve_coupled_block,
            dim=self.world_count,
            inputs=[
                self.residual,
                self.hessian,
                self.rigid_residual,
                self.rigid_hessian,
                self.cross_hessian,
                self.invalid_world,
                self.rigid_fallback,
                self.budget,
                self.relative_budget,
                self.path_length,
                self.rigid_path_length,
                self.relative_path_length,
                self.shift,
                self.rigid_shift,
                self.applications,
                self.rigid_applications,
            ],
            device=model.device,
        )
        wp.launch(
            _apply_rigid_block,
            dim=model.body_count,
            inputs=[self.body_groups, self.rigid_shift, state.body_q],
            device=model.device,
        )

    def validate_profile(self):
        """Check actual particle invariants on broad notifications, outside capture."""
        model = self.solver.model
        if (
            (model.particle_count, model.tet_count, model.world_count) != self._profile_counts
            or model.attachment_particle_body_count
            or model.spring_count
        ):
            raise ValueError("coarse translation particle topology changed; rebuild the solver")
        if self.rigid_translation:
            counts = (
                model.body_count,
                model.joint_count,
                model.shape_count,
                model.tendon_count,
                model.constraint_mimic_count,
                model.muscle_count,
            )
            if counts != self._rigid_counts:
                raise ValueError("coarse rigid translation assembly topology changed; rebuild the solver")
            for name, expected in self._rigid_profile.items():
                if not np.array_equal(getattr(model, name).numpy(), expected):
                    raise ValueError(f"coarse rigid translation frozen {name} changed; rebuild the solver")
        for name, expected in self._particle_profile.items():
            if not np.array_equal(getattr(model, name).numpy(), expected):
                raise ValueError(f"coarse translation frozen {name} changed; rebuild the solver")

    def begin_substep(self, world_mask=None):
        self._iteration = 0
        self.last_mode = "none"
        if self.object_rotation:
            wp.launch(
                _clear_rotation_diagnostics,
                dim=self.world_count,
                inputs=[
                    world_mask,
                    self.mask_indices,
                    self.centers,
                    self.radius,
                    self.max_radius,
                    self.angular_residual,
                    self.angular_hessian,
                    self.angular_linear_hessian,
                    self.rotation_vector,
                    self.rotation_applications,
                    self.rotation_path_angle,
                    self.rotation_budget_scale,
                    self.rotation_motion_bound,
                    self.rotation_fallback,
                    self.rotation_contact_active,
                ],
                device=self.solver.model.device,
            )
        if self.rigid_translation or self.object_rotation:
            wp.launch(
                _clear_rigid_diagnostics,
                dim=self.world_count,
                inputs=[
                    world_mask,
                    self.mask_indices,
                    self.rigid_path_length,
                    self.relative_path_length,
                    self.rigid_shift,
                    self.rigid_residual,
                    self.rigid_hessian,
                    self.cross_hessian,
                    self.rigid_applications,
                    self.rigid_fallback,
                ],
                device=self.solver.model.device,
            )
        if world_mask is not None:
            wp.launch(
                _clear_masked_diagnostics,
                dim=self.world_count,
                inputs=[
                    world_mask,
                    self.mask_indices,
                    self.path_length,
                    self.applications,
                    self.shift,
                    self.residual,
                    self.hessian,
                    self.invalid_world,
                ],
                device=self.solver.model.device,
            )
            return
        self.path_length.zero_()
        self.applications.zero_()
        self.shift.zero_()
        self.residual.zero_()
        self.hessian.zero_()
        self.invalid_world.zero_()

    def apply(self, state_in, state_out, contacts, dt):
        solver, model = self.solver, self.solver.model
        rotate = self.object_rotation and (not self.rigid_translation or self._iteration % 2 == 0)
        self._iteration += 1
        self.last_mode = (
            "object_se3" if rotate else ("coupled_translation" if self.rigid_translation else "object_translation")
        )
        self.residual.zero_()
        self.hessian.zero_()
        self.invalid_world.zero_()
        if rotate:
            self._prepare_object_rotation(state_in, dt)
        if self.rigid_translation and not rotate:
            self.rigid_residual.zero_()
            self.rigid_hessian.zero_()
            self.cross_hessian.zero_()
            self.rigid_fallback.zero_()
            wp.launch(
                _rigid_inertia_block,
                dim=model.body_count,
                inputs=[
                    dt,
                    self.body_groups,
                    state_in.body_q,
                    solver.body_inertia_q,
                    model.body_com,
                    model.body_mass,
                    self.rigid_residual,
                    self.rigid_hessian,
                ],
                device=model.device,
            )
        if not rotate:
            wp.launch(
                _inertia_block,
                dim=model.particle_count,
                inputs=[
                    dt,
                    state_in.particle_q,
                    solver.inertia,
                    model.particle_mass,
                    self.world,
                    self.residual,
                    self.hessian,
                ],
                device=model.device,
            )
        if contacts is not None:
            wp.launch(
                _check_contact_capacity,
                dim=self.world_count,
                inputs=[contacts.soft_contact_count, contacts.soft_contact_max, self.invalid_world],
                device=model.device,
            )
        if contacts is not None and contacts.soft_contact_max:
            if solver.integrate_with_external_rigid_solver:
                body_q, previous, body_qd = state_out.body_q, state_in.body_q, state_out.body_qd
            else:
                body_q = state_in.body_q
                previous = solver.body_q_prev if model.body_count else None
                body_qd = state_in.body_qd
            contact_filter = _make_soft_contact_filter_data(model, contacts, state_in.particle_q, body_q)
            wp.launch(
                _contact_block,
                dim=contacts.soft_contact_max,
                inputs=[
                    dt,
                    state_in.particle_q,
                    solver.particle_q_prev,
                    self.world,
                    model.particle_radius,
                    contacts.soft_contact_indices,
                    contacts.soft_contact_count,
                    contacts.soft_contact_max,
                    contacts.soft_contact_barycentric,
                    contact_filter,
                    solver.body_particle_contact_penalty_k,
                    solver.body_particle_contact_material_kd,
                    solver.body_particle_contact_material_mu,
                    solver.friction_epsilon,
                    solver.rigid_soft_contact_use_log_barrier,
                    model.shape_body,
                    body_q,
                    previous,
                    body_qd,
                    model.body_com,
                    contacts.soft_contact_shape,
                    contacts.soft_contact_body_pos,
                    contacts.soft_contact_body_vel,
                    contacts.soft_contact_normal,
                    model.shape_margin,
                    self.residual,
                    self.hessian,
                    self.invalid_world,
                    self.rigid_translation and not rotate,
                    self.body_groups,
                    self.rigid_residual,
                    self.rigid_hessian,
                    self.cross_hessian,
                    rotate,
                    self.centers,
                    self.radius,
                    self.angular_residual,
                    self.angular_hessian,
                    self.angular_linear_hessian,
                    self.rotation_contact_active,
                ],
                device=model.device,
            )
        if rotate:
            self._apply_object_rotation(state_in)
        elif self.rigid_translation:
            self._apply_coupled_solve(state_in, contacts, dt)
        else:
            wp.launch(
                _solve_block,
                dim=self.world_count,
                inputs=[
                    self.residual,
                    self.hessian,
                    self.invalid_world,
                    self.budget,
                    self.max_shift,
                    self.relaxation,
                    self.path_length,
                    self.shift,
                    self.applications,
                ],
                device=model.device,
            )
        if not rotate:
            wp.launch(
                _apply_block,
                dim=model.particle_count,
                inputs=[
                    self.world,
                    self.shift,
                    solver.pos_prev_collision_detection,
                    state_in.particle_q,
                    solver.particle_displacements,
                ],
                device=model.device,
            )
        wp.copy(state_out.particle_q, state_in.particle_q)


class SolverVBDCoarse(SolverVBD):
    """VBD with an opt-in translation block for one fused tetrahedral object per world.

    ``coarse_translation=False`` retains the base solver's stepping and reset
    behavior. Enabled, each particle iteration is followed by a fresh 3D solve
    of its existing inertial/contact objective along common object translation.
    The FEM material and deformation are preserved; no target object pose,
    attachment, or external manipulation force is introduced.

    Args:
        model: Newton Model, or SolverModelView with the supported particle layout.
        coarse_translation: Enable the restricted translation block (default False).
        coarse_contact_gap: Actual collision pipeline detection gap in meters.
        coarse_budget: Maximum cumulative additional translation path per physical
            substep in meters, at most one quarter of coarse_contact_gap.
        coarse_rigid_translation: Opt-in independent object/complete FREE-assembly
            translations using their native coupled 6D inertial/contact block.
            Requires coarse_translation and compliant ALM; default False.
        coarse_object_rotation: Opt-in finite whole-object SE3 correction about its
            current mass COM, with RMS-radius-scaled angular coordinates. Default
            False retains the translation-only path. With a FREE assembly, this
            block alternates with coupled translation each iteration (SE3 first);
            otherwise it runs every iteration, including fixed-base robot scenes.
            Each correction is capped at 0.1 degree and 0.25 mm conservative
            maximum point motion, inside the shared cumulative coarse_budget.
        coarse_relative_budget: Cumulative additional relative point motion allowed
            per substep while object/assembly contact Hessians are active [m].
            Both proposed shifts receive one common budget scale factor.
        coarse_deterministic: Ordered coarse reductions (default True). False
            uses native floating atomics, reducing graph scratch and runtime;
            roundoff and repeated-rollout agreement must be validated separately.
        **kwargs: Passed unchanged to Newton SolverVBD.

    Pins, attachments, springs, disconnected objects, mixed global/per-world
    particles, DAT, and self-contact are refused when the block is enabled.
    Overflow disables the block in every world. This budget limits additional
    block motion; it is not a collision-coverage/nonpenetration guarantee and
    does not limit local VBD or rigid-body motion. Base reset restores physical
    state/history; block diagnostics/budget clear on reset and every substep.

    Enabled instances freeze particle masses, flags, world ownership and tet
    topology. Broad model notifications revalidate those actual invariants;
    notifications alone do not imply an incompatible change. Body-property/
    inertial notifications avoid readback for coupled rigid proxy refreshes in
    object-only mode. The optional rigid block freezes and revalidates its body
    inertial, flag, world, shape mapping and joint-tree profile on all notifications.
    Unsupported active external rigid contacts fall back to object-only correction;
    inactive speculative queries do not. This block applies to FREE assemblies,
    not fixed-base robot arms. Object-only rotation has no FREE-assembly
    requirement and holds the other bodies at their current iterate. It changes
    no body poses, inertia/history/velocity buffers or actuation. Native
    finalization computes velocities from the corrected particle positions.
    Changing a frozen invariant invalidates the block and requires rebuilding
    the solver. Direct, unnotified in-place edits to the particle profile are
    unsupported. DAT, self-contact and coarse
    enablement cannot be toggled after construction. Python step entry checks
    these modes; captured graphs must also be discarded before configuration
    changes because replay does not execute Python checks.

    In rotation mode, coarse_block.path_length counts the conservative maximum
    point-motion bound, shared by alternating blocks. Rotation diagnostics are
    the latest SE3 assembly/correction (not an independent final-state residual):
    centers/radius/max_radius, angular_residual (torque divided by RMS radius),
    angular_hessian/angular_linear_hessian, rotation_vector [rad], cumulative
    rotation_applications/rotation_path_angle, rotation_budget_scale and
    rotation_motion_bound [m]. rotation_fallback is 0 for a valid SE3 solve,
    1 for invalid geometry/contact overflow, or 2 for translation-only fallback
    after a rejected linear solve. last_mode names the last chosen block.

    ``coarse_block.residual`` contains the force residual before the latest
    correction, not a fresh post-correction validation measurement.
    Coarse force/Hessian reductions default to Warp RUN_TO_RUN atomics;
    native solver and collision determinism settings remain independent. Module
    options apply process-wide; configure before capture and discard captured
    graphs before changing modes.
    """

    def __init__(
        self,
        model,
        *,
        coarse_translation=False,
        coarse_contact_gap=0.002,
        coarse_budget=0.00025,
        coarse_rigid_translation=False,
        coarse_relative_budget=0.00025,
        coarse_object_rotation=False,
        coarse_deterministic=True,
        **kwargs,
    ):
        if (coarse_rigid_translation or coarse_object_rotation) and not coarse_translation:
            raise ValueError("coarse rigid translation/rotation requires coarse_translation=True")
        self.coarse_rigid_translation = bool(coarse_rigid_translation)
        self.coarse_object_rotation = bool(coarse_object_rotation)
        self.coarse_block = None
        self._coarse_profile_invalid = False
        super().__init__(model, **kwargs)
        self.coarse_translation = bool(coarse_translation)
        self.coarse_contact_gap = float(coarse_contact_gap)
        self.coarse_budget = float(coarse_budget)
        if self.coarse_translation:
            # Preserve the ordered diagnostic baseline by default. Its per-launch
            # scratch is retained throughout CUDA graph lifetime; callers can
            # explicitly select floating atomics for throughput after validation.
            mode = wp.DeterministicMode.RUN_TO_RUN if coarse_deterministic else wp.DeterministicMode.NOT_GUARANTEED
            self._set_module_options(
                {"deterministic": mode, "deterministic_max_records": 0},
                module=_inertia_block.module,
            )
            self.coarse_block = _TranslationBlock(
                self,
                contact_gap_m=self.coarse_contact_gap,
                substep_budget_m=self.coarse_budget,
                rigid_translation=self.coarse_rigid_translation,
                relative_budget_m=coarse_relative_budget,
                object_rotation=self.coarse_object_rotation,
            )

    def notify_model_changed(self, flags):
        # Rigid proxy mass/flags can change without altering the supported
        # complete soft-object profile. RSS can emit broad notifications after
        # assembly, so compare actual particle invariants instead of treating
        # the flags themselves as evidence of an incompatible edit.
        allowed = int(ModelFlags.BODY_PROPERTIES | ModelFlags.BODY_INERTIAL_PROPERTIES)
        if self.coarse_block is not None and (self.coarse_rigid_translation or int(flags) & ~allowed):
            try:
                self.coarse_block.validate_profile()
            except ValueError:
                self._coarse_profile_invalid = True
                raise
        return super().notify_model_changed(flags)

    def step(self, state_in, state_out, control, contacts, dt, *, observables=None):
        if self.coarse_block is not None and self.coarse_object_rotation != self.coarse_block.object_rotation:
            raise ValueError("coarse object rotation enablement is fixed at construction; rebuild the solver")
        if self.coarse_block is not None and self.coarse_rigid_translation != self.coarse_block.rigid_translation:
            raise ValueError("coarse rigid translation enablement is fixed at construction; rebuild the solver")
        if self.coarse_translation != (self.coarse_block is not None):
            raise ValueError("coarse translation enablement is fixed at construction; rebuild the solver")
        if self.coarse_block is not None:
            if self.coarse_rigid_translation and (
                not self.rigid_compliant_alm or self.integrate_with_external_rigid_solver
            ):
                raise ValueError("coarse rigid translation integration/ALM mode changed; rebuild the solver")
            if self._coarse_profile_invalid:
                raise ValueError("coarse translation profile was invalidated; rebuild the solver")
            if self.rigid_soft_enable_dat or self.particle_enable_self_contact:
                raise ValueError("coarse translation does not support enabling DAT or self-contact; rebuild the solver")
        return super().step(state_in, state_out, control, contacts, dt, observables=observables)

    def reset(self, state, world_mask=None, flags=None):
        super().reset(state, world_mask=world_mask, flags=flags)
        if self.coarse_block is not None:
            self.coarse_block.begin_substep(world_mask)

    def _initialize_particles(self, state_in, state_out, contacts, dt):
        super()._initialize_particles(state_in, state_out, contacts, dt)
        if self.coarse_block is not None:
            self.coarse_block.begin_substep()

    def _solve_particle_iteration(self, state_in, state_out, contacts, dt):
        super()._solve_particle_iteration(state_in, state_out, contacts, dt)
        if self.coarse_block is not None:
            self.coarse_block.apply(state_in, state_out, contacts, dt)
