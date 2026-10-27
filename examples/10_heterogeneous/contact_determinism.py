# SPDX-License-Identifier: Apache-2.0
"""Native Newton 1.8/Warp contact allocation settings for this example.

Call before the first collision launch or CUDA capture. These private Newton
kernel names bind this helper to the checked-out dependency version. Settings
apply process-wide to the selected modules; they do not validate replay.
Rigid contacts use CollisionPipeline's native deterministic sorting option.
"""

from __future__ import annotations

import warp as wp


def _configure(modules, *, face_modules=()):
    selected = {module.name: module for module in modules}
    face_names = {module.name for module in face_modules}
    for name, module in selected.items():
        options = {"deterministic": wp.DeterministicMode.RUN_TO_RUN, "deterministic_max_records": 0}
        if name in face_names:
            # CUDA 13.4's default optimizer can spend minutes on the generic
            # face kernel. This example does not request differentiation.
            options.update(mode="release", optimization_level=1, enable_backward=False)
        wp.set_module_options(options, module=module)
    return sorted(selected)


def configure_soft_contacts():
    """Cover fused/compacted primitive soft contacts and NXN/explicit pairs.

    Compacted edge/face dispatch depends on compatible candidate counts, so
    configure every existing specialization without launching any kernels.
    Return the configured module names; no device arrays are read back.
    """
    from newton._src.geometry import broad_phase_nxn, kernels
    from newton._src.geometry import soft_contacts_sdf as sdf

    face_modules = (
        sdf.create_soft_face_contacts.module,
        sdf.classify_soft_face_contacts.module,
        sdf.create_soft_face_sdf_contacts.module,
    )
    modules = [
        wp.get_module(kernels.__name__),
        wp.get_module(broad_phase_nxn.__name__),
        *face_modules,
        sdf.create_soft_edge_contacts.module,
        sdf.classify_soft_edge_contacts.module,
        sdf.create_soft_edge_sdf_contacts.module,
        *(kernel.module for kernel in sdf._SDF_FACE_KERNEL_BY_GEO.values()),
        *(kernel.module for kernel in sdf._SDF_EDGE_KERNEL_BY_GEO.values()),
    ]
    return _configure(modules, face_modules=face_modules)


def configure_soft_record_capacity(collision):
    """Bound records per fallback thread from actual world-compatible pairs.

    Native compact fallbacks stride over a capped grid. A thread can therefore
    emit more than once; Warp's automatic static bound does not count dynamic
    loop trips. Call for every active pipeline before capture; shared modules
    retain the largest requested bound. This reads array shapes, not contents.
    """
    from newton._src.geometry import soft_contacts_sdf as sdf

    result = {}
    for kind, pairs, generic, specialized in (
        ("edge", collision.soft_edge_rigid_pairs, sdf.create_soft_edge_sdf_contacts, sdf._SDF_EDGE_KERNEL_BY_GEO),
        ("face", collision.soft_face_rigid_pairs, sdf.create_soft_face_sdf_contacts, sdf._SDF_FACE_KERNEL_BY_GEO),
    ):
        count = len(pairs)
        grid = min(count, sdf._SDF_FALLBACK_GRID_SIZE)
        bound = (count + grid - 1) // grid if grid else 1
        for kernel in (generic, *specialized.values()):
            current = wp.get_module_options(kernel.module).get("deterministic_max_records", 0)
            wp.set_module_options({"deterministic_max_records": max(current, bound)}, module=kernel.module)
        result[kind] = {"candidate_pairs": count, "fallback_records_per_thread": bound}
    return result
