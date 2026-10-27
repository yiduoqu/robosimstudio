# SPDX-License-Identifier: Apache-2.0
"""Core bounded-actuator check: fingers must stall at a table, not follow through it."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from gripper import ScriptedGripperSpec

import robosimstudio as rss


def check():
    vbd = rss.backend("vbd", iterations=12)
    scene = rss.Scene(
        cell="bench_075",
        physics=rss.ArrangementSpec(control_hz=30, substeps=20, use_cuda_graph=True, backends=(("*", vbd),)),
    )
    grip = scene.add(
        ScriptedGripperSpec(
            center=(0.0, 0.0, 0.06), half_gap=0.03, scripted=False, angular_ke=1.0, angular_kd=0.0008, torque_limit=0.1
        )
    )
    sim = scene.build(device="cuda:0", warmup=True)
    entity = grip.entity
    target = entity.initial_poses.numpy()[0].copy()
    target[:, 2] = 0.70
    entity.set_targets(target)
    sim.run(60).sync()
    poses = sim.pipeline.state_0.body_q.numpy()[entity.bodies]
    peak = entity.peak_force.numpy()
    lower_bound = poses[:, 2] - 0.008
    error = poses[:, 2] - target[:, 2]
    result = {
        "graph_captured": sim.pipeline.captured,
        "peak_applied_force_N": peak.tolist(),
        "body_z_m": poses[:, 2].tolist(),
        "target_error_z_m": error.tolist(),
        "conservative_finger_bottom_z_m": lower_bound.tolist(),
        "force_limit_N": entity.spec.force_limit,
        "passed": bool(
            np.isfinite(poses).all()
            and np.max(peak) <= 5.00001
            and np.min(error) > 0.04
            and np.min(lower_bound) > 0.7495
        ),
    }
    if not result["passed"]:
        raise AssertionError(result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = check()
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
