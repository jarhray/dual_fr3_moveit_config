#!/usr/bin/env python3
"""GPU smoke test: dimensions, gravity, attachment and measured robot motion."""
import argparse
import json
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import numpy as np

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import USB_LINK, load_config
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_maniskill.scenes.usb_cable import UsbCableSimulation
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
import warp as wp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=25)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--render-output")
    parser.add_argument("--fixture-closeup", action="store_true")
    parser.add_argument("--config")
    parser.add_argument("--report-output")
    args = parser.parse_args()
    config_path = resolve_cable_config(args.config)
    config = load_config(config_path)
    description, semantic = build_maniskill_description(scene="usb_cable", cable_config=config_path)
    with tempfile.TemporaryDirectory(prefix="usb_cable_check_") as directory:
        sim = UsbCableSimulation(prepare_assets(description, semantic, Path(directory)), cable_config=config,
                                 control_freq=50, sim_freq=500, viewer=args.viewer)
        try:
            # This point belonged to the old 1.5 m layout but lies inside the
            # actual right FR3 base mesh. Keep it as a positive collision test.
            probe = np.tile([.22714427, .9605302, .08], (len(sim.cable.local), 1))
            depths = sim.cable.contacts.audit(sim.cable.states[0],
                positions=wp.array(probe, dtype=wp.vec3, device=sim.cable.device))
            bodies = sim.cable.model.shape_body.numpy()
            base_shapes = [s for s, b in enumerate(bodies) if sim.cable.actors[b].name == 'right_fr3_link0']
            assert max(depths[base_shapes]) > .01, 'Right FR3 base mesh contact was not detected'
            sim.cable.check_contacts()
            initial = sim.cable.centerline.copy()
            initial_plug = sim.link_pose(USB_LINK).copy()
            initial_joint = sim.target[sim.indices["left_fr3_joint1"]]
            start = time.monotonic()
            max_penetration = 0.0
            for i in range(args.steps):
                sim.target[sim.indices["left_fr3_joint1"]] = initial_joint + .02 * min(1., (i + 1) / 25)
                sim.step()
                max_penetration = max(max_penetration, sim.cable.contacts.max_depth)
                if i % 25 == 0:
                    print(json.dumps({"step": i + 1, "wall_seconds": time.monotonic() - start,
                                      **sim.cable.diagnostics()}), flush=True)
            report = sim.cable.diagnostics()
            report["sim_seconds"] = sim.time
            report["wall_seconds"] = time.monotonic() - start
            report["plug_motion_m"] = float(np.linalg.norm(sim.link_pose(USB_LINK)[:3] - initial_plug[:3]))
            report["tip_drop_m"] = float(initial[-1, 2] - sim.cable.centerline[-1, 2])
            report["tracking_error_rad"] = float(np.max(np.abs(sim.positions - sim.target)))
            report["max_rigid_penetration_over_run_m"] = max_penetration
            print(json.dumps(report, indent=2), flush=True)
            if args.report_output:
                Path(args.report_output).write_text(json.dumps(report, indent=2) + '\n')
            assert report["attachment_error_m"] < 1e-5, report
            assert report["max_section_gap_m"] < config["cable"]["particle_spacing"] * 2, report
            assert max_penetration <= config["cable"]["penetration_tolerance"], report
            assert np.isfinite(sim.cable.states[0].struct.particle_F.numpy()).all(), report
            assert report["tip_drop_m"] > 0, report
            if args.steps <= 5 and config["cable"]["initial_layout"] == "straight":
                # The distant free end cannot feel the clamp this early.
                expected_drop = .5 * 9.81 * sim.time ** 2
                assert abs(report["tip_drop_m"] - expected_drop) < .2 * expected_drop, report
            assert report["plug_motion_m"] > 1e-6, report
            if args.render_output:
                from PIL import Image
                Image.fromarray(sim.render_image(fixture_closeup=args.fixture_closeup)).save(args.render_output)
            # A rejected reset must leave the current physical state intact.
            before_reset = sim.cable.positions.copy()
            invalid = np.tile([.4, .7, -.02], (len(before_reset), 1))
            with patch.object(sim.cable, 'layout', return_value=invalid):
                try:
                    sim.cable.reset()
                except RuntimeError as exc:
                    assert 'penetrates' in str(exc), exc
                else:
                    raise AssertionError('Reset accepted a cable embedded in the table')
            np.testing.assert_array_equal(sim.cable.positions, before_reset)
            sim.cable.reset()
            assert sim.cable.diagnostics()["attachment_error_m"] < 1e-5
            assert abs(sim.cable.diagnostics()["current_centerline_length_m"] - config["cable"]["length"]) < 1e-4
        finally:
            sim.close()


if __name__ == "__main__":
    main()
