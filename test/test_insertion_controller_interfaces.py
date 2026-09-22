"""Check installed ROS interfaces coexist with the existing launch helpers."""

import os
from pathlib import Path
import subprocess
import sys
import textwrap

from ament_index_python.packages import get_package_prefix


def installed_probe(tmp_path, source):
    """Use the installed package, without a previous controller package overlay."""
    prefix = Path(get_package_prefix("dual_fr3_moveit_config"))
    roots = [
        path.parent.parent
        for path in prefix.glob("**/dual_fr3_moveit_config/msg/_joint_target.py")
    ]
    assert len(roots) == 1, "Build and install the moveit_config interfaces first"
    python_root = roots[0].parent
    env = os.environ.copy()
    for key in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH"):
        env[key] = os.pathsep.join(
            item for item in env.get(key, "").split(os.pathsep)
            if item and "dual_fr3_insertion_controller" not in Path(item).parts
        )
    env["PYTHONPATH"] = os.pathsep.join(filter(None, (str(python_root), env["PYTHONPATH"])))
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(source)],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_installed_messages_and_existing_helpers_share_one_python_package(tmp_path):
    installed_probe(tmp_path, """
        from dual_fr3_moveit_config import backends, maniskill_resources, moveit_resources
        from dual_fr3_moveit_config.msg import JointTarget, JointTargetState

        assert backends.resolve_simulation_backend("real") == "real"
        assert moveit_resources.DEFAULT_PACKAGE == "dual_fr3_moveit_config"
        assert callable(maniskill_resources.build_maniskill_description)
        assert JointTarget.__module__.startswith("dual_fr3_moveit_config.msg.")
        assert JointTargetState.__module__.startswith("dual_fr3_moveit_config.msg.")
    """)


def test_target_and_stopped_feedback_preserve_protocol_on_serialization(tmp_path):
    installed_probe(tmp_path, """
        from builtin_interfaces.msg import Time
        from rclpy.serialization import deserialize_message, serialize_message
        from dual_fr3_moveit_config.msg import JointTarget, JointTargetState

        session = (1 << 63) + 11
        sequence = (1 << 62) + 3
        stamp = Time(sec=17, nanosec=123456789)
        positions = [0.1, -0.2, 0.3, -1.4, 0.5, 1.6, 0.7]
        target = JointTarget(stamp=stamp, session=session, sequence=sequence,
                             valid_for_s=0.08, positions=positions, velocities=[0.0] * 7)
        restored = deserialize_message(serialize_message(target), JointTarget)
        assert restored.stamp == stamp
        assert restored.session == session and restored.sequence == sequence
        assert restored.valid_for_s == 0.08
        assert list(restored.positions) == positions
        assert list(restored.velocities) == [0.0] * 7

        state = JointTargetState(stamp=stamp, session=session, last_sequence=sequence,
                                 state="STOPPED", reason="operator_stop",
                                 positions=positions, velocities=[0.0] * 7)
        feedback = deserialize_message(serialize_message(state), JointTargetState)
        assert feedback.stamp == stamp
        assert feedback.session == session and feedback.last_sequence == sequence
        assert feedback.state == "STOPPED" and feedback.reason == "operator_stop"
        assert list(feedback.positions) == positions
        assert list(feedback.velocities) == [0.0] * 7
    """)


def test_plugin_is_registered_by_existing_robot_configuration_package(tmp_path):
    installed_probe(tmp_path, """
        from pathlib import Path
        import xml.etree.ElementTree as ET
        from ament_index_python.resources import get_resource
        from ament_index_python.packages import get_packages_with_prefixes

        assert "dual_fr3_insertion_controller" not in get_packages_with_prefixes()
        resource, prefix = get_resource("controller_interface__pluginlib__plugin",
                                        "dual_fr3_moveit_config")
        matches = []
        for relative_path in resource.splitlines():
            root = ET.parse(Path(prefix) / relative_path).getroot()
            libraries = [root] if root.tag == "library" else root.findall("library")
            for library in libraries:
                for entry in library.findall("class"):
                    if entry.get("name") == "dual_fr3_moveit_config/JointTargetController":
                        matches.append(entry)
                        assert entry.get("base_class_type") == "controller_interface::ControllerInterface"
                        assert entry.get("type").startswith("dual_fr3_moveit_config::")
                        assert (Path(prefix) / "lib" / ("lib" + library.get("path") + ".so")).is_file()
        assert len(matches) == 1
    """)
