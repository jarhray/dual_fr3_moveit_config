"""Exercise real Xacro resources and launch expansion without starting physics."""
import importlib.util
import logging
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

_logger_class = logging.getLoggerClass()
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
logging.setLoggerClass(_logger_class)

from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import USB_LINK, load_config
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description


SOURCE = Path(__file__).resolve().parents[2]


def expand_launch(filename, *, package="dual_fr3_moveit_config", **overrides):
    spec = importlib.util.spec_from_file_location("scene_launch", SOURCE / package / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    nodes = []

    def visit(actions):
        for action in actions:
            if isinstance(action, LaunchDescription):
                visit(action.entities)
                continue
            if action.condition is not None and not action.condition.evaluate(context):
                continue
            if isinstance(action, Node):
                nodes.append(action)
            elif isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
                action.execute(context)
            elif isinstance(action, (IncludeLaunchDescription, OpaqueFunction)):
                visit(action.execute(context) or [])

    visit(module.generate_launch_description().entities)
    return nodes, context


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def node_parameters(node, context):
    # Humble exposes no public accessor for a Node's normalized parameters.
    result = {}
    for values in evaluate_parameters(context, node._Node__parameters):
        if isinstance(values, Path):
            values = yaml.safe_load(values.read_text())["dual_fr3_maniskill"]["ros__parameters"]
        result.update(values)
    return result


@pytest.mark.parametrize("filename,scene,overrides", [
    ("maniskill.launch.py", "robot", {}),
    ("maniskill.launch.py", "usb_cable", {"maniskill_scene": "usb_cable"}),
    ("maniskill.launch.py", "trunking_cable", {"maniskill_scene": "trunking_cable"}),
    ("usb_cable.launch.py", "usb_cable", {}),
    ("demo.launch.py", "robot", {"simulation_backend": "maniskill"}),
    ("demo.launch.py", "usb_cable", {"simulation_backend": "maniskill", "maniskill_scene": "usb_cable"}),
    ("demo.launch.py", "trunking_cable", {"simulation_backend": "maniskill", "maniskill_scene": "trunking_cable"}),
])
def test_all_entry_points_share_one_bridge_and_final_model(filename, scene, overrides):
    nodes, context = expand_launch(filename, **overrides)
    by_package = {resolve(context, n.node_package): n for n in nodes}
    assert len(nodes) == len(by_package) == 4
    assert set(by_package) == {"dual_fr3_maniskill", "robot_state_publisher", "moveit_ros_move_group", "rviz2"}
    parameters = {name: node_parameters(node, context) for name, node in by_package.items()}
    bridge = parameters["dual_fr3_maniskill"]
    expected_description, expected_semantic = build_maniskill_description(scene=scene)
    for values in parameters.values():
        assert values["robot_description"] == expected_description
        if "robot_description_semantic" in values:
            assert values["robot_description_semantic"] == expected_semantic
    for name in ("robot_state_publisher", "moveit_ros_move_group", "rviz2"):
        assert parameters[name]["use_sim_time"] is True
    executable = resolve(context, by_package["dual_fr3_maniskill"].node_executable)
    if scene in ("usb_cable", "trunking_cable"):
        assert executable == f"{scene}_bridge.py"
        assert (bridge["control_freq"], bridge["sim_freq"], bridge["publish_freq"]) == (50, 500, 25)
        assert bridge["cable_config"] == resolve_cable_config(scene=scene)
        if scene == "usb_cable":
            assert any("usb_cable.rviz" in resolve(context, arg) for arg in by_package["rviz2"].cmd)
        else:
            assert USB_LINK not in expected_description
            assert "research_finger/finger1.STL" in expected_description
        move_group = parameters["moveit_ros_move_group"]
        assert move_group["trajectory_execution.allowed_execution_duration_scaling"] == 10.
        assert move_group["trajectory_execution.allowed_goal_duration_margin"] == 5.
    else:
        assert executable == "maniskill_bridge.py"
        assert (bridge["control_freq"], bridge["sim_freq"], bridge["publish_freq"]) == (100, 500, 50)
        assert "cable_config" not in bridge
        assert USB_LINK not in expected_description


def test_robot_model_matches_existing_backend():
    share = Path(get_package_share_directory("dual_fr3_moveit_config"))
    description = xacro.process_file(str(share / "config/dual_fr3.urdf.xacro"), mappings={
        "load_left_ros2_control": "false", "load_right_ros2_control": "false",
        "load_gripper": "true", "ee_id": "franka_hand",
    }).toxml()
    semantic = xacro.process_file(str(share / "config/dual_fr3.srdf.xacro")).toxml()
    assert build_maniskill_description() == (description, semantic)


def test_custom_cable_configuration_reaches_model_and_bridge(tmp_path):
    config = load_config(resolve_cable_config())
    config["usb"]["mass"] = .025
    config["usb"]["grip_center"] = [.001, 0., 0.]
    path = tmp_path / "custom cable.yaml"
    path.write_text(yaml.safe_dump(config))
    sim_path = tmp_path / "custom bridge.yaml"
    sim_path.write_text(yaml.safe_dump({"dual_fr3_maniskill": {"ros__parameters": {
        "control_freq": 25, "sim_freq": 500, "publish_freq": 25}}}))
    nodes, context = expand_launch("maniskill.launch.py", maniskill_scene="usb_cable",
                                   cable_config=str(path), maniskill_config=str(sim_path))
    bridge = next(n for n in nodes if resolve(context, n.node_package) == "dual_fr3_maniskill")
    values = node_parameters(bridge, context)
    assert values["cable_config"] == str(path)
    assert values["control_freq"] == 25
    robot = ET.fromstring(values["robot_description"])
    assert float(robot.find(f"link[@name='{USB_LINK}']/inertial/mass").get("value")) == .025
    assert robot.find("joint[@name='usb_cable_demo_mount']/origin").get("xyz") != "0.0 0.0 0.0"
    links = {link.get("name") for link in robot.findall("link")}
    assert len(robot.findall(f"link[@name='{USB_LINK}']")) == 1
    for joint in robot.findall("joint"):
        assert joint.find("parent").get("link") in links
        assert joint.find("child").get("link") in links
    for mesh in robot.findall(f"link[@name='{USB_LINK}']//mesh"):
        assert mesh.get("filename") == "package://dual_fr3_maniskill/meshes/USB1.stl"
    # Resolve the same final model through the real simulator asset adapter.
    assets = prepare_assets(values["robot_description"], values["robot_description_semantic"], tmp_path)
    assert assets.initial_positions["left_fr3_joint7"] == pytest.approx(3 * 3.141592653589793 / 4)


def test_physics_only_launch_uses_the_same_scene_selection():
    description, semantic = build_maniskill_description(scene="usb_cable")
    nodes, context = expand_launch("sim.launch.py", package="dual_fr3_maniskill",
        maniskill_scene="usb_cable", robot_description=description, robot_description_semantic=semantic)
    assert len(nodes) == 1
    parameters = node_parameters(nodes[0], context)
    assert parameters["robot_description"] == description
    assert parameters["robot_description_semantic"] == semantic
    assert parameters["control_freq"] == 50


@pytest.mark.parametrize("overrides,error", [
    ({"maniskill_scene": "missing"}, "is not valid"),
    ({"maniskill_scene": "usb_cable", "load_gripper": "false"}, "requires load_gripper"),
    ({"maniskill_scene": "usb_cable", "ee_id": "other"}, "requires load_gripper"),
    ({"maniskill_scene": "usb_cable", "cable_config": "/nonexistent/cable.yaml"}, "No such file"),
])
def test_invalid_scene_configuration_fails_before_nodes_start(overrides, error):
    with pytest.raises((ValueError, RuntimeError, FileNotFoundError), match=error):
        expand_launch("maniskill.launch.py", **overrides)


def test_scene_model_building_does_not_import_physics_dependencies():
    script = '''
import sys
class RejectPhysics:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('warp', 'sapien', 'mani_skill2', 'mpm'):
            raise AssertionError('Model construction imported physics: ' + fullname)
sys.meta_path.insert(0, RejectPhysics())
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
build_maniskill_description()
build_maniskill_description(scene='usb_cable')
'''
    subprocess.run([sys.executable, "-c", script], check=True, capture_output=True, text=True)


@pytest.mark.parametrize("filename,overrides", [
    ("maniskill.launch.py", {"maniskill_scene": "trunking_cable"}),
    ("usb_cable.launch.py", {}),
    ("demo.launch.py", {"simulation_backend": "maniskill", "maniskill_scene": "trunking_cable"}),
])
def test_rope_solver_reaches_physics_bridge(filename, overrides):
    nodes, context = expand_launch(filename, cable_solver="rope_actor", cable_trace_dir="/tmp/rope trace", **overrides)
    bridge, = [n for n in nodes if resolve(context, n.node_package) == "dual_fr3_maniskill"]
    assert node_parameters(bridge, context)["cable_solver"] == "rope_actor"
    assert node_parameters(bridge, context)["cable_trace_dir"] == "/tmp/rope trace"


@pytest.mark.parametrize("scene,solver", [("robot", "rope_actor"), ("trunking_cable", "mpm")])
def test_trace_requires_rope_cable_scene(scene, solver):
    with pytest.raises(ValueError, match="cable_trace_dir requires"):
        expand_launch("maniskill.launch.py", maniskill_scene=scene, cable_solver=solver,
                      cable_trace_dir="/tmp/rope trace")


def test_rope_only_yaml_can_build_usb_moveit_geometry(tmp_path):
    config = load_config(resolve_cable_config())
    del config["mpm"]
    for key in ("particle_spacing", "young_modulus", "axial_young_modulus", "axial_iterations", "yield_stress", "poisson_ratio"):
        del config["cable"][key]
    path = tmp_path/"rope.yaml"
    path.write_text(yaml.safe_dump(config))
    nodes, context = expand_launch("usb_cable.launch.py", cable_solver="rope_actor", cable_config=str(path))
    bridge, = [n for n in nodes if resolve(context, n.node_package) == "dual_fr3_maniskill"]
    assert node_parameters(bridge, context)["cable_solver"] == "rope_actor"


def test_invalid_cable_solver_is_rejected_by_launch():
    with pytest.raises(RuntimeError, match="is not valid"):
        expand_launch("usb_cable.launch.py", cable_solver="unsupported")


@pytest.mark.parametrize("direction", ["forward", "reverse"])
@pytest.mark.parametrize("filename", ["maniskill.launch.py", "demo.launch.py"])
def test_leader_orientation_reaches_deferred_cable_bridge(filename, direction):
    nodes, context = expand_launch(filename, simulation_backend="maniskill",
        maniskill_scene="trunking_cable", leader_orientation_direction=direction)
    bridge, = [n for n in nodes if resolve(context, n.node_package) == "dual_fr3_maniskill"]
    assert node_parameters(bridge, context)["leader_orientation_direction"] == direction

@pytest.mark.parametrize("config_file", ["trunking_cable.yaml", "trunking_cable_simplified_2mm.yaml"])
def test_research_finger_bore_fits_the_mtc_cable(tmp_path, config_file):
    import trimesh
    import numpy as np
    from dual_fr3_maniskill.cable.mesh_contacts import finger_collision_meshes, closed_oriented_shells
    from dual_fr3_maniskill.assets import prepare_assets
    from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
    path = SOURCE/"dual_fr3_maniskill/config"/config_file
    config = load_config(path)
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=path)
    assets = prepare_assets(description, semantic, tmp_path)
    robot = ET.fromstring(description)
    tcp_z = float(robot.find("joint[@name='right_fr3_hand_tcp_joint']/origin").get('xyz').split()[2])
    for name, shapes in finger_collision_meshes(assets.urdf_path).items():
        if not name.startswith('right_'):
            continue
        vertices, indices = shapes[0]
        mesh = trimesh.Trimesh(vertices=vertices, faces=indices.reshape(-1, 3), process=True)
        assert closed_oriented_shells(mesh)
        samples = np.column_stack((np.linspace(-.012, .012, 101), np.zeros(101), np.full(101, tcp_z-.0584)))
        samples += config['guide']['center_offset']
        _, distance, _ = trimesh.proximity.closest_point_naive(mesh, samples)
        radius = config['cable']['diameter']/2
        assert distance.min() > radius + .0001
        if config_file == "trunking_cable.yaml":
            assert distance.min() > .0035
        else:
            assert distance.min() < .0013


@pytest.mark.parametrize("config_file,inherit_visual,mesh,origin", [
    ("trunking_cable.yaml", False, "Trunking.STL", "0 0 0"),
    ("trunking_cable_simplified_2mm.yaml", False, "Trunking_simplify.stl", "0.00014546 -0.00068397 0"),
    ("trunking_cable_simplified_2mm.yaml", True, "Trunking_simplify.stl", "0.00014546 -0.00068397 0"),
])
def test_mtc_mesh_selection_reaches_all_scene_consumers(tmp_path, config_file, inherit_visual, mesh, origin):
    path = SOURCE/"dual_fr3_maniskill/config"/config_file
    if inherit_visual:
        config = yaml.safe_load(path.read_text())
        del config["scene"]["trunking_visual_mesh"]
        path = tmp_path/"legacy.yaml"
        path.write_text(yaml.safe_dump(config))
    nodes, context = expand_launch("demo.launch.py", simulation_backend="maniskill",
                                  maniskill_scene="trunking_cable", cable_config=str(path))
    for node in nodes:
        root = ET.fromstring(node_parameters(node, context)["robot_description"])
        trunking = root.find("link[@name='trunking']")
        expected = {"collision": (mesh, origin),
                    "visual": (mesh, origin) if inherit_visual else ("Trunking.STL", "0 0 0")}
        for kind, (expected_mesh, expected_origin) in expected.items():
            shape = trunking.find(kind)
            assert shape.find("geometry/mesh").get("filename").endswith("/"+expected_mesh)
            assert shape.find("origin").get("xyz") == expected_origin
