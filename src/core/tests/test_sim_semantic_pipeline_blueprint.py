import pytest

from core.blueprints.full_stack import full_stack_blueprint


@pytest.mark.parametrize(
    ("robot", "driver_name"),
    [
        ("sim_mujoco", "MujocoDriverModule"),
        ("sim_ros2", "ROS2SimDriverModule"),
    ],
)
def test_sim_blueprint_wires_real_semantic_pipeline(robot, driver_name):
    system = full_stack_blueprint(
        robot=robot,
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    assert driver_name in system.modules
    assert "PerceptionModule" in system.modules
    assert "VisualServoModule" in system.modules
    assert "SemanticPlannerModule" in system.modules

    connections = set(system.connections)
    assert (driver_name, "camera_image", "PerceptionModule", "color_image") in connections
    assert (driver_name, "depth_image", "PerceptionModule", "depth_image") in connections
    assert (driver_name, "camera_info", "PerceptionModule", "camera_info") in connections
    assert (driver_name, "camera_image", "VisualServoModule", "color_image") in connections
    assert (driver_name, "depth_image", "VisualServoModule", "depth_image") in connections
    assert (driver_name, "camera_info", "VisualServoModule", "camera_info") in connections
    assert ("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph") in connections
    assert ("PerceptionModule", "scene_graph", "VisualServoModule", "scene_graph") in connections
    assert ("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections") in connections


def test_sim_mujoco_blueprint_enables_camera_for_semantics():
    system = full_stack_blueprint(
        robot="sim_mujoco",
        slam_profile="none",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
        enable_map_modules=False,
    ).build()

    driver = system.get_module("MujocoDriverModule")
    assert driver._enable_camera is True
