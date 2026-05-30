"""Canonical LingTu runtime interface contract.

This module is the Python source of truth for the runtime boundary shared by
real robot drivers and simulator adapters.  Different endpoints may own
different data sources, but they must normalize into the same frames, topics,
message formats, and algorithm surfaces before entering LingTu modules.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class RuntimeFrames:
    """Canonical frame names used at LingTu runtime boundaries."""

    map: str = "map"
    odom: str = "odom"
    body: str = "body"
    model_base: str = "base_link"
    lidar: str = "lidar_link"
    real_lidar: str = "livox_frame"
    camera: str = "camera_link"
    simulator_world: str = "world"
    axis_convention: str = "x_forward_y_left_z_up"
    body_aliases: tuple[str, ...] = ("base_link",)
    lidar_aliases: tuple[str, ...] = ("livox_frame",)

    @property
    def body_alias_note(self) -> str:
        return f"{self.model_base} == {self.body}"

    @property
    def map_frame(self) -> str:
        return self.map

    @property
    def odom_frame(self) -> str:
        return self.odom

    @property
    def body_frame(self) -> str:
        return self.body

    @property
    def model_base_frame(self) -> str:
        return self.model_base

    @property
    def lidar_frame(self) -> str:
        return self.lidar

    @property
    def camera_frame(self) -> str:
        return self.camera

    @property
    def world(self) -> str:
        return self.simulator_world

    @property
    def simulator_world_frame(self) -> str:
        return self.simulator_world


@dataclass(frozen=True)
class RuntimeTopics:
    """Canonical topic names for LingTu runtime modules."""

    raw_lidar_points: str = "/points_raw"
    raw_imu: str = "/imu_raw"
    lidar_scan: str = "/nav/lidar_scan"
    imu: str = "/nav/imu"

    odometry: str = "/nav/odometry"
    registered_cloud: str = "/nav/registered_cloud"
    map_cloud: str = "/nav/map_cloud"
    cumulative_map_cloud: str = "/nav/cumulative_map_cloud"
    saved_map_cloud: str = "/nav/saved_map_cloud"
    save_map_service: str = "/nav/save_map"
    dog_odometry: str = "/nav/dog_odometry"
    localization_quality: str = "/nav/localization_quality"
    localization_health: str = "/nav/localization_health"
    state_estimation_at_scan: str = "/nav/state_estimation_at_scan"
    relocalize_service: str = "/nav/relocalize"
    global_relocalize_service: str = "/nav/global_relocalize"
    relocalize_check_service: str = "/nav/relocalize_check"

    exploration_start: str = "/exploration/start"
    exploration_stop: str = "/exploration/stop"
    exploration_way_point: str = "/exploration/way_point"
    exploration_grid: str = "/nav/exploration_grid"
    exploration_status: str = "/exploration/status"

    global_path: str = "/nav/global_path"
    local_path: str = "/nav/local_path"
    far_reach_goal: str = "/nav/far_reach_goal"
    adapter_status: str = "/nav/adapter_status"
    mission_status: str = "/nav/mission_status"
    terrain_map: str = "/nav/terrain_map"
    terrain_map_ext: str = "/nav/terrain_map_ext"
    cmd_vel: str = "/nav/cmd_vel"
    stop: str = "/nav/stop"
    slow_down: str = "/nav/slow_down"
    nav_way_point: str = "/nav/way_point"
    speed: str = "/nav/speed"
    map_clearing: str = "/nav/map_clearing"
    cloud_clearing: str = "/nav/cloud_clearing"
    added_obstacles: str = "/nav/added_obstacles"
    check_obstacle: str = "/nav/check_obstacle"
    planner_status: str = "/nav/planner_status"
    nav_state: str = "/nav/nav_state"

    goal_pose: str = "/nav/goal_pose"
    goal_point: str = "/nav/goal_point"
    patrol_goals: str = "/nav/patrol_goals"
    cancel: str = "/nav/cancel"
    navigation_boundary: str = "/nav/navigation_boundary"

    semantic_detections_3d: str = "/nav/semantic/detections_3d"
    semantic_scene_graph: str = "/nav/semantic/scene_graph"
    semantic_query_service: str = "/nav/semantic/query"
    semantic_instruction: str = "/nav/semantic/instruction"
    semantic_cancel: str = "/nav/semantic/cancel"
    semantic_status: str = "/nav/semantic/status"
    semantic_costmap: str = "/nav/costmap"
    scg_plan_request: str = "/nav/scg/plan_request"
    scg_plan_result: str = "/nav/scg/plan_result"
    voice_response: str = "/nav/voice/response"

    reconstruction_semantic_cloud: str = "/nav/reconstruction/semantic_cloud"
    reconstruction_stats: str = "/nav/reconstruction/stats"
    reconstruction_save_ply_service: str = "/nav/reconstruction/save_ply"

    safety_state: str = "/nav/safety_state"
    execution_eval: str = "/nav/execution_eval"
    dialogue_state: str = "/nav/dialogue_state"

    map_command: str = "/nav/map/command"
    map_response: str = "/nav/map/response"
    poi_command: str = "/nav/poi/command"
    poi_response: str = "/nav/poi/response"
    patrol_command: str = "/nav/patrol/command"
    patrol_response: str = "/nav/patrol/response"
    geofence_command: str = "/nav/geofence/command"
    geofence_response: str = "/nav/geofence/response"
    schedule_command: str = "/nav/schedule/command"
    schedule_response: str = "/nav/schedule/response"
    history_query: str = "/nav/history/query"
    history_response: str = "/nav/history/response"

    visualization_detections: str = "/nav/detections"
    robot_marker: str = "/nav/robot_marker"
    building_cloud: str = "/nav/building_cloud"

    camera_color: str = "/camera/color/image_raw"
    camera_depth: str = "/camera/depth/image_raw"
    camera_info: str = "/camera/color/camera_info"


@dataclass(frozen=True)
class Transform3D:
    """Static child-frame mounting pose expressed in the parent frame.

    For a body->lidar mounting, x/y/z/rpy describe the LiDAR frame pose in
    body coordinates. Applying this transform to a LiDAR-local point returns
    the point in the body frame, matching ROS TF parent/body child/lidar use.
    """

    parent: str
    child: str
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def translation(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def rotation_xyzw(self) -> tuple[float, float, float, float]:
        return rpy_to_quaternion_xyzw(self.roll, self.pitch, self.yaw)


@dataclass(frozen=True)
class MessageFormat:
    """Runtime message contract for a topic or topic family."""

    name: str
    ros_type: str
    frame_role: str
    required_fields: tuple[str, ...] = ()
    note: str = ""


@dataclass(frozen=True)
class ArtifactFormat:
    """Saved-map artifact contract shared by mapping, relocalization, and PCT."""

    name: str
    path: str
    artifact_type: str
    frame_role: str
    required_fields: tuple[str, ...] = ()
    required_metadata: tuple[str, ...] = ()
    note: str = ""


@dataclass(frozen=True)
class AlgorithmInterface:
    """Inputs and outputs owned by one LingTu algorithm surface."""

    name: str
    inputs: tuple[str, ...]
    outputs: tuple[str, ...]
    owner: str
    map_dependency: str


@dataclass(frozen=True)
class DataSourceContract:
    """How an endpoint must normalize its native data into LingTu."""

    name: str
    provider: str
    owns: tuple[str, ...]
    normalized_outputs: tuple[str, ...]
    command_sink: str
    source_outputs: tuple[str, ...] = ()
    algorithm_entry_outputs: tuple[str, ...] = ()
    algorithm_context_outputs: tuple[str, ...] = ()
    lidar_extrinsic_profile: str | None = None
    slam_source: str = "not_declared"
    localization_source: str = "not_declared"
    mapping_source: str = "not_declared"


@dataclass(frozen=True)
class AdapterTopicAlias:
    """Legacy/native endpoint topic mapped into the LingTu runtime contract."""

    source: str
    target: str
    msg_format: str
    scope: str = "adapter_only"
    note: str = ""


@dataclass(frozen=True)
class ProfileDataSourceBinding:
    """Which endpoint data source a CLI/runtime profile is allowed to use."""

    profile: str
    data_source: str
    mode: str
    note: str = ""


FRAMES = RuntimeFrames()
TOPICS = RuntimeTopics()

CORE_ALGORITHM_ENTRY_TOPICS = (
    TOPICS.odometry,
    TOPICS.registered_cloud,
    TOPICS.map_cloud,
)

CANONICAL_NAV_TOPICS = (
    *CORE_ALGORITHM_ENTRY_TOPICS,
    TOPICS.exploration_grid,
    TOPICS.global_path,
    TOPICS.local_path,
    TOPICS.cmd_vel,
)

CORE_REQUIRED_TOPICS = (
    *CANONICAL_NAV_TOPICS,
    TOPICS.goal_pose,
)

LIDAR_EXTRINSICS = {
    "real_mid360": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.real_lidar,
        x=-0.011,
        y=-0.02329,
        z=0.04412,
    ),
    "gazebo_proxy": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.28,
        y=0.0,
        z=0.20,
    ),
    "mujoco_thunder_v3": Transform3D(
        parent=FRAMES.body,
        child=FRAMES.lidar,
        x=0.0,
        y=0.0,
        z=0.28,
    ),
}

MESSAGE_FORMATS = {
    "raw_livox_custom": MessageFormat(
        name="raw_livox_custom",
        ros_type="livox_ros_driver2/msg/CustomMsg",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("offset_time", "x", "y", "z", "reflectivity", "tag", "line"),
        note="Native Livox raw packet stream for Fast-LIO2 lidar_type=1.",
    ),
    "raw_timed_pointcloud2": MessageFormat(
        name="raw_timed_pointcloud2",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role="lidar_or_body_raw_fastlio_input",
        required_fields=("x", "y", "z", "intensity", "time", "ring"),
        note="Timed XYZI cloud for Fast-LIO2 lidar_type=2.",
    ),
    "registered_cloud": MessageFormat(
        name="registered_cloud",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role=FRAMES.body,
        required_fields=("x", "y", "z"),
        note="Robot/body-frame current scan after source normalization.",
    ),
    "map_cloud": MessageFormat(
        name="map_cloud",
        ros_type="sensor_msgs/msg/PointCloud2",
        frame_role=f"{FRAMES.map}_or_{FRAMES.odom}",
        required_fields=("x", "y", "z"),
        note="World/local-map cloud; never body-relative.",
    ),
    "odometry": MessageFormat(
        name="odometry",
        ros_type="nav_msgs/msg/Odometry",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Continuous localization pose normalized before LingTu modules.",
    ),
    "state_estimation_at_scan": MessageFormat(
        name="state_estimation_at_scan",
        ros_type="nav_msgs/msg/Odometry",
        frame_role=f"{FRAMES.odom}->{FRAMES.body}",
        note="Scan-synchronized odometry from external simulation adapters.",
    ),
    "cmd_vel": MessageFormat(
        name="cmd_vel",
        ros_type="geometry_msgs/msg/TwistStamped",
        frame_role=FRAMES.body,
        note="Muxed command; endpoint adapters decide whether to relay it.",
    ),
}

TOPIC_FORMATS = {
    TOPICS.raw_lidar_points: ("raw_livox_custom", "raw_timed_pointcloud2"),
    TOPICS.raw_imu: ("sensor_msgs/msg/Imu",),
    TOPICS.lidar_scan: ("raw_livox_custom", "raw_timed_pointcloud2"),
    TOPICS.imu: ("sensor_msgs/msg/Imu",),
    TOPICS.odometry: ("odometry",),
    TOPICS.state_estimation_at_scan: ("state_estimation_at_scan",),
    TOPICS.registered_cloud: ("registered_cloud",),
    TOPICS.map_cloud: ("map_cloud",),
    TOPICS.cumulative_map_cloud: ("map_cloud",),
    TOPICS.saved_map_cloud: ("map_cloud",),
    TOPICS.exploration_grid: ("nav_msgs/msg/OccupancyGrid",),
    TOPICS.save_map_service: ("service",),
    TOPICS.localization_health: ("std_msgs/msg/String",),
    TOPICS.global_path: ("nav_msgs/msg/Path",),
    TOPICS.local_path: ("nav_msgs/msg/Path",),
    TOPICS.terrain_map: ("map_cloud",),
    TOPICS.terrain_map_ext: ("map_cloud",),
    TOPICS.cmd_vel: ("cmd_vel",),
    TOPICS.stop: ("std_msgs/msg/Bool",),
    TOPICS.slow_down: ("std_msgs/msg/Float32",),
    TOPICS.speed: ("std_msgs/msg/Float32",),
    TOPICS.map_clearing: ("std_msgs/msg/Bool",),
    TOPICS.cloud_clearing: ("std_msgs/msg/Bool",),
    TOPICS.added_obstacles: ("sensor_msgs/msg/PointCloud2",),
    TOPICS.check_obstacle: ("std_msgs/msg/Bool",),
    TOPICS.planner_status: ("std_msgs/msg/String",),
    TOPICS.navigation_boundary: ("geometry_msgs/msg/PolygonStamped",),
    TOPICS.goal_pose: ("geometry_msgs/msg/PoseStamped",),
    TOPICS.goal_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.exploration_way_point: ("geometry_msgs/msg/PointStamped",),
    TOPICS.nav_way_point: ("geometry_msgs/msg/PointStamped",),
    "/livox/lidar": ("raw_livox_custom",),
    "/livox/imu": ("sensor_msgs/msg/Imu",),
    "/imu/data": ("sensor_msgs/msg/Imu",),
    "/lidar/scan": ("raw_livox_custom",),
    "/model/thunder/odometry": ("odometry",),
    "/lingtu/gazebo/raw/lidar_points": ("sensor_msgs/msg/PointCloud2",),
    "/lingtu/gazebo/raw/lidar_scan": ("sensor_msgs/msg/LaserScan",),
    "/lingtu/gazebo/cmd_vel": ("geometry_msgs/msg/Twist",),
    "/state_estimation": ("odometry",),
    "/state_estimation_at_scan": ("state_estimation_at_scan",),
    "/registered_scan": ("map_cloud",),
    "/terrain_map": ("map_cloud",),
    "/terrain_map_ext": ("map_cloud",),
    "/way_point": ("geometry_msgs/msg/PointStamped",),
    "/cmd_vel": ("geometry_msgs/msg/Twist",),
    "/Odometry": ("odometry",),
    "/cloud_registered": ("registered_cloud",),
    "/cloud_map": ("map_cloud",),
    "/map_clearing": ("std_msgs/msg/Bool",),
    "/cloud_clearing": ("std_msgs/msg/Bool",),
    "/path": ("nav_msgs/msg/Path",),
    "/speed": ("std_msgs/msg/Float32",),
    "/stop": ("std_msgs/msg/Bool",),
    "/slow_down": ("std_msgs/msg/Float32",),
    "/navigation_boundary": ("geometry_msgs/msg/PolygonStamped",),
    "/added_obstacles": ("sensor_msgs/msg/PointCloud2",),
    "/check_obstacle": ("std_msgs/msg/Bool",),
    "/planner_status": ("std_msgs/msg/String",),
}

ARTIFACT_FORMATS = {
    "map_pcd": ArtifactFormat(
        name="map_pcd",
        path="map.pcd",
        artifact_type="pcd_xyz_or_xyzi",
        frame_role=FRAMES.map,
        required_fields=("x", "y", "z"),
        required_metadata=(
            "source_profile",
            "data_source",
            "slam_source",
            "frame_id",
            "point_count",
            "sha256",
        ),
        note="Saved map point cloud. PCT/relocalization may consume it only with same-source metadata.",
    ),
    "tomogram": ArtifactFormat(
        name="tomogram",
        path="tomogram.pickle",
        artifact_type="pct_tomogram",
        frame_role=FRAMES.map,
        required_fields=("tomogram", "resolution", "origin"),
        required_metadata=(
            "source_map_sha256",
            "source_profile",
            "data_source",
            "frame_id",
            "shape",
        ),
        note="PCT global-planning volume derived from the same map source.",
    ),
    "occupancy_grid": ArtifactFormat(
        name="occupancy_grid",
        path="occupancy.npz",
        artifact_type="numpy_occupancy_grid",
        frame_role=FRAMES.map,
        required_fields=("grid", "resolution", "origin"),
        required_metadata=("source_map_sha256", "source_profile", "data_source", "frame_id"),
        note="2D occupancy artifact for A*/frontier and map review.",
    ),
    "metadata": ArtifactFormat(
        name="metadata",
        path="metadata.json",
        artifact_type="json",
        frame_role=FRAMES.map,
        required_fields=(
            "source_profile",
            "data_source",
            "slam_source",
            "localization_source",
            "mapping_source",
            "frame_id",
            "created_at",
            "artifacts",
        ),
        required_metadata=(),
        note="Provenance file binding saved artifacts back to the runtime data source.",
    ),
}

ALGORITHM_INTERFACES = {
    "fastlio_mapping": AlgorithmInterface(
        name="fastlio_mapping",
        inputs=(TOPICS.lidar_scan, TOPICS.imu),
        outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        owner="slam",
        map_dependency="none_live_canonical_sensor_slam",
    ),
    "fastlio_raw_validation": AlgorithmInterface(
        name="fastlio_raw_validation",
        inputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        owner="slam_sim_validation",
        map_dependency="none_live_raw_sensor_slam",
    ),
    "exploration_strategy": AlgorithmInterface(
        name="exploration_strategy",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.exploration_grid),
        outputs=(TOPICS.exploration_way_point,),
        owner="tare_or_frontier",
        map_dependency="live_map_or_occupancy_grid",
    ),
    "wavefront_frontier_exploration": AlgorithmInterface(
        name="wavefront_frontier_exploration",
        inputs=(TOPICS.odometry, TOPICS.exploration_grid),
        outputs=(TOPICS.goal_pose,),
        owner="lingtu_frontier",
        map_dependency="live_occupancy_grid_unknown_free_boundary",
    ),
    "tare_exploration": AlgorithmInterface(
        name="tare_exploration",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.terrain_map_ext),
        outputs=(TOPICS.exploration_way_point,),
        owner="tare_external_or_lingtu_tare_adapter",
        map_dependency="live_registered_scan_or_terrain_map_ext",
    ),
    "global_planning": AlgorithmInterface(
        name="global_planning",
        inputs=(TOPICS.odometry, TOPICS.map_cloud, TOPICS.exploration_way_point, TOPICS.goal_pose),
        outputs=(TOPICS.global_path,),
        owner="lingtu_navigation",
        map_dependency="live_or_saved_map_cloud",
    ),
    "astar_global_planning": AlgorithmInterface(
        name="astar_global_planning",
        inputs=(TOPICS.odometry, TOPICS.exploration_grid, TOPICS.goal_pose),
        outputs=(TOPICS.global_path,),
        owner="lingtu_navigation",
        map_dependency="live_or_saved_occupancy_grid",
    ),
    "pct_global_planning": AlgorithmInterface(
        name="pct_global_planning",
        inputs=(TOPICS.odometry, "artifact:tomogram", TOPICS.goal_pose),
        outputs=(TOPICS.global_path,),
        owner="lingtu_pct",
        map_dependency="same_source_saved_tomogram_required",
    ),
    "local_planning_and_following": AlgorithmInterface(
        name="local_planning_and_following",
        inputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.global_path),
        outputs=(TOPICS.local_path, TOPICS.cmd_vel),
        owner="lingtu_autonomy",
        map_dependency="current_registered_cloud_and_path",
    ),
}

DATA_SOURCE_CONTRACTS = {
    "in_process_stub": DataSourceContract(
        name="in_process_stub",
        provider="in_process",
        owns=("mock_odometry", "mock_map", "mock_commands"),
        normalized_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        command_sink="module_graph_driver_cmd_vel",
        source_outputs=(),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile=None,
        slam_source="none",
        localization_source="mock_or_in_process_odometry",
        mapping_source="mock_or_in_process_map",
    ),
    "real_s100p": DataSourceContract(
        name="real_s100p",
        provider="hardware",
        owns=("mid360_lidar", "imu", "robot_actuation"),
        normalized_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        command_sink="hardware_driver_after_cmd_vel_mux",
        source_outputs=(TOPICS.lidar_scan, TOPICS.imu),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile="real_mid360",
        slam_source="lingtu_fastlio_or_external_robot_slam",
        localization_source="slam_localizer",
        mapping_source="slam_map_cloud",
    ),
    "mujoco_module_graph": DataSourceContract(
        name="mujoco_module_graph",
        provider="mujoco",
        owns=("physics", "rendered_lidar", "simulation_actuation"),
        normalized_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        command_sink="mujoco_driver_module_cmd_vel",
        source_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile="mujoco_thunder_v3",
        slam_source="none",
        localization_source="mujoco_sim_odometry",
        mapping_source="mujoco_rendered_lidar_map_cloud",
    ),
    "mujoco_fastlio2_live": DataSourceContract(
        name="mujoco_fastlio2_live",
        provider="mujoco",
        owns=("physics", "mid360_pattern_lidar", "imu"),
        normalized_outputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        command_sink="mujoco_velocity_adapter",
        source_outputs=(TOPICS.raw_lidar_points, TOPICS.raw_imu),
        algorithm_entry_outputs=(TOPICS.odometry, TOPICS.registered_cloud, TOPICS.map_cloud),
        algorithm_context_outputs=(),
        lidar_extrinsic_profile="mujoco_thunder_v3",
        slam_source="lingtu_fastlio2",
        localization_source="fastlio2_odometry",
        mapping_source="fastlio2_map_cloud",
    ),
    "gazebo_industrial": DataSourceContract(
        name="gazebo_industrial",
        provider="gazebo",
        owns=("world_geometry", "physics", "rendered_lidar", "simulation_actuation"),
        normalized_outputs=(
            TOPICS.odometry,
            TOPICS.state_estimation_at_scan,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.exploration_grid,
        ),
        command_sink="/lingtu/gazebo/cmd_vel",
        source_outputs=(
            "/model/thunder/odometry",
            "/lingtu/gazebo/raw/lidar_points",
            "/lingtu/gazebo/raw/lidar_scan",
        ),
        algorithm_entry_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
        ),
        algorithm_context_outputs=(
            TOPICS.state_estimation_at_scan,
            TOPICS.exploration_grid,
        ),
        lidar_extrinsic_profile="gazebo_proxy",
        slam_source="none",
        localization_source="gazebo_sim_odometry",
        mapping_source="gazebo_lidar_derived_map",
    ),
    "cmu_unity_external": DataSourceContract(
        name="cmu_unity_external",
        provider="cmu_unity",
        owns=("world_geometry", "physics", "external_registered_scan", "external_tare"),
        normalized_outputs=(
            TOPICS.odometry,
            TOPICS.state_estimation_at_scan,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.terrain_map_ext,
            TOPICS.exploration_way_point,
        ),
        command_sink="/cmd_vel",
        source_outputs=(
            "/state_estimation",
            "/state_estimation_at_scan",
            "/registered_scan",
            "/terrain_map",
            "/terrain_map_ext",
            "/way_point",
        ),
        algorithm_entry_outputs=(
            TOPICS.odometry,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
        ),
        algorithm_context_outputs=(
            TOPICS.state_estimation_at_scan,
            TOPICS.terrain_map_ext,
            TOPICS.exploration_way_point,
        ),
        lidar_extrinsic_profile=None,
        slam_source="external_cmu_registered_scan",
        localization_source="cmu_state_estimation",
        mapping_source="cmu_registered_scan_and_terrain_map_ext",
    ),
}

ADAPTER_TOPIC_ALIASES = {
    "livox_driver": (
        AdapterTopicAlias(
            source="/livox/lidar",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Livox driver output normalized for LingTu SLAM input.",
        ),
        AdapterTopicAlias(
            source="/livox/imu",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Livox IMU output normalized for LingTu SLAM input.",
        ),
    ),
    "fastlio2": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
            note="Fast-LIO2 current scan in body frame.",
        ),
        AdapterTopicAlias(
            source="/cloud_map",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="Fast-LIO2 local/world map cloud.",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
            note="Fast-LIO2 odometry normalized to odom->body.",
        ),
        AdapterTopicAlias(
            source="/imu/data",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
            note="Canonical IMU input for Fast-LIO2 launch/service paths.",
        ),
        AdapterTopicAlias(
            source="/lidar/scan",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
            note="Canonical LiDAR input for Fast-LIO2 launch/service paths.",
        ),
        AdapterTopicAlias(
            source="save_map",
            target=TOPICS.save_map_service,
            msg_format="service",
            note="Fast-LIO2 map-save service alias.",
        ),
    ),
    "pgo": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
    ),
    "localizer": (
        AdapterTopicAlias(
            source="/cloud_registered",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
        ),
        AdapterTopicAlias(
            source="/Odometry",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="map_cloud",
            target=TOPICS.saved_map_cloud,
            msg_format="map_cloud",
            note="Static saved map from localizer; never the live map cloud.",
        ),
    ),
    "pointlio": (
        AdapterTopicAlias(
            source="cloud_registered_body",
            target=TOPICS.registered_cloud,
            msg_format="registered_cloud",
            note="Point-LIO body-frame current scan.",
        ),
        AdapterTopicAlias(
            source="cloud_registered",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="Point-LIO world-frame registered map cloud.",
        ),
        AdapterTopicAlias(
            source="aft_mapped_to_init",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="livox/imu",
            target=TOPICS.imu,
            msg_format="sensor_msgs/msg/Imu",
        ),
        AdapterTopicAlias(
            source="livox/lidar",
            target=TOPICS.lidar_scan,
            msg_format="raw_livox_custom",
        ),
        AdapterTopicAlias(
            source="save_map",
            target=TOPICS.save_map_service,
            msg_format="service",
        ),
    ),
    "tare": (
        AdapterTopicAlias(
            source="/registered_scan",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
            note="TARE/CMU registered scan is a world/map cloud, not body scan.",
        ),
        AdapterTopicAlias(
            source="/terrain_map",
            target=TOPICS.terrain_map,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/terrain_map_ext",
            target=TOPICS.terrain_map_ext,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/state_estimation",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/state_estimation_at_scan",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/way_point",
            target=TOPICS.exploration_way_point,
            msg_format="geometry_msgs/msg/PointStamped",
        ),
    ),
    "terrain_analysis": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/map_clearing", target=TOPICS.map_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
    ),
    "terrain_analysis_ext": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/cloud_clearing", target=TOPICS.cloud_clearing, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
    ),
    "local_planner": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/cloud_map", target=TOPICS.map_cloud, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map", target=TOPICS.terrain_map, msg_format="map_cloud"),
        AdapterTopicAlias(source="/terrain_map_ext", target=TOPICS.terrain_map_ext, msg_format="map_cloud"),
        AdapterTopicAlias(source="/way_point", target=TOPICS.nav_way_point, msg_format="geometry_msgs/msg/PointStamped"),
        AdapterTopicAlias(source="/speed", target=TOPICS.speed, msg_format="std_msgs/msg/Float32"),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(source="/stop", target=TOPICS.stop, msg_format="std_msgs/msg/Bool"),
        AdapterTopicAlias(source="/slow_down", target=TOPICS.slow_down, msg_format="std_msgs/msg/Float32"),
        AdapterTopicAlias(source="/navigation_boundary", target=TOPICS.navigation_boundary, msg_format="geometry_msgs/msg/PolygonStamped"),
        AdapterTopicAlias(source="/added_obstacles", target=TOPICS.added_obstacles, msg_format="sensor_msgs/msg/PointCloud2"),
        AdapterTopicAlias(source="/check_obstacle", target=TOPICS.check_obstacle, msg_format="std_msgs/msg/Bool"),
    ),
    "path_follower": (
        AdapterTopicAlias(source="/Odometry", target=TOPICS.odometry, msg_format="odometry"),
        AdapterTopicAlias(source="/path", target=TOPICS.local_path, msg_format="nav_msgs/msg/Path"),
        AdapterTopicAlias(source="/cmd_vel", target=TOPICS.cmd_vel, msg_format="cmd_vel"),
        AdapterTopicAlias(source="/planner_status", target=TOPICS.planner_status, msg_format="std_msgs/msg/String"),
    ),
}

ADAPTER_RELAY_ALIASES = {
    "cmu_unity": (
        AdapterTopicAlias(
            source="/state_estimation",
            target=TOPICS.odometry,
            msg_format="odometry",
        ),
        AdapterTopicAlias(
            source="/state_estimation_at_scan",
            target=TOPICS.state_estimation_at_scan,
            msg_format="state_estimation_at_scan",
        ),
        AdapterTopicAlias(
            source="/registered_scan",
            target=TOPICS.map_cloud,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/terrain_map",
            target=TOPICS.terrain_map,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source="/terrain_map_ext",
            target=TOPICS.terrain_map_ext,
            msg_format="map_cloud",
        ),
        AdapterTopicAlias(
            source=TOPICS.cmd_vel,
            target="/cmd_vel",
            msg_format="geometry_msgs/msg/TwistStamped",
            note="Simulation-only command relay; never a hardware output or hardware ROS domain.",
        ),
    ),
}

PROFILE_DATA_SOURCE_BINDINGS = {
    "stub": ProfileDataSourceBinding(
        profile="stub",
        data_source="in_process_stub",
        mode="framework_test",
    ),
    "dev": ProfileDataSourceBinding(
        profile="dev",
        data_source="in_process_stub",
        mode="semantic_pipeline_test",
    ),
    "sim_nav": ProfileDataSourceBinding(
        profile="sim_nav",
        data_source="in_process_stub",
        mode="pure_python_navigation_sim",
    ),
    "sim": ProfileDataSourceBinding(
        profile="sim",
        data_source="mujoco_module_graph",
        mode="module_graph_simulation",
        note="In-process MuJoCo profile; not the Fast-LIO raw sensor gate.",
    ),
    "sim_mujoco_live": ProfileDataSourceBinding(
        profile="sim_mujoco_live",
        data_source="mujoco_fastlio2_live",
        mode="mujoco_raw_mid360_fastlio_live",
        note="First-class raw-sensor MuJoCo path: MID-360 pattern LiDAR + IMU -> Fast-LIO -> /nav/*.",
    ),
    "sim_gazebo": ProfileDataSourceBinding(
        profile="sim_gazebo",
        data_source="gazebo_industrial",
        mode="gazebo_ros_native_simulation",
    ),
    "sim_industrial": ProfileDataSourceBinding(
        profile="sim_industrial",
        data_source="gazebo_industrial",
        mode="gazebo_industrial_delivery_demo",
    ),
    "sim_cmu_tare": ProfileDataSourceBinding(
        profile="sim_cmu_tare",
        data_source="cmu_unity_external",
        mode="external_cmu_unity_tare_adapter",
    ),
    "map": ProfileDataSourceBinding(
        profile="map",
        data_source="real_s100p",
        mode="real_robot_mapping",
    ),
    "nav": ProfileDataSourceBinding(
        profile="nav",
        data_source="real_s100p",
        mode="real_robot_saved_map_navigation",
    ),
    "explore": ProfileDataSourceBinding(
        profile="explore",
        data_source="real_s100p",
        mode="real_robot_live_exploration",
    ),
    "tare_explore": ProfileDataSourceBinding(
        profile="tare_explore",
        data_source="real_s100p",
        mode="real_robot_tare_exploration",
    ),
    "super_lio": ProfileDataSourceBinding(
        profile="super_lio",
        data_source="real_s100p",
        mode="real_robot_super_lio_mapping",
    ),
    "super_lio_relocation": ProfileDataSourceBinding(
        profile="super_lio_relocation",
        data_source="real_s100p",
        mode="real_robot_super_lio_relocalization",
    ),
}


def adapter_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return adapter-only legacy/native aliases for one endpoint surface."""

    try:
        return ADAPTER_TOPIC_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_TOPIC_ALIASES))
        raise ValueError(f"unknown adapter alias surface {surface!r}; available: {available}") from exc


def adapter_remappings(surface: str) -> dict[str, str]:
    """Return source->target remappings for a native launch/service surface."""

    return {alias.source: alias.target for alias in adapter_aliases(surface)}


def adapter_relay_aliases(surface: str) -> tuple[AdapterTopicAlias, ...]:
    """Return bidirectional relay aliases for simulation bridge surfaces."""

    try:
        return ADAPTER_RELAY_ALIASES[surface]
    except KeyError as exc:
        available = ", ".join(sorted(ADAPTER_RELAY_ALIASES))
        raise ValueError(f"unknown adapter relay surface {surface!r}; available: {available}") from exc


def adapter_relay_remappings(surface: str) -> dict[str, str]:
    """Return source->target relays for a simulation bridge surface."""

    return {alias.source: alias.target for alias in adapter_relay_aliases(surface)}


def topic_formats(topic: str) -> tuple[str, ...]:
    """Return the declared message format names or ROS types for a topic."""

    try:
        return TOPIC_FORMATS[topic]
    except KeyError as exc:
        raise ValueError(f"topic {topic!r} has no declared runtime format") from exc


def map_frame_id() -> str:
    """Return the canonical fixed map frame used at runtime boundaries."""

    return FRAMES.map


def odom_frame_id() -> str:
    """Return the canonical odometry frame used at runtime boundaries."""

    return FRAMES.odom


def body_frame_id() -> str:
    """Return the canonical body frame used at runtime boundaries."""

    return FRAMES.body


def lidar_frame_id() -> str:
    """Return the canonical normalized LiDAR frame used at runtime boundaries."""

    return FRAMES.lidar


def real_lidar_frame_id() -> str:
    """Return the physical Livox frame before runtime normalization."""

    return FRAMES.real_lidar


def camera_frame_id() -> str:
    """Return the canonical camera frame used at runtime boundaries."""

    return FRAMES.camera


def simulator_world_frame_id() -> str:
    """Return the simulator fixed-world frame used at runtime boundaries."""

    return FRAMES.simulator_world


def profile_data_source(profile: str) -> ProfileDataSourceBinding:
    """Return the declared endpoint data-source binding for one CLI profile."""

    try:
        return PROFILE_DATA_SOURCE_BINDINGS[profile]
    except KeyError as exc:
        available = ", ".join(sorted(PROFILE_DATA_SOURCE_BINDINGS))
        raise ValueError(f"unknown profile data-source binding {profile!r}; available: {available}") from exc


def runtime_contract_manifest() -> dict[str, object]:
    """Export the full runtime contract as machine-checkable plain data."""

    return {
        "schema_version": "lingtu.runtime_interface.v1",
        "frames": asdict(FRAMES),
        "topics": asdict(TOPICS),
        "core_required_topics": CORE_REQUIRED_TOPICS,
        "lidar_extrinsics": {
            name: asdict(transform)
            for name, transform in LIDAR_EXTRINSICS.items()
        },
        "message_formats": {
            name: asdict(format_spec)
            for name, format_spec in MESSAGE_FORMATS.items()
        },
        "topic_formats": {
            topic: formats
            for topic, formats in TOPIC_FORMATS.items()
        },
        "artifact_formats": {
            name: asdict(format_spec)
            for name, format_spec in ARTIFACT_FORMATS.items()
        },
        "algorithm_interfaces": {
            name: asdict(interface)
            for name, interface in ALGORITHM_INTERFACES.items()
        },
        "data_sources": {
            name: asdict(source)
            for name, source in DATA_SOURCE_CONTRACTS.items()
        },
        "adapter_aliases": {
            name: [asdict(alias) for alias in aliases]
            for name, aliases in ADAPTER_TOPIC_ALIASES.items()
        },
        "adapter_relays": {
            name: [asdict(alias) for alias in aliases]
            for name, aliases in ADAPTER_RELAY_ALIASES.items()
        },
        "profile_data_sources": {
            name: asdict(binding)
            for name, binding in PROFILE_DATA_SOURCE_BINDINGS.items()
        },
    }


def rpy_to_quaternion_xyzw(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw radians to a ROS xyzw quaternion."""

    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotate_xyz_by_quaternion(
    point: tuple[float, float, float],
    quat_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Rotate one XYZ point by a normalized or unnormalized xyzw quaternion."""

    x, y, z = point
    qx, qy, qz, qw = quat_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return point
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_xyz(
    point: tuple[float, float, float],
    transform: Transform3D,
) -> tuple[float, float, float]:
    """Transform a child/local-frame point into the transform parent frame."""

    rx, ry, rz = rotate_xyz_by_quaternion(point, transform.rotation_xyzw)
    return (rx + transform.x, ry + transform.y, rz + transform.z)


def lidar_extrinsic(profile: str) -> Transform3D:
    """Return a named body->LiDAR extrinsic or raise a clear error."""

    try:
        return LIDAR_EXTRINSICS[profile]
    except KeyError as exc:
        available = ", ".join(sorted(LIDAR_EXTRINSICS))
        raise ValueError(f"unknown LiDAR extrinsic profile {profile!r}; available: {available}") from exc
