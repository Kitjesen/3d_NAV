from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Save map after mapping (PGO publishes map->odom; save .pcd for Localizer in navigation):
#   ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/path/to/your_map.pcd'}"
# Then run navigation (system_launch) with: map_path:=/path/to/your_map  (no .pcd)


def generate_launch_description():
    """
    Launch file for Mapping Phase only.
    Use this to generate a map before running navigation.
    """

    # --- Livox Driver (MID360) ---
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        )
    )

    # Config Paths
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])

    pgo_config_path = PathJoinSubstitution([
        FindPackageShare("pgo"), "config", "pgo.yaml"
    ])
    
    # PGO Node (Pose Graph Optimization Backend)
    pgo_node = Node(
        package="pgo",
        namespace="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[{"config_path": pgo_config_path}]
    )
    
    # FASTLIO2 Node (Mapping Mode)
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path}],
        remappings=[
            ("body_cloud", "/cloud_registered"),
            ("lio_odom", "/Odometry"),
        ]
    )


    # NOTE: PGO node will publish TF: map -> odom dynamically after loop closure detection

    return LaunchDescription([
        livox_launch,
        lio_node,
        pgo_node
    ])
