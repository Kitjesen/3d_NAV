"""
【测试】PCT 规划 + Adapter + Local Planner + Path Follower，用于无真机时检查整条数据链。
无真定位时默认启动 fake_localization，在 RViz 用「2D Pose Estimate」设起点，「2D Goal Pose」设终点。
启动: ros2 launch pct_planner test/test_planning_launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_fake_localization_arg = DeclareLaunchArgument(
        'use_fake_localization',
        default_value='true',
        description='为 true 时启动 fake_localization（TF map->body，订阅 /initialpose）'
    )
    use_fake_localization = LaunchConfiguration('use_fake_localization', default='true')

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='spiral0.3_2',
        description='地图名（不含扩展名），会找 <map_path>.pcd 或 .pickle'
    )
    map_path = LaunchConfiguration('map_path')

    tomogram_resolution_arg = DeclareLaunchArgument('tomogram_resolution', default_value='0.2')
    tomogram_slice_dh_arg = DeclareLaunchArgument('tomogram_slice_dh', default_value='0.5')
    tomogram_ground_h_arg = DeclareLaunchArgument('tomogram_ground_h', default_value='0.0')
    tomogram_resolution = LaunchConfiguration('tomogram_resolution')
    tomogram_slice_dh = LaunchConfiguration('tomogram_slice_dh')
    tomogram_ground_h = LaunchConfiguration('tomogram_ground_h')

    fake_localization_node = Node(
        package='pct_planner',
        executable='fake_localization.py',
        name='fake_localization',
        output='screen',
        condition=IfCondition(use_fake_localization),
    )

    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"
    ld_lib_path = f"/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH', '')}"
    home_dir = os.environ.get('HOME', '/home/sunrise')

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[{
            'map_file': map_path,
            'map_frame': 'map',
            'robot_frame': 'body',
            'tomogram_resolution': tomogram_resolution,
            'tomogram_slice_dh': tomogram_slice_dh,
            'tomogram_ground_h': tomogram_ground_h,
        }],
        env={
            'PYTHONPATH': python_path,
            'LD_LIBRARY_PATH': ld_lib_path,
            'HOME': home_dir
        }
    )

    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter.py',
        name='pct_path_adapter',
        output='screen',
        parameters=[{
            'waypoint_distance': 0.5,
            'arrival_threshold': 0.5,
            'lookahead_dist': 1.0
        }]
    )

    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': path_folder,
            'vehicleLength': 0.6,
            'vehicleWidth': 0.6,
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': False,
            'checkObstacle': False,
            'checkRotObstacle': False,
            'adjacentRange': 3.5,
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'costHeightThre1': 0.15,
            'costHeightThre2': 0.1,
            'useCost': False,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'maxSpeed': 0.5,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
        }],
        remappings=[
            ('/terrain_map', '/terrain_map_dummy'),
            ('/cloud_registered', '/cloud_map')
        ]
    )

    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'realRobot': False,
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'pubSkipNum': 1,
            'twoWayDrive': True,
            'lookAheadDis': 0.5,
            'maxSpeed': 1.0,
            'maxAccel': 1.0,
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
        }]
    )

    return LaunchDescription([
        use_fake_localization_arg,
        map_path_arg,
        tomogram_resolution_arg,
        tomogram_slice_dh_arg,
        tomogram_ground_h_arg,
        fake_localization_node,
        pct_planner,
        pct_adapter,
        local_planner_node,
        path_follower_node
    ])
