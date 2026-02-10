"""
nav-planning 子系统启动文件
包含: Localizer + PCT Global Planner + PCT Path Adapter

使用参数:
  map_path  - 地图路径 (不含扩展名)
  x, y, z, yaw - 初始位姿
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ── 参数声明 ──
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2',
        description='地图路径 (不含扩展名)'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    map_path = LaunchConfiguration('map_path')
    static_map_path_pcd = PythonExpression(['"', map_path, '" + ".pcd"'])

    localizer_config_path = PathJoinSubstitution([
        FindPackageShare("localizer"), "config", "localizer.yaml"
    ])
    pct_adapter_config_path = PathJoinSubstitution([
        FindPackageShare("pct_adapters"), "config", "pct_path_adapter.yaml"
    ])

    # ── Localizer 节点 ──
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path},
            {"static_map_path": static_map_path_pcd},
            {"initial_x": LaunchConfiguration('x')},
            {"initial_y": LaunchConfiguration('y')},
            {"initial_z": LaunchConfiguration('z')},
            {"initial_yaw": LaunchConfiguration('yaw')},
        ],
    )

    # ── PCT 全局规划器 ──
    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[{
            'map_file': map_path,
            'tomogram_resolution': 0.2,
            'tomogram_slice_dh': 0.5,
            'tomogram_ground_h': 0.0,
        }],
        # 重要: 使用 additional_env 而非 env
        # env={} 会替换整个进程环境，导致丢失 LD_LIBRARY_PATH → librcl_action.so 找不到
        # additional_env={} 只追加/覆盖指定变量，保留 LD_LIBRARY_PATH 等系统路径
        additional_env={'PYTHONPATH': python_path}
    )

    # ── PCT 路径适配器 ──
    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter.py',
        name='pct_path_adapter',
        output='screen',
        parameters=[pct_adapter_config_path]
    )

    return LaunchDescription([
        map_path_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        localizer_node,
        pct_planner,
        pct_adapter,
    ])
