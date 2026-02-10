"""
nav-autonomy 子系统启动文件
包含: terrain_analysis + terrain_analysis_ext + local_planner + pathFollower

参数与 system_launch.py 保持一致
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    # -- 地形分析 (基础) --
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[{
            'vehicleHeight': 0.5,
            'terrainVoxelSize': 0.2,
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'checkCollision': True,
        }],
        remappings=[('/cloud_registered', '/cloud_map')]
    )

    # -- 地形分析 (扩展: 连通性检查 + 2.5D 高度图) --
    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[{
            'scanVoxelSize': 0.05,
            'decayTime': 2.0,
            'noDecayDis': 4.0,
            'clearingDis': 8.0,
            'useSorting': True,
            'quantileZ': 0.25,
            'vehicleHeight': 0.5,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'lowerBoundZ': -1.5,
            'upperBoundZ': 1.0,
            'disRatioZ': 0.2,
            'checkTerrainConn': True,
            'terrainUnderVehicle': -0.2,
            'terrainConnThre': 0.5,
            'ceilingFilteringThre': 2.0,
            'localTerrainMapRadius': 4.0,
        }],
        remappings=[
            ('/cloud_registered', '/cloud_map'),
            ('/terrain_map', '/terrain_map'),
        ]
    )

    # -- 局部规划器 --
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[{
            'pathFolder': path_folder,
            'vehicleLength': 1.0,
            'vehicleWidth': 0.6,
            'sensorOffsetX': 0.3,
            'sensorOffsetY': 0.0,
            'twoWayDrive': True,
            'laserVoxelSize': 0.05,
            'terrainVoxelSize': 0.2,
            'useTerrainAnalysis': True,
            'checkObstacle': True,
            'checkRotObstacle': False,
            'adjacentRange': 3.5,
            'obstacleHeightThre': 0.2,
            'groundHeightThre': 0.1,
            'costHeightThre1': 0.15,
            'costHeightThre2': 0.1,
            'useCost': False,
            'slowPathNumThre': 5,
            'slowGroupNumThre': 1,
            'pointPerPathThre': 2,
            'minRelZ': -0.5,
            'maxRelZ': 0.25,
            'dirWeight': 0.02,
            'dirThre': 90.0,
            'dirToVehicle': False,
            'pathScale': 1.0,
            'minPathScale': 0.75,
            'pathScaleStep': 0.25,
            'maxSpeed': 0.5,
            'pathScaleBySpeed': True,
            'minPathRange': 1.0,
            'pathRangeStep': 0.5,
            'pathRangeBySpeed': True,
            'pathCropByGoal': True,
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
            'joyToCheckObstacleDelay': 5.0,
            'freezeAng': 90.0,
            'freezeTime': 2.0,
            'omniDirGoalThre': 1.0,
            'goalClearRange': 0.5,
            'goalBehindRange': 0.8,
            'goalX': 0.0,
            'goalY': 0.0,
        }],
        remappings=[
            ('/cloud_registered', '/cloud_map'),
            ('/terrain_map', '/terrain_map'),
        ]
    )

    # -- 路径跟踪器 (Pure Pursuit) --
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[{
            'sensorOffsetX': 0.0,
            'sensorOffsetY': 0.0,
            'pubSkipNum': 1,
            'twoWayDrive': True,
            'lookAheadDis': 0.5,
            'baseLookAheadDis': 0.3,
            'lookAheadRatio': 0.5,
            'minLookAheadDis': 0.2,
            'maxLookAheadDis': 2.0,
            'yawRateGain': 7.5,
            'stopYawRateGain': 7.5,
            'maxYawRate': 45.0,
            'maxSpeed': 0.5,
            'maxAccel': 1.0,
            'switchTimeThre': 1.0,
            'dirDiffThre': 0.1,
            'omniDirGoalThre': 1.0,
            'omniDirDiffThre': 1.5,
            'stopDisThre': 0.2,
            'slowDwnDisThre': 1.0,
            'useInclRateToSlow': False,
            'inclRateThre': 120.0,
            'slowRate1': 0.25,
            'slowRate2': 0.5,
            'slowRate3': 0.75,
            'slowTime1': 2.0,
            'slowTime2': 2.0,
            'useInclToStop': False,
            'inclThre': 45.0,
            'stopTime': 5.0,
            'noRotAtStop': False,
            'noRotAtGoal': True,
            'autonomyMode': True,
            'autonomySpeed': 0.5,
            'joyToSpeedDelay': 2.0,
        }],
    )

    return LaunchDescription([
        terrain_analysis_node,
        terrain_analysis_ext_node,
        local_planner_node,
        path_follower_node,
    ])
