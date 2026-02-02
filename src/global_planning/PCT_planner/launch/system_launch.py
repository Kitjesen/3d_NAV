"""
    3D导航 PCT 规划系统 - 统一启动文件

    系统架构：
        感知层：
            - Livox MID360 激光雷达
            - FASTLIO2 (定位 + 建图)
            - Localizer (基于先验地图的定位)
        
        全局规划层：
            - PCT Global Planner (基于Tomogram的3D全局路径规划)
            - PCT Adapter (全局路径分段为局部航点)
        
        局部规划层：
            - Terrain Analysis (地形可通行性分析)
            - Local Planner (障碍物规避 + 轨迹规划)
            - Path Follower (路径跟踪 -> 速度指令)
        
        执行层：
            - Robot Driver (电机控制)
            - Joystick (手动控制接口)

    ================================================================================
    点云与地图数据流详解：
    ================================================================================

    1. 【原始激光扫描】
    来源: Livox MID360 雷达
    话题: /livox/lidar (原始点云)
    说明: 当前帧的原始激光扫描数据（sensor坐标系）
    
    2. 【当前帧配准点云】/cloud_registered
    来源: FASTLIO2 -> body_cloud
    坐标系: sensor/body 坐标系
    说明: 当前帧激光扫描，已经过IMU去畸变和点云配准
            这是"当前scan"，用于实时障碍物检测
    
    3. 【全局地图点云】/cloud_map  ⭐关键话题⭐
    来源: FASTLIO2 -> world_cloud  
    坐标系: map/world 全局坐标系
    说明: 累积的全局点云地图（所有历史扫描的集合）
            包含整个环境的3D结构信息
    
    使用者：
    ├─ Terrain Analysis       (订阅: /cloud_map)
    │  └─ 分析全局地形可通行性
    ├─ Terrain Analysis Ext   (订阅: /cloud_map)  
    │  └─ 生成增强地形高度图 -> /terrain_map
    └─ Local Planner          (订阅: /cloud_map)
        └─ 使用全局地图进行障碍物规避

    4. 【地形高度图】/terrain_map
    来源: Terrain Analysis Ext
    坐标系: map 全局坐标系
    说明: 从/cloud_map提取的2.5D地形表示
            每个网格存储高度和可通行性信息
    
    使用者：
    └─ Local Planner          (订阅: /terrain_map)
        └─ 结合地形信息选择可通行路径

    5. 【先验地图】map_path + ".pcd"
    来源: 离线建图（PGO保存）
    用途: Localizer精确定位
            PCT Global Planner路径规划（转为Tomogram）

    ================================================================================
    重要说明：
    - /cloud_registered: 当前scan（局部/实时），用于动态障碍物
    - /cloud_map: 全局地图（历史累积），用于地形分析和路径规划
    - 系统中所有订阅 /cloud_registered 的节点都被重映射为 /cloud_map
    目的：使用全局坐标系地图而非当前帧scan
    ================================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
        生成完整的3D导航系统启动描述
        
        包含模块：
        - FASTLIO2 (实时定位与建图)
        - PCT Global Planner (全局路径规划)
        - PCT Adapter (路径适配器)
        - Terrain Analysis (地形分析)
        - Local Planner (局部规划器)
        - Path Follower (路径跟踪器)
        - Robot Driver (机器人驱动)
    """

    # ==================== 启动参数配置 ====================
    # 地图路径（不含扩展名）
    # PCT规划器使用: map_path (自动加载.pickle或从.pcd构建)
    # Localizer使用: map_path + ".pcd" (用于定位)
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2',
        description='地图文件路径（不含扩展名）。PCT使用基础名称，Localizer加载.pcd文件'
    )
    map_path = LaunchConfiguration('map_path')
    
    # Tomogram地图构建参数（仅在.pickle不存在时从PCD构建时使用）
    tomogram_resolution_arg = DeclareLaunchArgument(
        'tomogram_resolution', 
        default_value='0.2', 
        description='体素网格分辨率 (米)，用于从PCD构建Tomogram'
    )
    tomogram_slice_dh_arg = DeclareLaunchArgument(
        'tomogram_slice_dh', 
        default_value='0.5', 
        description='层高步长 (米)，用于分层处理3D地图'
    )
    tomogram_ground_h_arg = DeclareLaunchArgument(
        'tomogram_ground_h', 
        default_value='0.0', 
        description='地面高度基准 (米)'
    )
    tomogram_resolution = LaunchConfiguration('tomogram_resolution')
    tomogram_slice_dh = LaunchConfiguration('tomogram_slice_dh')
    tomogram_ground_h = LaunchConfiguration('tomogram_ground_h')
    
    # 生成Localizer的PCD地图路径：map_path + ".pcd"
    static_map_path_pcd = PythonExpression(['"', map_path, '" + ".pcd"'])
    
    # --- 初始位姿参数 ---
    # 用于Localizer的初始定位位置（相对于地图坐标系）
    x_arg = DeclareLaunchArgument(
        'x', 
        default_value='0.0', 
        description='初始X坐标 (米)'
    )
    y_arg = DeclareLaunchArgument(
        'y', 
        default_value='0.0', 
        description='初始Y坐标 (米)'
    )
    z_arg = DeclareLaunchArgument(
        'z', 
        default_value='0.0', 
        description='初始Z坐标 (米)'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', 
        default_value='0.0', 
        description='初始偏航角 (弧度)'
    )
    
    initial_x = LaunchConfiguration('x')
    initial_y = LaunchConfiguration('y')
    initial_z = LaunchConfiguration('z')
    initial_yaw = LaunchConfiguration('yaw')

    # ==================== 配置文件路径 ====================
    
    # FASTLIO2配置文件
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])
    
    # Localizer配置文件
    localizer_config_path = PathJoinSubstitution([
        FindPackageShare("localizer"), "config", "localizer.yaml"
    ])

    # PCT路径适配器配置文件
    pct_adapter_config_path = PathJoinSubstitution([
        FindPackageShare("pct_adapters"), "config", "pct_path_adapter.yaml"
    ])

    # ==================== 可调参数集中区 ====================

    # PCT全局规划器参数
    pct_planner_params = {
        'map_file': map_path,                    # Tomogram地图文件
        'tomogram_resolution': tomogram_resolution,  # 体素分辨率
        'tomogram_slice_dh': tomogram_slice_dh,      # 层高步长
        'tomogram_ground_h': tomogram_ground_h,      # 地面高度
    }

    # 地形分析参数
    terrain_analysis_params = {
        'vehicleHeight': 0.5,              # 机器人高度 (米)
        'terrainVoxelSize': 0.2,           # 地形体素大小 (米)
        'obstacleHeightThre': 0.2,         # 障碍物高度阈值 (米)
        'groundHeightThre': 0.1,           # 地面高度阈值 (米)
        'checkCollision': True,            # 是否检测碰撞
    }

    # 扩展地形分析参数
    terrain_analysis_ext_params = {
        'scanVoxelSize': 0.05,             # 扫描体素大小 (米)
        'decayTime': 2.0,                  # 点云衰减时间 (秒)
        'noDecayDis': 4.0,                 # 不衰减距离 (米)
        'clearingDis': 8.0,                # 清除距离 (米)
        'useSorting': True,                # 使用排序优化
        'quantileZ': 0.25,                 # Z轴分位数
        'vehicleHeight': 0.5,              # 机器人高度 (米)
        'voxelPointUpdateThre': 100,       # 体素点更新阈值
        'voxelTimeUpdateThre': 2.0,        # 体素时间更新阈值 (秒)
        'lowerBoundZ': -1.5,               # Z轴下界 (米)
        'upperBoundZ': 1.0,                # Z轴上界 (米)
        'disRatioZ': 0.2,                  # Z轴距离比率
        'checkTerrainConn': True,          # 检查地形连通性
        'terrainUnderVehicle': -0.2,       # 机器人下方地形 (米)
        'terrainConnThre': 0.5,            # 地形连通阈值 (米)
        'ceilingFilteringThre': 2.0,       # 天花板过滤阈值 (米)
        'localTerrainMapRadius': 4.0,      # 局部地形图半径 (米)
    }

    # 获取局部规划器的路径库位置
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    # 局部规划器参数
    local_planner_params = {
        # --- 机器人物理参数 ---
        'pathFolder': path_folder,         # 预计算路径库位置
        'vehicleLength': 1.0,              # 机器人长度 (米)
        'vehicleWidth': 0.6,               # 机器人宽度 (米)
        'sensorOffsetX': 0.3,              # 传感器X偏移 (米)
        'sensorOffsetY': 0.0,              # 传感器Y偏移 (米)
        'twoWayDrive': True,               # 支持双向行驶
        
        # --- 点云处理参数 ---
        'laserVoxelSize': 0.05,            # 激光点云体素大小 (米)
        'terrainVoxelSize': 0.2,           # 地形体素大小 (米)
        'useTerrainAnalysis': True,        # 使用地形分析
        
        # --- 障碍物检测参数 ---
        'checkObstacle': True,             # 启用障碍物检测
        'checkRotObstacle': False,         # 检测旋转时的障碍物
        'adjacentRange': 3.5,              # 检测范围 (米)
        'obstacleHeightThre': 0.2,         # 障碍物高度阈值 (米)
        'groundHeightThre': 0.1,           # 地面高度阈值 (米)
        
        # --- 代价函数参数 ---
        'costHeightThre1': 0.15,           # 代价高度阈值1 (米)
        'costHeightThre2': 0.1,            # 代价高度阈值2 (米)
        'useCost': False,                  # 使用代价地图
        'slowPathNumThre': 5,              # 触发减速的路径数量阈值
        'slowGroupNumThre': 1,             # 触发减速的路径组阈值
        
        # --- 路径选择参数 ---
        'pointPerPathThre': 2,             # 每条路径点数阈值
        'minRelZ': -0.5,                   # 最小相对高度 (米)
        'maxRelZ': 0.25,                   # 最大相对高度 (米) - 关键：地面机器人的可通过高度范围
        'dirWeight': 0.02,                 # 方向权重
        'dirThre': 90.0,                   # 方向阈值 (度)
        'dirToVehicle': False,             # 方向基准是否指向车体
        'pathScale': 1.0,                  # 路径缩放比例
        'minPathScale': 0.75,              # 最小路径缩放
        'pathScaleStep': 0.25,             # 路径缩放步长
        
        # --- 速度控制参数 ---
        'maxSpeed': 0.5,                   # 最大速度 (米/秒)
        'pathScaleBySpeed': True,          # 根据速度缩放路径
        'minPathRange': 1.0,               # 最小路径范围 (米)
        'pathRangeStep': 0.5,              # 路径范围步长 (米)
        'pathRangeBySpeed': True,          # 根据速度调整路径范围
        'pathCropByGoal': True,            # 根据目标裁剪路径
        
        # --- 自主模式参数 ---
        'autonomyMode': True,              # 启用自主导航模式
        'autonomySpeed': 0.5,              # 自主模式速度 (米/秒)
        'joyToSpeedDelay': 2.0,            # 手柄切换延迟 (秒)
        'joyToCheckObstacleDelay': 5.0,    # 手柄避障切换延迟 (秒)
        'freezeAng': 90.0,                 # 冻结角度阈值 (度)
        'freezeTime': 2.0,                 # 冻结持续时间 (秒)
        'omniDirGoalThre': 1.0,            # 全向目标阈值 (米)
        'goalClearRange': 0.5,             # 目标清除范围 (米)
        'goalBehindRange': 0.8,            # 目标在车后范围 (米)
        'goalX': 0.0,                      # 默认目标X
        'goalY': 0.0,                      # 默认目标Y
    }

    # 路径跟踪器参数
    path_follower_params = {
        # --- 硬件接口 ---
        'sensorOffsetX': 0.0,              # 传感器X偏移 (米)
        'sensorOffsetY': 0.0,              # 传感器Y偏移 (米)
        'pubSkipNum': 1,                   # 发布跳帧数
        
        # --- 运动控制参数 ---
        'twoWayDrive': True,               # 支持双向行驶
        'lookAheadDis': 0.5,               # 前瞻距离 (米)
        'baseLookAheadDis': 0.3,           # 基础前瞻距离 (米)
        'lookAheadRatio': 0.5,             # 前瞻距离比例
        'minLookAheadDis': 0.2,            # 最小前瞻距离 (米)
        'maxLookAheadDis': 2.0,            # 最大前瞻距离 (米)
        'yawRateGain': 7.5,                # 转向增益
        'stopYawRateGain': 7.5,            # 停止时转向增益
        'maxYawRate': 45.0,                # 最大角速度 (度/秒)
        'maxSpeed': 0.5,                   # 最大速度 (米/秒)
        'maxAccel': 1.0,                   # 最大加速度 (米/秒²)
        'switchTimeThre': 1.0,             # 换向时间阈值 (秒)
        'dirDiffThre': 0.1,                # 方向差阈值
        'omniDirGoalThre': 1.0,            # 全向目标阈值 (米)
        'omniDirDiffThre': 1.5,            # 全向方向差阈值
        'stopDisThre': 0.2,                # 到达停止距离 (米)
        'slowDwnDisThre': 1.0,             # 减速距离 (米)
        'useInclRateToSlow': False,        # 使用角速度减速
        'inclRateThre': 120.0,             # 角速度阈值 (度/秒)
        'slowRate1': 0.25,                 # 减速比例1
        'slowRate2': 0.5,                  # 减速比例2
        'slowRate3': 0.75,                 # 减速比例3
        'slowTime1': 2.0,                  # 减速持续时间1 (秒)
        'slowTime2': 2.0,                  # 减速持续时间2 (秒)
        'useInclToStop': False,            # 使用倾角停止
        'inclThre': 45.0,                  # 倾角阈值 (度)
        'stopTime': 5.0,                   # 停止持续时间 (秒)
        'noRotAtStop': False,              # 停止时不旋转
        'noRotAtGoal': True,               # 到达目标不旋转
        
        # --- 自主模式参数 ---
        'autonomyMode': True,              # 启用自主导航模式
        'autonomySpeed': 0.5,              # 自主模式速度 (米/秒)
        'joyToSpeedDelay': 2.0,            # 手柄切换延迟 (秒)
    }

    # 手柄节点参数
    joy_params = {
        'device_id': LaunchConfiguration('joy_dev'),  # 设备ID
        'deadzone': 0.1,                              # 死区阈值
        'autorepeat_rate': 20.0,                      # 自动重复频率 (Hz)
    }
    
    # ==================== 感知层：定位与建图 ====================
    
    # FASTLIO2节点：实时激光SLAM
    # 功能：点云配准、里程计估计、实时地图构建
    # 输出：
    #   - /cloud_registered: 当前帧配准点云（body/sensor坐标系）
    #   - /cloud_map: 全局累积点云地图（map坐标系）⭐重要⭐
    #   - /Odometry: 位姿估计
    # 
    # 说明：
    #   - /cloud_registered 是"当前scan"，表示当前时刻的激光扫描
    #   - /cloud_map 是"全局地图"，包含所有历史扫描的累积
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path}],
        remappings=[
            # body_cloud是FASTLIO2内部名称，重映射为标准话题名
            ("body_cloud", "/cloud_registered"),  # 当前帧点云（已配准）
            ("world_cloud", "/cloud_map"),        # 全局地图点云（累积）⭐
            ("lio_odom", "/Odometry"),            # 里程计位姿估计
            ("lio_path", "/lio_path"),            # 运动轨迹
        ]
    )
    
    # Localizer节点：基于先验地图的定位
    # 功能：在已知地图中进行精确定位
    # 输入：
    #   - 先验地图PCD文件
    #   - 当前激光扫描
    # 输出：
    #   - 在地图坐标系下的精确位姿
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path},
            {"static_map_path": static_map_path_pcd},  # PGO保存的.pcd地图
            {"initial_x": initial_x},
            {"initial_y": initial_y},
            {"initial_z": initial_z},
            {"initial_yaw": initial_yaw}
        ],
    )
    
    # ==================== 激光雷达驱动 ====================
    
    # Livox MID360激光雷达驱动
    # 功能：采集360度视场的3D点云数据
    # 输出：原始点云数据
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        )
    )

    # ==================== 全局规划层 ====================
    
    # --- PCT全局规划器 ---
    # 功能：基于Tomogram 3D地图生成全局路径
    # 输入：
    #   - 目标点 (/goal_point)
    #   - Tomogram地图（3D体素地图）
    # 输出：
    #   - /pct_global_path: 完整的3D全局路径
    
    # 配置Python环境变量（包含PCT规划器的依赖路径）
    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')
    
    # 构建PYTHONPATH：包含planner目录（配置/库）和scripts目录（工具）
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[pct_planner_params],
        env={'PYTHONPATH': python_path}
    )
    
    # --- PCT路径适配器 ---
    # 功能：将全局路径切分为局部航点序列
    # 输入：
    #   - /pct_global_path: PCT生成的全局路径
    #   - /Odometry: 当前机器人位姿
    # 输出：
    #   - /way_point: 下一个航点目标
    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter.py',
        name='pct_path_adapter',
        output='screen',
        parameters=[pct_adapter_config_path]
    )


    # ==================== 地形分析层 ====================
    
    # --- 基础地形分析 ---
    # 功能：分析地形的基本可通行性
    # 输入：
    #   - /cloud_map: 全局坐标系点云（来自FASTLIO2的world_cloud）
    # 输出：
    #   - /terrain_cloud: 地形点云
    #   - /terrain_local: 局部地形信息
    # 
    # 说明：订阅/cloud_registered但通过remapping重定向到/cloud_map
    #       这样可以使用全局地图而非当前帧scan
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[terrain_analysis_params],
        remappings=[
            # 关键重映射：将订阅从当前scan改为全局地图
            ('/cloud_registered', '/cloud_map')
        ]
    )

    # --- 扩展地形分析 ---
    # 功能：高级地形建图与连通性分析
    # 输入：
    #   - /cloud_map: 全局坐标系点云（来自FASTLIO2的world_cloud）
    # 输出：
    #   - /terrain_map: 增强的地形高度图（2.5D表示）
    #   - /terrain_map_ext: 扩展地形信息
    # 
    # 工作流程：
    #   1. 接收全局点云地图 /cloud_map
    #   2. 构建体素化的地形表示
    #   3. 计算每个网格的高度、梯度、可通行性
    #   4. 检查地形连通性
    #   5. 输出2.5D地形高度图 -> /terrain_map
    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[terrain_analysis_ext_params],
        remappings=[
            # 关键重映射：将订阅从当前scan改为全局地图
            ('/cloud_registered', '/cloud_map'),  # 输入：全局点云地图
            ('/terrain_map', '/terrain_map')      # 输出：地形高度图
        ]
    )

    # ==================== 局部规划与控制层 ====================
    
    # --- 局部规划器 ---
    # 功能：实时障碍物规避 + 局部轨迹规划（2.5D导航）
    # 输入：
    #   - /way_point: 目标航点（来自PCT Adapter）
    #   - /cloud_map: 全局点云地图（来自FASTLIO2的world_cloud）
    #   - /terrain_map: 地形高度图（来自Terrain Analysis Ext）
    #   - /Odometry: 当前位姿
    # 输出：
    #   - /path: 局部轨迹（nav_msgs/Path）
    #   - /slow_down: 减速指令
    # 
    # 工作原理：
    #   1. 从预计算的路径库中选择候选路径
    #   2. 使用/cloud_map检测障碍物（全局地图信息）
    #   3. 使用/terrain_map评估地形可通行性（2.5D高度图）
    #   4. 根据目标方向、障碍物、地形综合评估路径
    #   5. 选择最优可通行路径
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[local_planner_params],
        remappings=[
            # 关键重映射：使用全局地图和地形图
            ('/cloud_registered', '/cloud_map'),  # 输入：全局点云地图（障碍物检测）
            ('/terrain_map', '/terrain_map')      # 输入：地形高度图（可通行性评估）
        ]
    )

    # --- 路径跟踪器 ---
    # 功能：Pure Pursuit路径跟踪算法，生成速度指令
    # 输入：
    #   - /path: 局部轨迹（来自Local Planner）
    #   - /Odometry: 当前位姿
    #   - /slow_down: 减速指令
    # 输出：
    #   - /cmd_vel: 速度指令（geometry_msgs/TwistStamped）
    # 
    # 工作原理：
    #   1. 在路径上寻找前瞻点（lookahead point）
    #   2. 计算到达前瞻点的转向角速度
    #   3. 根据路径曲率和障碍物调整线速度
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[path_follower_params],
        remappings=[
            # 订阅 /Odometry（来自FASTLIO2）
            # 发布 /cmd_vel（发送给机器人驱动）
        ]
    )
    
    # ==================== 手动控制接口 ====================
    
    # --- 手柄参数 ---
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='手柄设备ID（0 -> /dev/input/js0, 1 -> /dev/input/js1）'
    )

    # --- 手柄节点 ---
    # 功能：接收游戏手柄输入，用于手动控制
    # 
    # 发布话题：
    #   - /joy (sensor_msgs/Joy): 手柄状态信息
    #       └─ 包含所有按键和摇杆的实时状态
    # 
    # 订阅者（使用/joy话题的节点）：
    #   ├─ Local Planner (localPlanner)
    #   │  └─ 用途：手动/自主模式切换、障碍物检测开关、速度控制
    #   │  └─ 功能：
    #   │      - axes[2]: 自主模式开关 (>-0.1: 手动, <-0.1: 自主)
    #   │      - axes[5]: 障碍物检测开关 (>-0.1: 启用, <-0.1: 禁用)
    #   │      - axes[3], axes[4]: 手动方向控制（左摇杆）
    #   │
    #   └─ Path Follower (pathFollower)
    #      └─ 用途：手动驾驶控制、速度调节
    #      └─ 功能：
    #          - axes[3], axes[4]: 线速度和方向控制
    #          - axes[0]: 角速度控制（转向）
    #          - axes[2]: 自主模式开关
    #          - axes[5]: 手动模式开关
    # 
    # 用途总结：
    #   - 手动控制机器人移动（摇杆）
    #   - 自主/手动模式切换（扳机键）
    #   - 紧急停止和安全控制
    #   - 实时速度调节
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params]
    )
    
    # ==================== 坐标变换 ====================
    
    # 注意：在system_launch中，localizer_node会动态发布map->lidar变换
    # 不需要静态变换，否则会产生TF冲突（TF抖动）
    
    # ==================== 可视化工具 ====================
    
    # --- 可视化节点（可选）---
    # 功能：在RViz中可视化导航状态
    # 注意：当前已注释，根据需要启用
    # visualization_node = Node(
    #     package='visualization_tools',
    #     executable='visualizationTools',
    #     name='visualizationTools',
    #     output='screen',
    #     remappings=[
    #         ('/state_estimation', '/Odometry'),
    #         ('/registered_scan', '/cloud_registered')
    #     ]
    # )

    # ==================== 启动描述组装 ====================
    
    return LaunchDescription([
        # --- 启动参数 ---
        map_path_arg,                  # 地图路径参数
        tomogram_resolution_arg,       # Tomogram分辨率参数
        tomogram_slice_dh_arg,         # Tomogram层高参数
        tomogram_ground_h_arg,         # Tomogram地面高度参数
        x_arg, y_arg, z_arg, yaw_arg,  # 初始位姿参数
        joy_dev_arg,                   # 手柄设备参数
        
        # --- 感知层 ---
        livox_launch,                  # Livox MID360激光雷达驱动
        lio_node,                      # FASTLIO2 实时SLAM
        localizer_node,                # 基于先验地图的定位
        
        # --- 全局规划层 ---
        pct_planner,                   # PCT 3D全局路径规划器
        pct_adapter,                   # PCT路径适配器（全局→局部）
        
        # --- 地形分析层 ---
        terrain_analysis_node,         # 基础地形分析
        terrain_analysis_ext_node,     # 扩展地形分析
        
        # --- 局部规划与控制层 ---
        local_planner_node,            # 局部规划器（障碍物规避）
        path_follower_node,            # 路径跟踪器（Pure Pursuit）
        
        # --- 硬件与手动控制 ---
        # robot_driver,                # 机器人底盘驱动（已注释）
        joy_node,                      # 游戏手柄接口
        
        # --- 可视化 ---
    ])
