"""
    3D瀵艰埅 PCT 瑙勫垝绯荤粺 - 缁熶竴鍚姩鏂囦欢

    绯荤粺鏋舵瀯锛?        鎰熺煡灞傦細
            - Livox MID360 婵€鍏夐浄杈?            - FASTLIO2 (瀹氫綅 + 寤哄浘)
            - Localizer (鍩轰簬鍏堥獙鍦板浘鐨勫畾浣?

        鍏ㄥ眬瑙勫垝灞傦細
            - PCT Global Planner (鍩轰簬Tomogram鐨?D鍏ㄥ眬璺緞瑙勫垝)
            - PCT Adapter (鍏ㄥ眬璺緞鍒嗘涓哄眬閮ㄨ埅鐐?

        灞€閮ㄨ鍒掑眰锛?            - Terrain Analysis (鍦板舰鍙€氳鎬у垎鏋?
            - Local Planner (闅滅鐗╄閬?+ 杞ㄨ抗瑙勫垝)
            - Path Follower (璺緞璺熻釜 -> 閫熷害鎸囦护)

        鎵ц灞傦細
            - Robot Driver (鐢垫満鎺у埗)
            - Joystick (鎵嬪姩鎺у埗鎺ュ彛)

    ================================================================================
    鐐逛簯涓庡湴鍥炬暟鎹祦璇﹁В锛?    ================================================================================

    1. 銆愬師濮嬫縺鍏夋壂鎻忋€?    鏉ユ簮: Livox MID360 闆疯揪
    璇濋: /livox/lidar (鍘熷鐐逛簯)
    璇存槑: 褰撳墠甯х殑鍘熷婵€鍏夋壂鎻忔暟鎹紙sensor鍧愭爣绯伙級

    2. 銆愬綋鍓嶅抚閰嶅噯鐐逛簯銆?cloud_registered
    鏉ユ簮: FASTLIO2 -> body_cloud
    鍧愭爣绯? sensor/body 鍧愭爣绯?    璇存槑: 褰撳墠甯ф縺鍏夋壂鎻忥紝宸茬粡杩嘔MU鍘荤暩鍙樺拰鐐逛簯閰嶅噯
            杩欐槸"褰撳墠scan"锛岀敤浜庡疄鏃堕殰纰嶇墿妫€娴?
    3. 銆愬叏灞€鍦板浘鐐逛簯銆?cloud_map  猸愬叧閿瘽棰樷瓙
    鏉ユ簮: FASTLIO2 -> world_cloud
    鍧愭爣绯? map/world 鍏ㄥ眬鍧愭爣绯?    璇存槑: 绱Н鐨勫叏灞€鐐逛簯鍦板浘锛堟墍鏈夊巻鍙叉壂鎻忕殑闆嗗悎锛?            鍖呭惈鏁翠釜鐜鐨?D缁撴瀯淇℃伅

    浣跨敤鑰咃細
    鈹溾攢 Terrain Analysis       (璁㈤槄: /cloud_map)
    鈹? 鈹斺攢 鍒嗘瀽鍏ㄥ眬鍦板舰鍙€氳鎬?    鈹溾攢 Terrain Analysis Ext   (璁㈤槄: /cloud_map)
    鈹? 鈹斺攢 鐢熸垚澧炲己鍦板舰楂樺害鍥?-> /terrain_map
    鈹斺攢 Local Planner          (璁㈤槄: /cloud_map)
        鈹斺攢 浣跨敤鍏ㄥ眬鍦板浘杩涜闅滅鐗╄閬?
    4. 銆愬湴褰㈤珮搴﹀浘銆?terrain_map
    鏉ユ簮: Terrain Analysis Ext
    鍧愭爣绯? map 鍏ㄥ眬鍧愭爣绯?    璇存槑: 浠?cloud_map鎻愬彇鐨?.5D鍦板舰琛ㄧず
            姣忎釜缃戞牸瀛樺偍楂樺害鍜屽彲閫氳鎬т俊鎭?
    浣跨敤鑰咃細
    鈹斺攢 Local Planner          (璁㈤槄: /terrain_map)
        鈹斺攢 缁撳悎鍦板舰淇℃伅閫夋嫨鍙€氳璺緞

    5. 銆愬厛楠屽湴鍥俱€憁ap_path + ".pcd"
    鏉ユ簮: 绂荤嚎寤哄浘锛圥GO淇濆瓨锛?    鐢ㄩ€? Localizer绮剧‘瀹氫綅
            PCT Global Planner璺緞瑙勫垝锛堣浆涓篢omogram锛?
    ================================================================================
    閲嶈璇存槑锛?    - /cloud_registered: 褰撳墠scan锛堝眬閮?瀹炴椂锛夛紝鐢ㄤ簬鍔ㄦ€侀殰纰嶇墿
    - /cloud_map: 鍏ㄥ眬鍦板浘锛堝巻鍙茬疮绉級锛岀敤浜庡湴褰㈠垎鏋愬拰璺緞瑙勫垝
    - 绯荤粺涓墍鏈夎闃?/cloud_registered 鐨勮妭鐐归兘琚噸鏄犲皠涓?/cloud_map
    鐩殑锛氫娇鐢ㄥ叏灞€鍧愭爣绯诲湴鍥捐€岄潪褰撳墠甯can
    ================================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
        鐢熸垚瀹屾暣鐨?D瀵艰埅绯荤粺鍚姩鎻忚堪

        鍖呭惈妯″潡锛?        - FASTLIO2 (瀹炴椂瀹氫綅涓庡缓鍥?
        - PCT Global Planner (鍏ㄥ眬璺緞瑙勫垝)
        - PCT Adapter (璺緞閫傞厤鍣?
        - Terrain Analysis (鍦板舰鍒嗘瀽)
        - Local Planner (灞€閮ㄨ鍒掑櫒)
        - Path Follower (璺緞璺熻釜鍣?
        - Robot Driver (鏈哄櫒浜洪┍鍔?
    """

    # ==================== 鍚姩鍙傛暟閰嶇疆 ====================
    # 鍦板浘璺緞锛堜笉鍚墿灞曞悕锛?    # PCT瑙勫垝鍣ㄤ娇鐢? map_path (鑷姩鍔犺浇.pickle鎴栦粠.pcd鏋勫缓)
    # Localizer浣跨敤: map_path + ".pcd" (鐢ㄤ簬瀹氫綅)
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/sunrise/data/SLAM/navigation/src/global_planning/pct_planner/rsc/tomogram/spiral0.3_2',
        description='鍦板浘鏂囦欢璺緞锛堜笉鍚墿灞曞悕锛夈€侾CT浣跨敤鍩虹鍚嶇О锛孡ocalizer鍔犺浇.pcd鏂囦欢'
    )
    map_path = LaunchConfiguration('map_path')

    # Tomogram鍦板浘鏋勫缓鍙傛暟锛堜粎鍦?pickle涓嶅瓨鍦ㄦ椂浠嶱CD鏋勫缓鏃朵娇鐢級
    tomogram_resolution_arg = DeclareLaunchArgument(
        'tomogram_resolution',
        default_value='0.2',
        description='浣撶礌缃戞牸鍒嗚鲸鐜?(绫?锛岀敤浜庝粠PCD鏋勫缓Tomogram'
    )
    tomogram_slice_dh_arg = DeclareLaunchArgument(
        'tomogram_slice_dh',
        default_value='0.5',
        description='灞傞珮姝ラ暱 (绫?锛岀敤浜庡垎灞傚鐞?D鍦板浘'
    )
    tomogram_ground_h_arg = DeclareLaunchArgument(
        'tomogram_ground_h',
        default_value='0.0',
        description='鍦伴潰楂樺害鍩哄噯 (绫?'
    )
    tomogram_resolution = LaunchConfiguration('tomogram_resolution')
    tomogram_slice_dh = LaunchConfiguration('tomogram_slice_dh')
    tomogram_ground_h = LaunchConfiguration('tomogram_ground_h')

    # 鐢熸垚Localizer鐨凱CD鍦板浘璺緞锛歮ap_path + ".pcd"
    static_map_path_pcd = PythonExpression(['"', map_path, '" + ".pcd"'])

    # --- 鍒濆浣嶅Э鍙傛暟 ---
    # 鐢ㄤ簬Localizer鐨勫垵濮嬪畾浣嶄綅缃紙鐩稿浜庡湴鍥惧潗鏍囩郴锛?    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='鍒濆X鍧愭爣 (绫?'
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='鍒濆Y鍧愭爣 (绫?'
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='鍒濆Z鍧愭爣 (绫?'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='鍒濆鍋忚埅瑙?(寮у害)'
    )

    initial_x = LaunchConfiguration('x')
    initial_y = LaunchConfiguration('y')
    initial_z = LaunchConfiguration('z')
    initial_yaw = LaunchConfiguration('yaw')

    # ==================== 閰嶇疆鏂囦欢璺緞 ====================

    # FASTLIO2閰嶇疆鏂囦欢
    lio_config_path = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "config", "lio.yaml"
    ])

    # Localizer閰嶇疆鏂囦欢
    localizer_config_path = PathJoinSubstitution([
        FindPackageShare("localizer"), "config", "localizer.yaml"
    ])

    # PCT璺緞閫傞厤鍣ㄩ厤缃枃浠?    pct_adapter_config_path = PathJoinSubstitution([
        FindPackageShare("pct_adapters"), "config", "pct_path_adapter.yaml"
    ])

    # ==================== 鍙皟鍙傛暟闆嗕腑鍖?====================

    # PCT鍏ㄥ眬瑙勫垝鍣ㄥ弬鏁?    pct_planner_params = {
        'map_file': map_path,                    # Tomogram鍦板浘鏂囦欢
        'tomogram_resolution': tomogram_resolution,  # 浣撶礌鍒嗚鲸鐜?        'tomogram_slice_dh': tomogram_slice_dh,      # 灞傞珮姝ラ暱
        'tomogram_ground_h': tomogram_ground_h,      # 鍦伴潰楂樺害
    }

    # 鍦板舰鍒嗘瀽鍙傛暟
    terrain_analysis_params = {
        'vehicleHeight': 0.5,              # 鏈哄櫒浜洪珮搴?(绫?
        'terrainVoxelSize': 0.2,           # 鍦板舰浣撶礌澶у皬 (绫?
        'obstacleHeightThre': 0.2,         # 闅滅鐗╅珮搴﹂槇鍊?(绫?
        'groundHeightThre': 0.1,           # 鍦伴潰楂樺害闃堝€?(绫?
        'checkCollision': True,            # 鏄惁妫€娴嬬鎾?    }

    # 鎵╁睍鍦板舰鍒嗘瀽鍙傛暟
    terrain_analysis_ext_params = {
        'scanVoxelSize': 0.05,             # 鎵弿浣撶礌澶у皬 (绫?
        'decayTime': 2.0,                  # 鐐逛簯琛板噺鏃堕棿 (绉?
        'noDecayDis': 4.0,                 # 涓嶈“鍑忚窛绂?(绫?
        'clearingDis': 8.0,                # 娓呴櫎璺濈 (绫?
        'useSorting': True,                # 浣跨敤鎺掑簭浼樺寲
        'quantileZ': 0.25,                 # Z杞村垎浣嶆暟
        'vehicleHeight': 0.5,              # 鏈哄櫒浜洪珮搴?(绫?
        'voxelPointUpdateThre': 100,       # 浣撶礌鐐规洿鏂伴槇鍊?        'voxelTimeUpdateThre': 2.0,        # 浣撶礌鏃堕棿鏇存柊闃堝€?(绉?
        'lowerBoundZ': -1.5,               # Z杞翠笅鐣?(绫?
        'upperBoundZ': 1.0,                # Z杞翠笂鐣?(绫?
        'disRatioZ': 0.2,                  # Z杞磋窛绂绘瘮鐜?        'checkTerrainConn': True,          # 妫€鏌ュ湴褰㈣繛閫氭€?        'terrainUnderVehicle': -0.2,       # 鏈哄櫒浜轰笅鏂瑰湴褰?(绫?
        'terrainConnThre': 0.5,            # 鍦板舰杩為€氶槇鍊?(绫?
        'ceilingFilteringThre': 2.0,       # 澶╄姳鏉胯繃婊ら槇鍊?(绫?
        'localTerrainMapRadius': 4.0,      # 灞€閮ㄥ湴褰㈠浘鍗婂緞 (绫?
    }

    # 鑾峰彇灞€閮ㄨ鍒掑櫒鐨勮矾寰勫簱浣嶇疆
    local_planner_share = get_package_share_directory('local_planner')
    path_folder = os.path.join(local_planner_share, 'paths')

    # 灞€閮ㄨ鍒掑櫒鍙傛暟
    local_planner_params = {
        # --- 鏈哄櫒浜虹墿鐞嗗弬鏁?---
        'pathFolder': path_folder,         # 棰勮绠楄矾寰勫簱浣嶇疆
        'vehicleLength': 1.0,              # 鏈哄櫒浜洪暱搴?(绫?
        'vehicleWidth': 0.6,               # 鏈哄櫒浜哄搴?(绫?
        'sensorOffsetX': 0.3,              # 浼犳劅鍣╔鍋忕Щ (绫?
        'sensorOffsetY': 0.0,              # 浼犳劅鍣╕鍋忕Щ (绫?
        'twoWayDrive': True,               # 鏀寔鍙屽悜琛岄┒

        # --- 鐐逛簯澶勭悊鍙傛暟 ---
        'laserVoxelSize': 0.05,            # 婵€鍏夌偣浜戜綋绱犲ぇ灏?(绫?
        'terrainVoxelSize': 0.2,           # 鍦板舰浣撶礌澶у皬 (绫?
        'useTerrainAnalysis': True,        # 浣跨敤鍦板舰鍒嗘瀽

        # --- 闅滅鐗╂娴嬪弬鏁?---
        'checkObstacle': True,             # 鍚敤闅滅鐗╂娴?        'checkRotObstacle': False,         # 妫€娴嬫棆杞椂鐨勯殰纰嶇墿
        'adjacentRange': 3.5,              # 妫€娴嬭寖鍥?(绫?
        'obstacleHeightThre': 0.2,         # 闅滅鐗╅珮搴﹂槇鍊?(绫?
        'groundHeightThre': 0.1,           # 鍦伴潰楂樺害闃堝€?(绫?

        # --- 浠ｄ环鍑芥暟鍙傛暟 ---
        'costHeightThre1': 0.15,           # 浠ｄ环楂樺害闃堝€? (绫?
        'costHeightThre2': 0.1,            # 浠ｄ环楂樺害闃堝€? (绫?
        'useCost': False,                  # 浣跨敤浠ｄ环鍦板浘
        'slowPathNumThre': 5,              # 瑙﹀彂鍑忛€熺殑璺緞鏁伴噺闃堝€?        'slowGroupNumThre': 1,             # 瑙﹀彂鍑忛€熺殑璺緞缁勯槇鍊?
        # --- 璺緞閫夋嫨鍙傛暟 ---
        'pointPerPathThre': 2,             # 姣忔潯璺緞鐐规暟闃堝€?        'minRelZ': -0.5,                   # 鏈€灏忕浉瀵归珮搴?(绫?
        'maxRelZ': 0.25,                   # 鏈€澶х浉瀵归珮搴?(绫? - 鍏抽敭锛氬湴闈㈡満鍣ㄤ汉鐨勫彲閫氳繃楂樺害鑼冨洿
        'dirWeight': 0.02,                 # 鏂瑰悜鏉冮噸
        'dirThre': 90.0,                   # 鏂瑰悜闃堝€?(搴?
        'dirToVehicle': False,             # 鏂瑰悜鍩哄噯鏄惁鎸囧悜杞︿綋
        'pathScale': 1.0,                  # 璺緞缂╂斁姣斾緥
        'minPathScale': 0.75,              # 鏈€灏忚矾寰勭缉鏀?        'pathScaleStep': 0.25,             # 璺緞缂╂斁姝ラ暱

        # --- 閫熷害鎺у埗鍙傛暟 ---
        'maxSpeed': 0.5,                   # 鏈€澶ч€熷害 (绫?绉?
        'pathScaleBySpeed': True,          # 鏍规嵁閫熷害缂╂斁璺緞
        'minPathRange': 1.0,               # 鏈€灏忚矾寰勮寖鍥?(绫?
        'pathRangeStep': 0.5,              # 璺緞鑼冨洿姝ラ暱 (绫?
        'pathRangeBySpeed': True,          # 鏍规嵁閫熷害璋冩暣璺緞鑼冨洿
        'pathCropByGoal': True,            # 鏍规嵁鐩爣瑁佸壀璺緞

        # --- 鑷富妯″紡鍙傛暟 ---
        'autonomyMode': True,              # 鍚敤鑷富瀵艰埅妯″紡
        'autonomySpeed': 0.5,              # 鑷富妯″紡閫熷害 (绫?绉?
        'joyToSpeedDelay': 2.0,            # 鎵嬫焺鍒囨崲寤惰繜 (绉?
        'joyToCheckObstacleDelay': 5.0,    # 鎵嬫焺閬块殰鍒囨崲寤惰繜 (绉?
        'freezeAng': 90.0,                 # 鍐荤粨瑙掑害闃堝€?(搴?
        'freezeTime': 2.0,                 # 鍐荤粨鎸佺画鏃堕棿 (绉?
        'omniDirGoalThre': 1.0,            # 鍏ㄥ悜鐩爣闃堝€?(绫?
        'goalClearRange': 0.5,             # 鐩爣娓呴櫎鑼冨洿 (绫?
        'goalBehindRange': 0.8,            # 鐩爣鍦ㄨ溅鍚庤寖鍥?(绫?
        'goalX': 0.0,                      # 榛樿鐩爣X
        'goalY': 0.0,                      # 榛樿鐩爣Y
    }

    # 璺緞璺熻釜鍣ㄥ弬鏁?    path_follower_params = {
        # --- 纭欢鎺ュ彛 ---
        'sensorOffsetX': 0.0,              # 浼犳劅鍣╔鍋忕Щ (绫?
        'sensorOffsetY': 0.0,              # 浼犳劅鍣╕鍋忕Щ (绫?
        'pubSkipNum': 1,                   # 鍙戝竷璺冲抚鏁?
        # --- 杩愬姩鎺у埗鍙傛暟 ---
        'twoWayDrive': True,               # 鏀寔鍙屽悜琛岄┒
        'lookAheadDis': 0.5,               # 鍓嶇灮璺濈 (绫?
        'baseLookAheadDis': 0.3,           # 鍩虹鍓嶇灮璺濈 (绫?
        'lookAheadRatio': 0.5,             # 鍓嶇灮璺濈姣斾緥
        'minLookAheadDis': 0.2,            # 鏈€灏忓墠鐬昏窛绂?(绫?
        'maxLookAheadDis': 2.0,            # 鏈€澶у墠鐬昏窛绂?(绫?
        'yawRateGain': 7.5,                # 杞悜澧炵泭
        'stopYawRateGain': 7.5,            # 鍋滄鏃惰浆鍚戝鐩?        'maxYawRate': 45.0,                # 鏈€澶ц閫熷害 (搴?绉?
        'maxSpeed': 0.5,                   # 鏈€澶ч€熷害 (绫?绉?
        'maxAccel': 1.0,                   # 鏈€澶у姞閫熷害 (绫?绉捖?
        'switchTimeThre': 1.0,             # 鎹㈠悜鏃堕棿闃堝€?(绉?
        'dirDiffThre': 0.1,                # 鏂瑰悜宸槇鍊?        'omniDirGoalThre': 1.0,            # 鍏ㄥ悜鐩爣闃堝€?(绫?
        'omniDirDiffThre': 1.5,            # 鍏ㄥ悜鏂瑰悜宸槇鍊?        'stopDisThre': 0.2,                # 鍒拌揪鍋滄璺濈 (绫?
        'slowDwnDisThre': 1.0,             # 鍑忛€熻窛绂?(绫?
        'useInclRateToSlow': False,        # 浣跨敤瑙掗€熷害鍑忛€?        'inclRateThre': 120.0,             # 瑙掗€熷害闃堝€?(搴?绉?
        'slowRate1': 0.25,                 # 鍑忛€熸瘮渚?
        'slowRate2': 0.5,                  # 鍑忛€熸瘮渚?
        'slowRate3': 0.75,                 # 鍑忛€熸瘮渚?
        'slowTime1': 2.0,                  # 鍑忛€熸寔缁椂闂? (绉?
        'slowTime2': 2.0,                  # 鍑忛€熸寔缁椂闂? (绉?
        'useInclToStop': False,            # 浣跨敤鍊捐鍋滄
        'inclThre': 45.0,                  # 鍊捐闃堝€?(搴?
        'stopTime': 5.0,                   # 鍋滄鎸佺画鏃堕棿 (绉?
        'noRotAtStop': False,              # 鍋滄鏃朵笉鏃嬭浆
        'noRotAtGoal': True,               # 鍒拌揪鐩爣涓嶆棆杞?
        # --- 鑷富妯″紡鍙傛暟 ---
        'autonomyMode': True,              # 鍚敤鑷富瀵艰埅妯″紡
        'autonomySpeed': 0.5,              # 鑷富妯″紡閫熷害 (绫?绉?
        'joyToSpeedDelay': 2.0,            # 鎵嬫焺鍒囨崲寤惰繜 (绉?
    }

    # 鎵嬫焺鑺傜偣鍙傛暟
    joy_params = {
        'device_id': LaunchConfiguration('joy_dev'),  # 璁惧ID
        'deadzone': 0.1,                              # 姝诲尯闃堝€?        'autorepeat_rate': 20.0,                      # 鑷姩閲嶅棰戠巼 (Hz)
    }

    # ==================== 鎰熺煡灞傦細瀹氫綅涓庡缓鍥?====================

    # FASTLIO2鑺傜偣锛氬疄鏃舵縺鍏塖LAM
    # 鍔熻兘锛氱偣浜戦厤鍑嗐€侀噷绋嬭浼拌銆佸疄鏃跺湴鍥炬瀯寤?    # 杈撳嚭锛?    #   - /cloud_registered: 褰撳墠甯ч厤鍑嗙偣浜戯紙body/sensor鍧愭爣绯伙級
    #   - /cloud_map: 鍏ㄥ眬绱Н鐐逛簯鍦板浘锛坢ap鍧愭爣绯伙級猸愰噸瑕佲瓙
    #   - /Odometry: 浣嶅Э浼拌
    #
    # 璇存槑锛?    #   - /cloud_registered 鏄?褰撳墠scan"锛岃〃绀哄綋鍓嶆椂鍒荤殑婵€鍏夋壂鎻?    #   - /cloud_map 鏄?鍏ㄥ眬鍦板浘"锛屽寘鍚墍鏈夊巻鍙叉壂鎻忕殑绱Н
    lio_node = Node(
        package="fastlio2",
        namespace="fastlio2",
        executable="lio_node",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": lio_config_path}],
        remappings=[
            # body_cloud鏄疐ASTLIO2鍐呴儴鍚嶇О锛岄噸鏄犲皠涓烘爣鍑嗚瘽棰樺悕
            ("body_cloud", "/cloud_registered"),  # 褰撳墠甯х偣浜戯紙宸查厤鍑嗭級
            ("world_cloud", "/cloud_map"),        # 鍏ㄥ眬鍦板浘鐐逛簯锛堢疮绉級猸?            ("lio_odom", "/Odometry"),            # 閲岀▼璁′綅濮夸及璁?            ("lio_path", "/lio_path"),            # 杩愬姩杞ㄨ抗
        ]
    )

    # Localizer鑺傜偣锛氬熀浜庡厛楠屽湴鍥剧殑瀹氫綅
    # 鍔熻兘锛氬湪宸茬煡鍦板浘涓繘琛岀簿纭畾浣?    # 杈撳叆锛?    #   - 鍏堥獙鍦板浘PCD鏂囦欢
    #   - 褰撳墠婵€鍏夋壂鎻?    # 杈撳嚭锛?    #   - 鍦ㄥ湴鍥惧潗鏍囩郴涓嬬殑绮剧‘浣嶅Э
    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[
            {"config_path": localizer_config_path},
            {"static_map_path": static_map_path_pcd},  # PGO淇濆瓨鐨?pcd鍦板浘
            {"initial_x": initial_x},
            {"initial_y": initial_y},
            {"initial_z": initial_z},
            {"initial_yaw": initial_yaw}
        ],
    )

    # ==================== 婵€鍏夐浄杈鹃┍鍔?====================

    # Livox MID360婵€鍏夐浄杈鹃┍鍔?    # 鍔熻兘锛氶噰闆?60搴﹁鍦虹殑3D鐐逛簯鏁版嵁
    # 杈撳嚭锛氬師濮嬬偣浜戞暟鎹?    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch',
                'msg_MID360_launch.py'
            ])
        )
    )

    # ==================== 鍏ㄥ眬瑙勫垝灞?====================

    # --- PCT鍏ㄥ眬瑙勫垝鍣?---
    # 鍔熻兘锛氬熀浜嶵omogram 3D鍦板浘鐢熸垚鍏ㄥ眬璺緞
    # 杈撳叆锛?    #   - 鐩爣鐐?(/goal_point)
    #   - Tomogram鍦板浘锛?D浣撶礌鍦板浘锛?    # 杈撳嚭锛?    #   - /pct_global_path: 瀹屾暣鐨?D鍏ㄥ眬璺緞

    # 閰嶇疆Python鐜鍙橀噺锛堝寘鍚玃CT瑙勫垝鍣ㄧ殑渚濊禆璺緞锛?    pct_share = get_package_share_directory('pct_planner')
    planner_dir = os.path.join(pct_share, 'planner')
    scripts_dir = os.path.join(planner_dir, 'scripts')
    lib_dir = os.path.join(planner_dir, 'lib')

    # 鏋勫缓PYTHONPATH锛氬寘鍚玴lanner鐩綍锛堥厤缃?搴擄級鍜宻cripts鐩綍锛堝伐鍏凤級
    python_path = f"{planner_dir}:{scripts_dir}:{lib_dir}:{os.environ.get('PYTHONPATH', '')}"

    pct_planner = Node(
        package='pct_planner',
        executable='global_planner.py',
        name='pct_global_planner',
        output='screen',
        parameters=[pct_planner_params],
        env={'PYTHONPATH': python_path}
    )

    # --- PCT璺緞閫傞厤鍣?---
    # 鍔熻兘锛氬皢鍏ㄥ眬璺緞鍒囧垎涓哄眬閮ㄨ埅鐐瑰簭鍒?    # 杈撳叆锛?    #   - /pct_global_path: PCT鐢熸垚鐨勫叏灞€璺緞
    #   - /Odometry: 褰撳墠鏈哄櫒浜轰綅濮?    # 杈撳嚭锛?    #   - /way_point: 涓嬩竴涓埅鐐圭洰鏍?    pct_adapter = Node(
        package='pct_adapters',
        executable='pct_path_adapter.py',
        name='pct_path_adapter',
        output='screen',
        parameters=[pct_adapter_config_path]
    )


    # ==================== 鍦板舰鍒嗘瀽灞?====================

    # --- 鍩虹鍦板舰鍒嗘瀽 ---
    # 鍔熻兘锛氬垎鏋愬湴褰㈢殑鍩烘湰鍙€氳鎬?    # 杈撳叆锛?    #   - /cloud_map: 鍏ㄥ眬鍧愭爣绯荤偣浜戯紙鏉ヨ嚜FASTLIO2鐨剋orld_cloud锛?    # 杈撳嚭锛?    #   - /terrain_cloud: 鍦板舰鐐逛簯
    #   - /terrain_local: 灞€閮ㄥ湴褰俊鎭?    #
    # 璇存槑锛氳闃?cloud_registered浣嗛€氳繃remapping閲嶅畾鍚戝埌/cloud_map
    #       杩欐牱鍙互浣跨敤鍏ㄥ眬鍦板浘鑰岄潪褰撳墠甯can
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[terrain_analysis_params],
        remappings=[
            # 鍏抽敭閲嶆槧灏勶細灏嗚闃呬粠褰撳墠scan鏀逛负鍏ㄥ眬鍦板浘
            ('/cloud_registered', '/cloud_map')
        ]
    )

    # --- 鎵╁睍鍦板舰鍒嗘瀽 ---
    # 鍔熻兘锛氶珮绾у湴褰㈠缓鍥句笌杩為€氭€у垎鏋?    # 杈撳叆锛?    #   - /cloud_map: 鍏ㄥ眬鍧愭爣绯荤偣浜戯紙鏉ヨ嚜FASTLIO2鐨剋orld_cloud锛?    # 杈撳嚭锛?    #   - /terrain_map: 澧炲己鐨勫湴褰㈤珮搴﹀浘锛?.5D琛ㄧず锛?    #   - /terrain_map_ext: 鎵╁睍鍦板舰淇℃伅
    #
    # 宸ヤ綔娴佺▼锛?    #   1. 鎺ユ敹鍏ㄥ眬鐐逛簯鍦板浘 /cloud_map
    #   2. 鏋勫缓浣撶礌鍖栫殑鍦板舰琛ㄧず
    #   3. 璁＄畻姣忎釜缃戞牸鐨勯珮搴︺€佹搴︺€佸彲閫氳鎬?    #   4. 妫€鏌ュ湴褰㈣繛閫氭€?    #   5. 杈撳嚭2.5D鍦板舰楂樺害鍥?-> /terrain_map
    terrain_analysis_ext_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        output='screen',
        parameters=[terrain_analysis_ext_params],
        remappings=[
            # 鍏抽敭閲嶆槧灏勶細灏嗚闃呬粠褰撳墠scan鏀逛负鍏ㄥ眬鍦板浘
            ('/cloud_registered', '/cloud_map'),  # 杈撳叆锛氬叏灞€鐐逛簯鍦板浘
            ('/terrain_map', '/terrain_map')      # 杈撳嚭锛氬湴褰㈤珮搴﹀浘
        ]
    )

    # ==================== 灞€閮ㄨ鍒掍笌鎺у埗灞?====================

    # --- 灞€閮ㄨ鍒掑櫒 ---
    # 鍔熻兘锛氬疄鏃堕殰纰嶇墿瑙勯伩 + 灞€閮ㄨ建杩硅鍒掞紙2.5D瀵艰埅锛?    # 杈撳叆锛?    #   - /way_point: 鐩爣鑸偣锛堟潵鑷狿CT Adapter锛?    #   - /cloud_map: 鍏ㄥ眬鐐逛簯鍦板浘锛堟潵鑷狥ASTLIO2鐨剋orld_cloud锛?    #   - /terrain_map: 鍦板舰楂樺害鍥撅紙鏉ヨ嚜Terrain Analysis Ext锛?    #   - /Odometry: 褰撳墠浣嶅Э
    # 杈撳嚭锛?    #   - /path: 灞€閮ㄨ建杩癸紙nav_msgs/Path锛?    #   - /slow_down: 鍑忛€熸寚浠?    #
    # 宸ヤ綔鍘熺悊锛?    #   1. 浠庨璁＄畻鐨勮矾寰勫簱涓€夋嫨鍊欓€夎矾寰?    #   2. 浣跨敤/cloud_map妫€娴嬮殰纰嶇墿锛堝叏灞€鍦板浘淇℃伅锛?    #   3. 浣跨敤/terrain_map璇勪及鍦板舰鍙€氳鎬э紙2.5D楂樺害鍥撅級
    #   4. 鏍规嵁鐩爣鏂瑰悜銆侀殰纰嶇墿銆佸湴褰㈢患鍚堣瘎浼拌矾寰?    #   5. 閫夋嫨鏈€浼樺彲閫氳璺緞
    local_planner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[local_planner_params],
        remappings=[
            # 鍏抽敭閲嶆槧灏勶細浣跨敤鍏ㄥ眬鍦板浘鍜屽湴褰㈠浘
            ('/cloud_registered', '/cloud_map'),  # 杈撳叆锛氬叏灞€鐐逛簯鍦板浘锛堥殰纰嶇墿妫€娴嬶級
            ('/terrain_map', '/terrain_map')      # 杈撳叆锛氬湴褰㈤珮搴﹀浘锛堝彲閫氳鎬ц瘎浼帮級
        ]
    )

    # --- 璺緞璺熻釜鍣?---
    # 鍔熻兘锛歅ure Pursuit璺緞璺熻釜绠楁硶锛岀敓鎴愰€熷害鎸囦护
    # 杈撳叆锛?    #   - /path: 灞€閮ㄨ建杩癸紙鏉ヨ嚜Local Planner锛?    #   - /Odometry: 褰撳墠浣嶅Э
    #   - /slow_down: 鍑忛€熸寚浠?    # 杈撳嚭锛?    #   - /cmd_vel: 閫熷害鎸囦护锛坓eometry_msgs/TwistStamped锛?    #
    # 宸ヤ綔鍘熺悊锛?    #   1. 鍦ㄨ矾寰勪笂瀵绘壘鍓嶇灮鐐癸紙lookahead point锛?    #   2. 璁＄畻鍒拌揪鍓嶇灮鐐圭殑杞悜瑙掗€熷害
    #   3. 鏍规嵁璺緞鏇茬巼鍜岄殰纰嶇墿璋冩暣绾块€熷害
    path_follower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[path_follower_params],
        remappings=[
            # 璁㈤槄 /Odometry锛堟潵鑷狥ASTLIO2锛?            # 鍙戝竷 /cmd_vel锛堝彂閫佺粰鏈哄櫒浜洪┍鍔級
        ]
    )

    # ==================== 鎵嬪姩鎺у埗鎺ュ彛 ====================

    # --- 鎵嬫焺鍙傛暟 ---
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='鎵嬫焺璁惧ID锛? -> /dev/input/js0, 1 -> /dev/input/js1锛?
    )

    # --- 鎵嬫焺鑺傜偣 ---
    # 鍔熻兘锛氭帴鏀舵父鎴忔墜鏌勮緭鍏ワ紝鐢ㄤ簬鎵嬪姩鎺у埗
    #
    # 鍙戝竷璇濋锛?    #   - /joy (sensor_msgs/Joy): 鎵嬫焺鐘舵€佷俊鎭?    #       鈹斺攢 鍖呭惈鎵€鏈夋寜閿拰鎽囨潌鐨勫疄鏃剁姸鎬?    #
    # 璁㈤槄鑰咃紙浣跨敤/joy璇濋鐨勮妭鐐癸級锛?    #   鈹溾攢 Local Planner (localPlanner)
    #   鈹? 鈹斺攢 鐢ㄩ€旓細鎵嬪姩/鑷富妯″紡鍒囨崲銆侀殰纰嶇墿妫€娴嬪紑鍏炽€侀€熷害鎺у埗
    #   鈹? 鈹斺攢 鍔熻兘锛?    #   鈹?     - axes[2]: 鑷富妯″紡寮€鍏?(>-0.1: 鎵嬪姩, <-0.1: 鑷富)
    #   鈹?     - axes[5]: 闅滅鐗╂娴嬪紑鍏?(>-0.1: 鍚敤, <-0.1: 绂佺敤)
    #   鈹?     - axes[3], axes[4]: 鎵嬪姩鏂瑰悜鎺у埗锛堝乏鎽囨潌锛?    #   鈹?    #   鈹斺攢 Path Follower (pathFollower)
    #      鈹斺攢 鐢ㄩ€旓細鎵嬪姩椹鹃┒鎺у埗銆侀€熷害璋冭妭
    #      鈹斺攢 鍔熻兘锛?    #          - axes[3], axes[4]: 绾块€熷害鍜屾柟鍚戞帶鍒?    #          - axes[0]: 瑙掗€熷害鎺у埗锛堣浆鍚戯級
    #          - axes[2]: 鑷富妯″紡寮€鍏?    #          - axes[5]: 鎵嬪姩妯″紡寮€鍏?    #
    # 鐢ㄩ€旀€荤粨锛?    #   - 鎵嬪姩鎺у埗鏈哄櫒浜虹Щ鍔紙鎽囨潌锛?    #   - 鑷富/鎵嬪姩妯″紡鍒囨崲锛堟壋鏈洪敭锛?    #   - 绱ф€ュ仠姝㈠拰瀹夊叏鎺у埗
    #   - 瀹炴椂閫熷害璋冭妭
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params]
    )

    # ==================== 鍧愭爣鍙樻崲 ====================

    # 娉ㄦ剰锛氬湪system_launch涓紝localizer_node浼氬姩鎬佸彂甯僲ap->lidar鍙樻崲
    # 涓嶉渶瑕侀潤鎬佸彉鎹紝鍚﹀垯浼氫骇鐢烼F鍐茬獊锛圱F鎶栧姩锛?
    # ==================== 鍙鍖栧伐鍏?====================

    # --- 鍙鍖栬妭鐐癸紙鍙€夛級---
    # 鍔熻兘锛氬湪RViz涓彲瑙嗗寲瀵艰埅鐘舵€?    # 娉ㄦ剰锛氬綋鍓嶅凡娉ㄩ噴锛屾牴鎹渶瑕佸惎鐢?    # visualization_node = Node(
    #     package='visualization_tools',
    #     executable='visualizationTools',
    #     name='visualizationTools',
    #     output='screen',
    #     remappings=[
    #         ('/state_estimation', '/Odometry'),
    #         ('/registered_scan', '/cloud_registered')
    #     ]
    # )

    # ==================== 鍚姩鎻忚堪缁勮 ====================

    return LaunchDescription([
        # --- 鍚姩鍙傛暟 ---
        map_path_arg,                  # 鍦板浘璺緞鍙傛暟
        tomogram_resolution_arg,       # Tomogram鍒嗚鲸鐜囧弬鏁?        tomogram_slice_dh_arg,         # Tomogram灞傞珮鍙傛暟
        tomogram_ground_h_arg,         # Tomogram鍦伴潰楂樺害鍙傛暟
        x_arg, y_arg, z_arg, yaw_arg,  # 鍒濆浣嶅Э鍙傛暟
        joy_dev_arg,                   # 鎵嬫焺璁惧鍙傛暟

        # --- 鎰熺煡灞?---
        livox_launch,                  # Livox MID360婵€鍏夐浄杈鹃┍鍔?        lio_node,                      # FASTLIO2 瀹炴椂SLAM
        localizer_node,                # 鍩轰簬鍏堥獙鍦板浘鐨勫畾浣?
        # --- 鍏ㄥ眬瑙勫垝灞?---
        pct_planner,                   # PCT 3D鍏ㄥ眬璺緞瑙勫垝鍣?        pct_adapter,                   # PCT璺緞閫傞厤鍣紙鍏ㄥ眬鈫掑眬閮級

        # --- 鍦板舰鍒嗘瀽灞?---
        terrain_analysis_node,         # 鍩虹鍦板舰鍒嗘瀽
        terrain_analysis_ext_node,     # 鎵╁睍鍦板舰鍒嗘瀽

        # --- 灞€閮ㄨ鍒掍笌鎺у埗灞?---
        local_planner_node,            # 灞€閮ㄨ鍒掑櫒锛堥殰纰嶇墿瑙勯伩锛?        path_follower_node,            # 璺緞璺熻釜鍣紙Pure Pursuit锛?
        # --- 纭欢涓庢墜鍔ㄦ帶鍒?---
        # robot_driver,                # 鏈哄櫒浜哄簳鐩橀┍鍔紙宸叉敞閲婏級
        joy_node,                      # 娓告垙鎵嬫焺鎺ュ彛

        # --- 鍙鍖?---
    ])
