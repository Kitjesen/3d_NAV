尚未实现工程化管理，程序整体比较庞大。
a) 监控各模块健康状态
b) 未实现状态管理，当前服务心跳状态检测。


如果要做一个可视化的管理，那么这些配置参数都需要可视化操作：
"""
class SceneCommon():
    pcd = ScenePCD()
    pcd.file_name = 'scans.pcd'    # ← 关键！指定加载哪个PCD文件

    map = SceneMap()
    map.resolution = 0.10          # 地图分辨率
    map.ground_h = 0.0             # 地面高度
    map.slice_dh = 0.5             # 层高间隔

    trav = SceneTrav()             # 可穿越性参数
    trav.kernel_size = 7
    trav.slope_max = 0.40          # 最大坡度
    trav.step_max = 0.17           # 最大台阶
"""

保存地图:
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/path/save_dir', save_patches: true}"


除无效点的方法
1.距离过滤 2.高度范围过滤 3.时间衰减过滤（动态点云） 4.VoxelGrid 降采样（去除重复点） 5.动态障碍物过滤  6.天花板过滤  7.无效数据区域处理