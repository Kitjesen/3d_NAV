# PCT Planner

## Overview



---

## 1. 蹇€熷紑濮?

### 1.1 缂栬瘧瀹夎

```bash
cd planner/
./build_thirdparty.sh
./build.sh
```

### 1.2 鍑嗗鍦板浘锛堥€氳繃 PGO 淇濆瓨锛?
**鎺ㄨ崘浣跨敤 PGO 鐨?`/pgo/save_maps` 鏈嶅姟淇濆瓨鍦板浘**锛岃繖鏍峰彲浠ヨ幏寰楀洖鐜紭鍖栧悗鐨勯珮璐ㄩ噺鍦板浘銆?
#### 浣跨敤涓€浣撳寲寤哄浘 Launch锛堟帹鑽愶級
```bash
# 1. 鍚姩瀹屾暣鐨勫缓鍥炬祦绋嬶紙LiDAR + Fast-LIO2 + PGO锛?
ros2 launch pct_planner mapping_launch.py
# 2. 鍦ㄧ幆澧冧腑绉诲姩鏈哄櫒浜猴紝绛夊緟 PGO 妫€娴嬪埌鍥炵幆
#    瑙傚療 /pgo/loop_markers 璇濋纭鍥炵幆妫€娴?
# 3. 淇濆瓨浼樺寲鍚庣殑鍦板浘
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/to/save_dir', save_patches: true}"

# 4. 澶嶅埗鍦板浘鍒?pct_planner
#    PGO 浼氫繚瀛橈細map.pcd, poses.txt, patches/鐩綍
cp /path/to/save_dir/map.pcd rsc/pcd/my_map.pcd
```

#### 鏂瑰紡 B: 鎵嬪姩鍒嗗埆鍚姩

```bash
# 1. 鍒嗗埆鍚姩鍚勪釜鑺傜偣
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py
ros2 launch pgo pgo_launch.py

# 2. 淇濆瓨鍦板浘锛堝悓涓婏級
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/to/save_dir', save_patches: true}"
```

**涓轰粈涔堜笉浣跨敤 `/save_map` (Fast-LIO2)锛?*
- `/save_map` 淇濆瓨鐨勬槸**鏈紭鍖?*鐨勫師濮嬬偣浜?
- `/pgo/save_maps` 淇濆瓨鐨勬槸**鍥炵幆浼樺寲鍚?*鐨勫湴鍥撅紝婕傜Щ鏇村皬锛屾洿閫傚悎鍏ㄥ眬瑙勫垝
- PGO 淇濆瓨鐨勫湴鍥惧寘鍚畬鏁寸殑浣嶅Э鍥句俊鎭紝璐ㄩ噺鏇撮珮

### 1.3 閫夋嫨鍚姩鏂瑰紡

#### 鏂瑰紡 A: 绂荤嚎棰勫鐞嗭紙鎺ㄨ崘锛屽惎鍔ㄦ渶蹇級

```bash
# 1. 绂荤嚎鏋勫缓 Tomogram
python3 tomography/scripts/tomography.py --scene Common
# 鐢熸垚: rsc/tomogram/Common.pickle

# 2. 鍦ㄧ嚎鍔犺浇杩愯锛堜娇鐢?.pickle锛?
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"
```

#### 鏂瑰紡 B: 鍦ㄧ嚎瀹炴椂鏋勫缓锛堟柟渚匡紝棣栨鍚姩杈冩參锛?

```bash
# 鐩存帴浠?PCD 鏋勫缓骞惰繍琛岋紙鏃犻渶棰勫鐞嗭級
ros2 run pct_planner global_planner --ros-args -p map_file:="my_map.pcd"

# 棣栨杩愯浼氳嚜鍔ㄦ瀯寤哄苟缂撳瓨 .pickle锛屽悗缁惎鍔ㄥ姞閫?
```

---

## 2. 绯荤粺鏋舵瀯

```
鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?    鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?    鈹屸攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
鈹?  Tomography    鈹傗攢鈹€鈹€鈹€鈻垛攤  Global Planner 鈹傗攢鈹€鈹€鈹€鈻垛攤  Path Adapter   鈹?
鈹? (绂荤嚎/鍦ㄧ嚎)     鈹?    鈹? (ROS 2 Node)   鈹?    鈹?(pct_adapters)  鈹?
鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?    鈹斺攢鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹?
         鈹?                      鈹?                      鈹?
         鈹?.pickle/PCD          鈹?/pct_path            鈹?/way_point
         鈹?                      鈹?                      鈹?
         鈻?                      鈻?                      鈻?
   rsc/tomogram/           RViz鏄剧ず鐩爣鐐?         Local Planner
```

### 妯″潡璇存槑

| 妯″潡 | 鍔熻兘 | 鍚姩鏂瑰紡 |
|------|------|---------|
| **tomography** | 灏?PCD 鈫?Tomogram锛堝灞傚湴鍥撅級 | `python3 tomography.py --scene XXX` |
| **global_planner** | 鍔犺浇鍦板浘锛屾帴鏀剁洰鏍囩偣锛岃鍒掑叏灞€璺緞 | `ros2 run pct_planner global_planner` |
| **pct_adapters** | 灏嗗叏灞€璺緞杞崲涓哄眬閮ㄨ埅鐐瑰簭鍒?| `ros2 run pct_adapters pct_path_adapter` |

---

## 3. 璇︾粏浣跨敤鎸囧崡

### 3.1 鍦烘櫙閰嶇疆 (--scene)

`--scene` 鍙傛暟閫夋嫨涓€濂楅璁剧殑閰嶇疆锛圥CD鏂囦欢銆佸垎杈ㄧ巼銆佸彲绌胯秺鎬у弬鏁扮瓑锛夈€?

#### 浣跨敤鍐呯疆鍦烘櫙

```bash
# 閫氱敤骞冲湴鍦烘櫙
python3 tomography/scripts/tomography.py --scene Common

# 妤兼鍦烘櫙锛堟洿闄＄殑鍧″害瀹瑰繊锛?
python3 tomography/scripts/tomography.py --scene Stairs

# 瀹ゅ唴鍦版澘
python3 tomography/scripts/tomography.py --scene Floor
```

**鍐呯疆鍦烘櫙鍒楄〃**:

| 鍦烘櫙鍚?| 閫傜敤鐜 | 鍏抽敭鍙傛暟鐗圭偣 |
|--------|---------|-------------|
| `Common` | 閫氱敤骞冲湴 | 榛樿閰嶇疆 |
| `Stairs` | 妤兼鍦烘櫙 | `slope_max=0.60`, `step_max=0.2` |
| `Floor` | 瀹ゅ唴鍦版澘 | `slope_max=0.40`, `step_max=0.17` |
| `Room` | 灏忔埧闂?| 瀹夊叏杈圭晫杈冨皬 |
| `Plaza` | 骞垮満 | `slope_max=0.36` |
| `Building` | 寤虹瓚鐗?| 榛樿閰嶇疆 |
| `Spiral` | 铻烘棆璺緞 | `step_max=0.30`, 鍓嶇灮璺濈澶?|

#### 鍒涘缓鑷畾涔夊満鏅?

1. **鍑嗗PCD鏂囦欢**
   ```bash
   cp your_map.pcd rsc/pcd/my_scene.pcd
   ```

2. **鍒涘缓閰嶇疆鏂囦欢** (`tomography/config/scene_myscene.py`)
   ```python
   from .scene import ScenePCD, SceneMap, SceneTrav

   class SceneMyscene():
       pcd = ScenePCD()
       pcd.file_name = 'my_scene.pcd'    # PCD鏂囦欢鍚?

       map = SceneMap()
       map.resolution = 0.10              # 鍦板浘鍒嗚鲸鐜?m)
       map.ground_h = 0.0                 # 鍦伴潰楂樺害(m)
       map.slice_dh = 0.5                 # 灞傞珮闂撮殧(m)

       trav = SceneTrav()
       trav.kernel_size = 7               # 鍙┛瓒婃€у垎鏋愭牳澶у皬
       trav.slope_max = 0.40              # 鏈€澶у潯搴?tan鍊?
       trav.step_max = 0.20               # 鏈€澶у彴闃堕珮搴?m)
       trav.standable_ratio = 0.20        # 鍙珯绔嬫瘮渚嬮槇鍊?
       trav.cost_barrier = 50.0           # 闅滅浠ｄ环鍊?
       trav.safe_margin = 0.4             # 瀹夊叏杈圭晫(m)
       trav.inflation = 0.2               # 鑶ㄨ儉绯绘暟(m)
   ```

3. **娉ㄥ唽鍦烘櫙** (`tomography/config/__init__.py`)
   ```python
   from .scene_myscene import SceneMyscene
   ```

4. **杩愯**
   ```bash
   python3 tomography/scripts/tomography.py --scene Myscene
   ```

### 3.2 ROS 2 鍙傛暟閰嶇疆

`global_planner` 鏀寔閫氳繃 ROS 鍙傛暟鍔ㄦ€侀厤缃細

```bash
ros2 run pct_planner global_planner --ros-args \
  -p map_file:="Common.pickle" \
  -p map_frame:="map" \
  -p robot_frame:="body" \
  -p min_plan_interval:=2.0 \
  -p default_goal_height:=0.0 \
  -p tomogram_resolution:=0.1 \
  -p tomogram_slice_dh:=0.5 \
  -p publish_map_pointcloud:=true \
  -p publish_tomogram:=true
```

**鍙傛暟璇存槑**:

| 鍙傛暟鍚?| 绫诲瀷 | 榛樿鍊?| 璇存槑 |
|--------|------|--------|------|
| `map_file` | string | - | Tomogram鏂囦欢(.pickle)鎴朠CD鏂囦欢(.pcd) |
| `map_frame` | string | "map" | 鍦板浘鍧愭爣绯?|
| `robot_frame` | string | "body" | 鏈哄櫒浜哄潗鏍囩郴 |
| `min_plan_interval` | float | 2.0 | 鏈€灏忚鍒掗棿闅?绉?锛岄槻姝㈣繛缁Е鍙?|
| `default_goal_height` | float | 0.0 | 榛樿鐩爣楂樺害(m) |
| `tomogram_resolution` | float | 0.2 | Tomogram鍒嗚鲸鐜?浠匬CD鏋勫缓鏃剁敓鏁? |
| `tomogram_slice_dh` | float | 0.5 | 灞傞珮闂撮殧(浠匬CD鏋勫缓鏃剁敓鏁? |
| `tomogram_ground_h` | float | 0.0 | 鍦伴潰楂樺害(浠匬CD鏋勫缓鏃剁敓鏁? |
| `publish_map_pointcloud` | bool | false | 鏄惁鍙戝竷鍘熷鍦板浘鐐逛簯 |
| `publish_tomogram` | bool | false | 鏄惁鍙戝竷Tomogram鍙鍖?|

### 3.3 浜や簰寮忚鍒?

鍚姩鍚庯紝鍦?RViz 涓細

1. **璁剧疆鐩爣鐐?*: 鐐瑰嚮 "2D Goal Pose" 宸ュ叿锛屽湪鍦板浘涓婇€夋嫨鐩爣浣嶇疆
2. **鏌ョ湅璺緞**: 瑙勫垝鐨?`/pct_path` 浼氳嚜鍔ㄥ彂甯?
3. **鏌ョ湅鐘舵€?*: 璁㈤槄 `/pct_planner/status` 鏌ョ湅瑙勫垝鐘舵€?

**璇濋鎺ュ彛**:

| 璇濋鍚?| 绫诲瀷 | 鏂瑰悜 | 璇存槑 |
|--------|------|------|------|
| `/goal_pose` | PoseStamped | Sub | RViz 2D Goal Pose |
| `/clicked_point` | PointStamped | Sub | RViz Clicked Point |
| `/pct_path` | Path | Pub | 瑙勫垝鐨勫叏灞€璺緞 |
| `/pct_planner/status` | String | Pub | 瑙勫垝鐘舵€?(IDLE/PLANNING/SUCCESS/FAILED) |
| `/map_pointcloud` | PointCloud2 | Pub | 鍘熷鍦板浘鐐逛簯(鍙€? |
| `/tomogram` | PointCloud2 | Pub | Tomogram鍙鍖?鍙€? |

---

## 4. 绯荤粺闆嗘垚

### 4.1 涓庡眬閮ㄨ鍒掑櫒闆嗘垚

PCT Planner 浣滀负鍏ㄥ眬瑙勫垝鍣紝闇€瑕侀€氳繃 `pct_adapters` 涓?`local_planner` 鑱斿姩锛?

```bash
# 缁堢1: 鍚姩鍏ㄥ眬瑙勫垝
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"

# 缁堢2: 鍚姩璺緞閫傞厤鍣?
ros2 run pct_adapters pct_path_adapter

# 缁堢3: 鍚姩灞€閮ㄨ鍒掑櫒
ros2 launch local_planner local_planner.launch
```

**鏁版嵁娴?*:
```
RViz 2D Goal 鈹€鈹€鈻?global_planner 鈹€鈹€鈻?/pct_path 鈹€鈹€鈻?pct_adapters 鈹€鈹€鈻?/way_point 鈹€鈹€鈻?local_planner
```

### 4.2 瀹屾暣瀵艰埅鍚姩娴佺▼

```bash
# === 闃舵1: SLAM鍜屽畾浣?===
# 鏂瑰紡1: 浣跨敤涓€浣撳寲寤哄浘launch锛堝鏋滄槸寤哄浘闃舵锛?
ros2 launch pct_planner mapping_launch.py

# 鏂瑰紡2: 鎴栧垎鍒惎鍔紙濡傛灉鏄繍琛岄樁娈碉級
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fastlio2 lio_launch.py
ros2 launch localizer localizer_launch.py

# 閲嶅畾浣嶅埌棰勫缓鍦板浘锛堜娇鐢?PGO 淇濆瓨鐨勫湴鍥撅級
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/path/to/pgo_save_dir/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# === 闃舵2: 瑙勫垝鍜屾帶鍒?===
ros2 launch terrain_analysis terrain_analysis.launch
ros2 launch local_planner local_planner.launch

# === 闃舵3: 鍏ㄥ眬瑙勫垝 ===
ros2 run pct_planner global_planner --ros-args -p map_file:="Common.pickle"
ros2 run pct_adapters pct_path_adapter

# === 闃舵4: 鍦≧Viz涓偣鍑荤洰鏍囩偣 ===
```

---

## 5. 鏁呴殰鎺掓煡

### 闂1: `ModuleNotFoundError: No module named 'config'`

**瑙ｅ喅**: 纭繚鍦?`tomography/scripts/` 鐩綍涓嬭繍琛岋紝鎴栬缃?PYTHONPATH
```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/tomography/config
```

### 闂2: `No .pickle found; building tomogram from PCD`

**璇存槑**: 杩欐槸姝ｅ父琛屼负锛岄娆′粠 PCD 鏋勫缓闇€瑕佸嚑绉掑埌鍑犲崄绉掋€備細鑷姩缂撳瓨 `.pickle` 鏂囦欢銆?

**鍔犻€?*: 寤鸿鍏堢绾胯繍琛?`tomography.py` 鐢熸垚 `.pickle`锛屽啀鍦ㄧ嚎鍔犺浇銆?

### 闂3: 鍦板浘璐ㄩ噺宸紝瑙勫垝璺緞涓嶅噯纭?

**鍙兘鍘熷洜**: 浣跨敤浜?Fast-LIO2 鐨?`/save_map` 淇濆瓨鐨勬湭浼樺寲鍦板浘銆?

**瑙ｅ喅**: 浣跨敤 PGO 鐨?`/pgo/save_maps` 淇濆瓨鍥炵幆浼樺寲鍚庣殑鍦板浘锛?
```bash
# 閿欒锛氫繚瀛樼殑鏄湭浼樺寲鍦板浘
ros2 service call /save_map interface/srv/SaveMaps "{file_path: '/path/map.pcd'}"

# 姝ｇ‘锛氫繚瀛樼殑鏄紭鍖栧悗鐨勫湴鍥?
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/path/save_dir', save_patches: true}"
```

### 闂3: TF 鍙樻崲澶辫触

**妫€鏌?*:
```bash
ros2 run tf2_tools view_frames
# 纭繚 map 鈫?body 鐨勫彉鎹㈤摼瀹屾暣
```

### 闂4: 瑙勫垝澶辫触 (status: FAILED)

**鍙兘鍘熷洜**:
- 璧风偣鎴栫粓鐐瑰湪闅滅鐗╀腑
- 璧风偣鍜岀粓鐐逛箣闂存棤鍙繛閫氳矾寰?
- Tomogram 鍙傛暟涓嶅悎閫傦紙濡?`step_max` 澶皬锛?

**璋冭瘯**:
```bash
# 鏌ョ湅Tomogram鍙鍖?
ros2 param set /pct_global_planner publish_tomogram true
# 鍦≧Viz涓鏌?/tomogram 璇濋
```

---

## 6. 鏂囦欢缁勭粐

```
pct_planner/
鈹溾攢鈹€ planner/
鈹?  鈹溾攢鈹€ scripts/
鈹?  鈹?  鈹溾攢鈹€ global_planner.py      # ROS 2 瑙勫垝鑺傜偣
鈹?  鈹?  鈹斺攢鈹€ planner_wrapper.py     # TomogramPlanner 灏佽
鈹?  鈹溾攢鈹€ lib/                        # C++ 搴?(A*, 杞ㄨ抗浼樺寲)
鈹?  鈹斺攢鈹€ config/param.py            # ROS 鍙傛暟閰嶇疆
鈹?
鈹溾攢鈹€ tomography/
鈹?  鈹溾攢鈹€ scripts/
鈹?  鈹?  鈹溾攢鈹€ tomography.py          # 绂荤嚎/鍦ㄧ嚎 Tomogram 鏋勫缓
鈹?  鈹?  鈹斺攢鈹€ build_tomogram.py      # PCD 鈫?Tomogram 鏍稿績閫昏緫
鈹?  鈹斺攢鈹€ config/
鈹?      鈹溾攢鈹€ scene.py               # 鍩虹閰嶇疆绫?
鈹?      鈹溾攢鈹€ scene_common.py        # 閫氱敤鍦烘櫙
鈹?      鈹溾攢鈹€ scene_stairs.py        # 妤兼鍦烘櫙
鈹?      鈹斺攢鈹€ ...                    # 鍏朵粬鍦烘櫙
鈹?
鈹溾攢鈹€ rsc/
鈹?  鈹溾攢鈹€ pcd/                        # PCD鍦板浘鏂囦欢
鈹?  鈹斺攢鈹€ tomogram/                   # 鐢熸垚鐨?.pickle 鏂囦欢
鈹?
鈹斺攢鈹€ pct_adapters/                   # 璺緞閫傞厤鍣紙鍗曠嫭鍖咃級
    鈹斺攢鈹€ pct_path_adapter.py        # 鍏ㄥ眬鈫掑眬閮ㄦˉ姊?
```

---

## 7. Citing

If you use PCT Planner, please cite:

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography}, 
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

---

## 8. License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
