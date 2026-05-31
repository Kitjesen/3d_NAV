# SLAM Interface Services

鏈寘瀹氫箟浜嗙敤浜?SLAM 绯荤粺浜や簰鐨勮嚜瀹氫箟 ROS 2 鏈嶅姟鎺ュ彛锛屽寘鎷湴鍥句繚瀛樸€侀噸瀹氫綅銆佽建杩硅褰曠瓑鍔熻兘銆?

## 鏈嶅姟鍒楄〃

| 鏈嶅姟鍚嶇О | 鎺ュ彛绫诲瀷 | 鎻愪緵鑰?| 鍔熻兘 |
|---------|---------|--------|------|
| `/save_map` | `SaveMaps.srv` | FastLIO2, PGO | 淇濆瓨鍏ㄥ眬鍦板浘 |
| `/relocalize` | `Relocalize.srv` | Localizer | 鍔犺浇鍦板浘骞堕噸瀹氫綅 |
| `/relocalize_check` | `IsValid.srv` | Localizer | 妫€鏌ラ噸瀹氫綅鐘舵€?|
| `/pgo/save_maps` | `SaveMaps.srv` | PGO | 淇濆瓨浼樺寲鍚庣殑鍦板浘 |
| `/pgo/save_poses` | `SavePoses.srv` | PGO | 淇濆瓨鍏抽敭甯ц建杩?|

---

## 1. SaveMaps.srv - 淇濆瓨鍦板浘

### 鎺ュ彛瀹氫箟
```
# Request
string file_path      # 淇濆瓨鐩綍鐨勭粷瀵硅矾寰?
bool save_patches     # 鏄惁淇濆瓨鍒嗗潡鍦板浘锛堢敤浜庡ぇ鍦板浘鍦烘櫙锛?

# Response
bool success          # 鏄惁鎴愬姛
string message        # 鐘舵€佹秷鎭?
```

### 浣跨敤绀轰緥

#### 淇濆瓨 FastLIO2 鍦板浘锛堝崟鏂囦欢锛?
```bash
ros2 service call /save_map interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/office_map', save_patches: false}"
```

#### 淇濆瓨 PGO 浼樺寲鍦板浘锛堝垎鍧楋級
```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/warehouse', save_patches: true}"
```

### 杈撳嚭鏂囦欢缁撴瀯
```
/home/user/maps/office_map/
鈹溾攢鈹€ map.pcd              # 鍏ㄥ眬鐐逛簯鍦板浘
鈹溾攢鈹€ poses.txt            # 鍏抽敭甯т綅濮垮垪琛?(濡傛灉 save_patches=true)
鈹斺攢鈹€ patches/             # 鍒嗗潡鍦板浘 (濡傛灉 save_patches=true)
    鈹溾攢鈹€ 0.pcd
    鈹溾攢鈹€ 1.pcd
    鈹斺攢鈹€ ...
```

---

## 2. Relocalize.srv - 閲嶅畾浣?

### 鎺ュ彛瀹氫箟
```
# Request
string pcd_path       # 鍦板浘鏂囦欢璺緞 (*.pcd)
float32 x             # 鍒濆浣嶇疆浼拌 X (绫?
float32 y             # 鍒濆浣嶇疆浼拌 Y (绫?
float32 z             # 鍒濆浣嶇疆浼拌 Z (绫?
float32 yaw           # 鍒濆鍋忚埅瑙?(寮у害)
float32 pitch         # 鍒濆淇话瑙?(寮у害)
float32 roll          # 鍒濆婊氳浆瑙?(寮у害)

# Response
bool success
string message
```

### 浣跨敤鍦烘櫙
- 鏈哄櫒浜哄惎鍔ㄦ椂锛屽湪宸茬煡鍦板浘涓畾浣?
- 缁戞灦闂锛坘idnapped robot problem锛夋仮澶?
- 鍒囨崲鍒版柊鐨勫伐浣滃尯鍩?

### 浣跨敤绀轰緥

#### 绠€鍗曞畾浣嶏紙浠呮寚瀹氫綅缃級
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 0.0, y: 0.0, z: 0.0, \
    yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

#### 绮剧‘瀹氫綅锛堝凡鐭ュ垵濮嬪Э鎬侊級
```bash
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/warehouse/map.pcd', \
    x: 10.5, y: -3.2, z: 0.0, \
    yaw: 1.57, pitch: 0.0, roll: 0.0}"
```

### 鍙傛暟璇存槑
- **`pcd_path`**: 蹇呴』鏄?`.pcd` 鏍煎紡鐨勭偣浜戝湴鍥?
- **浣嶇疆浼拌**: 鍏佽 卤5 绫宠宸紝瀹氫綅绠楁硶浼氳嚜鍔ㄤ紭鍖?
- **濮挎€佷及璁?*: 鍏佽 卤30掳 璇樊锛坄yaw` 鏈€閲嶈锛宍pitch/roll` 閫氬父涓?0锛?

---

## 3. IsValid.srv - 妫€鏌ュ畾浣嶇姸鎬?

### 鎺ュ彛瀹氫箟
```
# Request
int32 code            # 鏌ヨ浠ｇ爜 (閫氬父涓?0)

# Response
bool valid            # 瀹氫綅鏄惁鏈夋晥
```

### 浣跨敤绀轰緥

#### 妫€鏌ラ噸瀹氫綅鏄惁瀹屾垚
```bash
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
```

#### 杩斿洖绀轰緥
```yaml
valid: true
```

### 鍏稿瀷宸ヤ綔娴?
```bash
# Step 1: 鍙戣捣閲嶅畾浣嶈姹?
ros2 service call /relocalize interface/srv/Relocalize ...

# Step 2: 绛夊緟 2-5 绉?

# Step 3: 妫€鏌ユ槸鍚︽垚鍔?
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"

# Step 4: 濡傛灉 valid=true锛屽彲浠ュ紑濮嬪鑸?
```

---

## 4. SavePoses.srv - 淇濆瓨杞ㄨ抗

### 鎺ュ彛瀹氫箟
```
# Request
string file_path      # 淇濆瓨璺緞 (*.txt)

# Response
bool success
string message
```

### 浣跨敤绀轰緥

```bash
ros2 service call /pgo/save_poses interface/srv/SavePoses \
  "{file_path: '/home/user/trajectories/run_001.txt'}"
```

### 杈撳嚭鏍煎紡
```
# file_name x y z qw qx qy qz
0.pcd 0.000 0.000 0.000 1.000 0.000 0.000 0.000
1.pcd 0.523 0.102 0.000 0.998 0.000 0.000 0.065
2.pcd 1.045 0.205 0.000 0.995 0.000 0.000 0.098
...
```

---

## 5. RefineMap.srv - 浼樺寲鍦板浘

### 鎺ュ彛瀹氫箟
```
# Request
string maps_path      # 鍦板浘鐩綍璺緞

# Response
bool success
string message
```

### 浣跨敤绀轰緥

```bash
ros2 service call /refine_map interface/srv/RefineMap \
  "{maps_path: '/home/user/maps/office_map'}"
```

---

## 甯歌宸ヤ綔娴佺▼

### 馃搷 鍦烘櫙1: 寤哄浘锛圡apping锛?

```bash
# 1. 鍚姩绯荤粺锛堝缓鍥炬ā寮忥級
ros2 launch pct_planner system_launch.py

# 2. 鎺у埗鏈哄櫒浜虹Щ鍔紙鎵嬪姩鎴栬嚜涓伙級
# 浣跨敤鎵嬫焺鎴栧彂閫佸鑸洰鏍?

# 3. 寤哄浘瀹屾垚鍚庝繚瀛?
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/user/maps/new_map', save_patches: true}"

# 4. 淇濆瓨杞ㄨ抗锛堝彲閫夛紝鐢ㄤ簬鍒嗘瀽锛?
ros2 service call /pgo/save_poses interface/srv/SavePoses \
  "{file_path: '/home/user/maps/new_map/trajectory.txt'}"
```

---

### 馃搷 鍦烘櫙2: 瀹氫綅涓庡鑸紙Localization & Navigation锛?

```bash
# 1. 鍚姩绯荤粺
ros2 launch pct_planner system_launch.py

# 2. 鍔犺浇宸叉湁鍦板浘骞堕噸瀹氫綅
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 3. 绛夊緟 2-3 绉掞紝妫€鏌ュ畾浣嶇姸鎬?
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"

# 4. 濡傛灉 valid=true锛屽彂閫佸鑸洰鏍?
ros2 topic pub /way_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 5.0, z: 0.0}}"
```

---

### 馃搷 鍦烘櫙3: 缁戞灦鎭㈠锛圞idnapped Robot Recovery锛?

```bash
# 1. 鏈哄櫒浜鸿鎼繍鍒版湭鐭ヤ綅缃紝TF 鏍戞柇瑁?

# 2. 鎵嬪姩浼拌澶ц嚧浣嶇疆锛堥€氳繃 RViz 鎴栧凡鐭ヤ俊鎭級
# 鍋囪鏈哄櫒浜哄湪鍦板浘鐨?(5, -2) 浣嶇疆锛屾湞涓滐紙yaw 鈮?0锛?

# 3. 閲嶆柊瀹氫綅
ros2 service call /relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/user/maps/office_map/map.pcd', \
    x: 5.0, y: -2.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# 4. 楠岃瘉
ros2 service call /relocalize_check interface/srv/IsValid "{code: 0}"
```

---

## 璋冭瘯鎶€宸?

### 鏌ョ湅鏈嶅姟鐘舵€?
```bash
# 鍒楀嚭鎵€鏈夋鍦ㄨ繍琛岀殑鏈嶅姟
ros2 service list

# 鏌ョ湅鏈嶅姟鎺ュ彛绫诲瀷
ros2 service type /save_map

# 鏌ョ湅鏈嶅姟璇︾粏淇℃伅
ros2 service info /relocalize
```

### 鐩戞帶瀹氫綅璐ㄩ噺
```bash
# 鏌ョ湅瀹氫綅璇樊锛堝鏋?Localizer 鍙戝竷璇ヨ瘽棰橈級
ros2 topic echo /localization_error

# 鍙鍖?TF 鏍?
ros2 run rqt_tf_tree rqt_tf_tree
```

---

