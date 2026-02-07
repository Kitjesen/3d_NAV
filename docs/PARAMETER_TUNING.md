# Parameter Tuning Guide

## Quick Reference

### 🎯 Start Here: Default Configurations

For most applications, use these presets:

| Scenario | Config File | Description |
|----------|-------------|-------------|
| **Indoor Navigation** | `config/indoor.yaml` | Slow speed, tight turns |
| **Outdoor Open Space** | `config/outdoor.yaml` | Fast speed, wide turns |
| **Rough Terrain** | `config/offroad.yaml` | Conservative, terrain-aware |

---

## Layer-by-Layer Tuning

### 1️⃣ Path Follower Parameters

**File**: `src/base_autonomy/local_planner/launch/pathFollower.launch.xml`

#### Speed & Acceleration

```yaml
maxSpeed: 1.0          # Maximum velocity (m/s)
  # 💡 Tip: Start at 0.5 m/s for testing
  # 🎯 Indoor: 0.6-1.0 | Outdoor: 1.5-3.0
  
maxAccel: 1.0          # Maximum acceleration (m/s²)
  # 💡 Tip: Higher = more responsive, but jerky
  # 🎯 Smooth: 0.5 | Aggressive: 2.0
```

#### Lookahead Distance (Adaptive)

```yaml
baseLookAheadDis: 0.3  # Base lookahead at 0 m/s
lookAheadRatio: 0.5    # Additional distance per m/s
minLookAheadDis: 0.2   # Minimum lookahead
maxLookAheadDis: 2.0   # Maximum lookahead

# Formula: L = base + ratio * speed
# Example: At 2 m/s → L = 0.3 + 0.5*2 = 1.3m
```

**Tuning Strategy**:
- **Too Small**: Oscillation, overshooting
- **Too Large**: Cuts corners, slow response
- **Recommended**: Start with defaults, adjust `lookAheadRatio` by ±0.1

#### Turning Control

```yaml
yawRateGain: 7.5       # Proportional gain for turning
  # 💡 Higher = faster rotation response
  # 🎯 Sluggish: Increase to 10-15
  # 🎯 Oscillating: Decrease to 5-7
  
maxYawRate: 45.0       # Maximum turn rate (deg/s)
  # 💡 Safety limit, not control gain
  # 🎯 Slow robots: 30 | Agile: 60-90
```

---

### 2️⃣ Local Planner Parameters

**File**: `src/base_autonomy/local_planner/launch/localPlanner.launch.xml`

#### Obstacle Detection

```yaml
laserVoxelSize: 0.1    # Point cloud downsampling (m)
  # ⚖️ Tradeoff: Smaller = more detail, higher CPU
  # 🎯 Fast CPU: 0.05 | Slow CPU: 0.15
  # ⚠️ Affects collision detection resolution

obstacleHeightThre: 0.2  # Minimum obstacle height (m)
  # 💡 Points taller than this = obstacle
  # 🎯 Flat ground: 0.15 | Rough: 0.25
```

#### Planning Horizon

```yaml
minPathRange: 2.5      # Minimum planning distance (m)
  # ⚠️ MUST be ≥ maxLookAheadDis (Path Follower)
  # 🎯 Hallways: 2.0 | Open: 3.5
  
adjacentRange: 3.5     # Obstacle consideration radius (m)
  # 💡 Larger = safer, but slower
  # 🎯 Cluttered: 2.5 | Sparse: 5.0
```

#### Path Selection

```yaml
dirWeight: 0.02        # Direction alignment weight
  # 💡 Higher = prefers straight paths
  # 🎯 Goal-seeking: 0.05 | Exploration: 0.01
  
useCost: true          # Enable terrain cost
  # 💡 Requires terrain_analysis running
  # 🎯 Flat: false | Slopes: true
```

---

### 3️⃣ Terrain Analysis Parameters

**File**: `src/base_autonomy/terrain_analysis/launch/terrainAnalysis.launch.xml`

#### Voxel Configuration

```yaml
scanVoxelSize: 0.1     # Scan downsampling (m)
  # ⚖️ Smaller = finer terrain map, more memory
  # 🎯 Default: 0.1 | High-res: 0.05

terrainVoxelSize: 1.0  # Terrain grid resolution (m)
  # 💡 Affects rolling map size
  # 🎯 Indoor: 0.5 | Outdoor: 2.0
```

#### Temporal Filtering

```yaml
decayTime: 10.0        # Point cloud lifetime (s)
  # 💡 How long to remember obstacles
  # 🎯 Static: 30 | Crowded: 5
  
noDecayDis: 2.0        # No decay within this radius (m)
  # ⚠️ Points near robot never decay
  # 🎯 Small robots: 1.0 | Large: 3.0
```

#### Ground Detection

```yaml
groundHeightThre: 0.1  # Ground classification (m)
  # 💡 Points below this = ground
  # 🎯 Smooth: 0.08 | Bumpy: 0.15
  
disRatioZ: 0.1         # Distance-based Z tolerance
  # 💡 Allows more Z variance far away
  # Formula: threshold = base + ratio * distance
```

---

### 4️⃣ Global Planner Parameters

**File**: `src/global_planning/PCT_planner/config/params.yaml`

#### A* Search

```yaml
w_traversability: 1.0  # Traversability cost weight
w_smoothness: 0.2      # Path smoothness weight
w_length: 0.1          # Path length weight

# 💡 Higher weight = more influence
# 🎯 Off-road: Increase traversability
# 🎯 Racing: Increase smoothness
```

#### Trajectory Optimization

```yaml
trajectory_resolution: 0.1  # Waypoint spacing (m)
max_iterations: 100         # Optimization iterations
convergence_threshold: 0.01 # Stop criteria

# ⚖️ More iterations = smoother, slower
```

---

## Common Tuning Scenarios

### 🐌 Robot is Too Slow

**Symptoms**: Robot crawls even with high joystick input

**Solutions**:
1. ✅ Increase `maxSpeed` (Path Follower)
2. ✅ Increase `autonomySpeed` if in autonomous mode
3. ✅ Check `/slow_down` topic isn't constantly triggering
4. ✅ Reduce `slowPathNumThre` (Local Planner)

---

### 📐 Robot Cuts Corners

**Symptoms**: Doesn't follow path accurately

**Solutions**:
1. ✅ Increase `baseLookAheadDis` (Path Follower)
2. ✅ Decrease `lookAheadRatio`
3. ✅ Increase `yawRateGain` for sharper turns
4. ✅ Reduce `maxSpeed` in tight spaces

---

### 🌀 Robot Oscillates

**Symptoms**: Snakes left-right around path

**Solutions**:
1. ✅ Decrease `yawRateGain` (Path Follower)
2. ✅ Increase `baseLookAheadDis`
3. ✅ Check odometry quality (`ros2 topic hz /Odometry`)
4. ✅ Smooth terrain map (`scanVoxelSize` → 0.15)

---

### 🚫 Too Many "Slow Down" Warnings

**Symptoms**: Robot constantly braking

**Solutions**:
1. ✅ Increase `slowPathNumThre` (Local Planner)
2. ✅ Increase `laserVoxelSize` (less dense obstacles)
3. ✅ Adjust `obstacleHeightThre` if ground is bumpy
4. ✅ Check LiDAR isn't seeing own robot parts

---

### 🗺️ Global Planner Fails Often

**Symptoms**: "No path found" errors

**Solutions**:
1. ✅ Check tomogram map covers goal area
2. ✅ Reduce `w_traversability` (less picky)
3. ✅ Increase `inflation_radius` for narrower gaps
4. ✅ Rebuild tomogram if environment changed

---

## Runtime Parameter Changes

### Using ROS 2 CLI

```bash
# View current parameters
ros2 param list /pathFollower

# Change a parameter
ros2 param set /pathFollower maxSpeed 1.5

# Dump all parameters to file
ros2 param dump /pathFollower > my_config.yaml
```

### Permanent Changes

Edit launch files, then rebuild:
```bash
colcon build --packages-select local_planner
source install/setup.bash
```

---

## Performance vs. Safety Tradeoffs

| Parameter | ← Conservative | Aggressive → |
|-----------|---------------|--------------|
| `maxSpeed` | 0.5 m/s | 2.0 m/s |
| `maxAccel` | 0.5 m/s² | 2.0 m/s² |
| `laserVoxelSize` | 0.05 m | 0.2 m |
| `adjacentRange` | 5.0 m | 2.0 m |
| `decayTime` | 30 s | 3 s |
| `obstacleHeightThre` | 0.1 m | 0.3 m |

**Recommendation**: Start conservative, tune aggressive once confident.

---

## Debugging Tools

### Visualize in RViz2

```bash
rviz2 -d src/base_autonomy/local_planner/rviz/navigation.rviz
```

**Key Topics**:
- `/free_paths`: See which paths are collision-free (green)
- `/terrain_map`: Check terrain analysis quality
- `/path`: Current selected path

### Monitor Metrics

```bash
# Path Follower performance
ros2 topic echo /cmd_vel

# Local Planner decisions
ros2 topic echo /slow_down

# Global Planner status
ros2 topic echo /planning_status
```

---

## 相关文档

- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 故障排查
- [BUILD_GUIDE.md](BUILD_GUIDE.md) — 编译指南
- [ARCHITECTURE.md](ARCHITECTURE.md) — 系统架构

---

*Last Updated: February 2026*
