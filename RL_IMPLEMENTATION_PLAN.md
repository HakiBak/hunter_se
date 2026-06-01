# Scout Mini RL Training Implementation Plan

## Overview

Add reinforcement learning (RL) training capabilities to Scout Mini robot in ROS2/Gazebo Harmonic, using TD3 algorithm with 3D LiDAR (Velodyne-style).

**Approach**: Step-by-step implementation with verification at each phase.

---

## Phase 1: Add 3D LiDAR Sensor

### 1.1 Create LiDAR Xacro Macro

**Create**: `src/ugv_gazebo_sim/scout/scout_description/urdf/lidar_3d.xacro`

- Define Velodyne-style 3D LiDAR macro for Gazebo Harmonic
- Use `gpu_lidar` sensor type with vertical scan
- Parameters: 16 vertical lasers, 1875 horizontal samples, 10Hz
- Topic: `scout_mini/points` (PointCloud2)

Key configuration:
```xml
<sensor name="velodyne" type="gpu_lidar">
  <topic>scout_mini/points</topic>
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1875</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <min_angle>-0.261799</min_angle>  <!-- -15 deg -->
        <max_angle>0.261799</max_angle>   <!-- +15 deg -->
      </vertical>
    </scan>
    <range>
      <min>0.3</min>
      <max>30.0</max>
    </range>
  </lidar>
</sensor>
```

### 1.2 Modify Scout Mini URDF

**Modify**: `src/ugv_gazebo_sim/scout/scout_description/urdf/scout_mini.xacro`

- Include the new lidar_3d.xacro
- Add LiDAR link and fixed joint on top of robot (z=0.2m above base_link)

### 1.3 Update Bridge Configuration

**Modify**: `src/ugv_gazebo_sim/scout/scout_gazebo_sim/config/scout_mini_bridge_ros_gz.yaml`

- Add PointCloud2 bridge entry:
```yaml
- ros_topic_name: "scout_mini/points"
  gz_topic_name: "scout_mini/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: "GZ_TO_ROS"
```

### 1.4 Verification
```bash
colcon build --packages-select scout_description scout_gazebo_sim
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
ros2 topic echo /scout_mini/points --once
# Visualize in RViz with PointCloud2 display
```

---

## Phase 2: Create Training World

### 2.1 Create Obstacles World

**Create**: `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_simple.world`

Components:
- 10m x 10m arena with boundary walls
- 4 static obstacles (varied shapes: boxes, cylinders)
- 4 movable box obstacles (for randomization during training)
- Ground plane with friction
- Proper physics settings (10ms step, 100Hz)
- Required Gazebo Harmonic plugins (Physics, Sensors, UserCommands, SceneBroadcaster)

### 2.2 Verification
```bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py world_name:=obstacles_simple.world
# Verify obstacles visible, robot can navigate, LiDAR detects obstacles
```

---

## Phase 3: Create RL Training Package (scout_rl)

### 3.1 Package Structure

**Create**: `src/scout_rl/`
```
src/scout_rl/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/scout_rl
├── scout_rl/
│   ├── __init__.py
│   ├── gazebo_interface.py    # Gazebo Harmonic control
│   ├── scout_env.py           # Gymnasium environment
│   ├── train_td3.py           # TD3 training script
│   ├── test_policy.py         # Test trained policy
│   └── utils.py               # Point cloud processing
├── config/
│   └── training_config.yaml
├── models/                    # Saved models
└── launch/
    └── training.launch.py
```

### 3.2 Gazebo Interface (`gazebo_interface.py`)

Key functions:
- `pause_simulation()` / `unpause_simulation()` - via `gz service`
- `reset_simulation()` - reset world state
- `set_entity_pose()` - teleport robot/obstacles
- `publish_cmd_vel()` - send velocity commands
- `get_robot_state()` - from odometry subscriber
- `get_point_cloud()` - from PointCloud2 subscriber

### 3.3 Gymnasium Environment (`scout_env.py`)

**Observation Space** (24 dimensions):
- 20 LiDAR bins (min distance per angular sector, processed from 3D point cloud)
- Goal distance
- Goal angle (relative to robot heading)
- Linear velocity
- Angular velocity

**Action Space** (2 continuous):
- Linear velocity: [0, 1] → [0, 1.0 m/s]
- Angular velocity: [-1, 1] → [-0.5, 0.5 rad/s]

**Reward Function**:
- +100 for reaching goal (< 0.4m)
- -100 for collision (min_laser < 0.4m)
- Progress reward: `(prev_dist - curr_dist) * 10`
- Forward motion reward: `linear_vel * 0.5`
- Steering penalty: `-|angular_vel| * 0.2`

**Episode**:
- Max 500 steps
- Random robot start position and orientation
- Random goal position
- Random movable obstacle positions

### 3.4 TD3 Training Script (`train_td3.py`)

Using Stable Baselines3:
```python
model = TD3(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    buffer_size=100000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    policy_kwargs=dict(net_arch=[400, 300]),
)
```

Features:
- TensorBoard logging
- Checkpoint saving every 10k steps
- Evaluation callback every 5k steps
- Resume from saved model support

### 3.5 Verification
```bash
colcon build --packages-select scout_rl
source install/setup.bash

# Terminal 1: Launch simulation
ros2 launch scout_rl training.launch.py

# Terminal 2: Run training (short test)
ros2 run scout_rl train_td3 --timesteps 10000

# Monitor
tensorboard --logdir ./logs
```

---

## Files to Create/Modify

| Action | File Path |
|--------|-----------|
| CREATE | `src/ugv_gazebo_sim/scout/scout_description/urdf/lidar_3d.xacro` |
| MODIFY | `src/ugv_gazebo_sim/scout/scout_description/urdf/scout_mini.xacro` |
| MODIFY | `src/ugv_gazebo_sim/scout/scout_gazebo_sim/config/scout_mini_bridge_ros_gz.yaml` |
| CREATE | `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_simple.world` |
| CREATE | `src/scout_rl/package.xml` |
| CREATE | `src/scout_rl/setup.py` |
| CREATE | `src/scout_rl/setup.cfg` |
| CREATE | `src/scout_rl/resource/scout_rl` |
| CREATE | `src/scout_rl/scout_rl/__init__.py` |
| CREATE | `src/scout_rl/scout_rl/gazebo_interface.py` |
| CREATE | `src/scout_rl/scout_rl/scout_env.py` |
| CREATE | `src/scout_rl/scout_rl/train_td3.py` |
| CREATE | `src/scout_rl/scout_rl/test_policy.py` |
| CREATE | `src/scout_rl/scout_rl/utils.py` |
| CREATE | `src/scout_rl/config/training_config.yaml` |
| CREATE | `src/scout_rl/launch/training.launch.py` |

---

## Reference Files

- **LiDAR pattern**: `/home/hakim/ros2_galaxy/agileX_shorty/DRL-robot-navigation/catkin_ws/src/velodyne_simulator/velodyne_description/urdf/VLP-16.urdf.xacro`
- **Gym env pattern**: `/home/hakim/ros2_galaxy/agileX_shorty/DRL-robot-navigation/TD3/velodyne_env.py`
- **Training pattern**: `/home/hakim/ros2_galaxy/agileX_shorty/DRL-robot-navigation/TD3/train_velodyne_td3.py`
- **World pattern**: `/home/hakim/ros2_galaxy/agileX_shorty/src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/empty.world`

---

## Dependencies to Install

```bash
pip install gymnasium stable-baselines3 tensorboard torch
```

---

## Quick Start (After Implementation)

```bash
# Build
colcon build --packages-select scout_description scout_gazebo_sim scout_rl

# Source
source install/setup.bash

# Launch simulation (Terminal 1)
ros2 launch scout_rl training.launch.py

# Train (Terminal 2)
ros2 run scout_rl train_td3 --timesteps 500000

# Test trained model
ros2 run scout_rl test_policy --model ./models/td3_final
```
