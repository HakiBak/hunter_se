# Differential Drive to Ackermann Steering Conversion Guide

## Scout Mini Robot - Conversion Analysis

This document outlines all files that need to be modified to convert the Scout Mini robot from differential drive to Ackermann steering.

---

## 1. URDF/Xacro File (MAJOR CHANGES)

**File:** `src/ugv_gazebo_sim/scout/scout_description/urdf/scout_mini.xacro`

| Change | Level | Details |
|--------|-------|---------|
| Add front steering links | Structural | Create `front_left_steer_link` and `front_right_steer_link` |
| Add front steering joints | Structural | Add revolute joints with Z-axis rotation, limits ±45° |
| Modify front wheel parent | Structural | Reparent front wheels from `base_link` to steering links |
| Replace DiffDrive plugin | Critical | Replace `gz-sim-diff-drive-system` with `gz-sim-ackermann-steering-system` |
| Update joint types | Moderate | Front wheels stay `continuous`, rear wheels stay `continuous` |
| Add kingpin geometry | Optional | Add visual links for steering kingpins |

### Plugin Replacement

**REMOVE:**
```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>front_left_wheel</left_joint>
    <right_joint>front_right_wheel</right_joint>
    <left_joint>rear_left_wheel</left_joint>
    <right_joint>rear_right_wheel</right_joint>
    <wheel_separation>0.490</wheel_separation>
    <wheel_radius>0.160</wheel_radius>
    <max_wheel_torque>20</max_wheel_torque>
    <max_linear_acceleration>1</max_linear_acceleration>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

**ADD:**
```xml
<plugin filename="gz-sim-ackermann-steering-system" name="gz::sim::systems::AckermannSteering">
    <left_steering_joint>front_left_steer</left_steering_joint>
    <right_steering_joint>front_right_steer</right_steering_joint>
    <left_joint>rear_left_wheel</left_joint>
    <right_joint>rear_right_wheel</right_joint>
    <wheel_separation>0.490</wheel_separation>
    <kingpin_width>0.416</kingpin_width>
    <wheel_base>0.464</wheel_base>
    <wheel_radius>0.160</wheel_radius>
    <steering_limit>0.785</steering_limit>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

### New Steering Joint Template

Add these joints for front wheel steering:

```xml
<!-- Front Left Steering Joint -->
<joint name="front_left_steer" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_steer_link"/>
    <origin xyz="0.232 0.208 -0.100" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="5"/>
</joint>

<link name="front_left_steer_link">
    <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
</link>

<!-- Front Right Steering Joint -->
<joint name="front_right_steer" type="revolute">
    <parent link="base_link"/>
    <child link="front_right_steer_link"/>
    <origin xyz="0.232 -0.208 -0.100" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="5"/>
</joint>

<link name="front_right_steer_link">
    <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
</link>
```

### Modified Front Wheel Joints

Change the parent of front wheels from `base_link` to the steering links.
**Note:** Each side has different `rpy` and `axis` values to account for wheel mesh orientation.

```xml
<!-- Front Right Wheel - CHILD OF STEERING LINK -->
<joint name="front_right_wheel" type="continuous">
    <origin rpy="1.57 0 0" xyz="0 0 0"/>
    <parent link="front_right_steer_link"/>
    <child link="front_right_wheel_link"/>
    <axis xyz="0 0 -1"/>
</joint>

<!-- Front Left Wheel - CHILD OF STEERING LINK -->
<joint name="front_left_wheel" type="continuous">
    <origin rpy="-1.57 0 0" xyz="0 0 0"/>
    <parent link="front_left_steer_link"/>  <!-- CHANGED from base_link -->
    <child link="front_left_wheel_link"/>
    <axis xyz="0 0 1"/>
</joint>
```

---

## 2. ROS-Gazebo Bridge Configuration (MINOR CHANGES)

**File:** `src/ugv_gazebo_sim/scout/scout_gazebo_sim/config/scout_mini_bridge_ros_gz.yaml`

| Change | Level | Details |
|--------|-------|---------|
| Update joint_states topic | Minor | May need to include new steering joints |
| Verify cmd_vel compatibility | Verify | Ackermann uses same Twist message format |

**Note:** The Ackermann plugin accepts the same `geometry_msgs/Twist` input, so minimal bridge changes required.

---

## 3. Launch Files (MINOR CHANGES)

**File:** `src/ugv_gazebo_sim/scout/scout_gazebo_sim/launch/scout_mini_empty_world.launch.py`

| Change | Level | Details |
|--------|-------|---------|
| No structural changes | None | Launch files remain compatible |
| Optional: Add steering params | Optional | Expose steering limits as launch arguments |

---

## 4. RViz Configuration (MINOR CHANGES)

**File:** `src/ugv_gazebo_sim/scout/scout_description/rviz/scout_mini.rviz`

| Change | Level | Details |
|--------|-------|---------|
| Add new TF frames | Minor | Include front steering link frames in visualization |

---

## 5. World File (NO CHANGES)

**File:** `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/empty.world`

No modifications required - physics and environment settings remain the same.

---

## Summary of Changes by Priority

### Critical (Must Change)
1. **scout_mini.xacro** - Complete restructure of front wheel joints and Gazebo plugin replacement

### Moderate (Likely Need Changes)
2. **scout_mini_bridge_ros_gz.yaml** - Add steering joint states to bridge

### Minor/Optional
3. **scout_mini.rviz** - Update TF visualization
4. **Launch files** - No changes required

---

## New Joint Hierarchy (After Conversion)

```
base_link
├── front_left_steer (revolute, Z-axis) ← NEW
│   └── front_left_steer_link ← NEW
│       └── front_left_wheel (continuous)
│           └── front_left_wheel_link
├── front_right_steer (revolute, Z-axis) ← NEW
│   └── front_right_steer_link ← NEW
│       └── front_right_wheel (continuous)
│           └── front_right_wheel_link
├── rear_left_wheel (continuous) ← UNCHANGED
│   └── rear_left_wheel_link
└── rear_right_wheel (continuous) ← UNCHANGED
    └── rear_right_wheel_link
```

---

## Key Geometric Parameters

| Parameter | Scout Mini Value | Description |
|-----------|-----------------|-------------|
| `wheel_separation` | 0.490 m | Lateral distance between wheels |
| `wheel_base` | 0.464 m | Longitudinal distance between axles |
| `wheel_radius` | 0.160 m | Wheel radius |
| `kingpin_width` | ~0.416 m | Distance between steering kingpins |
| `steering_limit` | 0.785 rad (45°) | Max steering angle |

---

## Ackermann Plugin Parameters Reference

| Parameter | Required | Description |
|-----------|----------|-------------|
| `left_steering_joint` | Yes | Left front steering joint name |
| `right_steering_joint` | Yes | Right front steering joint name |
| `left_joint` | Yes | Left rear wheel joint name |
| `right_joint` | Yes | Right rear wheel joint name |
| `wheel_separation` | Yes | Distance between left and right wheels |
| `wheel_base` | Yes | Distance between front and rear axles |
| `wheel_radius` | Yes | Wheel radius for odometry calculation |
| `kingpin_width` | No | Distance between steering kingpins (default: 0.8m) |
| `steering_limit` | No | Max steering angle in radians (default: 0.5) |
| `steering_only` | No | Control steering angle only, no drive (default: false) |
| `topic` | No | Input cmd_vel topic (default: cmd_vel) |
| `odom_topic` | No | Output odometry topic (default: odom) |
| `tf_topic` | No | Output TF topic (default: tf) |
| `frame_id` | No | Odometry frame (default: odom) |
| `child_frame_id` | No | Robot base frame (default: base_link) |
| `odom_publish_frequency` | No | Odometry publish rate Hz (default: 50) |

---

## Useful References

- [Gazebo AckermannSteering Class Reference](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html)
- [gz_ros2_control Ackermann Demo](https://control.ros.org/rolling/doc/gz_ros2_control/doc/index.html)
- [Example Ackermann Vehicle Project](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle)
- [ROS 2 Ackermann Vehicle with Nav2](https://github.com/alitekes1/ros2-ackermann-vehicle-gz-sim-harmonic-nav2)

---

## Reference Implementation: steer_bot

The `steer_bot` folder in this repository contains a reference implementation of Ackermann steering that can be used as inspiration:

- **URDF Structure:** `steer_bot/steer_bot_description/urdf/steer_bot.urdf.xacro`
- **Wheel Macros:** `steer_bot/steer_bot_description/urdf/wheel.xacro`
- **Controller Config:** `steer_bot/steer_bot_control/config/ctrl_ackermann_steering_controller.yaml`
- **Hardware Config:** `steer_bot/steer_bot_control/config/ctrl_steer_bot_hardware_gazebo.yaml`

**Note:** The steer_bot uses ROS 1 / Classic Gazebo, while scout uses ROS 2 / Gazebo Harmonic. The URDF structure concepts are transferable, but the plugin system differs.
