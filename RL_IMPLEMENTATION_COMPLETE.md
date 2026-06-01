# Scout Mini RL Training - Complete Implementation

## Overview

Add reinforcement learning (RL) training capabilities to Scout Mini robot in ROS2/Gazebo Harmonic, using TD3 algorithm with 3D LiDAR.

---

## Phase 1: Add 3D LiDAR Sensor

### 1.1 Create LiDAR Xacro Macro

**File**: `src/ugv_gazebo_sim/scout/scout_description/urdf/lidar_3d.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="lidar_3d" params="parent:=base_link">

        <link name="velodyne_link">
            <inertial>
                <mass value="0.01"/>
                <origin xyz="0 0 0"/>
                <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
            </inertial>
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <cylinder radius="0.05" length="0.07"/>
                </geometry>
                <material name="lidar_material">
                    <color rgba="0.5 0.5 0.5 0.5"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <cylinder radius="0.05" length="0.07"/>
                </geometry>
            </collision>
        </link>

        <joint name="velodyne_joint" type="fixed">
            <parent link="${parent}"/>
            <child link="velodyne_link"/>
            <origin xyz="0 0 0.2" rpy="0 0 0"/>
        </joint>

        <gazebo reference="velodyne_link">
            <sensor name="velodyne" type="gpu_lidar">
                <topic>scout_mini/points</topic>
                <update_rate>10</update_rate>
                <lidar>
                    <scan>
                        <horizontal>
                            <samples>1875</samples>
                            <resolution>1</resolution>
                            <min_angle>-3.14159</min_angle>
                            <max_angle>3.14159</max_angle>
                        </horizontal>
                        <vertical>
                            <samples>16</samples>
                            <resolution>1</resolution>
                            <min_angle>-0.261799</min_angle>
                            <max_angle>0.261799</max_angle>
                        </vertical>
                    </scan>
                    <range>
                        <min>0.3</min>
                        <max>30</max>
                        <resolution>0.01</resolution>
                    </range>
                </lidar>
                <always_on>1</always_on>
                <visualize>false</visualize>
            </sensor>
        </gazebo>
    </xacro:macro>
</robot>
```

### 1.2 Modify Scout Mini URDF

**File**: `src/ugv_gazebo_sim/scout/scout_description/urdf/scout_mini.xacro`

Add these lines:
```xml
<!-- Near the top, with other includes -->
<xacro:include filename="$(find scout_description)/urdf/lidar_3d.xacro"/>

<!-- Before closing </robot> tag -->
<xacro:lidar_3d parent="base_link"/>
```

### 1.3 Update Bridge Configuration

**File**: `src/ugv_gazebo_sim/scout/scout_gazebo_sim/config/scout_mini_bridge_ros_gz.yaml`

Add at end of file:
```yaml
# 3D LiDAR PointCloud
- ros_topic_name: "scout_mini/points"
  gz_topic_name: "scout_mini/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: "GZ_TO_ROS"
```

### 1.4 Verification

```bash
colcon build --packages-select scout_description scout_gazebo_sim
source install/setup.bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py world_name:=obstacles_easy.world
ros2 topic list | grep points
ros2 topic echo /scout_mini/points --once
```

---

## Phase 2: Create Training World

### 2.1 Create Obstacles World

**File**: `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_easy.world`

```xml
<?xml version="1.0"?>

<sdf version="1.7">
    <world name="obstacles_easy">
        <!-- Physics settings: 10ms step, 100Hz -->
        <physics name='10ms' type='ignored'>
            <max_step_size>0.01</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>100</real_time_update_rate>
        </physics>

        <!-- Gazebo Harmonic plugins -->
        <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
        <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
        <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
        <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'>
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
        <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

        <scene>
            <ambient>1.0 1.0 1.0</ambient>
            <background>0.8 0.8 0.8</background>
        </scene>

        <!-- Sun -->
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <!-- Ground plane with friction -->
        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.3 0.3 0.3 1</ambient>
                        <diffuse>0.3 0.3 0.3 1</diffuse>
                        <specular>0.1 0.1 0.1 1</specular>
                    </material>
                </visual>
            </link>
        </model>

        <!-- BOUNDARY WALLS (10m x 10m arena) -->

        <!-- North wall -->
        <model name="wall_north">
            <static>true</static>
            <pose>0 5 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>10 0.2 1</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>10 0.2 1</size></box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.5 1</ambient>
                        <diffuse>0.5 0.5 0.5 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- South wall -->
        <model name="wall_south">
            <static>true</static>
            <pose>0 -5 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>10 0.2 1</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>10 0.2 1</size></box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.5 1</ambient>
                        <diffuse>0.5 0.5 0.5 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- East wall -->
        <model name="wall_east">
            <static>true</static>
            <pose>5 0 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>0.2 10 1</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.2 10 1</size></box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.5 1</ambient>
                        <diffuse>0.5 0.5 0.5 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- West wall -->
        <model name="wall_west">
            <static>true</static>
            <pose>-5 0 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>0.2 10 1</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.2 10 1</size></box>
                    </geometry>
                    <material>
                        <ambient>0.5 0.5 0.5 1</ambient>
                        <diffuse>0.5 0.5 0.5 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- STATIC OBSTACLES (4 varied shapes) -->

        <!-- Static obstacle 1: Large box -->
        <model name="static_box_1">
            <static>true</static>
            <pose>2 2 0.4 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>0.8 0.8 0.8</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.8 0.8 0.8</size></box>
                    </geometry>
                    <material>
                        <ambient>0.8 0.2 0.2 1</ambient>
                        <diffuse>0.8 0.2 0.2 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Static obstacle 2: Cylinder -->
        <model name="static_cylinder_1">
            <static>true</static>
            <pose>-2 2 0.5 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>1.0</length>
                        </cylinder>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <cylinder>
                            <radius>0.4</radius>
                            <length>1.0</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>0.2 0.8 0.2 1</ambient>
                        <diffuse>0.2 0.8 0.2 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Static obstacle 3: Small box -->
        <model name="static_box_2">
            <static>true</static>
            <pose>-2 -2 0.3 0 0 0.785</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <box><size>0.6 0.6 0.6</size></box>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.6 0.6 0.6</size></box>
                    </geometry>
                    <material>
                        <ambient>0.2 0.2 0.8 1</ambient>
                        <diffuse>0.2 0.2 0.8 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Static obstacle 4: Tall cylinder -->
        <model name="static_cylinder_2">
            <static>true</static>
            <pose>2 -2 0.6 0 0 0</pose>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <cylinder>
                            <radius>0.3</radius>
                            <length>1.2</length>
                        </cylinder>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <cylinder>
                            <radius>0.3</radius>
                            <length>1.2</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.2 1</ambient>
                        <diffuse>0.8 0.8 0.2 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- MOVABLE OBSTACLES (4 boxes for randomization) -->

        <!-- Movable box 1 -->
        <model name="movable_box_1">
            <static>false</static>
            <pose>1 0 0.25 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>5.0</mass>
                    <inertia>
                        <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.1</iyy><iyz>0</iyz>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.5 0.1 1</ambient>
                        <diffuse>0.9 0.5 0.1 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Movable box 2 -->
        <model name="movable_box_2">
            <static>false</static>
            <pose>-1 0 0.25 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>5.0</mass>
                    <inertia>
                        <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.1</iyy><iyz>0</iyz>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.5 0.1 1</ambient>
                        <diffuse>0.9 0.5 0.1 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Movable box 3 -->
        <model name="movable_box_3">
            <static>false</static>
            <pose>0 1 0.25 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>5.0</mass>
                    <inertia>
                        <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.1</iyy><iyz>0</iyz>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.5 0.1 1</ambient>
                        <diffuse>0.9 0.5 0.1 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- Movable box 4 -->
        <model name="movable_box_4">
            <static>false</static>
            <pose>0 -1 0.25 0 0 0</pose>
            <link name="link">
                <inertial>
                    <mass>5.0</mass>
                    <inertia>
                        <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
                        <iyy>0.1</iyy><iyz>0</iyz>
                        <izz>0.1</izz>
                    </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>0.8</mu>
                                <mu2>0.8</mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box><size>0.5 0.5 0.5</size></box>
                    </geometry>
                    <material>
                        <ambient>0.9 0.5 0.1 1</ambient>
                        <diffuse>0.9 0.5 0.1 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

    </world>
</sdf>
```

### 2.2 Verification

```bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py world_name:=obstacles_easy.world
```

---

## Phase 3: Create RL Training Package (scout_rl)

### 3.1 Package Structure

```bash
mkdir -p src/scout_rl/scout_rl
mkdir -p src/scout_rl/config
mkdir -p src/scout_rl/models
mkdir -p src/scout_rl/launch
mkdir -p src/scout_rl/resource
touch src/scout_rl/resource/scout_rl
```

### 3.2 package.xml

**File**: `src/scout_rl/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>scout_rl</name>
  <version>0.0.1</version>
  <description>Reinforcement Learning training package for Scout Mini robot</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <depend>visualization_msgs</depend>
  <depend>ros_gz_bridge</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 3.3 setup.py

**File**: `src/scout_rl/setup.py`

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scout_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='RL training package for Scout Mini',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_td3 = scout_rl.train_td3:main',
            'test_policy = scout_rl.test_policy:main',
        ],
    },
)
```

### 3.4 setup.cfg

**File**: `src/scout_rl/setup.cfg`

```ini
[develop]
script_dir=$base/lib/scout_rl
[install]
install_scripts=$base/lib/scout_rl
```

### 3.5 __init__.py

**File**: `src/scout_rl/scout_rl/__init__.py`

```python
# Scout RL Package
```

### 3.6 utils.py

**File**: `src/scout_rl/scout_rl/utils.py`

```python
"""
Utility functions for point cloud processing and transformations.
"""

import math
import numpy as np


def process_point_cloud(points, num_bins=20):
    """
    Process 3D point cloud into distance bins for RL observation.
    """
    velodyne_data = np.ones(num_bins) * 10.0

    gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / num_bins]]
    for m in range(num_bins - 1):
        gaps.append([gaps[m][1], gaps[m][1] + np.pi / num_bins])
    gaps[-1][-1] += 0.03

    for point in points:
        x, y, z = point
        if z > -0.2:
            mag1 = math.sqrt(x**2 + y**2)
            if mag1 < 0.001:
                continue
            beta = math.acos(x / mag1) * np.sign(y)
            dist = math.sqrt(x**2 + y**2 + z**2)

            for j, gap in enumerate(gaps):
                if gap[0] <= beta < gap[1]:
                    velodyne_data[j] = min(velodyne_data[j], dist)
                    break

    return velodyne_data


def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z
```

### 3.7 gazebo_interface.py

**File**: `src/scout_rl/scout_rl/gazebo_interface.py`

```python
"""
Gazebo Harmonic interface for RL training.
"""

import subprocess
import math
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2

from .utils import process_point_cloud, quaternion_to_euler


class GazeboInterface(Node):
    """Interface to control Gazebo Harmonic simulation for RL training."""

    def __init__(self, environment_dim=20):
        super().__init__('gazebo_interface')

        self.environment_dim = environment_dim
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.velodyne_data = np.ones(environment_dim) * 10.0
        self.last_odom = None
        self.data_lock = threading.Lock()

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.vel_pub = self.create_publisher(Twist, 'scout_mini/cmd_vel', 10)
        self.goal_pub = self.create_publisher(MarkerArray, 'goal_marker', 3)

        self.odom_sub = self.create_subscription(
            Odometry, 'scout_mini/odom', self.odom_callback, sensor_qos)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'scout_mini/points', self.pointcloud_callback, sensor_qos)

        self.get_logger().info('Gazebo interface initialized')

    def odom_callback(self, msg):
        with self.data_lock:
            self.last_odom = msg
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            _, _, self.odom_yaw = quaternion_to_euler(q.w, q.x, q.y, q.z)
            self.linear_vel = msg.twist.twist.linear.x
            self.angular_vel = msg.twist.twist.angular.z

    def pointcloud_callback(self, msg):
        try:
            points = list(pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')))
            with self.data_lock:
                self.velodyne_data = process_point_cloud(points, self.environment_dim)
        except Exception as e:
            self.get_logger().warn(f'Error processing point cloud: {e}')

    def publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    def get_robot_state(self):
        with self.data_lock:
            return {
                'x': self.odom_x, 'y': self.odom_y, 'yaw': self.odom_yaw,
                'linear_vel': self.linear_vel, 'angular_vel': self.angular_vel
            }

    def get_velodyne_data(self):
        with self.data_lock:
            return self.velodyne_data.copy()

    def publish_goal_marker(self, goal_x, goal_y):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
        self.goal_pub.publish(marker_array)

    @staticmethod
    def pause_simulation():
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/obstacles_easy/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', 'pause: true'],
                capture_output=True, timeout=5)
        except Exception as e:
            print(f'Error pausing simulation: {e}')

    @staticmethod
    def unpause_simulation():
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/obstacles_easy/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', 'pause: false'],
                capture_output=True, timeout=5)
        except Exception as e:
            print(f'Error unpausing simulation: {e}')

    @staticmethod
    def reset_world():
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/obstacles_easy/control',
                 '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '2000', '--req', 'reset: {all: true}'],
                capture_output=True, timeout=5)
        except Exception as e:
            print(f'Error resetting world: {e}')

    @staticmethod
    def set_entity_pose(model_name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        pose_req = (f'name: "{model_name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                    f'orientation: {{w: {qw}, x: {qx}, y: {qy}, z: {qz}}}')

        try:
            subprocess.run(
                ['gz', 'service', '-s', '/world/obstacles_easy/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', pose_req],
                capture_output=True, timeout=5)
        except Exception as e:
            print(f'Error setting entity pose: {e}')
```

### 3.8 scout_env.py

**File**: `src/scout_rl/scout_rl/scout_env.py`

```python
"""
Gymnasium environment for Scout Mini robot RL training.
"""

import math
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .gazebo_interface import GazeboInterface

GOAL_REACHED_DIST = 0.4
COLLISION_DIST = 0.4
TIME_DELTA = 0.1
MAX_LINEAR_VEL = 1.0
MAX_ANGULAR_VEL = 0.5


class ScoutEnv(gym.Env):
    """Custom Gymnasium environment for Scout Mini robot navigation."""

    metadata = {'render_modes': ['human']}

    def __init__(self, environment_dim=20):
        super().__init__()

        self.environment_dim = environment_dim
        state_dim = environment_dim + 4

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(state_dim,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.goal_x = 3.0
        self.goal_y = 0.0
        self.prev_distance = None
        self.episode_step = 0
        self.max_episode_steps = 500
        self.arena_size = 4.5

        self.static_obstacles = [(2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0), (2.0, -2.0)]

        if not rclpy.ok():
            rclpy.init()

        self.gazebo = GazeboInterface(environment_dim)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gazebo)

        print("Waiting for sensors...")
        time.sleep(2.0)
        self._spin_once()

    def _spin_once(self):
        self.executor.spin_once(timeout_sec=0.1)

    def _get_observation(self):
        self._spin_once()
        laser_state = self.gazebo.get_velodyne_data()
        robot_state = self.gazebo.get_robot_state()

        distance = math.sqrt(
            (robot_state['x'] - self.goal_x)**2 +
            (robot_state['y'] - self.goal_y)**2)

        angle_to_goal = math.atan2(
            self.goal_y - robot_state['y'], self.goal_x - robot_state['x'])
        theta = angle_to_goal - robot_state['yaw']

        while theta > math.pi: theta -= 2 * math.pi
        while theta < -math.pi: theta += 2 * math.pi

        robot_obs = np.array([distance, theta, robot_state['linear_vel'],
                              robot_state['angular_vel']], dtype=np.float32)
        observation = np.concatenate([laser_state, robot_obs]).astype(np.float32)

        return observation, distance, robot_state

    def _get_reward(self, distance, collision, reached_goal, action, min_laser):
        if reached_goal: return 100.0
        if collision: return -100.0

        progress_reward = (self.prev_distance - distance) * 10.0 if self.prev_distance else 0.0
        forward_reward = action[0] * 0.5
        steering_penalty = -abs(action[1]) * 0.2
        proximity_penalty = -(1.0 - min_laser) * 0.5 if min_laser < 1.0 else 0.0

        return progress_reward + forward_reward + steering_penalty + proximity_penalty

    def step(self, action):
        self.episode_step += 1

        linear_vel = (action[0] + 1.0) / 2.0 * MAX_LINEAR_VEL
        angular_vel = action[1] * MAX_ANGULAR_VEL

        GazeboInterface.unpause_simulation()
        self.gazebo.publish_cmd_vel(linear_vel, angular_vel)
        time.sleep(TIME_DELTA)
        GazeboInterface.pause_simulation()

        observation, distance, robot_state = self._get_observation()
        min_laser = min(self.gazebo.get_velodyne_data())

        collision = min_laser < COLLISION_DIST
        reached_goal = distance < GOAL_REACHED_DIST
        truncated = self.episode_step >= self.max_episode_steps
        terminated = collision or reached_goal

        reward = self._get_reward(distance, collision, reached_goal, action, min_laser)
        self.prev_distance = distance

        info = {'collision': collision, 'reached_goal': reached_goal,
                'distance_to_goal': distance, 'min_laser': min_laser}

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_step = 0
        self.prev_distance = None

        GazeboInterface.reset_world()
        time.sleep(0.5)

        robot_x, robot_y, robot_yaw = self._get_random_robot_pose()
        GazeboInterface.set_entity_pose('scout_mini', robot_x, robot_y, 0.1, yaw=robot_yaw)

        self.goal_x, self.goal_y = self._get_random_goal_pose(robot_x, robot_y)
        self._randomize_obstacles(robot_x, robot_y)
        self.gazebo.publish_goal_marker(self.goal_x, self.goal_y)

        GazeboInterface.unpause_simulation()
        time.sleep(0.3)
        GazeboInterface.pause_simulation()

        observation, distance, _ = self._get_observation()
        self.prev_distance = distance

        return observation, {'goal_x': self.goal_x, 'goal_y': self.goal_y}

    def _get_random_robot_pose(self):
        while True:
            x = np.random.uniform(-self.arena_size + 1, self.arena_size - 1)
            y = np.random.uniform(-self.arena_size + 1, self.arena_size - 1)
            valid = all(math.sqrt((x - ox)**2 + (y - oy)**2) >= 1.5
                       for ox, oy in self.static_obstacles)
            if valid:
                return x, y, np.random.uniform(-math.pi, math.pi)

    def _get_random_goal_pose(self, robot_x, robot_y):
        while True:
            x = np.random.uniform(-self.arena_size + 0.5, self.arena_size - 0.5)
            y = np.random.uniform(-self.arena_size + 0.5, self.arena_size - 0.5)
            if math.sqrt((x - robot_x)**2 + (y - robot_y)**2) < 2.0:
                continue
            if all(math.sqrt((x - ox)**2 + (y - oy)**2) >= 1.0
                   for ox, oy in self.static_obstacles):
                return x, y

    def _randomize_obstacles(self, robot_x, robot_y):
        for i in range(4):
            while True:
                x = np.random.uniform(-3.5, 3.5)
                y = np.random.uniform(-3.5, 3.5)
                dist_robot = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                dist_goal = math.sqrt((x - self.goal_x)**2 + (y - self.goal_y)**2)
                if dist_robot > 1.5 and dist_goal > 1.0:
                    if all(math.sqrt((x - ox)**2 + (y - oy)**2) >= 1.0
                           for ox, oy in self.static_obstacles):
                        GazeboInterface.set_entity_pose(f'movable_box_{i+1}', x, y, 0.25)
                        break

    def close(self):
        self.gazebo.stop_robot()
        self.gazebo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

### 3.9 train_td3.py

**File**: `src/scout_rl/scout_rl/train_td3.py`

```python
"""
TD3 Training script using Stable Baselines3.
"""

import os
import argparse
import numpy as np

from stable_baselines3 import TD3
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, CallbackList
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.monitor import Monitor

from .scout_env import ScoutEnv


def main():
    parser = argparse.ArgumentParser(description='Train TD3 on Scout Mini')
    parser.add_argument('--timesteps', type=int, default=500000)
    parser.add_argument('--load-model', type=str, default=None)
    parser.add_argument('--save-dir', type=str, default='./models')
    parser.add_argument('--log-dir', type=str, default='./logs')
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    print("Creating environment...")
    env = Monitor(ScoutEnv(environment_dim=20), args.log_dir)

    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

    if args.load_model:
        print(f"Loading model from {args.load_model}")
        model = TD3.load(args.load_model, env=env)
        model.action_noise = action_noise
    else:
        print("Creating new TD3 model...")
        model = TD3(
            "MlpPolicy", env,
            learning_rate=3e-4, buffer_size=100000, learning_starts=10000,
            batch_size=256, tau=0.005, gamma=0.99,
            train_freq=(1, "episode"), gradient_steps=-1,
            action_noise=action_noise,
            policy_kwargs=dict(net_arch=[400, 300]),
            tensorboard_log=args.log_dir, verbose=1)

    callbacks = CallbackList([
        CheckpointCallback(save_freq=10000, save_path=args.save_dir, name_prefix='td3_scout'),
        EvalCallback(env, best_model_save_path=args.save_dir, log_path=args.log_dir,
                     eval_freq=5000, n_eval_episodes=5, deterministic=True)])

    print(f"Starting training for {args.timesteps} timesteps...")
    try:
        model.learn(total_timesteps=args.timesteps, callback=callbacks, progress_bar=True)
    except KeyboardInterrupt:
        print("\nTraining interrupted")

    model.save(os.path.join(args.save_dir, 'td3_final'))
    print(f"Model saved to {args.save_dir}/td3_final")
    env.close()


if __name__ == '__main__':
    main()
```

### 3.10 test_policy.py

**File**: `src/scout_rl/scout_rl/test_policy.py`

```python
"""
Test trained TD3 policy.
"""

import argparse
import time
import numpy as np
from stable_baselines3 import TD3
from .scout_env import ScoutEnv


def main():
    parser = argparse.ArgumentParser(description='Test TD3 policy')
    parser.add_argument('--model', type=str, required=True)
    parser.add_argument('--episodes', type=int, default=10)
    args = parser.parse_args()

    env = ScoutEnv(environment_dim=20)
    model = TD3.load(args.model)

    total_rewards, successes, collisions = [], 0, 0

    for ep in range(args.episodes):
        obs, info = env.reset()
        done, truncated, episode_reward, steps = False, False, 0, 0

        print(f"\nEpisode {ep + 1}/{args.episodes} - Goal: ({info['goal_x']:.2f}, {info['goal_y']:.2f})")

        while not done and not truncated:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1

        total_rewards.append(episode_reward)
        if info.get('reached_goal'): successes += 1; print(f"  SUCCESS in {steps} steps")
        elif info.get('collision'): collisions += 1; print(f"  COLLISION at step {steps}")
        else: print(f"  TIMEOUT after {steps} steps")
        print(f"  Reward: {episode_reward:.2f}")

    print(f"\n{'='*50}\nSUMMARY\n{'='*50}")
    print(f"Success: {successes}/{args.episodes} ({100*successes/args.episodes:.1f}%)")
    print(f"Collision: {collisions}/{args.episodes} ({100*collisions/args.episodes:.1f}%)")
    print(f"Avg reward: {np.mean(total_rewards):.2f} (+/- {np.std(total_rewards):.2f})")

    env.close()


if __name__ == '__main__':
    main()
```

### 3.11 training_config.yaml

**File**: `src/scout_rl/config/training_config.yaml`

```yaml
# Training Configuration
environment:
  lidar_bins: 20
  max_episode_steps: 500
  time_delta: 0.1
  arena_size: 4.5

robot:
  max_linear_vel: 1.0
  max_angular_vel: 0.5

rewards:
  goal_reached: 100.0
  collision: -100.0
  progress_scale: 10.0

thresholds:
  goal_reached: 0.4
  collision: 0.4

td3:
  learning_rate: 0.0003
  buffer_size: 100000
  batch_size: 256
  tau: 0.005
  gamma: 0.99
  net_arch: [400, 300]

training:
  total_timesteps: 500000
  checkpoint_freq: 10000
  eval_freq: 5000
```

### 3.12 training.launch.py

**File**: `src/scout_rl/launch/training.launch.py`

```python
"""
Launch file for Scout Mini RL training.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    scout_gazebo_dir = get_package_share_directory('scout_gazebo_sim')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='obstacles_easy.world'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_gazebo_dir, 'launch', 'scout_mini_empty_world.launch.py')),
            launch_arguments={'world_name': LaunchConfiguration('world_name')}.items()),
    ])
```

---

## Dependencies

```bash
pip install gymnasium stable-baselines3 tensorboard torch
```

---

## Quick Start

```bash
# Build
colcon build --packages-select scout_description scout_gazebo_sim scout_rl
source install/setup.bash

# Terminal 1: Launch simulation
ros2 launch scout_rl training.launch.py

# Terminal 2: Train
ros2 run scout_rl train_td3 --timesteps 500000

# Terminal 3: Monitor
tensorboard --logdir ./logs

# Test trained model
ros2 run scout_rl test_policy --model ./models/td3_final
```
