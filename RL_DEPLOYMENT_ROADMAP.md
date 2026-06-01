# Roadmap: RL Training → Real-World Deployment

## Scout Mini Ackermann Steering Robot

---

## Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  PHASE 1          PHASE 2          PHASE 3          PHASE 4    │
│  Simulation  →    RL Setup    →    Training    →    Real Robot │
│  Enhancement      Integration      & Tuning         Deployment │
└─────────────────────────────────────────────────────────────────┘
```

---

## Current Status (Completed)

- [x] Ackermann steering conversion (differential → Ackermann)
- [x] Steering joints and links configured
- [x] Gazebo Ackermann plugin working
- [x] Wheel friction parameters set
- [x] Velocity/acceleration limits configured
- [x] Basic simulation functional

---

## Phase 1: Simulation Enhancement

### 1.1 Add LiDAR Sensor

**Purpose:** Primary sensor for obstacle detection and navigation

**Files to modify:**
- `src/ugv_gazebo_sim/scout/scout_description/urdf/scout_mini.xacro`

**Procedure:**
1. Add LiDAR link to URDF (position on top of robot)
2. Add fixed joint connecting LiDAR to base_link
3. Add Gazebo LiDAR sensor plugin
4. Configure scan parameters (range, angle, resolution)
5. Update ROS-Gazebo bridge to forward /scan topic
6. Test: `ros2 topic echo /scout_mini/scan`

**Parameters to configure:**
```xml
- update_rate: 10-30 Hz
- samples: 360-720 (resolution)
- min_angle: -π
- max_angle: π
- min_range: 0.1 m
- max_range: 10-30 m
- noise: Gaussian (stddev ~0.01)
```

---

### 1.2 Add Camera (Optional)

**Purpose:** Visual input for RL, depth perception

**Procedure:**
1. Add camera link to URDF
2. Add camera joint (fixed to base_link)
3. Add Gazebo camera plugin
4. Configure image size, FOV, frame rate
5. Update bridge config for image topics
6. Test: `ros2 topic echo /scout_mini/rgb_cam/image_raw`

---

### 1.3 Create Training Worlds

**Purpose:** Diverse environments for robust RL training

**Files to create:**
- `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_simple.world`
- `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_complex.world`
- `src/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/maze.world`

**Procedure:**
1. Create simple world with static obstacles (boxes, cylinders)
2. Create complex world with varied obstacle shapes/sizes
3. Create maze-like environment for navigation challenges
4. Add ground plane with appropriate friction
5. Test each world with robot spawned

**World features:**
```
- Static obstacles (walls, boxes, cylinders)
- Varying obstacle density
- Different floor textures/friction
- Boundary walls to contain robot
- Goal markers (visual reference)
```

---

### 1.4 Create cmd_vel Filter Node

**Purpose:** Prevent extreme steering at low velocities, smooth control

**Files to create:**
- `src/ugv_gazebo_sim/scout/scout_gazebo_sim/scripts/cmd_vel_filter.py`

**Procedure:**
1. Create Python ROS2 node
2. Subscribe to `/cmd_vel_raw` (from RL agent)
3. Apply velocity/steering constraints
4. Publish to `/scout_mini/cmd_vel`
5. Add to launch file
6. Test with manual commands

**Filter logic:**
```python
min_linear_velocity = 0.3  # m/s
min_turn_radius = 0.5      # m
max_angular_velocity = 1.5 # rad/s

if abs(linear_x) < min_linear_velocity:
    angular_z = 0  # No steering at very low speeds
else:
    max_angular = abs(linear_x) / min_turn_radius
    angular_z = clamp(angular_z, -max_angular, max_angular)
```

---

### 1.5 Physics Parameter Tuning

**Purpose:** Realistic simulation for better sim-to-real transfer

**Parameters to tune in URDF:**
```yaml
Wheel friction:
  mu1: 0.8 - 1.2 (longitudinal)
  mu2: 0.6 - 1.0 (lateral)
  kp: 1000000.0 (contact stiffness)
  kd: 1.0 - 100.0 (contact damping)

Robot mass/inertia:
  - Verify matches real robot
  - Adjust center of mass if needed

Steering dynamics:
  damping: 0.5 - 2.0
  friction: 0.1 - 0.5
```

---

## Phase 2: RL Integration

### 2.1 Create Gym Environment

**Purpose:** Standard interface for RL algorithms

**Files to create:**
- `src/scout_rl/scout_rl/__init__.py`
- `src/scout_rl/scout_rl/envs/__init__.py`
- `src/scout_rl/scout_rl/envs/scout_nav_env.py`
- `src/scout_rl/setup.py`
- `src/scout_rl/package.xml`

**Procedure:**
1. Create new ROS2 package `scout_rl`
2. Implement Gymnasium environment class
3. Define observation space (LiDAR, odometry, goal)
4. Define action space (linear_vel, angular_vel)
5. Implement step(), reset(), render() methods
6. Register environment with Gymnasium
7. Test with random actions

---

### 2.2 Define Observation Space

**Components:**
```python
observation_space = Dict({
    # LiDAR scan (downsampled)
    "lidar": Box(low=0, high=30, shape=(36,), dtype=float32),

    # Robot velocity
    "velocity": Box(low=[-3, -3], high=[3, 3], shape=(2,), dtype=float32),

    # Goal relative position (distance, angle)
    "goal": Box(low=[-50, -π], high=[50, π], shape=(2,), dtype=float32),

    # Previous action (for smoothness)
    "prev_action": Box(low=[-1, -1], high=[1, 1], shape=(2,), dtype=float32),
})
```

---

### 2.3 Define Action Space

**Options:**

Option A - Continuous:
```python
action_space = Box(
    low=np.array([0.0, -1.0]),   # [min_linear, min_angular]
    high=np.array([1.0, 1.0]),   # [max_linear, max_angular]
    dtype=np.float32
)
# Scale to actual velocities in step()
```

Option B - Discrete (simpler):
```python
action_space = Discrete(5)
# 0: Forward
# 1: Forward-Left
# 2: Forward-Right
# 3: Slow Forward
# 4: Stop
```

---

### 2.4 Design Reward Function

**Components:**
```python
def compute_reward(self):
    reward = 0.0

    # Goal progress reward
    reward += (prev_goal_dist - curr_goal_dist) * 10.0

    # Goal reached bonus
    if goal_reached:
        reward += 100.0

    # Collision penalty
    if collision:
        reward -= 50.0

    # Time penalty (encourage efficiency)
    reward -= 0.1

    # Smoothness reward (penalize jerky motion)
    action_diff = abs(curr_action - prev_action)
    reward -= action_diff.sum() * 0.5

    # Velocity reward (encourage movement)
    if linear_vel > 0.3:
        reward += 0.1

    return reward
```

---

### 2.5 Simulation Control Interface

**Purpose:** Control Gazebo from RL environment

**Implement:**
```python
class GazeboInterface:
    def __init__(self):
        # ROS2 node
        self.node = rclpy.create_node('rl_interface')

        # Publishers
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Subscribers
        self.scan_sub = self.node.create_subscription(LaserScan, '/scout_mini/scan', ...)
        self.odom_sub = self.node.create_subscription(Odometry, '/scout_mini/odom', ...)

        # Service clients
        self.reset_client = self.node.create_client(Empty, '/reset_simulation')
        self.spawn_client = self.node.create_client(SpawnEntity, '/spawn_entity')

    def reset_robot(self, position, orientation):
        # Reset robot pose
        pass

    def step(self, action):
        # Apply action, get observation
        pass
```

---

### 2.6 Integrate with Stable Baselines3

**Files to create:**
- `src/scout_rl/scripts/train.py`
- `src/scout_rl/scripts/evaluate.py`
- `src/scout_rl/config/training_config.yaml`

**Training script structure:**
```python
from stable_baselines3 import PPO, SAC
from scout_rl.envs import ScoutNavEnv

# Create environment
env = ScoutNavEnv(
    world="obstacles_simple",
    max_steps=1000,
    goal_threshold=0.5
)

# Create model
model = PPO(
    "MultiInputPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    verbose=1,
    tensorboard_log="./logs/"
)

# Train
model.learn(total_timesteps=1_000_000)

# Save
model.save("scout_nav_ppo")
```

---

## Phase 3: Training & Tuning

### 3.1 Initial Training

**Procedure:**
1. Start with simple environment (few obstacles)
2. Use small network architecture
3. Train for 100k steps, evaluate
4. Monitor with TensorBoard
5. Adjust hyperparameters based on results

**Key hyperparameters:**
```yaml
PPO:
  learning_rate: 3e-4 → 1e-4 (decay)
  n_steps: 2048
  batch_size: 64
  n_epochs: 10
  gamma: 0.99
  gae_lambda: 0.95
  clip_range: 0.2
  ent_coef: 0.01

SAC (alternative):
  learning_rate: 3e-4
  buffer_size: 1_000_000
  batch_size: 256
  tau: 0.005
  gamma: 0.99
```

---

### 3.2 Curriculum Learning

**Procedure:**
1. Stage 1: No obstacles, reach goal (100k steps)
2. Stage 2: Few static obstacles (200k steps)
3. Stage 3: Many obstacles (300k steps)
4. Stage 4: Complex maze environment (400k steps)

**Implementation:**
```python
def get_curriculum_stage(total_steps):
    if total_steps < 100_000:
        return "empty"
    elif total_steps < 300_000:
        return "simple"
    elif total_steps < 600_000:
        return "complex"
    else:
        return "maze"
```

---

### 3.3 Domain Randomization

**Purpose:** Improve sim-to-real transfer

**Randomize:**
```python
# Physics
friction_range = [0.6, 1.2]
mass_range = [0.9, 1.1]  # multiplier

# Sensors
lidar_noise_range = [0.0, 0.05]
lidar_dropout_prob = [0.0, 0.1]

# Environment
obstacle_positions = random
goal_positions = random
initial_robot_pose = random

# Control
action_delay_range = [0, 3]  # steps
velocity_scale_range = [0.9, 1.1]
```

---

### 3.4 Evaluation & Metrics

**Metrics to track:**
```
- Success rate (% goals reached)
- Average episode length
- Average reward
- Collision rate
- Path efficiency (actual / optimal distance)
- Smoothness (acceleration variance)
```

**Evaluation procedure:**
1. Run 100 episodes in test environment
2. Record all metrics
3. Visualize trajectories
4. Compare with baseline (random, heuristic)

---

## Phase 4: Real Robot Deployment

### 4.1 Hardware Requirements

**Components needed:**
```
- Ackermann steering robot chassis
- Motor controllers (rear drive wheels)
- Steering actuator (front wheels)
- LiDAR sensor (same model as simulated)
- Onboard computer (Jetson, Pi, etc.)
- Battery and power management
```

---

### 4.2 ROS2 Hardware Interface

**Files to create:**
- `src/scout_hardware/scout_hardware/motor_driver.py`
- `src/scout_hardware/scout_hardware/steering_driver.py`
- `src/scout_hardware/launch/robot_bringup.launch.py`

**Interface structure:**
```python
class ScoutHardwareInterface:
    def __init__(self):
        # Motor controller connection (CAN/Serial)
        self.motor = MotorDriver(port='/dev/ttyUSB0')

        # Steering servo
        self.steering = SteeringDriver(port='/dev/ttyUSB1')

        # cmd_vel subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/scout_mini/cmd_vel', self.cmd_callback, 10
        )

    def cmd_callback(self, msg):
        # Convert Twist to motor commands
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate steering angle
        if abs(linear_vel) > 0.1:
            turn_radius = linear_vel / angular_vel
            steering_angle = atan(wheel_base / turn_radius)
        else:
            steering_angle = 0

        # Send to hardware
        self.motor.set_velocity(linear_vel)
        self.steering.set_angle(steering_angle)
```

---

### 4.3 Sensor Calibration

**Procedure:**
1. Mount LiDAR at same position as in simulation
2. Calibrate LiDAR-to-base_link transform
3. Compare real scans vs simulated scans
4. Adjust URDF sensor position if needed
5. Verify scan matches environment

---

### 4.4 Model Deployment

**Procedure:**
1. Export trained model to ONNX (optional, for speed)
2. Create inference node
3. Test in controlled environment first
4. Gradually increase complexity
5. Monitor and collect failure cases

**Inference node:**
```python
class PolicyNode:
    def __init__(self):
        # Load trained model
        self.model = PPO.load("scout_nav_ppo")

        # Observation buffers
        self.lidar_data = None
        self.odom_data = None
        self.goal = None

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def control_loop(self):
        obs = self.get_observation()
        action, _ = self.model.predict(obs, deterministic=True)
        self.publish_action(action)
```

---

### 4.5 Safety Systems

**Implement:**
```python
class SafetyMonitor:
    def __init__(self):
        self.min_obstacle_dist = 0.3  # meters
        self.max_velocity = 1.0  # m/s
        self.emergency_stop = False

    def check_safety(self, lidar_scan, cmd_vel):
        # Check for obstacles
        min_dist = min(lidar_scan.ranges)
        if min_dist < self.min_obstacle_dist:
            return Twist()  # Stop

        # Limit velocity
        cmd_vel.linear.x = clamp(cmd_vel.linear.x, 0, self.max_velocity)

        return cmd_vel
```

---

## File Structure (Final)

```
agileX_shorty/
├── src/
│   ├── ugv_gazebo_sim/
│   │   └── scout/
│   │       ├── scout_description/
│   │       │   ├── urdf/
│   │       │   │   └── scout_mini.xacro (with LiDAR)
│   │       │   └── rviz/
│   │       └── scout_gazebo_sim/
│   │           ├── launch/
│   │           ├── worlds/
│   │           │   ├── empty.world
│   │           │   ├── obstacles_simple.world
│   │           │   ├── obstacles_complex.world
│   │           │   └── maze.world
│   │           ├── config/
│   │           └── scripts/
│   │               └── cmd_vel_filter.py
│   │
│   ├── scout_rl/  (NEW)
│   │   ├── scout_rl/
│   │   │   ├── __init__.py
│   │   │   └── envs/
│   │   │       ├── __init__.py
│   │   │       └── scout_nav_env.py
│   │   ├── scripts/
│   │   │   ├── train.py
│   │   │   └── evaluate.py
│   │   ├── config/
│   │   │   └── training_config.yaml
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── scout_hardware/  (FOR REAL ROBOT)
│       ├── scout_hardware/
│       │   ├── motor_driver.py
│       │   └── steering_driver.py
│       ├── launch/
│       │   └── robot_bringup.launch.py
│       └── package.xml
│
├── models/  (trained models)
│   └── scout_nav_ppo.zip
│
├── logs/  (TensorBoard)
│
└── RL_DEPLOYMENT_ROADMAP.md (this file)
```

---

## Quick Reference Commands

```bash
# Build
colcon build --packages-select scout_description scout_gazebo_sim scout_rl

# Launch simulation
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py world_name:=obstacles_simple.world

# Test cmd_vel
ros2 topic pub /scout_mini/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"

# View LiDAR in RViz
ros2 run rviz2 rviz2

# Train RL
python3 src/scout_rl/scripts/train.py

# Evaluate
python3 src/scout_rl/scripts/evaluate.py --model models/scout_nav_ppo.zip

# TensorBoard
tensorboard --logdir logs/
```

---

## Timeline Estimate

| Phase | Tasks | Estimated Effort |
|-------|-------|------------------|
| Phase 1 | Simulation Enhancement | 1-2 weeks |
| Phase 2 | RL Integration | 1-2 weeks |
| Phase 3 | Training & Tuning | 2-4 weeks |
| Phase 4 | Real Robot | 2-4 weeks |

**Total: 6-12 weeks** (depending on experience and hardware availability)

---

## Next Action

Start with **Phase 1.1: Add LiDAR Sensor** to the Scout Mini URDF.
