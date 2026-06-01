# Scout RL Bug Fix Plan

## Overview
This plan addresses **all 15 identified bugs** and issues in the `scout_rl` package, prioritized by severity. The fixes are based on comparison with the reference `DRL-robot-navigation` implementation and analysis of the actual Gazebo world configuration.

### User Preferences (Confirmed)
- **Reverse Movement:** Asymmetric (30% reverse speed)
- **Scope:** All 15 fixes
- **Network Size:** [512, 512, 256]

---

## Phase 1: Critical Fixes

### 1.1 Make World Name Configurable
**File:** `scout_rl/gazebo_interface.py`
**Lines:** 110-163

**Problem:** World name `obstacles_easy` is hardcoded in 4 places. If world changes, all Gazebo commands fail silently.

**Fix:**
```python
class GazeboInterface(Node):
    def __init__(self, environment_dim=20, world_name='obstacles_easy'):
        self.world_name = world_name
        # ... rest of init

    def pause_simulation(self):
        # Use self.world_name instead of hardcoded string
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world_name}/control', ...])
```

**Changes needed:**
- Add `world_name` parameter to `__init__`
- Convert static methods to instance methods (or pass world_name)
- Update all 4 occurrences of `/world/obstacles_easy/`

---

### 1.2 Add Proper Obstacle Zone Validation
**File:** `scout_rl/scout_env.py`
**Lines:** 46, 146-176

**Problem:** Only 4 corner points defined as obstacles. Missing wall boundaries and cylinder obstacles.

**Fix:** Add comprehensive `check_pos()` function based on actual world file:

```python
def _is_valid_position(self, x, y, min_dist_from_obstacles=0.5):
    """Check if position is valid (not inside walls or obstacles)."""
    # Arena boundaries (walls at ±5, but leave margin)
    if abs(x) > 4.5 or abs(y) > 4.5:
        return False

    # Static obstacles from obstacles_easy.world:
    # static_box_1: (2, 2) - 0.8x0.8m box
    # static_cylinder_1: (-2, 2) - r=0.4m cylinder
    # static_box_2: (-2, -2) - 0.6x0.6m box (rotated)
    # static_cylinder_2: (2, -2) - r=0.3m cylinder

    obstacles = [
        {'pos': (2.0, 2.0), 'radius': 0.6},    # box_1 (0.8/2 + margin)
        {'pos': (-2.0, 2.0), 'radius': 0.6},   # cylinder_1 (0.4 + margin)
        {'pos': (-2.0, -2.0), 'radius': 0.5},  # box_2 (0.6/2 + margin)
        {'pos': (2.0, -2.0), 'radius': 0.5},   # cylinder_2 (0.3 + margin)
    ]

    for obs in obstacles:
        dist = math.sqrt((x - obs['pos'][0])**2 + (y - obs['pos'][1])**2)
        if dist < obs['radius'] + min_dist_from_obstacles:
            return False
    return True
```

**Update these methods to use validation:**
- `_get_random_robot_pose()` - line 146
- `_get_random_goal_pose()` - line 155
- `_randomize_obstacles()` - line 165

---

### 1.3 Load and Use Configuration File
**Files:** `scout_rl/scout_env.py`, `scout_rl/train_td3.py`

**Problem:** `training_config.yaml` exists but is never loaded.

**Fix:** Create config loader utility and use throughout:

```python
# scout_rl/config_loader.py (new file)
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_config():
    pkg_dir = get_package_share_directory('scout_rl')
    config_path = os.path.join(pkg_dir, 'config', 'training_config.yaml')
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)
```

**Update scout_env.py:**
```python
from .config_loader import load_config

class ScoutEnv(gym.Env):
    def __init__(self, config=None):
        self.config = config or load_config()
        self.environment_dim = self.config['environment']['lidar_bins']
        self.max_episode_steps = self.config['environment']['max_episode_steps']
        # ... use config values throughout
```

---

### 1.4 Fix Linear Velocity Mapping (Allow Reverse)
**File:** `scout_rl/scout_env.py`
**Line:** 98

**Problem:** Current mapping `(action[0] + 1.0) / 2.0` maps [-1,1] to [0,1]. Robot cannot reverse.

**Fix:** Asymmetric bidirectional movement (user confirmed):
```python
# Asymmetric: Full forward speed, 30% reverse speed
MAX_REVERSE_VEL = 0.3  # 30% of max forward

if action[0] >= 0:
    linear_vel = action[0] * MAX_LINEAR_VEL  # 0 to 1.0 m/s
else:
    linear_vel = action[0] * MAX_REVERSE_VEL  # -0.3 to 0 m/s
```

---

## Phase 2: Significant Fixes

### 2.1 Add Curriculum Learning for Goals
**File:** `scout_rl/scout_env.py`
**Lines:** 155-163

**Problem:** Fixed goal placement bounds. Reference gradually expands difficulty.

**Fix:**
```python
class ScoutEnv(gym.Env):
    def __init__(self, ...):
        # ... existing init
        self.goal_distance_min = 1.5  # Start with nearby goals
        self.goal_distance_max = 3.0  # Initial max distance
        self.curriculum_increment = 0.002  # Expand per episode
        self.curriculum_max = self.arena_size - 0.5

    def reset(self, ...):
        # ... existing reset code
        # Gradually increase difficulty
        if self.goal_distance_max < self.curriculum_max:
            self.goal_distance_max += self.curriculum_increment
        # ...

    def _get_random_goal_pose(self, robot_x, robot_y):
        while True:
            # Random angle from robot
            angle = np.random.uniform(-math.pi, math.pi)
            distance = np.random.uniform(self.goal_distance_min, self.goal_distance_max)
            x = robot_x + distance * math.cos(angle)
            y = robot_y + distance * math.sin(angle)

            if self._is_valid_position(x, y) and abs(x) < self.arena_size - 0.5:
                return x, y
```

---

### 2.2 Fix Sensor Data Synchronization
**File:** `scout_rl/gazebo_interface.py`

**Problem:** Single `spin_once()` may return stale data.

**Fix:** Add multi-spin with freshness check:
```python
def wait_for_fresh_data(self, timeout=1.0):
    """Wait for fresh sensor data after simulation step."""
    start_time = time.time()
    initial_odom = self.last_odom

    while time.time() - start_time < timeout:
        self.executor.spin_once(timeout_sec=0.05)
        if self.last_odom is not initial_odom:
            return True
    return False
```

**Update scout_env.py step():**
```python
def step(self, action):
    # ... publish cmd_vel
    GazeboInterface.unpause_simulation()
    time.sleep(TIME_DELTA)
    GazeboInterface.pause_simulation()

    # Wait for fresh sensor data
    self.gazebo.wait_for_fresh_data(timeout=0.5)

    observation, distance, robot_state = self._get_observation()
```

---

### 2.3 Adjust Thresholds to Match Reference
**File:** `scout_rl/scout_env.py`
**Lines:** 16-17

**Current:**
```python
GOAL_REACHED_DIST = 0.4
COLLISION_DIST = 0.4
```

**Fix:**
```python
GOAL_REACHED_DIST = 0.3   # Tighter goal precision
COLLISION_DIST = 0.35     # Better collision detection
```

---

### 2.4 Add Exploration Noise Decay
**File:** `scout_rl/train_td3.py`

**Problem:** Constant noise σ=0.1 throughout training.

**Fix:** Use decaying noise with custom callback:
```python
from stable_baselines3.common.callbacks import BaseCallback

class NoiseDecayCallback(BaseCallback):
    def __init__(self, initial_noise=0.5, final_noise=0.1, decay_steps=300000):
        super().__init__()
        self.initial_noise = initial_noise
        self.final_noise = final_noise
        self.decay_steps = decay_steps

    def _on_step(self):
        progress = min(1.0, self.num_timesteps / self.decay_steps)
        current_noise = self.initial_noise - progress * (self.initial_noise - self.final_noise)
        self.model.action_noise.sigma = current_noise * np.ones(self.model.action_space.shape[-1])
        return True

# In main():
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.5 * np.ones(n_actions))
callbacks = CallbackList([
    NoiseDecayCallback(initial_noise=0.5, final_noise=0.1, decay_steps=300000),
    CheckpointCallback(...),
    EvalCallback(...)
])
```

---

### 2.5 Increase Network Size and Buffer
**File:** `scout_rl/train_td3.py`
**Lines:** 42-46

**Current:**
```python
buffer_size=100000,
policy_kwargs=dict(net_arch=[400, 300])
```

**Fix (user confirmed [512, 512, 256]):**
```python
buffer_size=500000,  # 5x larger (compromise between 100k and 1M)
policy_kwargs=dict(net_arch=[512, 512, 256])  # 3-layer network with more capacity
```

This 3-layer architecture provides:
- Better feature extraction (512 neurons per layer)
- Deeper representation learning
- ~2.5x more parameters than current [400, 300]

---

## Phase 3: Medium Priority Fixes

### 3.1 Add Safety Escape Mechanism
**File:** `scout_rl/scout_env.py`

**Add escape behavior during training:**
```python
def step(self, action):
    # Check if too close to obstacle
    min_laser = min(self.gazebo.get_velodyne_data())

    # Emergency escape behavior (10% chance when too close)
    if min_laser < 0.6 and np.random.random() > 0.9:
        # Execute escape sequence
        for _ in range(5):
            escape_action = np.array([-0.5, np.random.uniform(-1, 1)])
            self._execute_action(escape_action)

    # Normal action execution
    # ... rest of step
```

---

### 3.2 Improve Reward Function
**File:** `scout_rl/scout_env.py`
**Lines:** 84-93

**Enhanced reward with better shaping:**
```python
def _get_reward(self, distance, collision, reached_goal, action, min_laser, robot_state):
    if reached_goal:
        return 100.0
    if collision:
        return -100.0

    # Progress reward (most important)
    progress = (self.prev_distance - distance) * 15.0 if self.prev_distance else 0.0

    # Velocity reward (use actual velocity, not action)
    velocity_reward = robot_state['linear_vel'] * 0.3

    # Steering penalty (penalize excessive turning)
    steering_penalty = -abs(robot_state['angular_vel']) * 0.3

    # Proximity penalty (exponential, stronger when closer)
    if min_laser < 1.0:
        proximity_penalty = -((1.0 - min_laser) ** 2) * 2.0
    else:
        proximity_penalty = 0.0

    # Goal heading bonus (encourage facing goal)
    # theta is already in observation
    heading_bonus = 0.1 * (1.0 - abs(self.current_theta) / math.pi)

    return progress + velocity_reward + steering_penalty + proximity_penalty + heading_bonus
```

---

### 3.3 Increase Training Duration
**File:** `scout_rl/config/training_config.yaml`
**Line:** 43

**Change:**
```yaml
training:
  total_timesteps: 1000000  # Was 500000, now 1M
  learning_starts: 25000    # Was 10000
```

---

## Phase 4: Code Quality Improvements

### 4.1 Add Proper Logging
**File:** `scout_rl/scout_env.py`

```python
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('scout_rl')

class ScoutEnv:
    def reset(self, ...):
        logger.info(f"Episode reset - Robot: ({robot_x:.2f}, {robot_y:.2f}), "
                   f"Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")
```

### 4.2 Add Type Hints
Add type hints to all functions for better maintainability.

### 4.3 Create Unit Tests
Create `test/` directory with tests for:
- `test_utils.py` - Test point cloud processing, quaternion conversions
- `test_scout_env.py` - Test environment reset, step, reward calculation

---

## Implementation Order

| Priority | Task | Estimated Complexity |
|----------|------|---------------------|
| 1 | Fix obstacle zone validation | Medium |
| 2 | Fix linear velocity (allow reverse) | Low |
| 3 | Load config file | Medium |
| 4 | Make world name configurable | Low |
| 5 | Adjust thresholds | Low |
| 6 | Add curriculum learning | Medium |
| 7 | Fix sensor synchronization | Medium |
| 8 | Add noise decay callback | Medium |
| 9 | Increase network/buffer size | Low |
| 10 | Improve reward function | Medium |
| 11 | Add safety escape mechanism | Low |
| 12 | Increase training duration | Low |

---

## Files to Modify

1. **scout_rl/scout_env.py** - Main environment (most changes)
2. **scout_rl/gazebo_interface.py** - Gazebo interface fixes
3. **scout_rl/train_td3.py** - Training script improvements
4. **scout_rl/config_loader.py** - New file for config loading
5. **config/training_config.yaml** - Update default values

---

## Verification Steps

After implementation:
1. Run `ros2 launch scout_rl training.launch.py` - Verify world loads
2. Test robot positioning - Confirm no spawns inside obstacles
3. Test goal placement - Confirm goals are reachable
4. Verify sensor data - Check point cloud is received
5. Run short training (1000 steps) - Verify no crashes
6. Monitor rewards - Should see positive progress over time
