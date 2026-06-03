"""
Gymnasium environment for Hunter SE robot RL training.
"""

import math
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .gazebo_interface import GazeboInterface

GOAL_REACHED_DIST = 0.5
COLLISION_DIST = 0.5  # Hunter SE front bumper reaches x=0.475 from lidar; obstacle in frontal contact reads ~0.47
TIME_DELTA = 0.1 # modified from 0.1 to increase training time 
MAX_LINEAR_VEL = 1.0   # < real top speed 1.33 m/s
MAX_ANGULAR_VEL = 0.6  # feasible turn rate at speed given min radius ~1.8 m (real Ackermann limit)


class HunterSEEnv(gym.Env):
    """Custom Gymnasium environment for Hunter SE robot navigation."""

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
        self.max_episode_steps = 1000
        self.arena_size = 9.0

        self.static_obstacles = [(2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0), (2.0, -2.0)]

        if not rclpy.ok():
            rclpy.init()

        self.gazebo = GazeboInterface(environment_dim)

        print("Waiting for sensors...")
        self.gazebo.unpause_simulation()
        time.sleep(2.0)

    def _get_observation(self):
        laser_state = self.gazebo.get_velodyne_data(wait_for_fresh=False)
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
        idle_penalty = -2.0 if action[0] < -0.5 else 0.0

        return progress_reward + forward_reward + steering_penalty + proximity_penalty + idle_penalty

    def step(self, action):
        self.episode_step += 1

        linear_vel = (action[0] + 1.0) / 2.0 * MAX_LINEAR_VEL
        angular_vel = action[1] * MAX_ANGULAR_VEL

        self.gazebo.publish_cmd_vel(linear_vel, angular_vel)
        self.gazebo.get_velodyne_data(wait_for_fresh=True)

        observation, distance, robot_state = self._get_observation()
        min_laser = min(self.gazebo.get_velodyne_data(wait_for_fresh=False))

        collision = min_laser < COLLISION_DIST
        print(f"STEP - min_laser: {min_laser:.3f}, collision: {collision}")
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

        self.gazebo.stop_robot()
        self.gazebo.reset_world()
        time.sleep(0.2)

        robot_x, robot_y, robot_yaw = self._get_random_robot_pose()
        self.gazebo.set_entity_pose('hunter_se', robot_x, robot_y, 0.31, yaw=robot_yaw)

        self.goal_x, self.goal_y = self._get_random_goal_pose(robot_x, robot_y)
        self._randomize_obstacles(robot_x, robot_y)
        self.gazebo.publish_goal_marker(self.goal_x, self.goal_y)

        self.gazebo.unpause_simulation()
        self.gazebo.get_velodyne_data(wait_for_fresh=True)

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
                x = np.random.uniform(-8.5, 8.5)
                y = np.random.uniform(-8.5, 8.5)
                dist_robot = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
                dist_goal = math.sqrt((x - self.goal_x)**2 + (y - self.goal_y)**2)
                if dist_robot > 1.5 and dist_goal > 1.0:
                    if all(math.sqrt((x - ox)**2 + (y - oy)**2) >= 1.0
                           for ox, oy in self.static_obstacles):
                        self.gazebo.set_entity_pose(f'movable_box_{i+1}', x, y, 0.25)
                        break

    def close(self):
        self.gazebo.stop_robot()
        self.gazebo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()