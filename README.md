# Hunter SE — TD3 Reinforcement Learning Navigation

Goal-directed, obstacle-avoiding navigation for the **Hunter SE** Ackermann robot,
trained with **TD3** (Stable-Baselines3) in **Gazebo Harmonic** on **ROS 2 Jazzy**.

The learned policy maps a 24-dimensional observation (20 lidar distance bins + goal
distance, heading error, and current linear/angular velocity) to a 2-dimensional
velocity action. It is trained in simulation and intended for later transfer to the
physical Hunter SE robot.

---

## Repository layout

```
.
├── src/hunter_se_rl/                     # RL package (ament_python)
│   └── hunter_se_rl/
│       ├── hunter_se_env.py              # Gymnasium environment (HunterSEEnv)
│       ├── gazebo_interface.py           # ROS 2 <-> Gazebo bridge (lidar, odom, cmd_vel, reset)
│       ├── utils.py                      # point cloud -> 20 lidar bins, quaternion helpers
│       ├── train_td3.py                  # training entry point
│       └── test_policy.py                # evaluation entry point
│   ├── launch/training.launch.py
│   └── config/training_config.yaml
│
├── ugv_gazebo_sim/hunter_se/
│   ├── hunter_se_description/            # URDF/xacro, meshes, 3D lidar, RViz config
│   └── hunter_se_gazebo_sim/            # sim launch, ros_gz bridge, obstacles_easy.world
│
└── models/
    ├── td3_final.zip                     # final trained policy
    └── best_model.zip                    # best policy by evaluation reward
```

---

## Prerequisites

- **ROS 2 Jazzy** — sourced from `/opt/ros/jazzy`
- **Gazebo Harmonic** with `ros_gz_sim` and `ros_gz_bridge`
- **RViz2** (optional, for visualization)
- Python 3.12 with: `stable_baselines3`, `torch`, `gymnasium`, `numpy`

Install the Python dependencies (system interpreter):

```bash
python3 -m pip install "stable-baselines3[extra]" gymnasium numpy
```

---

## Build

```bash
# from the workspace root (the folder containing this README)
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

To build only the packages used here:

```bash
colcon build --packages-select hunter_se_rl hunter_se_description hunter_se_gazebo_sim
```

---

## Quick start: run & test the trained policy

Use **two terminals**. In each one, first set up the environment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Terminal 1 — launch the simulation** (Gazebo + robot + ros_gz bridge; RViz on by default):

```bash
ros2 launch hunter_se_gazebo_sim hunter_se_simulation.launch.py
# headless (no RViz):
ros2 launch hunter_se_gazebo_sim hunter_se_simulation.launch.py use_rviz:=false
```

Wait until the robot has spawned and lidar is publishing.

**Terminal 2 — evaluate the policy:**

```bash
ros2 run hunter_se_rl test_policy --model ./models/td3_final --episodes 10
# or evaluate the best checkpoint saved during training:
ros2 run hunter_se_rl test_policy --model ./models/best_model --episodes 10
```

Each episode randomizes the start pose, goal, and obstacles, then drives the policy
until it reaches the goal, collides, or times out. A summary of success / collision /
timeout rates and average reward prints at the end.

---

## Train a new model

Training reuses the same simulation, so the setup mirrors testing: launch the sim in one
terminal, run the trainer in another. Source the environment in **both** first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**Terminal 1 — launch the simulation** (headless, no RViz, for speed):

```bash
ros2 launch hunter_se_gazebo_sim hunter_se_simulation.launch.py use_rviz:=false
```

**Terminal 2 — start training:**

```bash
ros2 run hunter_se_rl train_td3 --timesteps 1000000 --save-dir ./models --log-dir ./logs
```

| Flag | Meaning |
|---|---|
| `--timesteps` | total environment steps to train for |
| `--save-dir` | where checkpoints and final/best models are written |
| `--log-dir` | where TensorBoard logs are written |
| `--load-model ./models/td3_final` | *(optional)* resume from an existing model instead of starting fresh |

**Outputs** (written to `--save-dir` / `--log-dir`):

| Output | File / Location |
|---|---|
| Periodic checkpoints (every 10k steps) | `models/td3_hunter_se_*_steps.zip` |
| Best model (highest evaluation reward) | `models/best_model.zip` |
| Final model (end of training) | `models/td3_final.zip` |


**Algorithm:** TD3 with `MlpPolicy` `[400, 300]`, learning rate `3e-4`, replay buffer
`100k`, batch size `256`, discount `γ = 0.99`.

> **Faster training:** the simulation runs in real time by default. To run physics
> faster than real time, edit
> [`obstacles_easy.world`](ugv_gazebo_sim/hunter_se/hunter_se_gazebo_sim/worlds/obstacles_easy.world):
> comment out the `realtime` physics block and uncomment the `fast_physics` block
> (`real_time_factor = 8`), then rebuild `hunter_se_gazebo_sim`.

---

**Leftover Gazebo / ROS processes between runs:**

```bash
pkill -9 -f gazebo; pkill -9 -f "gz sim"; pkill -9 -f ros2; pkill -9 -f train_td3
```
---

## License

MIT — see package manifests for details.
