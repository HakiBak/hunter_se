pkill -9 -f gazebo && pkill -9 -f gz && pkill -9 -f ros && pkill -9 -f train_td3 && pkill -9 -f python3

pkill -9 -f gazebo
pkill -9 -f "gz sim"
pkill -9 -f ros2
pkill -9 -f train_td3


Step 1: Rebuild

cd /home/hakim/ros2_galaxy/similar_bot

colcon build --packages-select scout_rl

source /opt/ros/jazzy/setup.bash
source install/setup.bash

Step 2: Launch Gazebo (Terminal 1)

Option A - Using the training launch file:

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
ros2 launch scout_rl training.launch.py

Option B - Manual world launch:

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py

Step 3: Launch Monitor (Terminal 2)

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
python3 /tmp/lidar_monitor.py
Step 4: Launch Training (Terminal 3)

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
ros2 run scout_rl train_td3 --timesteps 500000 2>/dev/null


# TRAINING WITHOUT GUI TO SPEED UP THE TRAINING TIME

# Speed Optimization Steps
# Step 1: Modify the World File (increase RTF)
File: /home/hakim/ros2_galaxy/similar_bot/ugv_gazebo_sim/scout/scout_gazebo_sim/worlds/obstacles_easy.world

Replace lines 6-10:


<physics name='10ms' type='ignored'>
    <max_step_size>0.01</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>100</real_time_update_rate>
</physics>

With:

<physics name='fast_physics' type='ignored'>
    <max_step_size>0.004</max_step_size>
    <real_time_factor>4</real_time_factor>
    <real_time_update_rate>250</real_time_update_rate>
</physics>

# Step 2: Adjust TIME_DELTA
File: /home/hakim/ros2_galaxy/similar_bot/src/scout_rl/scout_rl/scout_env.py

Change line 18 from:

TIME_DELTA = 0.1
To:
TIME_DELTA = 0.025

# Step 3: Reduce Service Timeouts
File: /home/hakim/ros2_galaxy/similar_bot/src/scout_rl/scout_rl/gazebo_interface.py

Line 161: timeout_sec=1.0 → timeout_sec=0.2
Line 169: timeout_sec=1.0 → timeout_sec=0.2
Line 177: timeout_sec=2.0 → timeout_sec=0.2

# Step 4: Reduce Reset Sleeps
File: /home/hakim/ros2_galaxy/similar_bot/src/scout_rl/scout_rl/scout_env.py

Line 130: time.sleep(0.5) → time.sleep(0.2)
Line 141: time.sleep(0.3) → time.sleep(0.15)

# Step 5: Enable Headless Mode (optional)
File: /home/hakim/ros2_galaxy/similar_bot/ugv_gazebo_sim/scout/scout_gazebo_sim/launch/scout_mini_empty_world.launch.py

Change line 74 from:


"gz_args": ["-v 4 -r ", world],
To:


"gz_args": ["-v 4 -r -s ", world],


# Step 6: Rebuild
cd /home/hakim/ros2_galaxy/similar_bot
colcon build --packages-select scout_rl scout_gazebo_sim

# Step 7: Launch Training
# Terminal 1:

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py world_name:=obstacles_easy.world use_rviz:=false

# Terminal 2:

cd /home/hakim/ros2_galaxy/similar_bot
source install/setup.bash
ros2 run scout_rl train_td3 --timesteps 500000