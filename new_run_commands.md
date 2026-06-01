pkill -9 -f gazebo && pkill -9 -f gz && pkill -9 -f ros && pkill -9 -f train_td3 && pkill -9 -f python3

source /opt/ros/jazzy/setup.bash

Rebuild

cd /home/hakim/ros2_galaxy/similar_bot_check_point_20260215

rm -rf build/scout_gazebo_sim install/scout_gazebo_sim

colcon build --packages-select scout_gazebo_sim

source install/setup.bash

OR Rubuild
colcon build --packages-select scout_rl

T1

cd /home/hakim/ros2_galaxy/similar_bot_check_point_20260215

source install/setup.bash

ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py use_rviz:=true

T2

cd /home/hakim/ros2_galaxy/similar_bot_check_point_20260215
source install/setup.bash
ros2 run scout_rl train_td3 --timesteps 500000 --save-dir ./models --log-dir ./logs


ros2 run scout_rl test_policy --model ./models/td3_final --episodes 10

