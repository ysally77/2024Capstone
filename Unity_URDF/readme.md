
$ cd ~/Workspace/ros_ws
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch car_tutorial unity.launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard
