# Tricycle Steering Robot Simulation ROS2 Jazzy

### Launch Open World, Spawn Robot and cmd_vel_bridge:
    ros2 launch launch/all_node.launch.py

### Run Teleop Twist Keyboard:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

### Slam:
    ros2 launch antbot_ros2_control slam.launch.py

### Navigation:
    ros2 launch antbot_ros2_control navigation.launch.py

![Navigation](/images/navigation.gif?raw=true "Navigation")    