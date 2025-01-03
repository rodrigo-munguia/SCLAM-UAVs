


# Launch first ROS 2 node in a new console
gnome-terminal -- bash -c ". install/setup.bash;ros2 launch webot_package robot_launch.py"

# Launch second ROS 2 node in another new console
gnome-terminal -- bash -c ". install/setup.bash;ros2 run keyboard keyboard_node"


. install/setup.bash
ros2 launch launch/quad_slam2.launch.py