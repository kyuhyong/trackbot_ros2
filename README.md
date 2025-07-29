# trackbot_ros2

$ colcon build --packages-select trackbot_gazebo --symlink-install


Task	Command
Check model spawned:	gz service -l
Check Ignition topics:	gz topic -l
Start bridge:   	    ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
Publish Twist cmd:	    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}'
