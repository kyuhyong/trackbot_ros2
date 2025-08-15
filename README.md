# trackbot_ros2

This package is to simulate a **track type vehicle** in gazebo simulation environment.

Since there is no realistic track model in gazeob, tracks are modeled as multiple bogies with prismatic suspensions under the main body.

Instead of driving the main drive wheel, each bogies are working as a drive wheel.

## Support

This package support and tested
- Ubuntu 22.04
- ROS2 Humble
- ignition gazebo 6.17.0 installed with following guide https://gazebosim.org/docs/fortress/install_ubuntu/#binary-installation-on-ubuntu 

# Install pacakge

```
$ colcon build --packages-select trackbot_gazebo trackbot_controller --symlink-install
$ source install/setup.bash
```

# Run 

## Gazebo simulation

To start trackbot in gazebo simulation
```
$ ros2 launch trackbot_gazebo gazebo_world.launch.py profile:=empty
```
## Teleopration
To launch a keyboard controller node
```
$ ros2 launch trackbot_controller teleop_keyboard.launch.py
```

## Extra tips
- Check model spawned:	`gz service -l`
- Check Ignition topics:	`gz topic -l`
- To kill all ign process:   `sudo pkill -f ign`
