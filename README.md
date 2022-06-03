## Aerial manipulation MBZIRC 2022 repo

ROS2 aerial manipulation MBZIRC 2022 repo. 

### Launch commands: 

Send `cmd_vel` command to `UAV`: 

```
ros2 topic pub /am_L/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.01, z: 0.01}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Run uav_joy_ctl node

Run `uav_joy_ctl` node with following command: 
```
ros2 run mbzirc_aerial_manipulation uav_joy_ctl
```

### Use teleop twist key: 

Use existing `teleop-twist-keyboard` package to send messages to `cmd_vel` topic. 

Run `teleop-twist-keyboard` with: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### Docs: 

Official mbzirc [github repo](https://github.com/osrf/mbzirc)

Official mbzirc [wiki](https://github.com/osrf/mbzirc/wiki)

ROS2 [wiki](https://docs.ros.org/en/foxy/index.html)

Book about [ROS2](https://osrf.github.io/ros2multirobotbook/)

TurtleSim class as a good reference for [dev](https://github.com/ros/ros_tutorials/blob/galactic-devel/turtlesim/src/turtle.cpp) 

### Battery stuff: 

Use parameter `flightTime` to increase battery life!

### Plugins that are used and that cause trouble 

Plugins that causes trouble during some development is game development plugin. 

### Temp: 

PCL build: https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html
VTK build: https://github.com/Kitware/VTK/blob/master/Documentation/dev/build.md#linux-getting-started

### Simulation models

Simulation models that can be use can be found [here](https://app.gazebosim.org/OpenRobotics/fuel/collections/mbzirc). 

### TODO: 

- [x] Add launch for launch UAVs with manipulators 
- [x] Add teleop twist keyboard to enable basic UAV ctl
- [x] Add open/close gripper service 
- [x] Add joystick control to enable flying with aerial manipulators
- [x] Add objects to the world to enable manipulation with those objects
- [x] Try to manipulate object 
- [ ] Add ROS1 bridge 
- [x] Check how to update battery life 
- [x] Check camera RGBD output
- [ ] Check detach topic that stems from `<plugin filename="ignition-gazebo-detachable-joint-system" name="ignition::gazebo::systems::DetachableJoint">`
- [x] Pose estimation/Sensor positioning (maybe logical to be on bottom of UAV?) - test different scenarios 
