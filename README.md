## Aerial manipulation MBZIRC 2022 repo

ROS2 aerial manipulation MBZIRC 2022 repo. 

### Launch commands: 

Send `cmd_vel` command to `UAV`: 

```
ros2 topic pub /am_L/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0,y: 0.01, z: 0.01}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### Docs: 

Official mbzirc [github repo](https://github.com/osrf/mbzirc)

Official mbzirc [wiki](https://github.com/osrf/mbzirc/wiki)

ROS2[wiki](https://docs.ros.org/en/foxy/index.html) 


### TODO: 

- [ ] Add launch for launch UAVs with manipulators 
- [ ] Add joystick control to enable flying with aerial manipulators
- [ ] Add objects to the world to enable manipulation with those objects
- [ ] Try to manipulate object 
- [ ] Add ROS1 bridge 
- [ ] Check how to update battery life 

