# Development


maintainer: `filip.zoric@fer.hr`. 

purpose: `Development report of mbzirc_aerial_manipulation github.com` 

dependend packages: [mbzirc_ros](git@github.com:fzoric8/mbzirc.git) 

done: 
 - [x] Integrated joystick to control small(am_S) and large UAV(am_L) 
 - [x] Created simple world to test mbzirc aerial manipulation 
 - [x] Created simple modes of control for slower and faster object approach 
 - [x] Added gripper services for turning gripper on and off 
 - [x] Tested lifting of small objects of interest 

todo: 
 - [x] Add upstream of mbzirc repo 
 - [ ] Estimate object position with ICP --> postponed
 - [ ] Create ICP ROS2 package  --> postponed  
 - [ ] Test output of ICP and create simple aerial servoing to navigate UAV to the obstacle (simple, yet efficient) --> postponed
 - [ ] Think of states for lifting object 
 - [ ] Think of collaborative object lifting 
 - [ ] Add namespacing 
 - [ ] Add pose estimation
 - [ ] Add service to change current joy command
 - [ ] Create joy node to enable control of multiple UAVs 
 
conclusions: 
 - [x] RGBD camera kill sim factor (99% - 20%)
