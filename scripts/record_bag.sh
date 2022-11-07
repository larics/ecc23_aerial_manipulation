#!/bin/bash

ros2 bag record /uav1/pose_gt \
                /uav1/pose_ref \
                /uav1/pos_ref \
                /uav1/odometry \
                /uav1/imu/data \
                /uav1/comp_val \
                /uav1/comp_factor \
                /uav1/comp_est_vel \
                /uav1/cmd_vel \
                /uav1/vel_gt \
                /uav1/drop_off_point \
                /uav1/state_debug \
                /uav1/arm/drone_detection/detected_point \
                /usv/uav1/arm/drone_detection/vessel_detected_point \
                /usv/uav1/arm/drone_detection/detected_point \
                /uav1/arm/drone_detection/vessel_detected_point \
                /uav1/arm/drone_detection/detected_point \

# Not recognized msg type from bag, remove it from bag :) 
#/uav1/tx \
#/usv/tx \
#/uav1/rx \
#/usv/rx

