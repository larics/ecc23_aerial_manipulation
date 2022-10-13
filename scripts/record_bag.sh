#!/bin/bash

ros2 bag record /uav1/pose_gt \
                /uav1/pose_ref \
                /uav1/odometry \
                /uav1/imu/data \
                /uav1/compensation_x \
                /uav1/compensation_y \
                /uav1/compensation_z \
                /uav1/cmd_vel \
                /uav1/vel_gt \
                /uav1/compensation_factor_xy \
                /uav1/compensation_factor_z \
                /uav1/arm/drone_detection/detected_point \
#                /uav1/arm/drone_detection/vessel_detected_point 
