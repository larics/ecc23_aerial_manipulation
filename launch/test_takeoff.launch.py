
# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import mbzirc_ign.bridges

import os

def launch(context, *args, **kwargs):

    world_name = 'simple_demo'

    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_ign_gazebo'), 'launch'),
        '/ign_gazebo.launch.py']),
        launch_arguments = {'ign_args': f"-v 50 -r {world_name}.sdf"}.items()) 

    #spawn_small_aerial_manipulator
    spawn_small_aerial_manipulator1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mbzirc_ign'), 'launch'), 
        '/spawn.launch.py']), 
        launch_arguments = {
                'name': 'uav1', 
                'world': f'{world_name}', 
                'model': 'mbzirc_quadrotor',
                'x': '0', 
                'y': '-1', 
                'z': '1.0', 
                'R': '0', 
                'P': '0', 
                'Y': '0', 
                'gripper':'mbzirc_suction_gripper', 
                #'slot0':'mbzirc_hd_camera', 
                #'slot0' : 'mbzirc_rgbd_camera',
                'type':'uav2', 
                'flightTime': '6000', 
                #'capacity': '100.0' #'flightTime':'1200'     # This is probably a parameter to enable flightTime duration (battery)
                }.items())

    # Control node for small UAV (example)
    uav1_ctl_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_ctl", 
        namespace="uav1", # Add node namespace for UAV ctl 
        output="screen",
        parameters=[{'world_name': f'{world_name}'}],
    )

    # Add spawning of UAVs
    return [ign_gazebo,
            spawn_small_aerial_manipulator1,
            uav1_ctl_node, 
            ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ign_args', default_value='',
            description='Arguments to be passed to Ignition Gazebo'),
        OpaqueFunction(function = launch)
        ])
