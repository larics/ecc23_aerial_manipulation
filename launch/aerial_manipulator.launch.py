
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

    world_name = 'empty_platform'

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
                'z': '0.5', 
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

    #spawn_small_aerial_manipulator
    spawn_small_aerial_manipulator2= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mbzirc_ign'), 'launch'), 
        '/spawn.launch.py']), 
        launch_arguments = {
                'name': 'uav2', 
                'world': f'{world_name}', 
                'model': 'mbzirc_quadrotor',
                'x': '0', 
                'y': '-4', 
                'z': '0.5', 
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

    spawn_large_aerial_manipulator1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mbzirc_ign'), 'launch'), 
        '/spawn.launch.py']),
        launch_arguments = {
                'name' : 'uav3', 
                'world': f'{world_name}', 
                'model': 'mbzirc_hexrotor', 
                'x': '-2.5', 
                'y': '-1.9',
                'z': '2.0', 
                'R': '0', 
                'P': '0', 
                'Y': '0', 
                'type':'uav1',
                #'slot0':'mbzirc_vga_camera', 
                #'slot3': 'mbzirc_rgbd_camera', 
                'gripper':'mbzirc_oberon7_gripper', 
                'flightTime' : '6000' #'flightTime':'1020'
                }.items())


    spawn_large_aerial_manipulator2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('mbzirc_ign'), 'launch'), 
        '/spawn.launch.py']),
        launch_arguments = {
                'name' : 'uav4', 
                'world': f'{world_name}', 
                'model': 'mbzirc_hexrotor', 
                'x': '-5.0', 
                'y': '-1.9',
                'z': '2.0', 
                'R': '0', 
                'P': '0', 
                'Y': '0', 
                'type':'uav1',
                #'slot0':'mbzirc_vga_camera', 
                #'slot3': 'mbzirc_rgbd_camera', 
                'gripper':'mbzirc_oberon7_gripper', 
                'flightTime' : '6000' #'flightTime':'1020'
                }.items())
    
    # https://index.ros.org/p/joy/ --> joy node as joystick (Create subscriber that takes cmd_vel)
    joy_node = Node(
        package='joy', 
        executable="joy_node", 
        output="screen", 
        arguments={'device_name':'js0'}.items()
    )

    ros2_ign_score_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/score@std_msgs/msg/Float32@ignition.msgs.Float'],
    )

    ros2_ign_run_clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/run_clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'],
    )

    ros2_ign_phase_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/mbzirc/phase@std_msgs/msg/String@ignition.msgs.StringMsg'],
    )

    # Control node for small UAV (example)
    uav1_ctl_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_ctl", 
        namespace="uav1", # Add node namespace for UAV ctl 
        output="screen",
        parameters=[{'world_name': f'{world_name}'}],
    )

    # Control node for large UAV (example)
    uav2_ctl_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_ctl", 
        namespace="uav2", 
        output="screen",
        parameters=[{'world_name': f'{world_name}'}],
    )

    # Control node for large UAV (example)
    uav3_ctl_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_ctl", 
        namespace="uav3", 
        output="screen",
        parameters=[{'world_name': f'{world_name}'}],
    )

    # Control node for large UAV (example)
    uav4_ctl_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_ctl", 
        namespace="uav4", 
        output="screen",
        parameters=[{'world_name': f'{world_name}'}],
    )

    uav_joy_node = Node(
        package="mbzirc_aerial_manipulation", 
        executable="uav_joy", 
        output="screen" 
    )

    # Add spawning of UAVs
    return [ign_gazebo,
            joy_node,  
            spawn_small_aerial_manipulator1,
            #spawn_small_aerial_manipulator2,  
            #spawn_large_aerial_manipulator1,
            #spawn_large_aerial_manipulator2, 
            #ros2_ign_score_bridge, 
            #ros2_ign_run_clock_bridge, 
            #ros2_ign_phase_bridge,
            uav_joy_node, 
            uav1_ctl_node, 
            #uav2_ctl_node,
            #uav3_ctl_node, 
            #uav4_ctl_node, 
            ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ign_args', default_value='',
            description='Arguments to be passed to Ignition Gazebo'),
        OpaqueFunction(function = launch)
        ])
