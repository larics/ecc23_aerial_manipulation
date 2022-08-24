
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
from launch.actions import ExecuteProcess, TimerAction
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
        launch_arguments = {'ign_args': f"-v 4 -r {world_name}.sdf"}.items()) 

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
                'z': '1.5', 
                'R': '0', 
                'P': '0', 
                'Y': '0', 
                'gripper':'mbzirc_suction_gripper_light', 
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
                'z': '1.5', 
                'R': '0', 
                'P': '0', 
                'Y': '0', 
                'gripper':'mbzirc_suction_gripper_light', 
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

    ## Movements
    duration_ = 5
    uav_takeoff_action = create_takeoff_action("uav1", duration_); 
    uav_send_to_pose_action = create_send_to_pose_action("uav1", duration_, x_pose=-2, y_pose=2, z_pose=3)
    uav_land1 = create_send_to_pose_action("uav1", duration_, x_pose=-1.90, y_pose=2, z_pose=2.75)
    uav_land2 = create_send_to_pose_action("uav1", duration_, x_pose=-1.90, y_pose=2, z_pose=2.5)
    uav_land3 = create_send_to_pose_action("uav1", duration_, x_pose=-1.90, y_pose=2, z_pose=2.3)
    uav_land4 = create_send_to_pose_action("uav1", duration_, x_pose=-1.90, y_pose=2, z_pose=1.0)
    uav_grasp = create_grasp_action("uav1"); 
    uav_fly = create_send_to_pose_action("uav1", duration_*5, x_pose=-1.95, y_pose=2, z_pose=2.5)
    # Possible to create grasp but hard to manipulate uav after that
    # uav_grasp = 
   
    start_time = 0
    uav_first_movement = create_timed_description(uav_takeoff_action, 0.0)
    uav_second_movement = create_timed_description(uav_send_to_pose_action, 5.0)
    uav_third_movement = create_timed_description(uav_land1, 10.0)
    uav_fourth_movement = create_timed_description(uav_land2, 15.0)
    uav_fifth_movement = create_timed_description(uav_land3, 20.0)
    uav_sixth_movement = create_timed_description(uav_land4, 25.0)
    uav_grasp = create_timed_description(uav_grasp, 30.0)
    uav_carry = create_timed_description(uav_fly, 35.0)

    # Add spawning of UAVs
    return [ign_gazebo,
            joy_node,  
            spawn_small_aerial_manipulator1,
            #spawn_small_aerial_manipulator2,  
            spawn_large_aerial_manipulator1,
            #spawn_large_aerial_manipulator2, 
            #ros2_ign_score_bridge, 
            #ros2_ign_run_clock_bridge, 
            #ros2_ign_phase_bridge,
            uav_joy_node, 
            uav1_ctl_node, 
            uav3_ctl_node, 
            #uav_first_movement, 
            #uav_second_movement, 
            #uav_third_movement, 
            #uav_fourth_movement, 
            #uav_fifth_movement, 
            #uav_sixth_movement, 
            #uav_grasp, 
            #uav_carry
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


def create_takeoff_action(uav_name, duration):

    action =  ExecuteProcess(
        name="pub_twist_cmd", 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', '/{}/cmd_vel'.format(uav_name), 'geometry_msgs/msg/Twist',
                '"{linear: {x: 0.0,y: 0.00, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
        output="screen", 
        shell=True)
        
    return action

def create_send_to_pose_action(uav_name, duration, x_pose=0.0, y_pose=0.0, z_pose=0.0): 

    position_str = "x: {}, y: {}, z: {}".format(x_pose, y_pose, z_pose)
    cmd_str = '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'uav1\'}, pose: {position: {%s}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"' %position_str

    action =  ExecuteProcess(
        name="pub_twist_cmd", 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', '-r 5', '/{}/pose_ref'.format(uav_name), 'geometry_msgs/msg/PoseStamped',
                cmd_str],#.format(x_cmd, y_cmd, z_cmd)],
        output="screen", 
        shell=True)
        
    return action

def create_move_action(uav_name, duration, x_cmd, y_cmd, z_cmd): 
    action =  ExecuteProcess(
        name="pub_twist_cmd", 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', '/{}/cmd_vel'.format(uav_name), 'geometry_msgs/msg/Twist',
            '"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],#.format(x_cmd, y_cmd, z_cmd)],
        output="screen", 
        shell=True)
        
    return action

def create_stand_action(uav_name, duration): 
    action =  ExecuteProcess(
        name ="pub_twist_cmd", 
        # Added -r as rate for topic publishing and -t as how many messages is send 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', '/{}/cmd_vel'.format(uav_name), 'geometry_msgs/msg/Twist',
            '"{linear: {x: 0.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
        output ="screen", 
        shell =True)
        
    return action

def create_grasp_action(uav_name): 
    
    action =  ExecuteProcess(
        name ="pub_grasp_cmd", 
        # Added -r as rate for topic publishing and -t as how many messages is send 
        cmd = ['ros2', 'topic', 'pub', '/{}/gripper/suction_on'.format(uav_name), 'std_msgs/msg/Bool',
            '"data: true"'],
        output ="screen", 
        shell =True)
        
    return action


def create_timed_description(action, start_time): 

    launch_desc = LaunchDescription([TimerAction(
                    period = start_time,
                    actions = [action])])

    return launch_desc