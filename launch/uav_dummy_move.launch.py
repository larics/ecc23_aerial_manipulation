import os
from venv import create
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    main_config_path = os.path.join(
        get_package_share_directory('mbzirc_fer'), 
        'config',
        'dry_run'
        )

    ns = LaunchConfiguration('ns_arg')
    ns_arg = DeclareLaunchArgument('ns_arg')


    ###############
    # Communication
    ###############
    uav_takeoff_action = create_takeoff_action(ns, 10)
    uav_takeoff = create_timed_description(uav_takeoff_action, 10)
    uav_move_action = create_move_action(ns, 5)
    # TODO: Try to move action 
    uav_move = create_timed_description(uav_move_action, 15)
    uav_stand_action = create_stand_action(ns, 10)
    uav_stand = create_timed_description(uav_stand_action, 25)

    
    launch_list = []
    launch_list.append(ns_arg)
    launch_list.extend(uav_takeoff)
    launch_list.extend(uav_move)
    launch_list.extend(uav_stand)
    # launch_list.extend(estimation)

    return LaunchDescription(launch_list)


def create_takeoff_action(uav_ns, duration):

    action =  ExecuteProcess(
        name="pub_twist_cmd", 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', uav_ns, '/cmd_vel', 'geometry_msgs/msg/Twist',
                '"{linear: {x: 0.0,y: 0.00, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
        output="screen", 
        shell=True)
        
    return action

def create_move_action(uav_ns, duration, x_cmd, y_cmd, z_cmd): 

    action =  ExecuteProcess(
        name="pub_twist_cmd", 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', uav_ns, '/cmd_vel', 'geometry_msgs/msg/Twist',
            '"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
        output="screen", 
        shell=True)
        
    return action

def create_stand_action(uav_ns, duration): 
    
    action =  ExecuteProcess(
        name ="pub_twist_cmd", 
        # Added -r as rate for topic publishing and -t as how many messages is send 
        cmd = ['timeout {}s'.format(duration), 'ros2', 'topic', 'pub', uav_ns, '/cmd_vel', 'geometry_msgs/msg/Twist',
            '"{linear: {x: 0.0,y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
        output ="screen", 
        shell =True)
        
    return action

def create_timed_description(action, start_time): 

    launch_desc = LaunchDescription([TimerAction(
                    period = start_time,
                    actions = [action])])

    return launch_desc

