
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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