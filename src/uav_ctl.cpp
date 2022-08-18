#include "uav_ctl.hpp"


// How to call this method with a param? 
UavCtl::UavCtl(): Node("uav_ctl")
{
 
    // Initalize 
    init(); 

    init_ctl(); 

    roll = 0.0; 
    pitch = 0.0; 
    //yaw = 0.0;
    nodeInitialized = true; 

}

void UavCtl::init()
{   

    // Take node namespace as uav_name (easiest way to capture ns param)
    ns_ = this->get_namespace(); 	

    // Publishers 
    cmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>(ns_ + std::string("/cmd_vel"), 1); 
    poseGtPub_             = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1); 
    gripperCmdPosLeftPub_  = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_left/cmd_pos"), 1); 
    gripperCmdPosRightPub_ = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_right/cmd_pos"), 1); 
    gripperCmdSuctionPub_  = this->create_publisher<std_msgs::msg::Bool>(ns_ + std::string("/gripper/suction_on"), 1); 
    
    // Subscribers
    poseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>(ns_ + std::string("/pose_static"), 1, std::bind(&UavCtl::pose_callback, this, _1));
    currPoseSub_           = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1, std::bind(&UavCtl::curr_pose_callback, this, _1)); 
    cmdPoseSub_            = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_ref"), 1, std::bind(&UavCtl::cmd_pose_callback, this, _1)); 

    // Services
    openGripperSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper"), std::bind(&UavCtl::open_gripper, this, _1, _2)); 
    closeGripperSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"),  std::bind(&UavCtl::close_gripper, this, _1, _2)); 
    startSuctionSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/start_suction"), std::bind(&UavCtl::start_suction, this, _1, _2)); 
    stopSuctionSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/stop_suction"), std::bind(&UavCtl::stop_suction, this, _1, _2)); 

    // Initial position
    cmdPose_.pose.position.x = 0.0; 
    cmdPose_.pose.position.y = 0.0; 
    cmdPose_.pose.position.z = 0.5; 

    // Initial orientation
    cmdPose_.pose.orientation.x = 0.0; cmdPose_.pose.orientation.y = 0.0; 
    cmdPose_.pose.orientation.z = 0.0; cmdPose_.pose.orientation.w = 1.0; 

    // possible to use milliseconds and duration
    std::chrono::duration<double> SYSTEM_DT(0.1);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&UavCtl::timer_callback, this)); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!");

}

void UavCtl::init_ctl()
{

    // Controller -> set pid with (kp, ki, kd) gains    
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up controllers!");
    
    // Height controller
    pid.kp = 1.0; pid.ki = 0.0; pid.kd = 0.0; 
    z_controller_.set_pid(std::move(pid)); 
    z_controller_.set_plant_state(0);

    // Horizontal - x controller
    pid.kp = 0.5; pid.ki = 0.0; pid.kd = 0.0; 
    x_controller_.set_pid(std::move(pid)); 
    x_controller_.set_plant_state(0);

    // Horizontal - y controller 
    pid.kp = 0.5; pid.ki = 0.0; pid.kd = 0.0;  
    y_controller_.set_pid(std::move(pid)); 
    y_controller_.set_plant_state(0);  

    /*
    Gains 
    ----------
    How to configure gains for normal flight? 
    Small I and small D lead reference to instability? 
    Which PID to use? 
    */
    // yaw controller; 

}

void UavCtl::curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{   
    // TODO: Fix this part!
    currPose_.header.frame_id = msg->header.frame_id;
    currPose_.pose.position.x = msg->pose.position.x;  
    currPose_.pose.position.y = msg->pose.position.y;
    currPose_.pose.position.z = msg->pose.position.z;

    tf2::Quaternion q(currPose_.pose.orientation.x, 
                      currPose_.pose.orientation.y, 
                      currPose_.pose.orientation.z, 
                      currPose_.pose.orientation.w); 
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);

    // Output yaw (needed for pose estimation)
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current yaw is: " << yaw); 


}

void UavCtl::cmd_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{   
    // TODO: Fix this part!
    cmdPose_.header.frame_id = msg->header.frame_id; 
    cmdPose_.pose.position.x = msg->pose.position.x; 
    cmdPose_.pose.position.y = msg->pose.position.y; 
    cmdPose_.pose.position.z = msg->pose.position.z; 
    cmdReciv = true; 

}

void UavCtl::pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) 
{       

        geometry_msgs::msg::TransformStamped transform_stamped;
        bool pose_estimate = false; 
        bool check_complete = false; 
        // Get uav ns
        std::string uav_ns = this->ns_;
        // Remove backslash 
        uav_ns.erase(0, 1); 

        // TODO: Bear in mind that world_name is variable in local scope, should be passed as arg
        std::string world_name = "empty_platform";

        for (int i = 0; i < static_cast<int>(std::size(msg->transforms)); ++i) {
            // https://www.theconstructsim.com/ros-qa-045-publish-subscribe-array-vector-message/
            geometry_msgs::msg::TransformStamped transform_msg; 
            transform_msg = msg->transforms.at(i); 

            // tf's
            std::string frame_id; std::string child_frame_id;  
            frame_id        = transform_msg.header.frame_id; 
            child_frame_id  = transform_msg.child_frame_id; 

            // publish pose_gt for this uav
            if (frame_id == world_name && child_frame_id == uav_ns) 
            {   
                pose_estimate = true; 
                geometry_msgs::msg::PoseStamped msg; 

                // This can also be ground truth pose
                msg.header = transform_msg.header; 
                msg.header.frame_id = child_frame_id; 
                msg.pose.position.x = transform_msg.transform.translation.x; 
                msg.pose.position.y = transform_msg.transform.translation.y; 
                msg.pose.position.z = transform_msg.transform.translation.z; 
                msg.pose.orientation = transform_msg.transform.rotation; 

                poseGtPub_->publish(msg); 
                break; 

            }; 

            check_complete = true; 

        }
        // publish warning if there's no pose estimate
        if(!pose_estimate && check_complete){
            RCLCPP_WARN(this->get_logger(), "No pose estimate found for %s.",  uav_ns.c_str()); 
        }

}

bool UavCtl::close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                              std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.0; 

    gripperCmdPosLeftPub_->publish(finger_pos_msg); 
    gripperCmdPosRightPub_->publish(finger_pos_msg); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

    return true; 
}
bool UavCtl::open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                            std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.5; 

    gripperCmdPosLeftPub_->publish(finger_pos_msg); 
    gripperCmdPosRightPub_->publish(finger_pos_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Opening gripper!");

    return true;  
        
}

bool UavCtl::start_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res)
{
    bool start_suction = true; 

    std_msgs::msg::Bool msg; 
    msg.data = start_suction; 

    gripperCmdSuctionPub_->publish(msg);
    
}

bool UavCtl::stop_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                          std_srvs::srv::Empty::Response::SharedPtr res)
{
    bool start_suction = false; 

    std_msgs::msg::Bool msg; 
    msg.data = start_suction; 

    gripperCmdSuctionPub_->publish(msg); 


}
// Timer callback executes every 1.0/0.2 (5s)
void UavCtl::timer_callback()
{
    // TODO: Add control here for PID control :) 
    // get current uav_state
    double cmd_x; double cmd_y; double cmd_z; double cmd_yaw; 

    // this PID is used only for controlling z axis
    if (nodeInitialized && cmdReciv)
    {
        // send commands only if command is recieved
        //RCLCPP_INFO_STREAM(this->get_logger(), "Command recieved: "<< cmdPose_.pose.position.z); 

        // feedback loop --> should be included in PID
        float err_x; float err_y; float err_z; float err_yaw; 
        err_z = cmdPose_.pose.position.z - currPose_.pose.position.z; 
        // TODO: Add yaw to calculation
        err_x = currPose_.pose.position.x - cmdPose_.pose.position.x; 
        err_y = currPose_.pose.position.y - cmdPose_.pose.position.y; 
        
        // threading issue --> add to methods maybe :) 
        x_controller_.set_plant_state(currPose_.pose.position.x); 
        x_controller_.set_setpoint(cmdPose_.pose.position.x); 
        x_controller_.update(); 
        cmd_x = x_controller_.get_control_effort(); 

        y_controller_.set_plant_state(currPose_.pose.position.y); 
        y_controller_.set_setpoint(cmdPose_.pose.position.y);
        y_controller_.update();
        cmd_y = y_controller_.get_control_effort(); 

        z_controller_.set_plant_state(currPose_.pose.position.z); 
        z_controller_.set_setpoint(cmdPose_.pose.position.z);  
        z_controller_.update();
        cmd_z = z_controller_.get_control_effort(); 

        //RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << cmdPose_.pose.position.z); 
        //RCLCPP_INFO_STREAM(this->get_logger(), "curr z: " << currPose_.pose.position.z); 
        //RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << z_controller_.get_control_effort()); 
        
    
        geometry_msgs::msg::Twist cmdVel_;
        cmdVel_.linear.x = cmd_x; 
        cmdVel_.linear.y = cmd_y; 
        cmdVel_.linear.z = cmd_z; 

        cmdVel_.angular.x = 0; 
        cmdVel_.angular.y = 0; 
        cmdVel_.angular.z = 0.0;  

        cmdVelPub_->publish(cmdVel_); 
    
    }

}   






