
#include "uav_ctl.hpp"


// How to call this method with a param? 
UavCtl::UavCtl(): Node("uav_ctl")
{
 
    // Initalize 
    init(); 
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

    // tf buffer
    //amSTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    // tf listener
    //amSTransformListener = std::make_shared<tf2_ros::TransformListener>(*amSTfBuffer);

    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up controller!");
    // set pid with (kp, ki, kd) gains    
    pid.kp = 5; pid.ki = 0; pid.kd = 0; controller_.set_pid(std::move(pid)); 
    controller_.set_plant_state(0); 
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!");

    nodeInitialized = true; 
    // possible to use milliseconds and duration
    std::chrono::duration<double> SYSTEM_DT(0.1);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&UavCtl::timer_callback, this)); 

}

void UavCtl::curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{   
    // TODO: Fix this part!
    //currentPose_->header.frame_id = msg->header.frame_id; 
    //currentPose_->pose.position.z = msg->pose.position.z; 
    int a; 
}

void UavCtl::cmd_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) const
{   
    // TODO: Fix this part!
    //wantedPose_->header.frame_id = msg->header.frame_id; 
    //wantedPose_->pose.position.z = msg->pose.position.z; 
    int b; 

}

void UavCtl::pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
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

                msg.header = transform_msg.header; 
                msg.header.frame_id = child_frame_id; 
                msg.pose.position.x = transform_msg.transform.translation.x; 
                msg.pose.position.y = transform_msg.transform.translation.y; 
                msg.pose.position.z = transform_msg.transform.translation.z; 
                msg.pose.orientation = transform_msg.transform.rotation; 

                poseGtPub_->publish(msg); 
                check_complete = true; 
                break; 

            }; 
        }
        
        /*
        // publish warning if there's no pose estimate
        if(!pose_estimate && check_complete){
            RCLCPP_WARN(this->get_logger(), "No pose estimate found for %s.",  uav_ns.c_str()); 
        }*/

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

    // this PID is used only for controlling z axis
    controller_.set_setpoint(100); 
    controller_.update();
    double state; 
    state = controller_.get_control_effort();

    
    RCLCPP_INFO_STREAM(this->get_logger(), "State is: " << state); 
    //RCLCPP_INFO_STREAM(this->get_logger(), "Timer works!");


}   






