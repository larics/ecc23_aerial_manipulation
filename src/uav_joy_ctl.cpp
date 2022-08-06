
#include "uav_joy_ctl.hpp"


// How to call this method with a param? 
UavJoyCtl::UavJoyCtl(): Node("uav_joy_ctl")
{
 
    // Initalize 
    init(); 
}

void UavJoyCtl::init()
{   


    std::string ns_ = this->get_namespace(); 	
    // Publishers 
    amLCmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>(ns_ + std::string("/cmd_vel"), 1); 
    
    
    amLGripperCmdPosLeftPub_  = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_left/cmd_pos"), 1); 
    amLGripperCmdPosRightPub_ = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_right/cmd_pos"), 1); 
    amSGripperCmdSuctionPub_  = this->create_publisher<std_msgs::msg::Bool>("/am_S/gripper/suction_on", 1); 
            
    // Subscribers
    joySub_                   = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&UavJoyCtl::joy_callback, this, _1)); 
    teleopSub_                = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&UavJoyCtl::teleop_callback, this, _1)); 
    amLPoseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>("/am_L/pose_static", 1, std::bind(&UavJoyCtl::amL_pose_callback, this, _1)); 
    amSPoseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>("/am_S/pose_static", 1, std::bind(&UavJoyCtl::amS_pose_callback, this, _1)); 

    // Services
    openGripperSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper"), std::bind(&UavJoyCtl::open_gripper, this, _1, _2)); 
    closeGripperSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"),  std::bind(&UavJoyCtl::close_gripper, this, _1, _2)); 

    // Clients 
    openGripperClient_        = this->create_client<std_srvs::srv::Empty>(ns_ + std::string("open_gripper")); 
    closeGripperClient_       = this->create_client<std_srvs::srv::Empty>(ns_ + std::string("close_gripper"));

    // tf buffer
    amSTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    amLTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 

    // tf listener
    amSTransformListener = std::make_shared<tf2_ros::TransformListener>(*amSTfBuffer);
    amLTransformListener= std::make_shared<tf2_ros::TransformListener>(*amLTfBuffer); 

    // std::chrono::duration<double> SYSTEM_DT(0.2);
    // timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&UavJoyCtl::timer_callback, this)); 

    //startSuctionService_ = this->create_service<std_srvs::srv::Triger>("/am_S/suction")
    //TODO: Decouple large and small UAV --> define basic methods  

}

// Timer callback executes every 1.0/0.2 (5s)
void UavJoyCtl::timer_callback()
{
    auto message = std_msgs::msg::String(); 
    message.data = "Hello, world! " + std::to_string(count_++); 
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
}        

void UavJoyCtl::teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->linear.x); //%s'", msg->data.c_str()); 

    //msg_->linear.z = msg->linear.x; // / 5.0; 
    auto teleop_msg = geometry_msgs::msg::Twist(); 
    teleop_msg.linear.z = msg->linear.x; 

    //* This should work!
    RCLCPP_INFO_STREAM(this->get_logger(), "I want to publish: " << teleop_msg.linear.x); 

    //* Publish current speed to aerial manipulator (dummy callback to check current ctl)
    amLCmdVelPub_->publish(teleop_msg); 
}
    
//
//void UavJoyCtl::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const
//{
    // TODO: Get velocity from comparison of poses
    // TODO: Create method that will publish odometry 
//    if (!first_pose_reciv)
//    {
//        pose = msg.pose
//    }

//    else
//    {   
        //TODO: Add method to determine time difference between current pose and previous pose
        // Get current pose
//        pose_ = msg.pose
        // Calculate previous pose  
//        dp = pose_ - pose 
//        pose = msg.pose

//    }

//    first_pose_reciv = true; 
//}

void UavJoyCtl::amL_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
{
        RCLCPP_INFO_STREAM(this->get_logger(), "I recieved amL pose!"); 
        geometry_msgs::msg::TransformStamped transform_stamped;
        std::string toFrameRel("child_link"); 
        std::string fromFrameRel("parent_link"); 

        try {
          transform_stamped = amLTfBuffer->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
}

void UavJoyCtl::amS_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const 
{
        RCLCPP_INFO_STREAM(this->get_logger(), "I recieved amS pose!"); 

}


void UavJoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
{   
            
    float scale_factor; 
    float scale_factor_height; 
    bool switch_ctl = false; 

    // If L1 pressed, ctl small UAV
    if (msg->buttons.at(4) == 1){
        switch_ctl = true; 
    }

    // Call twist publisher based on joy readings 
    std::vector<float> axes_;
    axes_ = msg->axes; 

    // Populate teleop_twist msg
    float pitch; float height;  float roll; float yaw;             
    roll = axes_.at(3); pitch = axes_.at(4); 
    yaw  = axes_.at(0); height = axes_.at(1);

    // Change mode of control at R1
    if (msg->buttons.at(5) == 1){
        scale_factor = 1.1;
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE_L]: Drive"); 

    }else{
        scale_factor = 0.2; 
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION MODE_L]: Approach!"); 

    }

    if (msg->buttons[3] == 1){
        scale_factor_height = 10.0; 
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE_L]: Lifting!"); 
    }else{
        scale_factor_height = scale_factor; 
    }  

    // TODO: Make sure that velocity is only thing we can command to UAVs
    auto teleop_msg             = geometry_msgs::msg::Twist(); 
    teleop_msg.linear.x         = pitch     * scale_factor; 
    teleop_msg.linear.y         = roll      * scale_factor;  
    teleop_msg.linear.z         = height    * scale_factor_height * 0.2; 
    teleop_msg.angular.z        = yaw       * scale_factor * 0.4;

    if (switch_ctl)

    {
        std_msgs::msg::Bool suction_msg; 

        // Call start suction 
        if (msg->buttons.at(2) == 1){
            suction_msg.data = true;
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE_S] Suction on!");                    
        }else{
            suction_msg.data = false; 
        }

        amSCmdVelPub_->publish(teleop_msg); 
        amSGripperCmdSuctionPub_->publish(suction_msg); 

    }else{
        
        amLCmdVelPub_->publish(teleop_msg); 
        // Call open gripper w
        if (msg->buttons.at(0) == 1){                    
            //https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
            // https://answers.ros.org/question/340389/client-doesnt-return-when-declared-inside-c-class-in-ros-2/
            RCLCPP_INFO_STREAM(this->get_logger(), "Opening Gripper!");
            auto req_ = std::make_shared<std_srvs::srv::Empty::Request>();    
            openGripperClient_->async_send_request(req_); 
        }

        // Call close gripper
        if (msg->buttons.at(2) == 1){
            RCLCPP_INFO_STREAM(this->get_logger(), "Closing Gripper!");
            auto req_ = std::make_shared<std_srvs::srv::Empty::Request>(); 
            closeGripperClient_->async_send_request(req_); 
        }


    }

}


bool UavJoyCtl::close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                              std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.0; 

    amLGripperCmdPosLeftPub_->publish(finger_pos_msg); 
    amLGripperCmdPosRightPub_->publish(finger_pos_msg); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

    return true; 
}

bool UavJoyCtl::open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                            std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.5; 

    amLGripperCmdPosLeftPub_->publish(finger_pos_msg); 
    amLGripperCmdPosRightPub_->publish(finger_pos_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Opening gripper!");

    return true;  
        
}








