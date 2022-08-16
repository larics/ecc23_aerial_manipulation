
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
    amLCmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>(ns_ + std::string("/cmd_vel"), 1); 
    amLGripperCmdPosLeftPub_  = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_left/cmd_pos"), 1); 
    amLGripperCmdPosRightPub_ = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_right/cmd_pos"), 1); 
    amSGripperCmdSuctionPub_  = this->create_publisher<std_msgs::msg::Bool>("/am_S/gripper/suction_on", 1); 
            
    // Subscribers
    // amLPoseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>("/am_L/pose_static", 1, std::bind(&UavCtl::amL_pose_callback, this, _1)); 
    // amSPoseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>("/am_S/pose_static", 1, std::bind(&UavCtl::amS_pose_callback, this, _1)); 
    poseSub_                  = this->create_subscription<tf2_msgs::msg::TFMessage>(ns_ + std::string("/pose_static"), 1, std::bind(&UavCtl::pose_callback, this, _1));

    // Services
    openGripperSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper"), std::bind(&UavCtl::open_gripper, this, _1, _2)); 
    closeGripperSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"),  std::bind(&UavCtl::close_gripper, this, _1, _2)); 
    startSuctionSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/start_suction"), std::bind(&UavCtl::start_suction, this, _1, _2)); 
    stopSuctionSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/stop_suction"), std::bind(&UavCtl::stop_suction, this, _1, _2)); 

    // Clients --> TODO: Move this clients to uav_ctl 
    openGripperClient_        = this->create_client<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper")); 
    closeGripperClient_       = this->create_client<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"));

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
void UavCtl::timer_callback()
{
    auto message = std_msgs::msg::String(); 
    message.data = "Hello, world! " + std::to_string(count_++); 
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
}        

void UavCtl::pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
{       

        //RCLCPP_INFO_STREAM(this->get_logger(), "I recieved uav pose!"); 
        geometry_msgs::msg::TransformStamped transform_stamped;

        std::string uav_ns = this->ns_;
        // Remove backslash 
        uav_ns.erase(0, 1); 
        std::string toFrameRel("empty_platform"); 
        std::string fromFrameRel(uav_ns); 

        // This message is list of transforms, check how to inspect list of transforms in ROS2

        //RCLCPP_INFO_STREAM(this->get_logger(), "Recieved following msg: %msg" << msg->c_str()); 
        RCLCPP_INFO_STREAM(this->get_logger(), "Recieved following msg:" << msg.c_str()); 

        try {
          // Looking at wrong buffer!
          transform_stamped = amLTfBuffer->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);

        } catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Found transform!"); 
}

bool UavCtl::close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                              std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.0; 

    amLGripperCmdPosLeftPub_->publish(finger_pos_msg); 
    amLGripperCmdPosRightPub_->publish(finger_pos_msg); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

    return true; 
}
bool UavCtl::open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                            std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.5; 

    amLGripperCmdPosLeftPub_->publish(finger_pos_msg); 
    amLGripperCmdPosRightPub_->publish(finger_pos_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Opening gripper!");

    return true;  
        
}

bool UavCtl::start_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res)
{
    return true; 
}

bool UavCtl::stop_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                          std_srvs::srv::Empty::Response::SharedPtr res)
{
    return true; 
}








