#include "uav_joy.hpp"

UavJoy::UavJoy(): Node("uav_joy")
{
	// Initialize 
	init();

    setScaleFactor(1); 

    enableJoy_ = true; 

	// TODO: Add clients into initialization 
	// TODO: Change service to integrate type of UAV into consideration 
}

void UavJoy::init()
{

    // publishers
    cmdVelPub_ 		    = this->create_publisher<geometry_msgs::msg::Twist>("/am_L/cmd_vel", 1); 

    // subscribers
    joySub_ 		    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&UavJoy::joy_callback, this, _1)); 

    // services 
    chooseUavSrv_ 	    = this->create_service<mbzirc_aerial_manipulation_msgs::srv::ChooseUav>("/choose_uav", std::bind(&UavJoy::choose_uav, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Initialized uav_joy"); 
}

void UavJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{   
    
	float pitch; float thrust; float roll; float yaw; 
	std::vector<float> axes_ = msg->axes; 
	
	roll = axes_.at(3); pitch = axes_.at(4); 
	yaw = axes_.at(0); thrust = axes_.at(1);     

    // Enabling joystick functionality
    // Arrow up + R1 --> turnsOff
    if (msg->axes.at(7) == 1 && msg->buttons.at(4))
    { 
        RCLCPP_INFO(this->get_logger(), "Turning joystick on!");    
        setEnableJoy(true); 
    }

    // Arrow up + R2 --> turnsOn
    if (msg->axes.at(7) == 1 && msg->axes.at(2) == -1)
    {
        
        RCLCPP_INFO(this->get_logger(), "Turning joystick off!"); 
        setEnableJoy(false); 
    }

    enableJoy_ = getEnableJoy(); 



    float sF_; 
    int sF = getScaleFactor();
    // https://www.quantstart.com/articles/Passing-By-Reference-To-Const-in-C/ 
    if (msg->buttons.at(5) == 1){
        // crazy flight mode
        sF_ = sF; 
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE]: Crazy!"); 

    }else{
        // normal flight mode
        sF_ = static_cast<float>(sF) / 10.0;  
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION MODE]: Slow!"); 
    }


	// Create teleop msg
	auto teleop_msg 	    = geometry_msgs::msg::Twist(); 
	teleop_msg.linear.x	    = pitch  * sF_; 
	teleop_msg.linear.y 	= roll   * sF_; 
	teleop_msg.linear.z	    = thrust * sF_; 
	teleop_msg.angular.z 	= yaw    * sF_; 
    
    if (enableJoy_){
        cmdVelPub_->publish(teleop_msg); 
    }
    
    // [ ] --> open gripper
    if (msg->buttons.at(0) == 1){                    
        // https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
        // https://answers.ros.org/question/340389/client-doesnt-return-when-declared-inside-c-class-in-ros-2/
        RCLCPP_INFO_STREAM(this->get_logger(), "Releasing object!");
        auto req_ = std::make_shared<std_srvs::srv::Empty::Request>();    
        openGripperClient_->async_send_request(req_); 
        stopSuctionClient_->async_send_request(req_); 
            
    }

    // X --> close_gripper
    if (msg->buttons.at(2) == 1){
        RCLCPP_INFO_STREAM(this->get_logger(), "Grasping object!");
        auto req_ = std::make_shared<std_srvs::srv::Empty::Request>(); 
        closeGripperClient_->async_send_request(req_); 
        startSuctionClient_->async_send_request(req_); 
    }

    // △ --> increase scale factor by one
    if (msg->buttons.at(3) == 1){
        sF++; 
        setScaleFactor(sF);  
        RCLCPP_INFO_STREAM(this->get_logger(), "Increasing scale factor: " << scale_factor);
    }
    
    // O --> reset scale factor on one
    if (msg->buttons.at(1) == 1){
        RCLCPP_INFO_STREAM(this->get_logger(), "Resetting scale factor."); 
        setScaleFactor(1); 
    }

}

void UavJoy::choose_uav(const mbzirc_aerial_manipulation_msgs::srv::ChooseUav::Request::SharedPtr req, 
		mbzirc_aerial_manipulation_msgs::srv::ChooseUav::Response::SharedPtr res) 
{
	// Maybe will be neccessary to have mutex lock to prevent breaking when trying to publish 
	// message during topic change
	std::string uav_namespace; 
    uav_namespace = static_cast<std::string>(req->uav_ns); 

    // Object manipulation clients
    openGripperClient_        = this->create_client<std_srvs::srv::Empty>(uav_namespace + std::string("/open_gripper")); 
    closeGripperClient_       = this->create_client<std_srvs::srv::Empty>(uav_namespace + std::string("/close_gripper"));
    startSuctionClient_       = this->create_client<std_srvs::srv::Empty>(uav_namespace + std::string("/start_suction")); 
    stopSuctionClient_        = this->create_client<std_srvs::srv::Empty>(uav_namespace + std::string("/stop_suction")); 

	RCLCPP_INFO(this->get_logger(), "Joystick control switched to %s!", uav_namespace.c_str()); 

    cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>(uav_namespace + std::string("/cmd_vel"), 1); 
}

// Methods that set scale factor 
void UavJoy::setScaleFactor(int value)
{
    scale_factor = value; 
}

int UavJoy::getScaleFactor() const
{
    return scale_factor; 
}

void UavJoy::setEnableJoy(bool val) 
{
    enableJoy_ = val; 
}

bool UavJoy::getEnableJoy() const
{
    return enableJoy_; 
}