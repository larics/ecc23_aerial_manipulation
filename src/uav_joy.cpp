#include "uav_joy.hpp"

UavJoy::UavJoy(): Node("uav_joy")
{
	// Initialize 
	init();

	// TODO: Add clients into initialization 
	// TODO: Change service to integrate type of UAV into consideration 


}

void UavJoy::init()
{
    cmdVelPub_ 		= 	this->create_publisher<geometry_msgs::msg::Twist>("/uav_name/cmd_vel", 1); 

    joySub_ 		= 	this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&UavJoy::joy_callback, this, _1)); 

    chooseUavSrv_ 	=	this->create_service<mbzirc_aerial_manipulation::srv::ChooseUav>("/choose_uav", std::bind(&UavJoy::choose_uav, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Initialized uav_joy"); 
}

void UavJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
{
	float pitch; float thrust; float roll; float yaw; 
	std::vector<float> axes_ = msg->axes; 
	
	roll = axes_.at(3); pitch = axes_.at(4); 
	yaw = axes_.at(0); thrust = axes_.at(1); 

	// Create teleop msg
	auto teleop_msg 	= geometry_msgs::msg::Twist(); 
	teleop_msg.linear.x	= pitch; 
	teleop_msg.linear.y 	= roll; 
	teleop_msg.linear.z	= thrust; 
	teleop_msg.angular.z 	= yaw; 

	cmdVelPub_->publish(teleop_msg); 

	/* Code from original uav_ctl node 
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
	*/

}

void UavJoy::choose_uav(const mbzirc_aerial_manipulation::srv::ChooseUav::Request::SharedPtr req, 
		mbzirc_aerial_manipulation::srv::ChooseUav::Response::SharedPtr res) 
{
	std::cout << req->uav_ns  << std::endl;

	// Maybe will be neccessary to have mutex lock to prevent breaking when trying to publish 
	// message during topic change
	std::string uav_namespace; 
        uav_namespace = static_cast<std::string>(req->uav_ns); 
	RCLCPP_INFO(this->get_logger(), "Joystick control switched to %s!", uav_namespace.c_str()); 
        cmdVelPub_ = this->create_publisher<geometry_msgs::msg::Twist>(uav_namespace + std::string("/cmd_vel"), 1); 	
}
