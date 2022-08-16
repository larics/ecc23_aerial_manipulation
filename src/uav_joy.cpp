#include "uav_joy.hpp"

UavJoy::UavJoy(): Node("uav_joy")
{
	// Initialize 
	init();


}

void UavJoy::init()
{
    cmdVelPub_ 		= 	this->create_publisher<geometry_msgs::msg::Twist>("/uav_name/cmd_vel", 1); 

    joySub_ 		= 	this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&UavJoy::joy_callback, this, _1)); 

    chooseUavSrv_ 	=	this->create_service<mbzirc_aerial_manipulation::srv::ChooseUav>("/choose_uav", std::bind(&UavJoy::choose_uav, this, _1, _2));

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

}

void UavJoy::choose_uav(const mbzirc_aerial_manipulation::srv::ChooseUav::Request::SharedPtr req, 
		mbzirc_aerial_manipulation::srv::ChooseUav::Response::SharedPtr res) 
{
	std::cout << req << std::endl; 
}
