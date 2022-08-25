#ifndef UAV_JOY_H
#define UAV_JOY_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

//* ros 
#include "rclcpp/rclcpp.hpp"

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

//* srvs
#include "mbzirc_aerial_manipulation_msgs/srv/choose_uav.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2; 

class UavJoy: public rclcpp::Node 
{
	public:
		UavJoy(); 
		//~UavJoy(); 

	private:

		// vars
		bool 		enableJoy_; 
		mutable int scale_factor;  
	    
		// publishers	
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_; 
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr 		suctionPub_; 
		
		// subscribers
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_; 

		// services
		rclcpp::Service<mbzirc_aerial_manipulation_msgs::srv::ChooseUav>::SharedPtr chooseUavSrv_; 

		// clients 
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr		chooseUavClient_; // Could be used for initing all UAVs
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr		openGripperClient_; 
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr     closeGripperClient_; 
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr		startSuctionClient_; 
		rclcpp::Client<std_srvs::srv::Empty>::SharedPtr		stopSuctionClient_; 

		void init(); 
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); 
		void choose_uav(const mbzirc_aerial_manipulation_msgs::srv::ChooseUav::Request::SharedPtr req,
						 mbzirc_aerial_manipulation_msgs::srv::ChooseUav::Response::SharedPtr res); 

		// Setting them as const to be usable by joy_callback which is also const
		void setScaleFactor(int value); 
		int getScaleFactor() const; 
		void setEnableJoy(bool val); 
		bool getEnableJoy() const; 

		// TODO: Add service to turn joystick on and off

};

#endif


