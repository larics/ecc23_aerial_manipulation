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
#include "mbzirc_aerial_manipulation/srv/choose_uav.hpp"

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2; 

class UavJoy: public rclcpp::Node 
{
	public:
		UavJoy(); 
		//~UavJoy(); 

	private:
	        // publishers	
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_; 
		
		// subscribers
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_; 

		// services
		rclcpp::Service<mbzirc_aerial_manipulation::srv::ChooseUav>::SharedPtr chooseUavSrv_; 

		void init(); 
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const; 
		void choose_uav(const mbzirc_aerial_manipulation::srv::ChooseUav::Request::SharedPtr req, mbzirc_aerial_manipulation::srv::ChooseUav::Response::SharedPtr res); 

};

#endif


