#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals; 

/* This example creates a subclass of Node and uses std::bind() to register a 
* member function as callback from the timer. */

class UavJoyCtl : public rclcpp::Node
{
    public: 
        UavJoyCtl()
        : Node ("uav_joy_ctl"), count_(0)
        {   
            testPublisher_  = this->create_publisher<std_msgs::msg::String>("/test_pub", 1); 
            amLPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/am_L", 1); 
            timer_ = this->create_wall_timer(
            500ms, std::bind(&UavJoyCtl::timer_callback, this)); 
        }
    private: 
        void timer_callback()
        {
            auto message = std_msgs::msg::String(); 
            message.data = "Hello, world! " + std::to_string(count_++); 
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
            testPublisher_->publish(message); 
        }
        
        //void joy_callback()
        //{
        //
        //}
        
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr testPublisher_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr amLPublisher_; 
        size_t count_; 
};

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<UavJoyCtl>()); 
    rclcpp::shutdown(); 
    return 0; 

}

