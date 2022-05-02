#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
/*#include "joy_msgs/msg/joy.hpp"*/

using namespace std::chrono_literals; 
using std::placeholders::_1;

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
            
            //joySubscriber_ = this->create_subscription<joy_msgs::msg::Joy>("/joy", 10, 
            //std::bind(&UavJoyCtl::joy_callback, this, _1)); 

            teleopSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
            std::bind(&UavJoyCtl::teleop_callback, this, _1)); 
            
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

        void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->linear.x); //%s'", msg->data.c_str()); 
        }

        
        /* void joy_callback(const joy_msgs::msg::Joy::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), )
            // Call twist publisher based on joy readings 

        }
        */

        // * Timers
        rclcpp::TimerBase::SharedPtr timer_; 

        // * Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr testPublisher_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr amLPublisher_; 

        // * Subscribers
        /* rclcpp::Subscription<joy_msgs::msg::Joy>::SharedPtr joySubscriber_; */
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleopSubscriber_; 

        // * Services 
        size_t count_; 
};

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<UavJoyCtl>()); 
    rclcpp::shutdown(); 
    return 0; 

}

