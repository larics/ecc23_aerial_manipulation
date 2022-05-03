#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

//* srvs
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"


using namespace std::chrono_literals; 
using std::placeholders::_1;
using std::placeholders::_2; 

/* This example creates a subclass of Node and uses std::bind() to register a 
* member function as callback from the timer. */

class UavJoyCtl : public rclcpp::Node
{
    public: 
        UavJoyCtl()
        : Node ("uav_joy_ctl"), count_(0)
        {   
            // Publishers
            testPublisher_  = this->create_publisher<std_msgs::msg::String>("/test_pub", 1); 
            amLCmdVelPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/am_L/cmd_vel", 1); 
            amLGripperCmdPosLeftPublisher_ = this->create_publisher<std_msgs::msg::Float64>("/am_L/gripper/joint/finger_left/cmd_pos", 1); 
            amLGripperCmdPosRightPublisher_ = this->create_publisher<std_msgs::msg::Float64>("/am_L/gripper/joint/finger_right/cmd_pos", 1); 
            
            // Subscribers
            joySubscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
                                                                              std::bind(&UavJoyCtl::joy_callback, this, _1)); 

            teleopSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                                                                                     std::bind(&UavJoyCtl::teleop_callback, this, _1)); 

            // Services
            openGripperService_ = this->create_service<std_srvs::srv::Trigger>("/am_L/open_gripper",
                                                                                std::bind(&UavJoyCtl::open_gripper, this, _1, _2)); 
            closeGripperService_ = this->create_service<std_srvs::srv::Trigger>("/am_L/close_gripper",  
                                                                                std::bind(&UavJoyCtl::close_gripper, this, _1, _2)); 
            
            /*timer_ = this->create_wall_timer(
            500ms, std::bind(&UavJoyCtl::timer_callback, this)); */
        }

     
    private: 
        /*
        void timer_callback()
        {
            auto message = std_msgs::msg::String(); 
            message.data = "Hello, world! " + std::to_string(count_++); 
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
            testPublisher_->publish(message); 
        }
        */

        void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg->linear.x); //%s'", msg->data.c_str()); 


            //msg_->linear.z = msg->linear.x; // / 5.0; 
            auto teleop_msg = geometry_msgs::msg::Twist(); 
            teleop_msg.linear.z = msg->linear.x; 

            //* This should work!
            RCLCPP_INFO_STREAM(this->get_logger(), "I want to publish: " << teleop_msg.linear.x); 

            //* Publish current speed to aerial manipulator (dummy callback to check current ctl)
            amLCmdVelPublisher_->publish(teleop_msg); 
        }

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
        {
            
            //RCLCPP_INFO_STREAM(this->get_logger() , "Buttons are: " << msg->buttons); 
            RCLCPP_INFO_STREAM(this->get_logger() , "Joy callback active! "); 
            // Call twist publisher based on joy readings 
            std::vector<float> axes_;
            axes_ = msg->axes; 

            // Populate teleop_twist msg
            float pitch; float height;  float roll; float yaw;             
            roll = axes_.at(2);
            pitch = axes_.at(3); 
            yaw = axes_.at(4); 
            height = axes_.at(5);

            auto teleop_msg = geometry_msgs::msg::Twist(); 
            //teleop_msg.linear.x     = pitch * 5; 
            //teleop_msg.linear.y     = roll  * 5; 
            teleop_msg.linear.z     = height * 0.5; 
            teleop_msg.angular.z    = yaw * 0.5;
            teleop_msg.linear.x    = pitch * 15; 
            teleop_msg.linear.y    = roll * 15;  

            amLCmdVelPublisher_->publish(teleop_msg); 

        }

        bool close_gripper(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                           std_srvs::srv::Trigger::Response::SharedPtr res){

            auto finger_pos_msg = std_msgs::msg::Float64(); 
            finger_pos_msg.data = 0.0; 

            amLGripperCmdPosLeftPublisher_->publish(finger_pos_msg); 
            amLGripperCmdPosRightPublisher_->publish(finger_pos_msg); 

            RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

            return true; 

        }

        bool open_gripper(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                          std_srvs::srv::Trigger::Response::SharedPtr res){

            auto finger_pos_msg = std_msgs::msg::Float64(); 
            finger_pos_msg.data = 0.5; 

            amLGripperCmdPosLeftPublisher_->publish(finger_pos_msg); 
            amLGripperCmdPosRightPublisher_->publish(finger_pos_msg);

            RCLCPP_INFO_STREAM(this->get_logger(), "Opening gripper!");

            return true;  
        
        }

        // Timers
        rclcpp::TimerBase::SharedPtr timer_; 

        // Publishers
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     testPublisher_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr amLCmdVelPublisher_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    amLGripperCmdPosLeftPublisher_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    amLGripperCmdPosRightPublisher_; 

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_; 
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleopSubscriber_; 

        //  Services 
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr openGripperService_; 
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr closeGripperService_; 

        // * Hello World cound
        size_t count_; 
};

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<UavJoyCtl>()); 
    rclcpp::shutdown(); 
    return 0; 

}

