#ifndef UAV_JOY_CTRL_H
#define UAV_JOY_CTRL_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

//* ros
#include "rclcpp/rclcpp.hpp"

//* tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//* srvs
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;  
using std::placeholders::_1;
using std::placeholders::_2; 

class UavJoyCtl: public rclcpp::Node
{

    public: 

        UavJoyCtl(); 
        //~UavJoyCtl(); 


    private: 

     

        // publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         amLCmdVelPub_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         amSCmdVelPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            amLGripperCmdPosLeftPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr            amLGripperCmdPosRightPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               amSGripperCmdSuctionPub_; 

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr          joySub_;  
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      teleopSub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr       amLPoseSub_; 
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr       amSPoseSub_; 

        // services 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                openGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                closeGripperSrv_; 

        // clients
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                 openGripperClient_; 
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                 closeGripperClient_; 


        // timers
        rclcpp::TimerBase::SharedPtr                                    timer_{nullptr};

        // tf_buffers
        std::unique_ptr<tf2_ros::Buffer>                                amSTfBuffer{nullptr};
        std::unique_ptr<tf2_ros::Buffer>                                amLTfBuffer{nullptr};

        // transform_listener
        std::shared_ptr<tf2_ros::TransformListener>                     amSTransformListener{nullptr}; 
        std::shared_ptr<tf2_ros::TransformListener>                     amLTransformListener{nullptr}; 

        int operationMode;
        size_t count_; 

        void init(); 
        void timer_callback(); 
        void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const; 
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const; 
        void amL_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const; 
        void amS_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const; 
        bool close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 

};

#endif  // CONTROL_ARM_H
