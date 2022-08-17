#ifndef UAV_CTRL_H
#define UAV_CTRL_H

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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix.h"
#include "tf2_msgs/msg/tf_message.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//* srvs
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

//* controller
#include <jlb_pid/controller.hpp>
#include <jlb_pid/pid.hpp>
#include <jlb_pid/config.hpp>


using namespace std::chrono_literals;  
using std::placeholders::_1;
using std::placeholders::_2; 

class UavCtl: public rclcpp::Node
{

    public: 

        UavCtl(); 
        //~UavCtl(); 

        std::string ns_; 

    private: 

        // publishers --> TODO: Change names!
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmdVelPub_; 
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       poseGtPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                gripperCmdPosLeftPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                gripperCmdPosRightPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   gripperCmdSuctionPub_; 

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              joySub_;  
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr           poseSub_; 
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    currPoseSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    cmdPoseSub_; 

        // services --> spawn services relating to gripper depending on UAV type 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    openGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    closeGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    startSuctionSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    stopSuctionSrv_; 
        
        // timers
        rclcpp::TimerBase::SharedPtr                                        timer_;

        // controllers
        jlbpid::Controller                                                  v_controller_; 
        jlbpid::Controller                                                  h_controller_; 
        jlbpid::Controller                                                  y_controller_; 
        jlbpid::PID                                                         pid; 

        // tf_buffers
        std::unique_ptr<tf2_ros::Buffer>                                    amSTfBuffer{nullptr};

        // transform_listener
        std::shared_ptr<tf2_ros::TransformListener>                         amSTransformListener{nullptr}; 

        int                                                                 operationMode;
        bool                                                                nodeInitialized = false; 
        bool                                                                cmdReciv = false; 
        geometry_msgs::msg::PoseStamped                                     currPose_; 
        geometry_msgs::msg::PoseStamped                                     cmdPose_; 
        geometry_msgs::msg::Vector3                                         currEuler_; 


        // init methods
        void init(); 
        void init_ctl(); 

        // timer callback 
        void timer_callback(); 

        // sub callbacks
        void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg); 
        void curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 
        void cmd_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 

        // service callbacks
        bool close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool start_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool stop_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 

};

#endif  
