#ifndef UAV_CTRL_H
#define UAV_CTRL_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

//* ros
#include "rclcpp/rclcpp.hpp"

//* tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_msgs/msg/tf_message.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//* custom msgs
#include "mbzirc_aerial_manipulation_msgs/msg/pose_euler.hpp"

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

        // parameters
        std::string                                                         world_name_;

        // publishers 
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr              absPoseDistPub_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmdVelPub_; 
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       poseGtPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                gripperCmdPosLeftPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                gripperCmdPosRightPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   gripperCmdSuctionPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   fullSuctionContactPub_; 

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                          joySub_;  
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr                       poseSub_; 
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                currPoseSub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr                detObjSub_;
        rclcpp::Subscription<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>::SharedPtr     cmdPoseSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            bottomContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            leftContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            rightContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            centerContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                            topContactSub_; 

        // services --> spawn services relating to gripper depending on UAV type 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    openGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    closeGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    startSuctionSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                    stopSuctionSrv_; 
        
        // timers
        rclcpp::TimerBase::SharedPtr                                        timer_;

        // controllers
        jlbpid::Controller                                                  x_controller_; 
        jlbpid::Controller                                                  y_controller_;
        jlbpid::Controller                                                  z_controller_; 
        jlbpid::Controller                                                  yaw_controller_;  
        jlbpid::PID                                                         pid; 
        jlbpid::Config                                                      config; 

        // tf_buffers
        std::unique_ptr<tf2_ros::Buffer>                                    amSTfBuffer{nullptr};

        // transform_listener
        std::shared_ptr<tf2_ros::TransformListener>                         amSTransformListener{nullptr};

        // transform_broadcaster
        std::unique_ptr<tf2_ros::TransformBroadcaster>                      staticPoseTfBroadcaster_;

        int                                                                 operationMode;
        bool                                                                nodeInitialized = false; 
        bool                                                                cmdReciv = false; 
        bool                                                                bottomC, topC, leftC, rightC, centerC; 
        int                                                                 contactCounter_=0; 
        float                                                               roll, pitch;
        float                                                               currentYaw_, cmdYaw_;  
        geometry_msgs::msg::PoseStamped                                     currPose_; 
        geometry_msgs::msg::PoseStamped                                     cmdPose_; 
        geometry_msgs::msg::PointStamped                                    detObjPose_; 
        geometry_msgs::msg::Pose                                            poseDist_; 
        geometry_msgs::msg::Vector3                                         currEuler_; 

            // State machine
        enum state 
        {   
            IDLE = 0, 
            JOYSTICK = 1, 
            POSITION = 2, 
            SERVOING = 3, 
            APPROACH = 4, 
            GRASP = 5, 
            ALIGN_GRASP = 6      
        };

        enum state                                                          current_state_ = IDLE;


        // init methods
        void init(); 
        void init_ctl(); 

        // timer callback 
        void timer_callback(); 

        // methods
        void    get_pose_dist();        
        double  calculate_yaw_setpoint();  

        // sub callbacks
        void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg); 
        void curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 
        void cmd_pose_callback(const mbzirc_aerial_manipulation_msgs::msg::PoseEuler::SharedPtr msg);      
        void det_obj_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);    
        // contacts
        void bottom_contact_callback(const std_msgs::msg::Bool::SharedPtr msg); 
        void left_contact_callback(const std_msgs::msg::Bool::SharedPtr msg); 
        void right_contact_callback(const std_msgs::msg::Bool::SharedPtr msg); 
        void top_contact_callback(const std_msgs::msg::Bool::SharedPtr msg); 
        void center_contact_callback(const std_msgs::msg::Bool::SharedPtr msg); 

        // service callbacks
        bool close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool start_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 
        bool stop_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res); 

        static void limitCommand(double& cmd, double limit);                    

        // getters
        float getCurrentYaw(); 
        float getCmdYaw(); 
        bool checkContacts() const; 
        void printContacts() const; 
        int getNumContacts() const; 
        void generateContactRef(double& cmd_x, double& cmd_y); 

        

};

#endif  
