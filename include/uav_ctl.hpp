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
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//* custom msgs
#include "mbzirc_aerial_manipulation_msgs/msg/pose_euler.hpp"
#include "mbzirc_aerial_manipulation_msgs/msg/pose_error.hpp"
#include "mbzirc_aerial_manipulation_msgs/srv/change_state.hpp"
#include "mbzirc_aerial_manipulation_msgs/srv/takeoff.hpp"
#include "mbzirc_msgs/srv/usv_manipulate_object.hpp"

//* srvs
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"

//* controller
#include <jlb_pid/controller.hpp>
#include <jlb_pid/pid.hpp>
#include <jlb_pid/config.hpp>

#define stringify( name ) # name


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
        std::string                                                                     world_name_;
        std::string                                                                     detected_object_topic;
        std::string                                                                     detected_drone_topic; 
        bool                                                                            use_gt_;
        float                                                                           Kp_h, Kp_x, Kp_y, Kp_yaw; 
        float                                                                           Kd_h, Kd_x, Kd_y, Kd_yaw;
        OnSetParametersCallbackHandle::SharedPtr                                        callback_handle_;
        rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

        // callback groups
        rclcpp::CallbackGroup::SharedPtr                                                takeoff_group_;

        // publishers 
        rclcpp::Publisher<mbzirc_aerial_manipulation_msgs::msg::PoseError>::SharedPtr   absPoseDistPub_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                         cmdVelPub_; 
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr                   poseGtPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                            gripperCmdPosLeftPub_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                            gripperCmdPosRightPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                               gripperCmdSuctionPub_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                               fullSuctionContactPub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                             currentStatePub_;  
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                             startManipulationPub_; 

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                              joySub_;  
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr                           poseSub_; 
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr                           poseSubUsv_; 
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr                    currPoseSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                            currOdomSub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr                   detObjSub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr                   usvDropPoseSub_; 
        rclcpp::Subscription<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>::SharedPtr    cmdPoseSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                bottomContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                leftContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                rightContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                centerContactSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                topContactSub_; 
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                              imuSub_; 
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr                    magneticFieldSub_; 
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                                dockingFinishedSub_; 

        // services --> spawn services relating to gripper depending on UAV type 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                    openGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                    closeGripperSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                    startSuctionSrv_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                    stopSuctionSrv_; 
        rclcpp::Service<mbzirc_aerial_manipulation_msgs::srv::ChangeState>::SharedPtr       changeStateSrv_; 
        rclcpp::Service<mbzirc_aerial_manipulation_msgs::srv::Takeoff>::SharedPtr           takeoffSrv_; 

        rclcpp::Client<mbzirc_msgs::srv::UsvManipulateObject>::SharedPtr                    callArmClient_; 
        
        // timers
        rclcpp::TimerBase::SharedPtr                                        timer_;

        // controllers
        jlbpid::Controller                                                  x_controller_; 
        jlbpid::Controller                                                  x_drop_controller_; 
        jlbpid::Controller                                                  y_controller_;
        jlbpid::Controller                                                  y_drop_controller_; 
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
        bool                                                                usvPosReciv = false; 
        bool                                                                usvFinishedDocking_ = false; 
        bool                                                                bottomC, topC, leftC, rightC, centerC; 
        int                                                                 contactCounter_=0; 
        float                                                               roll, pitch;
        float                                                               currentYaw_, cmdYaw_;
        float                                                               magHeadingRad_, magHeadingDeg_; 

        float                                                               imuMeasuredPitch_, imuMeasuredRoll_, imuMeasuredYaw_; 
        geometry_msgs::msg::PoseStamped                                     currPose_; 
        geometry_msgs::msg::PoseStamped                                     cmdPose_; 
        geometry_msgs::msg::PointStamped                                    detObjPose_; 
        geometry_msgs::msg::PointStamped                                    dropOffPoint_; 
        mbzirc_aerial_manipulation_msgs::msg::PoseError                     poseError_; 
        geometry_msgs::msg::Vector3                                         currEuler_; 
        sensor_msgs::msg::Imu                                               currImuData_; 
        geometry_msgs::msg::Twist                                           cmdVel_;

        double                                                              lift_open_loop_z_ = 0.0;
        double                                                              lift_open_loop_v_ = 0.0;

        double go_to_drop_vel_x_ = 0.0;
        double go_to_drop_vel_y_ = 0.0;
        double go_to_drop_vel_z_ = 0.0;

        double go_to_drop_pos_x_ = 0.0;
        double go_to_drop_pos_y_ = 0.0;
        double go_to_drop_pos_z_ = 0.0;

        double go_to_drop_compensate_x_ = 0.0;
        double go_to_drop_compensate_y_ = 0.0;
        double go_to_drop_compensate_z_ = 0.0;

        double ex_dropOffPoint_stamp_ = 0.0;

        bool first_time_entering_go_to_drop_ = true;

        uint32_t compensation_counter_ = 0;
        uint32_t compensation_iterations_ = 200;

        double compensation_factor_start_xy_ = 0.3;
        double compensation_factor_end_xy_ = 0.02;
        double compensation_factor_start_z_ = 0.05;
        double compensation_factor_end_z_ = 0.02;

        // State machine
        enum state 
        {   
            IDLE = 0, 
            JOYSTICK = 1, 
            POSITION = 2,  
            SERVOING = 3, 
            APPROACH = 4,  
            PRE_GRASP = 5, 
            GRASP = 6, 
            LIFT = 7, 
            GO_TO_DROP = 8, 
            DROP = 9,
            INIT_STATE = 10
        };

         // depends on num states
        const char* stateNames[11] = 
        {
            stringify( IDLE ), 
            stringify( JOYSTICK ), 
            stringify( POSITION ), 
            stringify( SERVOING ), 
            stringify( APPROACH ), 
            stringify( PRE_GRASP ), 
            stringify( GRASP ), 
            stringify( LIFT ), 
            stringify( GO_TO_DROP ),
            stringify( DROP ),
            stringify( INIT_STATE )
        }; 

      
        enum state                                                          current_state_ = INIT_STATE;


        // init methods
        void init(); 
        void init_ctl(); 
        void init_params(); 

        // timer callback 
        void timer_callback(); 

        // methods
        void    get_pose_dist();        
        double  calculate_yaw_setpoint();  

        // sub callbacks
        void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg); 
        void pose_callback_usv(const tf2_msgs::msg::TFMessage::SharedPtr msg); 
        void curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg); 
        void curr_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); 
        void cmd_pose_callback(const mbzirc_aerial_manipulation_msgs::msg::PoseEuler::SharedPtr msg);      
        void det_obj_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);   
        void det_uav_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg); 
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);  
        void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg); 
        void docking_finished_callback(const std_msgs::msg::Bool::SharedPtr msg); 

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
        bool change_state(const mbzirc_aerial_manipulation_msgs::srv::ChangeState::Request::SharedPtr req, 
                            mbzirc_aerial_manipulation_msgs::srv::ChangeState::Response::SharedPtr res); 
        bool take_off(const mbzirc_aerial_manipulation_msgs::srv::Takeoff::Request::SharedPtr req, 
                      mbzirc_aerial_manipulation_msgs::srv::Takeoff::Response::SharedPtr res); 


        // getters
        float getCurrentYaw(); 
        float getCmdYaw(); 
        // suction gripper related
        int getNumContacts() const; 
        bool checkContacts() const; 
        void printContacts() const; 
        // controller related
        static void limitCommand(double& cmd, double limit);                    
        double calcPropCmd(double gainP, double cmd_sp, double cmd_mv, double limit_command); 
        double calcPidCmd(jlbpid::Controller& controller, double cmd_sp, double cmd_mv); 
        void setPidController(jlbpid::Controller& controller, jlbpid::PID pid, jlbpid::Config config); 
        // state related 
        void graspControl(); 
        void positionControl(geometry_msgs::msg::Twist& cmdVel); 
        void servoControl(geometry_msgs::msg::Twist& cmdVel); 
        void approachControl(geometry_msgs::msg::Twist& cmdVel); 
        void preGraspControl(geometry_msgs::msg::Twist& cmdVel); 
        void liftControl(geometry_msgs::msg::Twist& cmdVel); 
        void goToDropControl(geometry_msgs::msg::Twist& cmdVel); 
        void dropControl(geometry_msgs::msg::Twist& cmdVel); 







       


        

};

#endif  
