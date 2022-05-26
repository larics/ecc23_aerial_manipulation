#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
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
            testPublisher_                  = this->create_publisher<std_msgs::msg::String>("/test_pub", 1); 
            amLCmdVelPublisher_             = this->create_publisher<geometry_msgs::msg::Twist>("/am_L/cmd_vel", 1); 
            amSCmdVelPublisher_             = this->create_publisher<geometry_msgs::msg::Twist>("/am_S/cmd_vel", 1); 
            amLGripperCmdPosLeftPublisher_  = this->create_publisher<std_msgs::msg::Float64>("/am_L/gripper/joint/finger_left/cmd_pos", 1); 
            amLGripperCmdPosRightPublisher_ = this->create_publisher<std_msgs::msg::Float64>("/am_L/gripper/joint/finger_right/cmd_pos", 1); 
            amSGripperCmdSuctionPublisher_  = this->create_publisher<std_msgs::msg::Bool>("/am_S/gripper/suction_on", 1); 
            
            // Subscribers
            joySubscriber_                  = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
                                                                              std::bind(&UavJoyCtl::joy_callback, this, _1)); 

            teleopSubscriber_               = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, 
                                                                                     std::bind(&UavJoyCtl::teleop_callback, this, _1)); 
            // Services
            openGripperService_             = this->create_service<std_srvs::srv::Empty>("/am_L/open_gripper",
                                                                                std::bind(&UavJoyCtl::open_gripper, this, _1, _2)); 
            closeGripperService_            = this->create_service<std_srvs::srv::Empty>("/am_L/close_gripper",  
                                                                                std::bind(&UavJoyCtl::close_gripper, this, _1, _2)); 

            // Clients 
            openGripperClient_ = this->create_client<std_srvs::srv::Empty>("/am_L/open_gripper"); 
            closeGripperClient_ = this->create_client<std_srvs::srv::Empty>("/am_L/close_gripper");
             
            //startSuctionService_ = this->create_service<std_srvs::srv::Triger>("/am_S/suction")
            
            
            //constexpr static double SYSTEM_DT = 0.2;
            //timer_ = this->create_wall_timer(std::chrono::duration<double>(SYSTEM_DT), std::bind(&UavJoyCtl::timer_callback, this)); 
        }

     
    private:


        // Timer callback executes every 1.0/0.2 (5s)
        void timer_callback()
        {
            auto message = std_msgs::msg::String(); 
            message.data = "Hello, world! " + std::to_string(count_++); 
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 

        }        

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
            
            float scale_factor; 
            bool switch_ctl = false; 

            // If L1 pressed, ctl small UAV
            if (msg->buttons.at(4) == 1){
                switch_ctl = true; 
            }

            // Call twist publisher based on joy readings 
            std::vector<float> axes_;
            axes_ = msg->axes; 

            // Populate teleop_twist msg
            float pitch; float height;  float roll; float yaw;             
            roll = axes_.at(2); pitch = axes_.at(3); 
            yaw = axes_.at(4); height = axes_.at(5);

            // Change mode of control at R1
            if (msg->buttons.at(5) == 1){
                scale_factor = 1.0;
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE]: Drive"); 

            }else{
                scale_factor = 0.1; 
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION MODE]: Approach!"); 

            }


            // TODO: Make sure that velocity is only thing we can command to UAVs
            auto teleop_msg = geometry_msgs::msg::Twist(); 
            teleop_msg.linear.x         = pitch * scale_factor; 
            teleop_msg.linear.y         = roll * scale_factor;  
            teleop_msg.linear.z         = height * scale_factor * 0.2; 
            teleop_msg.angular.z        = yaw * scale_factor* 0.4;

            if (switch_ctl)

            {
                std_msgs::msg::Bool suction_msg; 

                // Call start suction 
                if (msg->buttons.at(5) == 1){

                    suction_msg.data = true;                     

                }else{

                    suction_msg.data = false; 
                }

                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Controlling small UAV!"); 
                amSCmdVelPublisher_->publish(teleop_msg); 
                amSGripperCmdSuctionPublisher_->publish(suction_msg); 

            }else{

                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Controlling large UAV!"); 
                amLCmdVelPublisher_->publish(teleop_msg); 

                // Call open gripper w
                if (msg->buttons.at(0) == 1){
                    
                    //https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
                    // https://answers.ros.org/question/340389/client-doesnt-return-when-declared-inside-c-class-in-ros-2/
                    RCLCPP_INFO_STREAM(this->get_logger(), "Opening Gripper!");
                    auto req_ = std::make_shared<std_srvs::srv::Empty::Request>();    
                    openGripperClient_->async_send_request(req_); 

                }

                // Call close gripper
                if (msg->buttons.at(2) == 1){

                    RCLCPP_INFO_STREAM(this->get_logger(), "Closing Gripper!");
                    auto req_ = std::make_shared<std_srvs::srv::Empty::Request>(); 
                    closeGripperClient_->async_send_request(req_); 

                }


            }

        }

        bool close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res){

            auto finger_pos_msg = std_msgs::msg::Float64(); 
            finger_pos_msg.data = 0.0; 

            amLGripperCmdPosLeftPublisher_->publish(finger_pos_msg); 
            amLGripperCmdPosRightPublisher_->publish(finger_pos_msg); 

            RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

            return true; 

        }

        bool open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                          std_srvs::srv::Empty::Response::SharedPtr res){

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
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         testPublisher_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     amLCmdVelPublisher_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr        amLGripperCmdPosLeftPublisher_; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     amSCmdVelPublisher_; 
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr        amLGripperCmdPosRightPublisher_; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           amSGripperCmdSuctionPublisher_; 

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      joySubscriber_; 
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  teleopSubscriber_; 

        // Services 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr          openGripperService_; 
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr          closeGripperService_;

        // Clients 
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr           openGripperClient_; 
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr           closeGripperClient_;  

        // * Hello World count
        size_t count_; 
        int operation_mode; 
};

int main(int argc, char * argv [])
{

    rclcpp::init(argc, argv); 

    // Create node 
    rclcpp::Node::SharedPtr node = std::make_shared<UavJoyCtl>(); 
    
    // Add multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node); 
    executor.spin();

    rclcpp::shutdown(); 
    return 0; 

}

