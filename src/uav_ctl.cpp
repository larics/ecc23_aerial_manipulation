#include "uav_ctl.hpp"


// How to call this method with a param? 
UavCtl::UavCtl(): Node("uav_ctl")
{
 
    // Initalize 
    init(); 

    init_ctl(); 

    roll = 0.0; 
    pitch = 0.0; 
    cmdYaw_ = 0.0; 
    nodeInitialized = true; 

}

void UavCtl::init()
{   
    // Take node namespace as uav_name (easiest way to capture ns param)
    ns_ = this->get_namespace(); 	

    // Parameters
    this->declare_parameter<std::string>("world_name", "coast");
    this->get_parameter("world_name", world_name_);
    
    // Publishers 
    cmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>(ns_ + std::string("/cmd_vel"), 1); 
    poseGtPub_             = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1); 
    gripperCmdPosLeftPub_  = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_left/cmd_pos"), 1); 
    gripperCmdPosRightPub_ = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_right/cmd_pos"), 1); 
    gripperCmdSuctionPub_  = this->create_publisher<std_msgs::msg::Bool>(ns_ + std::string("/gripper/suction_on"), 1); 
    absPoseDistPub_        = this->create_publisher<geometry_msgs::msg::Pose>(ns_ + std::string("/pose_dist"), 1); 
    // suction_related
    fullSuctionContactPub_ = this->create_publisher<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/all"), 1); 

    // Subscribers
    poseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>(ns_ + std::string("/pose_static"), 1, std::bind(&UavCtl::pose_callback, this, _1));
    currPoseSub_           = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1, std::bind(&UavCtl::curr_pose_callback, this, _1)); 
    cmdPoseSub_            = this->create_subscription<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>(ns_ + std::string("/pose_ref"), 1, std::bind(&UavCtl::cmd_pose_callback, this, _1)); 
    // suction_related
    bottomContactSub_      = this->create_subscription<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/bottom"), 1, std::bind(&UavCtl::bottom_contact_callback, this, _1)); 
    leftContactSub_        = this->create_subscription<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/left"), 1, std::bind(&UavCtl::left_contact_callback, this, _1)); 
    rightContactSub_       = this->create_subscription<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/right"), 1, std::bind(&UavCtl::right_contact_callback, this, _1)); 
    topContactSub_         = this->create_subscription<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/top"), 1, std::bind(&UavCtl::top_contact_callback, this, _1)); 
    centerContactSub_      = this->create_subscription<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/center"), 1, std::bind(&UavCtl::center_contact_callback, this, _1)); 
    
    // Services
    openGripperSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper"), std::bind(&UavCtl::open_gripper, this, _1, _2)); 
    closeGripperSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"),  std::bind(&UavCtl::close_gripper, this, _1, _2)); 
    startSuctionSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/start_suction"), std::bind(&UavCtl::start_suction, this, _1, _2)); 
    stopSuctionSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/stop_suction"), std::bind(&UavCtl::stop_suction, this, _1, _2)); 

    //Servoing state 
    detObjSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(std::string("/hsv_filter/detected_point"), 1, std::bind(&UavCtl::det_obj_callback, this, _1)); 


    // TF
    staticPoseTfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initial position
    cmdPose_.pose.position.x = 0.0; 
    cmdPose_.pose.position.y = 0.0; 
    cmdPose_.pose.position.z = 0.5; 

    // Initial orientation
    cmdPose_.pose.orientation.x = 0.0; cmdPose_.pose.orientation.y = 0.0; 
    cmdPose_.pose.orientation.z = 0.0; cmdPose_.pose.orientation.w = 1.0; 

    // Set contacts on false
    bottomC = false; topC = false; leftC = false; rightC = false; centerC = false; 

    // possible to use milliseconds and duration
    // TODO: Add as reconfigurable param
    std::chrono::duration<double> SYSTEM_DT(0.05);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&UavCtl::timer_callback, this)); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!");
}

void UavCtl::init_ctl()
{

    // Controller -> set pid with (kp, ki, kd) gains    
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up controllers!");
    
    // Height controller
    // TODO: Config I 
    pid.kp = 1; pid.ki = 0.01;  pid.kd = 0.01; 
    config.windup_limit = 5.0;
    config.upper_limit = 9.0; 
    config.lower_limit = -2.0;  
    z_controller_.set_pid(std::move(pid)); 
    z_controller_.set_config(std::move(config)); 
    z_controller_.set_plant_state(0);

    // Horizontal - x controller
    pid.kp = 0.5; pid.ki = 0.0; pid.kd = 0.0; 
    x_controller_.set_pid(std::move(pid)); 
    x_controller_.set_config(std::move(config)); 
    x_controller_.set_plant_state(0);

    // Horizontal - y controller 
    pid.kp = 0.5; pid.ki = 0.0; pid.kd = 0.0;  
    y_controller_.set_pid(std::move(pid)); 
    y_controller_.set_config(std::move(config)); 
    y_controller_.set_plant_state(0);  

    pid.kp = 1.0; pid.ki = 0.0; pid.kd = 0.0; 
    yaw_controller_.set_pid(std::move(pid));
    yaw_controller_.set_plant_state(0); 

    /*
    Gains 
    ----------
    How to configure gains for normal flight? 
    Small I and small D lead reference to instability? 
    Which PID to use? 
    */
    // yaw controller; 

}

void UavCtl::curr_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{   
    // TODO: Fix this part!
    currPose_.header.frame_id = msg->header.frame_id;
    currPose_.pose.position.x = msg->pose.position.x;  
    currPose_.pose.position.y = msg->pose.position.y;
    currPose_.pose.position.z = msg->pose.position.z;
    currPose_.pose.orientation.x = msg->pose.orientation.x; 
    currPose_.pose.orientation.y = msg->pose.orientation.y; 
    currPose_.pose.orientation.z = msg->pose.orientation.z; 
    currPose_.pose.orientation.w = msg->pose.orientation.w; 

    tf2::Quaternion q(currPose_.pose.orientation.x, 
                      currPose_.pose.orientation.y, 
                      currPose_.pose.orientation.z, 
                      currPose_.pose.orientation.w); 
    
    tf2::Matrix3x3 m(q);

    //RCLCPP_INFO_STREAM(this->get_logger(), "Current rotational matrix is: " << m); 
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);   

    currentYaw_ = yaw; 

    // Output yaw (needed for pose estimation)
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current yaw is: " << yaw); 


}

void UavCtl::cmd_pose_callback(const mbzirc_aerial_manipulation_msgs::msg::PoseEuler::SharedPtr msg) 
{   
    RCLCPP_INFO_STREAM(this->get_logger(), "Recieved cmd_pose!"); 
    // TODO: Fix this part! --> missess orientation check
    // Add time check to publish poseDist_ message if there's no command for 5 secs
    // cmdPose_.header.frame_id = msg->header.frame_id; 
    // Currently no header in CMD message
    cmdPose_.pose.position.x = msg->position.x; 
    cmdPose_.pose.position.y = msg->position.y; 
    cmdPose_.pose.position.z = msg->position.z; 

    // TODO: Determine if commanded in radians or degrees
    cmdYaw_ = msg->heading.data; 

    cmdReciv = true; 
}

void UavCtl::pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) 
{       

        geometry_msgs::msg::TransformStamped transform_stamped;
        bool pose_estimate = false; 
        bool check_complete = false; 
        // Get uav ns
        std::string uav_ns = this->ns_;
        // Remove backslash 
        uav_ns.erase(0, 1); 

        for (int i = 0; i < static_cast<int>(std::size(msg->transforms)); ++i) {
            // https://www.theconstructsim.com/ros-qa-045-publish-subscribe-array-vector-message/
            geometry_msgs::msg::TransformStamped transform_msg; 
            transform_msg = msg->transforms.at(i); 

            // tf's
            std::string frame_id; std::string child_frame_id;  
            frame_id        = transform_msg.header.frame_id; 
            child_frame_id  = transform_msg.child_frame_id; 

            // publish pose_gt for this uav
            if (frame_id == world_name_ && child_frame_id == uav_ns) 
            {   
                pose_estimate = true; 
                geometry_msgs::msg::PoseStamped msg; 

                // This can also be ground truth pose
                msg.header = transform_msg.header; 
                msg.header.frame_id = child_frame_id; 
                msg.pose.position.x = transform_msg.transform.translation.x; 
                msg.pose.position.y = transform_msg.transform.translation.y; 
                msg.pose.position.z = transform_msg.transform.translation.z; 
                msg.pose.orientation = transform_msg.transform.rotation; 

                poseGtPub_->publish(msg); 
                
                // Also broadcast as tf
                staticPoseTfBroadcaster_->sendTransform(transform_msg);

                break; 

            }; 

            check_complete = true; 

        }
        // publish warning if there's no pose estimate
        if(!pose_estimate && check_complete){
            RCLCPP_WARN(this->get_logger(), "No pose estimate found for %s.",  uav_ns.c_str()); 
        }

}

void UavCtl::det_obj_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)

{

    // Timestamp comparison

    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved first detected_obj_callback"); 
    detObjPose_.header = msg->header; 
    detObjPose_.point.x = msg->point.z;
    detObjPose_.point.y = msg->point.y; 
    detObjPose_.point.z = - msg->point.x; 

    float x = detObjPose_.point.x; 
    float y = detObjPose_.point.y;
    float z = detObjPose_.point.z; 

    bool cond_x = std::abs(x) < 1.0;
    bool cond_y = std::abs(y) < 1.0;

    RCLCPP_INFO_STREAM(this->get_logger(), "x: " << std::abs(x) << "\ny: " << std::abs(y) << "\nz" << z); 
    //RCLCPP_INFO_STREAM(this->get_logger(), "abs dist y" << std::abs(y)); 


    if(cond_x && cond_y ){
        current_state_= SERVOING; 
    }

}

void UavCtl::bottom_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved bottom suction"); 
    bottomC = msg->data; 
}

void UavCtl::left_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved left suction"); 
    leftC = msg->data; 
}

void UavCtl::right_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved right suction"); 
    rightC = msg->data; 
}

void UavCtl::center_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved center suction"); 
    centerC = msg->data; 
}

void UavCtl::top_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved top suction"); 
    topC = msg->data; 
}

bool UavCtl::close_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                              std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.0; 

    gripperCmdPosLeftPub_->publish(finger_pos_msg); 
    gripperCmdPosRightPub_->publish(finger_pos_msg); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Closing gripper!");

    return true; 
}
bool UavCtl::open_gripper(const std_srvs::srv::Empty::Request::SharedPtr req, 
                            std_srvs::srv::Empty::Response::SharedPtr res)
{

    auto finger_pos_msg = std_msgs::msg::Float64(); 
    finger_pos_msg.data = 0.5; 

    gripperCmdPosLeftPub_->publish(finger_pos_msg); 
    gripperCmdPosRightPub_->publish(finger_pos_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Opening gripper!");

    return true;  
        
}

bool UavCtl::start_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                           std_srvs::srv::Empty::Response::SharedPtr res)
{
    bool start_suction = true; 

    std_msgs::msg::Bool msg; 
    msg.data = start_suction; 

    gripperCmdSuctionPub_->publish(msg);
    
}

bool UavCtl::stop_suction(const std_srvs::srv::Empty::Request::SharedPtr req, 
                          std_srvs::srv::Empty::Response::SharedPtr res)
{
    bool start_suction = false; 

    std_msgs::msg::Bool msg; 
    msg.data = start_suction; 

    gripperCmdSuctionPub_->publish(msg); 


}

void UavCtl::get_pose_dist()
{

    poseDist_.position.x = std::abs(cmdPose_.pose.position.x - currPose_.pose.position.x);
    poseDist_.position.y = std::abs(cmdPose_.pose.position.y - currPose_.pose.position.y);
    poseDist_.position.z = std::abs(cmdPose_.pose.position.z - currPose_.pose.position.z);
    // TODO: Figure out what happens to orientation
    // Check heading differences!

}

double UavCtl::calculate_yaw_setpoint()
{
  auto                  yawRef = getCmdYaw();
  auto                  yawMv  = getCurrentYaw();
  static constexpr auto tol    = M_PI;

  // Compensate for wrapping to [-PI, PI]
  if (yawRef - yawMv > tol) {
    yawRef -= 2 * M_PI;
  } else if (yawRef - yawMv < -tol) {
    yawRef += 2 * M_PI;
  }

  return yawRef; 
}


// Timer callback executes every 1.0/0.2 (5s)
void UavCtl::timer_callback()
{
    // TODO: Add control here for PID control :) 
    // get current uav_state
    double cmd_x; double cmd_y; double cmd_z; double cmd_yaw; 
    geometry_msgs::msg::Pose msg; 

    // this PID is used only for controlling z axis
    if (nodeInitialized){
        
        geometry_msgs::msg::Twist cmdVel_;

        if(cmdReciv && current_state_ == POSITION){
        // TODO: Add in fuctions!    
        // Publish current pose difference
        get_pose_dist(); 
        absPoseDistPub_->publish(poseDist_);
        
        // Using only update as suggested in jlbpid docs results in threading issue
        x_controller_.set_plant_state(currPose_.pose.position.x); 
        x_controller_.set_setpoint(cmdPose_.pose.position.x); 
        x_controller_.update(); 
        cmd_x = x_controller_.get_control_effort(); 

        y_controller_.set_plant_state(currPose_.pose.position.y); 
        y_controller_.set_setpoint(cmdPose_.pose.position.y);
        y_controller_.update();
        cmd_y = y_controller_.get_control_effort(); 

        z_controller_.set_plant_state(currPose_.pose.position.z); 
        z_controller_.set_setpoint(cmdPose_.pose.position.z);  
        z_controller_.update();
        cmd_z = z_controller_.get_control_effort(); 

        // RCLCPP_INFO_STREAM(this->get_logger(), "cmd yaw: " << calculate_yaw_setpoint()); 
        yaw_controller_.set_plant_state(getCurrentYaw());
        yaw_controller_.set_setpoint(calculate_yaw_setpoint()); 
        yaw_controller_.update(); 
        cmd_yaw = yaw_controller_.get_control_effort(); 

        // RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << cmdPose_.pose.position.z); 
        // RCLCPP_INFO_STREAM(this->get_logger(), "curr z: " << currPose_.pose.position.z); 
        // RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << z_controller_.get_control_effort());  

        // RCLCPP_INFO_STREAM(this->get_logger(), "Current yaw is: " << getCurrentYaw());      
        // RCLCPP_INFO_STREAM(this->get_logger(), "cos(yaw)" << cos(getCurrentYaw())); 
        // RCLCPP_INFO_STREAM(this->get_logger(), "sin(yaw)" << sin(getCurrentYaw())); 
    
        cmdVel_.linear.x = cmd_x * cos(getCurrentYaw()) + cmd_y * sin(getCurrentYaw());  
        cmdVel_.linear.y = cmd_y * cos(getCurrentYaw()) - cmd_x * sin(getCurrentYaw()) ; 
        cmdVel_.linear.z = cmd_z; 

        cmdVel_.angular.x = 0; 
        cmdVel_.angular.y = 0; 
        cmdVel_.angular.z = cmd_yaw;   

        cmdVelPub_->publish(cmdVel_);


        }


        if (current_state_ == SERVOING){

            RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Servoing on object!"); 

            // P gain
            float Kp_xy = 0.5;
            float Kp_z = 0.5; 
            float limit_xy = 0.5; 
            float limit_z = 0.4; 

            // Align x, y
            double cmd_x = - Kp_xy * (0 - detObjPose_.point.x); 
            limitCommand(cmd_x, limit_xy);
            cmdVel_.linear.x = cmd_x;  
            double cmd_y = - Kp_xy * (0 - detObjPose_.point.y); 
            limitCommand(cmd_y, limit_xy);
            cmdVel_.linear.y = cmd_y; 

            // Send z
            if(std::abs(detObjPose_.point.x) < 0.1 && std::abs(detObjPose_.point.y < 0.1))
            {
                double cmd_z = Kp_z * (0.35 - std::abs(detObjPose_.point.z));
                limitCommand(cmd_z, limit_z); 
                cmdVel_.linear.z = cmd_z;
            }
            
            if (std::abs(detObjPose_.point.z) < 0.3) {
                cmdVel_.linear.x = 0.0; 
                cmdVel_.linear.y = 0.0; 
                cmdVel_.linear.z = 0.0;             
                current_state_ = APPROACH; 
               
            }



        }

        if (current_state_ == APPROACH)
        {   
            cmdVel_.linear.z = -0.2; 
            current_state_ == GRASP
            
        }

        if (current_state_ == GRASP){

            suction_msg.data = true; 
            fullSuctionContactPub_->publish(suction_msg); 
            double cmd_z = Kp_z * (3.0 - currPose_.pose.position.z);

        }

        cmdVelPub_->publish(cmdVel_); 


        // Publish speed



        std_msgs::msg::Bool suction_msg; 
        // Publish contacts if all 5 are touching object
        //RCLCPP_INFO_STREAM(this->get_logger(), "bottom: " << bottomC);

        // Check during service call and return true or false depends on current value
        if(bottomC && rightC && leftC && topC && centerC)
        {
            suction_msg.data = true; 
            fullSuctionContactPub_->publish(suction_msg); 
            //gripperCmdSuctionPub_->publish(suction_msg); 
            RCLCPP_INFO(this->get_logger(),"Suction ready!");    

        }else{
            suction_msg.data = false; 
            fullSuctionContactPub_->publish(suction_msg); 
            //RCLCPP_INFO(this->get_logger(),"Suction not ready!");  
        }          
       
    } 

}   


// GETTERS
float UavCtl::getCurrentYaw()
{
    return currentYaw_; 
}

float UavCtl::getCmdYaw()
{
    return cmdYaw_; 
}

void UavCtl::limitCommand(double& cmd, double limit)
{
    if (cmd > limit)
    {
        cmd = limit;
    }

    if (cmd < - limit) {
        cmd = -limit; 
    }
}



