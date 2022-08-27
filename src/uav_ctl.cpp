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
    this->declare_parameter<std::string>("world_name", "simple_demo");
    this->get_parameter("world_name", world_name_);
    
    // Publishers 
    cmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>(ns_ + std::string("/cmd_vel"), 1); 
    poseGtPub_             = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1); 
    gripperCmdPosLeftPub_  = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_left/cmd_pos"), 1); 
    gripperCmdPosRightPub_ = this->create_publisher<std_msgs::msg::Float64>(ns_ + std::string("/gripper/joint/finger_right/cmd_pos"), 1); 
    gripperCmdSuctionPub_  = this->create_publisher<std_msgs::msg::Bool>(ns_ + std::string("/gripper/suction_on"), 1); 
    currentStatePub_       = this->create_publisher<std_msgs::msg::String>(ns_ + std::string("/state"), 1); 
    absPoseDistPub_        = this->create_publisher<mbzirc_aerial_manipulation_msgs::msg::PoseError>(ns_ + std::string("/pose_dist"), 1); 
    // suction_related
    fullSuctionContactPub_ = this->create_publisher<std_msgs::msg::Bool>(ns_ + std::string("/gripper/contacts/all"), 1); 


    // Subscribers
    poseSub_               = this->create_subscription<tf2_msgs::msg::TFMessage>(ns_ + std::string("/pose_static"), 1, std::bind(&UavCtl::pose_callback, this, _1));
    poseSubUsv_            = this->create_subscription<tf2_msgs::msg::TFMessage>(std::string("/usv/pose_static"), 1, std::bind(&UavCtl::pose_callback_usv, this, _1));
    currPoseSub_           = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns_ + std::string("/pose_gt"), 1, std::bind(&UavCtl::curr_pose_callback, this, _1)); 
    cmdPoseSub_            = this->create_subscription<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>(ns_ + std::string("/pose_ref"), 1, std::bind(&UavCtl::cmd_pose_callback, this, _1)); 

    imuSub_ 		       = this->create_subscription<sensor_msgs::msg::Imu>(ns_ + std::string("/imu/data"), 1, std::bind(&UavCtl::imu_callback, this, _1)); 
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
    changeStateSrv_           = this->create_service<mbzirc_aerial_manipulation_msgs::srv::ChangeState>(ns_ + std::string("/change_state"), std::bind(&UavCtl::change_state, this, _1, _2)); 

    //Servoing state 
    detObjSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(std::string("/hsv_filter/detected_point"), 1, std::bind(&UavCtl::det_obj_callback, this, _1)); 

    // USV integration (TODO: Think how to decouple control and integration)
    usvDropPoseSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(std::string("/drone_detection/detected_point"), 1, std::bind(&UavCtl::det_uav_callback, this, _1)); 

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
    // Add time check to publish poseError_ message if there's no command for 5 secs
    // cmdPose_.header.frame_id = msg->header.frame_id; 
    // Currently no header in CMD message
    cmdPose_.pose.position.x = msg->position.x; 
    cmdPose_.pose.position.y = msg->position.y; 
    cmdPose_.pose.position.z = msg->position.z; 

    // TODO: Determine if commanded in radians or degrees
    cmdYaw_ = msg->heading.data; 

    cmdReciv = true; 
    current_state_= POSITION; 

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


void UavCtl::pose_callback_usv(const tf2_msgs::msg::TFMessage::SharedPtr msg) 
{       

        geometry_msgs::msg::TransformStamped transform_stamped;
        bool pose_estimate = false; 
        bool check_complete = false; 

        for (int i = 0; i < static_cast<int>(std::size(msg->transforms)); ++i) {
            // https://www.theconstructsim.com/ros-qa-045-publish-subscribe-array-vector-message/
            geometry_msgs::msg::TransformStamped transform_msg; 
            transform_msg = msg->transforms.at(i); 

            // tf's
            std::string frame_id; std::string child_frame_id;  
            frame_id        = transform_msg.header.frame_id; 
            child_frame_id  = transform_msg.child_frame_id; 

            // publish pose_gt for this uav
            if (frame_id == world_name_ && child_frame_id == "usv") 
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
            RCLCPP_WARN(this->get_logger(), "No pose estimate found for usv"); 
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

    // RCLCPP_INFO_STREAM(this->get_logger(), "x: " << std::abs(x) << "\ny: " << std::abs(y) << "\nz" << z); 
    //RCLCPP_INFO_STREAM(this->get_logger(), "abs dist y" << std::abs(y)); 

    if(cond_x && cond_y && current_state_ == IDLE){
        current_state_= SERVOING; 
    }

}

void UavCtl::det_uav_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if(current_state_ == DROP || current_state_ == LIFT)
    {
        usvPosReciv = true;
        dropOffPoint_.header = msg->header;  
        dropOffPoint_.point.x = msg->point.x;
        dropOffPoint_.point.y = msg->point.y; 
        dropOffPoint_.point.z = msg->point.z; 
    }
}

void UavCtl::bottom_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Positioning of suction gripper is not synonimous to real positions so we 
    // assign values as they correspond to our suction gripper
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved bottom suction"); 
    rightC = msg->data; 
}

void UavCtl::left_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved left suction"); 
    topC = msg->data; 
}

void UavCtl::right_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved right suction"); 
    bottomC = msg->data; 
}

void UavCtl::center_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved center suction"); 
    centerC = msg->data; 
}

void UavCtl::top_contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved top suction"); 
    leftC = msg->data; 
}

void UavCtl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "Recieved imu"); 
    currImuData_ = *msg; 

    // this can be generalized
    tf2::Quaternion q(currImuData_.orientation.x, 
                      currImuData_.orientation.y, 
                      currImuData_.orientation.z, 
                      currImuData_.orientation.w); 
    
    tf2::Matrix3x3 m(q);

    //RCLCPP_INFO_STREAM(this->get_logger(), "Current rotational matrix is: " << m); 
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);   

    imuMeasuredPitch_ = pitch; 
    imuMeasuredRoll_ = roll; 
    imuMeasuredYaw_ = yaw; 
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

bool UavCtl::change_state(const mbzirc_aerial_manipulation_msgs::srv::ChangeState::Request::SharedPtr req, 
                          mbzirc_aerial_manipulation_msgs::srv::ChangeState::Response::SharedPtr res)
{

    // Why would this be boolean? 

    auto itr = std::find(std::begin(stateNames), std::end(stateNames), req->state);
    

    if ( itr != std::end(stateNames))
    {
        int wantedIndex_ = std::distance(stateNames, itr); 
        current_state_  = (state)wantedIndex_; 
        RCLCPP_INFO_STREAM(this->get_logger(), "Switching state!");
        res->success = true;  
    }else{
        RCLCPP_INFO_STREAM(this->get_logger(), "Failed switching to state " << req->state); 
        res->success = false; 
    } 
        

}

void UavCtl::get_pose_dist()
{
    poseError_.position.x = std::abs(cmdPose_.pose.position.x - currPose_.pose.position.x);
    poseError_.position.y = std::abs(cmdPose_.pose.position.y - currPose_.pose.position.y);
    poseError_.position.z = std::abs(cmdPose_.pose.position.z - currPose_.pose.position.z);
    poseError_.abs_position = std::sqrt(std::pow(poseError_.position.x, 2)
                                       + std::pow(poseError_.position.y, 2)
                                       + std::pow(poseError_.position.z, 2));
    auto angle_diff = std::fmod(getCmdYaw() - getCurrentYaw(), 360.0);
    if (angle_diff < -180.0) angle_diff += 360.0; 
    if (angle_diff >= 180.0) angle_diff -= 360.0;
    poseError_.heading = std::abs(angle_diff);
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

void UavCtl::timer_callback()
{
    // TODO: Add control here for PID control :) 
    // get current uav_state
    double cmd_x; double cmd_y; double cmd_z; double cmd_yaw; 
    geometry_msgs::msg::Pose msg; 
    std_msgs::msg::Bool suction_msg; 

    // this PID is used only for controlling z axis
    if (nodeInitialized){
        
        geometry_msgs::msg::Twist cmdVel_;

        if (current_state_ == POSITION)
        {

        RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Position control!"); 
        // TODO: Add in fuctions!    
        // Publish current pose difference
        get_pose_dist(); 
        absPoseDistPub_->publish(poseError_);
        
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

        if (current_state_ == SERVOING)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Servoing on object!"); 
            // P gain
            float Kp_xy = 0.5; float Kp_z = 1.5; // servo gains
            float limit_xy = 0.5; float limit_z = 0.5; // servo limits 
            double cmd_x = - calcPropCmd(Kp_xy, 0, detObjPose_.point.x, limit_xy); 
            double cmd_y = - calcPropCmd(Kp_xy, 0, detObjPose_.point.y, limit_xy); 
            cmdVel_.linear.x = cmd_x;
            cmdVel_.linear.y = cmd_y; 
            // Send z
            if(std::abs(detObjPose_.point.x) < 0.1 && std::abs(detObjPose_.point.y < 0.1))
            {
                double cmd_z = calcPropCmd(Kp_z, 0.0, std::abs(detObjPose_.point.z), limit_z); 
                cmdVel_.linear.z = cmd_z;
            }
            
            if (std::abs(detObjPose_.point.z) < 0.4) {
                cmdVel_.linear.x = 0.0; 
                cmdVel_.linear.y = 0.0; 
                //cmdVel_.linear.z = 0.0;             
                current_state_ = APPROACH;                
            }

        }

        if (current_state_ == APPROACH)
        {   
            RCLCPP_INFO_ONCE(this->get_logger(), "[APPROACH] Approaching an object!"); 
            //cmdVel_.linear.x = -0.1; 
            cmdVel_.linear.z = -1.0; 

            RCLCPP_INFO_STREAM(this->get_logger(), "[APPROACH] Current num contacts: " << getNumContacts()); 
            if (checkContacts())
            {
                current_state_ = ALIGN_GRASP; 
            }
            
        }

        if (current_state_ == ALIGN_GRASP)
        {   
            // Name is redundant atm
            RCLCPP_INFO_STREAM(this->get_logger(), "[ALIGN GRASP] in progress. "); 
            RCLCPP_INFO_STREAM(this->get_logger(), "[ALIGN GRASP] num_contacts: " << getNumContacts()); 
            // Apply constant pressure on gripper and move it left/right until 
            // on middle of a case
            cmdVel_.linear.z = -5.0; 
            if (getNumContacts() > 4){
                contactCounter_++; 
            }else{
                contactCounter_ = 0; 
            }
            if(contactCounter_> 3){
                current_state_ = GRASP; 
            }
        }

        if (current_state_ == GRASP)
        {
            float Kp_z = 0.5; 
            RCLCPP_INFO_ONCE(this->get_logger(), "[GRASP] Grasping an object!"); 
            suction_msg.data = true; 

            if (getNumContacts() > 4)
            {
                contactCounter_++; 
            }
            // uses current pose!
            if (contactCounter_ > 5 && contactCounter_ < 15)
            {   
                RCLCPP_INFO_ONCE(this->get_logger(), "[GRASP] Started sucking big time!"); 
                gripperCmdSuctionPub_->publish(suction_msg); 
                
            }
            if (contactCounter_ > 15)
            {
                current_state_ = LIFT;
            }

        }

        if (current_state_ == LIFT)
        {

            RCLCPP_INFO_ONCE(this->get_logger(), "[LIFT] active!"); 
            // Go to height 2
            float Kp_z = 1.0; float Kp_yaw = 0.5; 
            cmd_z = calcPropCmd(Kp_z, 8.0, currPose_.pose.position.z, 1.0); 
            cmd_yaw = calcPropCmd(Kp_yaw, 0.0, imuMeasuredYaw_, 0.25); 
            cmd_x = 0.0; cmd_y = 0.0; 
            RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredPitch_ = " << imuMeasuredPitch_ << "\n");
            RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredRoll_ = " << imuMeasuredRoll_ << "\n");

            cmdVel_.linear.x = cmd_x; 
            cmdVel_.linear.y = cmd_y;  
            cmdVel_.linear.z = cmd_z; 
            cmdVel_.angular.z = cmd_yaw; 

            if (std::abs(cmd_yaw) < 0.05 && usvPosReciv)
            {
                current_state_ = GO_TO_DROP; 
            }

        }

        if (current_state_ == GO_TO_DROP)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "[GO_TO_DROP] Active!");
            // Compare time to know when time recieved
            if(usvPosReciv){
                // goToPose with heading
                // Add time check and possible timeout
                // Could basically reuse SERVOING state (however than we have to have determination which SERVOING it is)
                // Align x, y
                float Kp_xy = 0.5; float Kp_z = 0.5; // servo gains
                float limit_xy = 0.5; float limit_z = 0.4; // servo limits 
                double cmd_x = calcPropCmd(Kp_xy, 0, dropOffPoint_.point.x, limit_xy); 
                double cmd_y = calcPropCmd(Kp_xy, 0, dropOffPoint_.point.y, limit_xy); 

                cmdVel_.linear.x = cmd_x;
                cmdVel_.linear.y = cmd_y; 
                cmdVel_.linear.z = 0.0; 

                RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredYaw_ = " << imuMeasuredYaw_ << "\n");
                // Send z
                if(std::abs(dropOffPoint_.point.x) < 0.1 && std::abs(dropOffPoint_.point.y < 0.1))
                {   
                    double cmd_z = calcPropCmd(Kp_z, 0.35, std::abs(dropOffPoint_.point.z), limit_z); 
                    cmdVel_.linear.z = cmd_z;
                }
                
                if (std::abs(dropOffPoint_.point.z) < 0.3) {
                    cmdVel_.linear.x = 0.0; 
                    cmdVel_.linear.y = 0.0; 
                    cmdVel_.linear.z = 0.0;             
                    current_state_ = DROP; 
                
                }    
            }
            else 
            {
                std::cout << "nema poze\n";
            }
        }

        if( current_state_ == DROP){
            RCLCPP_INFO(this->get_logger(), "[DROP] Dropping food!");
            suction_msg.data = false; 
            gripperCmdSuctionPub_->publish(suction_msg); 

        }

        cmdVelPub_->publish(cmdVel_); 
        // Publish current state
        std_msgs::msg::String state_msg; state_msg.data = stateNames[current_state_]; 
        currentStatePub_->publish(state_msg); 

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

bool UavCtl::checkContacts() const
{   
    bool contacts = bottomC || rightC || leftC || topC || centerC;

    RCLCPP_INFO_STREAM(this->get_logger(), "Contacts: " << contacts); 
    //printContacts(); 

    return contacts; 
}

int UavCtl::getNumContacts() const
{   

    int numContacts = int(bottomC) + int(rightC) + int(leftC) + int(topC) + int(centerC);

    return numContacts; 

}

double UavCtl::calcPropCmd(double Kp, double cmd_sp, double cmd_mv, double limit_command)
{
    double cmd = Kp * (cmd_sp - cmd_mv);
    limitCommand(cmd, limit_command); 
    return cmd; 
}


// redundant
void UavCtl::generateContactRef(double& cmd_x, double& cmd_y)
{   
    // trying to make an informed reference
    cmd_x = 0.0; cmd_y = 0.0;

    // go backwards
    if(topC && !bottomC){
        cmd_x = -0.2; 
    }

    // go forward
    if(!topC && bottomC){
        cmd_x = 0.2; 
    }

    // go right
    //if(!rightC && leftC){
    //    cmd_y = -0.1;
    //}

    // go left
    //if(rightC && !leftC){
    //    cmd_y = 0.1; 
    //}



}

void UavCtl::printContacts() const
{

    std::string strTopC, strBottomC, strRightC, strLeftC, strCenterC; 

    if(topC){
        strTopC = "O";
    }else{strTopC = "X";}
    if(bottomC){
        strBottomC = "O";
    }else{strBottomC = "X";}
    if(rightC){
        strRightC = "O";
    }else{strRightC = "X";}
    if(leftC){
        strLeftC = "O";
    }else{strLeftC = "X";}
    if(centerC){
        strCenterC = "O";
    }else{strCenterC = "X";}

    RCLCPP_INFO_STREAM(this->get_logger(), "\t \t" << "==========================" << "\t \t"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "\t \t" << strTopC << "\t \t"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "\t \t" << strLeftC << " " << strCenterC << " " << strRightC);
    RCLCPP_INFO_STREAM(this->get_logger(), "\t \t" << strBottomC << "\t \t"); 
    RCLCPP_INFO_STREAM(this->get_logger(), "\t \t" << "==========================" << "\t \t"); 


}



