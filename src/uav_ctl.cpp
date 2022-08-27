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

    // initialize_parameters
    init_params(); 
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UavCtl::parametersCallback, this, std::placeholders::_1));

    // Callback groups
    takeoff_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
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
    dockingFinishedSub_    = this->create_subscription<std_msgs::msg::Bool>("docking_finished", 1, std::bind(&UavCtl::docking_finished_callback, this, _1)); 
    
    // Services
    openGripperSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/open_gripper"), std::bind(&UavCtl::open_gripper, this, _1, _2)); 
    closeGripperSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/close_gripper"),  std::bind(&UavCtl::close_gripper, this, _1, _2)); 
    startSuctionSrv_          = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/start_suction"), std::bind(&UavCtl::start_suction, this, _1, _2)); 
    stopSuctionSrv_           = this->create_service<std_srvs::srv::Empty>(ns_ + std::string("/stop_suction"), std::bind(&UavCtl::stop_suction, this, _1, _2)); 
    changeStateSrv_           = this->create_service<mbzirc_aerial_manipulation_msgs::srv::ChangeState>(ns_ + std::string("/change_state"), std::bind(&UavCtl::change_state, this, _1, _2)); 
    takeoffSrv_               = this->create_service<mbzirc_aerial_manipulation_msgs::srv::Takeoff>(ns_ + std::string("/takeoff"), std::bind(&UavCtl::take_off, this, _1, _2), rmw_qos_profile_services_default, takeoff_group_);
    
    // Object detection
    detObjSub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(std::string("/hsv_filter/detected_point"), 1, std::bind(&UavCtl::det_obj_callback, this, _1)); 
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

void UavCtl::init_params()
{
    // Parameters
    // TODO: Add cfg with init params 
    this->declare_parameter<std::string>("world_name", "simple_demo");
    this->get_parameter("world_name", world_name_);
    this->declare_parameter<float>("Kp_h", 1.0); 
    this->get_parameter("Kp_h", Kp_h); 
    this->declare_parameter<float>("Kd_h", 0.05); 
    this->get_parameter("Kd_h", Kp_h); 
    this->declare_parameter<float>("Kp_x", 1.0); 
    this->get_parameter("Kp_x", Kp_x); 
    this->declare_parameter<float>("Kd_x", 0.05); 
    this->get_parameter("Kd_x", Kd_x); 
    this->declare_parameter<float>("Kp_y", 1.0); 
    this->get_parameter("Kp_y", Kp_y); 
    this->declare_parameter<float>("Kd_y", 0.05); 
    this->get_parameter("Kd_y", Kd_y); 
    this->declare_parameter<float>("Kp_yaw", 0.05); 
    this->get_parameter("Kp_yaw", Kd_y); 
    this->declare_parameter<float>("Kd_yaw", 0.05); 
    this->get_parameter("Kd_yaw", Kd_y); 
}

rcl_interfaces::msg::SetParametersResult UavCtl::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        // Here update class attributes, do some actions, etc.

        for (const auto &param: parameters)
        {
            if (param.get_name() == "Kp_h")
                Kp_h = param.as_double();
            else if (param.get_name() == "Kd_h")
                Kp_h = param.as_double();
            else if (param.get_name() == "Kp_x")
                Kp_x = param.as_double();
            else if (param.get_name() == "Kd_x")
                Kd_x = param.as_double();
            else if (param.get_name() == "Kp_y")
                Kp_y = param.as_double();
            else if (param.get_name() == "Kd_y")
                Kd_y = param.as_double();
            else if (param.get_name() == "Kp_yaw")
                Kd_y = param.as_double();
            else if (param.get_name() == "Kd_yaw")
                Kd_y = param.as_double();
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Parameters updated!");

        return result;
    }

void UavCtl::init_ctl()
{

    // Controller -> set pid with (kp, ki, kd) gains    
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up controllers!");
    
    // UAV position control ---> NO OBJECT
    config.windup_limit = 5.0;
    config.upper_limit = 9.0; 
    config.lower_limit = -2.0;  
    pid.kp = Kp_h; pid.ki = 0.0;  pid.kd = Kd_h; 
    setPidController(z_controller_, pid, config); 

    // Horizontal - x controller
    pid.kp = Kp_x; pid.ki = 0.0; pid.kd = Kd_x; 
    setPidController(x_controller_, pid, config); 

    // Horizontal - y controller 
    pid.kp = Kp_y; pid.ki = 0.0; pid.kd = Kd_y;  
    setPidController(y_controller_, pid, config); 

    pid.kp = Kp_yaw; pid.ki = 0.0; pid.kd = Kd_yaw; 
    setPidController(yaw_controller_, pid, config); 

    // UAV position control ---> WITH OBJECT
    config.windup_limit = 2.0;
    config.upper_limit = 1.0; 
    config.lower_limit = -1.0;  
    pid.kp = 1.0; pid.ki = 0.1; pid.kd = 0.0; 
    setPidController(x_drop_controller_, pid, config); 

    pid.kp = 1.0; pid.ki = 0.1; pid.kd = 0.0; 
    setPidController(y_controller_, pid, config); 

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

void UavCtl::magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    // https://answers.ros.org/question/385299/imu-with-magnetometer-read-absolute-position/
    // https://digilent.com/blog/how-to-convert-magnetometer-data-into-compass-heading/
    double mag_x, mag_y, mag_z; 
    mag_x = msg->magnetic_field.x; 
    mag_y = msg->magnetic_field.y; 
    mag_z = msg->magnetic_field.z; 

    if(mag_x != 0){
        magHeadingRad_ = atan2(mag_y, mag_x);
        magHeadingDeg_ = magHeadingRad_ * 180/M_PI; 
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
    if(current_state_ == DROP || current_state_ == LIFT || current_state_ == GO_TO_DROP)
    {
        usvPosReciv = true;
        dropOffPoint_.header = msg->header;  
        dropOffPoint_.point.x = msg->point.x;
        dropOffPoint_.point.y = msg->point.y; 
        dropOffPoint_.point.z = msg->point.z; 
    }
}

void UavCtl::docking_finished_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // Maybe check time 
    usvFinishedDocking_ = true; 
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

bool UavCtl::take_off(const mbzirc_aerial_manipulation_msgs::srv::Takeoff::Request::SharedPtr req, 
                          mbzirc_aerial_manipulation_msgs::srv::Takeoff::Response::SharedPtr res)
{
    // Set takeoff commanded position.
    auto cmd_pos = std::make_shared<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>();
    cmd_pos->position.x = currPose_.pose.position.x;
    cmd_pos->position.y = currPose_.pose.position.y;
    cmd_pos->position.z = currPose_.pose.position.z + req->relative_height;
    cmd_pos->heading.data = getCurrentYaw();
    cmd_pose_callback(cmd_pos);
    get_pose_dist();

    // Wait until position is reached.
    while (poseError_.abs_position > 0.1);

    // Change state and report back
    current_state_ = IDLE;

    res->success = true;

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
            positionControl(cmdVel_); 
        }
       
        if (current_state_ == SERVOING)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Servoing on object!"); 
            // P gain
            float Kp_xy = 0.5;  // servo gains
            float limit_xy = 0.5;
            double cmd_x = - calcPropCmd(Kp_xy, 0, detObjPose_.point.x, limit_xy); 
            double cmd_y = - calcPropCmd(Kp_xy, 0, detObjPose_.point.y, limit_xy); 
            cmdVel_.linear.x = cmd_x;
            cmdVel_.linear.y = cmd_y; 
            // Send z
            if (usvFinishedDocking_ && std::abs(detObjPose_.point.x) < 0.1 && std::abs(detObjPose_.point.y < 0.1))
            {
                current_state_= APPROACH; 
            }
        }

        if (current_state_ == APPROACH){
            
            float Kp_z = 1.5; float limit_z = 0.5; // servo limits
            float Kp_xy = 0.5;  // servo gains
            float limit_xy = 0.5;
            double cmd_x = - calcPropCmd(Kp_xy, 0, detObjPose_.point.x, limit_xy); 
            double cmd_y = - calcPropCmd(Kp_xy, 0, detObjPose_.point.y, limit_xy); 
            double cmd_z = calcPropCmd(Kp_z, 0.0, std::abs(detObjPose_.point.z), limit_z); 
            cmdVel_.linear.x = cmd_x;
            cmdVel_.linear.y = cmd_y;  
            cmdVel_.linear.z = cmd_z;
                
            if (std::abs(detObjPose_.point.z) < 0.4) 
            {
            cmdVel_.linear.x = 0.0; 
            cmdVel_.linear.y = 0.0; 
            cmdVel_.linear.z = -1.0; 

            if (checkContacts()) current_state_ = PRE_GRASP;   
            }

        }
                 
        if (current_state_ == PRE_GRASP)
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
            float Kp_z = 2.0; float Kp_yaw = 0.5; 
            cmd_z = calcPropCmd(Kp_z, 8.0, currPose_.pose.position.z, 2.0); 
            cmd_yaw = calcPropCmd(Kp_yaw, 0.0, imuMeasuredYaw_, 0.25); 
            cmd_x = 0.0; cmd_y = 0.0; 
            //RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredPitch_ = " << imuMeasuredPitch_ << "\n");
            //RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredRoll_ = " << imuMeasuredRoll_ << "\n");

            cmdVel_.linear.x = cmd_x; 
            cmdVel_.linear.y = cmd_y;  
            cmdVel_.linear.z = cmd_z; 
            cmdVel_.angular.z = cmd_yaw; 

            if (std::abs(cmd_yaw) < 0.05 && usvPosReciv)
            {
                cmdVel_.linear.x = 0.0; 
                cmdVel_.linear.y = 0.0;  
                cmdVel_.linear.z = 0.0; 
                cmdVel_.angular.z = 0.0; 
                current_state_ = GO_TO_DROP; 
            }

        }

        if (current_state_ == GO_TO_DROP)
        {
            /*
            RCLCPP_INFO_ONCE(this->get_logger(), "[GO_TO_DROP] Active!");
            // Compare time to know when time recieved
            double time_diff = this->get_clock()->now().seconds() - dropOffPoint_.header.stamp.sec;
            RCLCPP_INFO_STREAM(this->get_logger(), "time diff  = " << time_diff << "\n");
            RCLCPP_INFO_STREAM(this->get_logger(), "stamp  = " << dropOffPoint_.header.stamp.sec << "\n");
            RCLCPP_INFO_STREAM(this->get_logger(), "clock now  = " << this->get_clock()->now().seconds() << "\n");
            */
            double time_diff = 0.0;

            if(usvPosReciv && time_diff < 0.5){
                // goToPose with heading
                // Add time check and possible timeout
                // Could basically reuse SERVOING state (however than we have to have determination which SERVOING it is)
                // Align x, y
                float Kp_z = 0.5; // servo gains
                float Kp_yaw = 0.5; 
                float limit_z = 0.4; // servo limits 
                double cmd_x = -calcPidCmd(x_drop_controller_, 0, dropOffPoint_.point.x); 
                double cmd_y = -calcPidCmd(y_drop_controller_, 0, dropOffPoint_.point.y); 
                cmd_yaw = calcPropCmd(Kp_yaw, 0.0, imuMeasuredYaw_, 0.25); 
                
                //RCLCPP_INFO_STREAM(this->get_logger(), "detected point  = " << dropOffPoint_.point.x << ", " << dropOffPoint_.point.y);
                //RCLCPP_INFO_STREAM(this->get_logger(), "velocity  = " << cmd_x << ", " << cmd_y);

                cmdVel_.linear.x = cmd_x;
                cmdVel_.linear.y = cmd_y; 
                cmdVel_.linear.z = 0.0; 
                cmdVel_.angular.z = cmd_yaw; 

                //RCLCPP_INFO_STREAM(this->get_logger(), "imuMeasuredYaw_ = " << imuMeasuredYaw_ << "\n");
                // Send z
                if(std::abs(dropOffPoint_.point.x) < 1 && std::abs(dropOffPoint_.point.y < 1))
                {   
                    current_state_ = DROP; 
                    Kp_z = 3.0;
                    limit_z = 2.0;
                    double cmd_z = calcPropCmd(Kp_z, 0.35, std::abs(dropOffPoint_.point.z), limit_z); 
                    cmdVel_.linear.z = cmd_z;
                }
                /*
                if (std::abs(dropOffPoint_.point.z) < 0.3) {
                    cmdVel_.linear.x = 0.0; 
                    cmdVel_.linear.y = 0.0; 
                    cmdVel_.linear.z = 0.0;             
                    current_state_ = DROP; 
                } 
                */ 
            }
            else 
            {
                cmdVel_.linear.x = 0.0;
                cmdVel_.linear.y = 0.0; 
                cmdVel_.linear.z = 0.0; 
                cmdVel_.angular.z = 0.0; 
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

double UavCtl::calcPidCmd(jlbpid::Controller& controller, double cmd_sp, double cmd_mv)
{
    // Using only update as suggested in jlbpid docs results in threading issue
    controller.set_plant_state(cmd_mv); 
    controller.set_setpoint(cmd_sp); 
    controller.update(); 

    return controller.get_control_effort(); 
}

void UavCtl::setPidController(jlbpid::Controller& controller, jlbpid::PID pid, jlbpid::Config config)
{
    controller.set_pid(std::move(pid));
    controller.set_config(std::move(config));    
    controller.set_plant_state(0); 
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


// State ctl 
void UavCtl::positionControl(geometry_msgs::msg::Twist& cmdVel)
{
    double cmd_x, cmd_y, cmd_z, cmd_yaw; 
    RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Position control!"); 
        
    // Publish current pose difference
    get_pose_dist(); 
    absPoseDistPub_->publish(poseError_);
        
    cmd_x = calcPidCmd(x_controller_, cmdPose_.pose.position.x, currPose_.pose.position.x); 
    cmd_y = calcPidCmd(y_controller_, cmdPose_.pose.position.y, currPose_.pose.position.y); 
    cmd_z = calcPidCmd(z_controller_, cmdPose_.pose.position.z, currPose_.pose.position.z); 
    cmd_yaw = calcPidCmd(yaw_controller_, calculate_yaw_setpoint(), getCurrentYaw()); 

    // RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << cmdPose_.pose.position.z); 
    // RCLCPP_INFO_STREAM(this->get_logger(), "curr z: " << currPose_.pose.position.z); 
    // RCLCPP_INFO_STREAM(this->get_logger(), "cmd z: " << z_controller_.get_control_effort());  
    
    cmdVel.linear.x = cmd_x * cos(getCurrentYaw()) + cmd_y * sin(getCurrentYaw());  
    cmdVel.linear.y = cmd_y * cos(getCurrentYaw()) - cmd_x * sin(getCurrentYaw()) ; 
    cmdVel.linear.z = cmd_z; 
    cmdVel.angular.x = 0; 
    cmdVel.angular.y = 0; 
    cmdVel.angular.z = cmd_yaw;       

}

/*
bool UavCtl::switchToState(state switch_state)
{
    current_state_ = state; 
}
*/