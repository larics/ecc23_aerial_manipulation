#include "simple_uav_ctl.hpp"


// How to call this method with a param? 
SimpleUavCtl::SimpleUavCtl(): Node("simple_uav_ctl")
{
 
    // Initalize 
    init(); 

    init_ctl(); 

    roll = 0.0; 
    pitch = 0.0; 
    cmdYaw_ = 0.0; 
    nodeInitialized = true; 

}

void SimpleUavCtl::init()
{   
    // Take node namespace as uav_name (easiest way to capture ns param)
    ns_ = this->get_namespace(); 	

    // initialize_parameters
    init_params(); 
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&SimpleUavCtl::parametersCallback, this, std::placeholders::_1));

    // Callback groups
    takeoff_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Publishers 
    cmdVelPub_             = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1); 
    poseGtPub_             = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_gt", 1); 
    absPoseDistPub_        = this->create_publisher<mbzirc_aerial_manipulation_msgs::msg::PoseError>("pose_dist", 1); 

    // Subscribers
    cmdPoseSub_            = this->create_subscription<mbzirc_aerial_manipulation_msgs::msg::PoseEuler>("pose_ref", 1, std::bind(&SimpleUavCtl::cmd_pose_callback, this, _1)); 
    currOdomSub_           = this->create_subscription<nav_msgs::msg::Odometry>("odometry", 1, std::bind(&SimpleUavCtl::curr_odom_callback, this, _1)); 
    imuSub_ 		       = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&SimpleUavCtl::imu_callback, this, _1)); 
    
    // Services
    takeoffSrv_            = this->create_service<mbzirc_aerial_manipulation_msgs::srv::Takeoff>("takeoff", std::bind(&SimpleUavCtl::take_off, this, _1, _2), rmw_qos_profile_services_default, takeoff_group_);
    
    // TF
    staticPoseTfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initial position
    cmdPose_.pose.position.x = 0.0; 
    cmdPose_.pose.position.y = 0.0; 
    cmdPose_.pose.position.z = 0.5; 

    // Initial orientation
    cmdPose_.pose.orientation.x = 0.0; cmdPose_.pose.orientation.y = 0.0; 
    cmdPose_.pose.orientation.z = 0.0; cmdPose_.pose.orientation.w = 1.0; 

    // possible to use milliseconds and duration
    // TODO: Add as reconfigurable param
    std::chrono::duration<double> SYSTEM_DT(0.05);
    timer_ = this->create_wall_timer(SYSTEM_DT, std::bind(&SimpleUavCtl::timer_callback, this)); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Initialized node!");
}

void SimpleUavCtl::init_params()
{
    // Parameters
    // TODO: Add cfg with init params 
    this->declare_parameter<std::string>("world_name", "simple_demo");
    this->get_parameter("world_name", world_name_);
    this->declare_parameter<bool>("use_gt", true);
    this->get_parameter("use_gt", use_gt_);

    if (use_gt_)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Using ground truth!");
    }

    this->declare_parameter<float>("Kp_h", 0.3); 
    this->get_parameter("Kp_h", Kp_h); 
    this->declare_parameter<float>("Kd_h", 0.05); 
    this->get_parameter("Kd_h", Kd_h); 
    this->declare_parameter<float>("Kp_x", 0.3); 
    this->get_parameter("Kp_x", Kp_x); 
    this->declare_parameter<float>("Kd_x", 0.05); 
    this->get_parameter("Kd_x", Kd_x); 
    this->declare_parameter<float>("Kp_y", 0.3); 
    this->get_parameter("Kp_y", Kp_y); 
    this->declare_parameter<float>("Kd_y", 0.05); 
    this->get_parameter("Kd_y", Kd_y); 
    this->declare_parameter<float>("Kp_yaw", 2); 
    this->get_parameter("Kp_yaw", Kp_yaw); 
    this->declare_parameter<float>("Kd_yaw", 0); 
    this->get_parameter("Kd_yaw", Kd_yaw); 
}

rcl_interfaces::msg::SetParametersResult SimpleUavCtl::parametersCallback(
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
                Kd_h = param.as_double();
            else if (param.get_name() == "Kp_x")
                Kp_x = param.as_double();
            else if (param.get_name() == "Kd_x")
                Kd_x = param.as_double();
            else if (param.get_name() == "Kp_y")
                Kp_y = param.as_double();
            else if (param.get_name() == "Kd_y")
                Kd_y = param.as_double();
            else if (param.get_name() == "Kp_yaw")
                Kp_yaw = param.as_double();
            else if (param.get_name() == "Kd_yaw")
                Kd_yaw = param.as_double();
        }

        init_ctl();

        RCLCPP_INFO_STREAM(this->get_logger(), "Parameters updated!");

        return result;
    }

void SimpleUavCtl::init_ctl()
{

    // Controller -> set pid with (kp, ki, kd) gains    
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up controllers!");
    
    // UAV position control ---> NO OBJECT
    config.windup_limit = 5.0;
    config.upper_limit = 5.0; 
    config.lower_limit = -5.0;  
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

    RCLCPP_INFO_STREAM(this->get_logger(), "____________________________________" ); 
}


void SimpleUavCtl::curr_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
{   

    
    // TODO: Fix this part!
    currPose_.header.frame_id = msg->header.frame_id;
    currPose_.pose.position.x = msg->pose.pose.position.x;  
    currPose_.pose.position.y = msg->pose.pose.position.y;
    currPose_.pose.position.z = msg->pose.pose.position.z;
    currPose_.pose.orientation.x = msg->pose.pose.orientation.x; 
    currPose_.pose.orientation.y = msg->pose.pose.orientation.y; 
    currPose_.pose.orientation.z = msg->pose.pose.orientation.z; 
    currPose_.pose.orientation.w = msg->pose.pose.orientation.w; 

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

void SimpleUavCtl::cmd_pose_callback(const mbzirc_aerial_manipulation_msgs::msg::PoseEuler::SharedPtr msg) 
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


void SimpleUavCtl::magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
{
    // https://answers.ros.org/question/385299/imu-with-magnetometer-read-absolute-position/
    // https://digilent.com/blog/how-to-convert-magnetometer-data-into-compass-heading/
    double mag_x, mag_y;
    mag_x = msg->magnetic_field.x; 
    mag_y = msg->magnetic_field.y; 

    if(mag_x != 0){
        magHeadingRad_ = atan2(mag_y, mag_x);
        magHeadingDeg_ = magHeadingRad_ * 180/M_PI; 
    }
}


void SimpleUavCtl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

// service callbacks
bool SimpleUavCtl::take_off(const mbzirc_aerial_manipulation_msgs::srv::Takeoff::Request::SharedPtr req, 
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
    while (poseError_.abs_position > 0.3);

    res->success = true;
    return res->success;
}

void SimpleUavCtl::get_pose_dist()
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

double SimpleUavCtl::calculate_yaw_setpoint()
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

void SimpleUavCtl::timer_callback()
{   
    
    if (nodeInitialized){
        

        if (current_state_ == POSITION) positionControl(cmdVel_); 

        if (current_state_ != INIT_STATE)
        {
            cmdVelPub_->publish(cmdVel_); 
        }
        
    } 

}   

// getters
float SimpleUavCtl::getCurrentYaw()
{
    return currentYaw_; 
}

float SimpleUavCtl::getCmdYaw()
{
    return cmdYaw_; 
}


void SimpleUavCtl::limitCommand(double& cmd, double limit)
{
    if (cmd > limit)
    {
        cmd = limit;
    }

    if (cmd < - limit) {
        cmd = -limit; 
    }
}

double SimpleUavCtl::calcPropCmd(double Kp, double cmd_sp, double cmd_mv, double limit_command)
{
    double cmd = Kp * (cmd_sp - cmd_mv);
    limitCommand(cmd, limit_command); 
    return cmd; 
}

double SimpleUavCtl::calcPidCmd(jlbpid::Controller& controller, double cmd_sp, double cmd_mv)
{
    // Using only update as suggested in jlbpid docs results in threading issue
    controller.set_plant_state(cmd_mv); 
    controller.set_setpoint(cmd_sp); 
    controller.update(); 

    return controller.get_control_effort(); 
}

void SimpleUavCtl::setPidController(jlbpid::Controller& controller, jlbpid::PID pid, jlbpid::Config config)
{
    controller.set_pid(std::move(pid));
    controller.set_config(std::move(config));    
    controller.set_plant_state(0); 

    RCLCPP_INFO_STREAM(this->get_logger(), "Kp: " << pid.kp << " Ki: " << pid.ki << " Kd: " << pid.kd); 
}

// State control 
void SimpleUavCtl::positionControl(geometry_msgs::msg::Twist& cmdVel)
{
    RCLCPP_INFO_ONCE(this->get_logger(), "[SERVOING] Position control!"); 
        
    // Publish current pose difference
    get_pose_dist(); 
    absPoseDistPub_->publish(poseError_);
        
    double cmd_x = calcPidCmd(x_controller_, cmdPose_.pose.position.x, currPose_.pose.position.x); 
    double cmd_y = calcPidCmd(y_controller_, cmdPose_.pose.position.y, currPose_.pose.position.y); 
    double cmd_z = calcPidCmd(z_controller_, cmdPose_.pose.position.z, currPose_.pose.position.z); 
    double cmd_yaw = calcPidCmd(yaw_controller_, calculate_yaw_setpoint(), getCurrentYaw()); 

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