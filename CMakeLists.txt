cmake_minimum_required(VERSION 3.8)
project(mbzirc_aerial_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(joy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
# PID controller
find_package(jlb_pid REQUIRED)
# mbzirc_aerial_manipulation_msgs
find_package(mbzirc_aerial_manipulation_msgs REQUIRED)
#mbzirc_msgs
find_package(mbzirc_msgs REQUIRED)


## Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/monitor_topic.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ files
add_executable(uav_ctl src/uav_ctl_node.cpp src/uav_ctl.cpp)
add_executable(uav_joy src/uav_joy_node.cpp src/uav_joy.cpp)
add_executable(simple_uav_ctl src/simple_uav_ctl_node.cpp src/simple_uav_ctl.cpp)

target_include_directories(uav_ctl 
  PRIVATE 
  include/
)

target_include_directories(uav_joy
  PRIVATE
  include/
)

target_include_directories(simple_uav_ctl 
  PRIVATE 
  include/
)
  
# Add target dependencies for correct linking
ament_target_dependencies(uav_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs nav_msgs tf2_ros tf2_msgs jlb_pid mbzirc_aerial_manipulation_msgs mbzirc_msgs)

ament_target_dependencies(uav_joy
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs mbzirc_aerial_manipulation_msgs)

ament_target_dependencies(simple_uav_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs nav_msgs tf2_ros tf2_msgs jlb_pid mbzirc_aerial_manipulation_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  uav_ctl 
  uav_joy
  simple_uav_ctl
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  src 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
