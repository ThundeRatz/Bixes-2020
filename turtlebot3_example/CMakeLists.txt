################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot3_drive scripts/turtlebot3_drive.cpp)
add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  scripts/turtlebot3_drive_py.py
  scripts/turtlebot3_obstacle.py
  scripts/turtlebot3_pointop_key.py
  scripts/turtlebot3_teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS turtlebot3_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
