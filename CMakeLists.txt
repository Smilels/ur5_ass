cmake_minimum_required(VERSION 2.8.3)
project(ur5_ass)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
#find_package(Eigen3 REQUIRED)

#if(NOT EIGEN3_INCLUDE_DIRS)
#  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roscpp
  rospy
  pluginlib
  std_msgs
  sensor_msgs
  tf
  pcl_conversions
	pcl_ros
  geometry_msgs
  visualization_msgs
  message_generation
)
find_package(PCL REQUIRED)
#find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
   FILES
  ObjectPose.msg
  TargetPose.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_ass
  CATKIN_DEPENDS moveit_core moveit_ros_planning_interface roscpp rospy std_msgs interactive_markers pcl_conversions visualization_msgs DEPENDS
	PCL
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
#include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(ur5move src/ur5move.cpp)
target_link_libraries(ur5move ${catkin_LIBRARIES})
add_executable(icp src/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(icp1 src/icp1.cpp)
target_link_libraries (icp1 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(frame src/frame.cpp)
target_link_libraries (frame ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(frame1 src/frame1.cpp)
target_link_libraries (frame1 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(frame2 src/frame2.cpp)
target_link_libraries (frame2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(icp_kinect2 src/icp_kinect2.cpp)
target_link_libraries (icp_kinect2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(icp_kinectsample src/icp_kinectsample.cpp)
target_link_libraries (icp_kinectsample ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(qq src/qq.cpp)
target_link_libraries (qq  ${catkin_LIBRARIES})
add_executable(onecallback src/onecallback.cpp)
target_link_libraries (onecallback  ${catkin_LIBRARIES})
add_executable(multitopic src/multitopic.cpp)
target_link_libraries (multitopic  ${catkin_LIBRARIES})
add_executable(icp_samesample src/icp_samesample.cpp)
target_link_libraries (icp_samesample ${PCL_LIBRARIES} ${catkin_LIBRARIES})
