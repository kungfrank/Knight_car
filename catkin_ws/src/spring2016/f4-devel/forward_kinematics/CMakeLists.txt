cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  std_msgs
  visualization_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES
  std_msgs
  visualization_msgs
  duckietown_msgs
)

catkin_package(
#   INCLUDE_DIRS include
   CATKIN_DEPENDS duckietown_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)
target_link_libraries(forward_kinematics_node 
  ${catkin_LIBRARIES})