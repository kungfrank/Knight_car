cmake_minimum_required(VERSION 2.8.3)
project(duckie_rr_bridge)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  rospy
  std_msgs
  sensor_msgs
)

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
  	RobotRaconteur 
  	duckietown_msgs 
  	rospy 
  	std_msgs
  	sensor_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
