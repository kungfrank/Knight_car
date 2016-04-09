cmake_minimum_required(VERSION 2.8.3)
project(cone_detector)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  geometry_msgs
  sensor_msgs
  duckietown_msgs
  std_msgs
  tf_conversions
  image_transport
  tf
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES cone_detector
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable

add_executable(echo src/echo.cpp)
add_dependencies(echo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(echo ${OpenCV_LIBS} ${catkin_LIBRARIES}  )

add_executable(morphology_object_tracking src/morphology_object_tracking.cpp)
add_dependencies(morphology_object_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(morphology_object_tracking ${OpenCV_LIBS} ${catkin_LIBRARIES}  )
