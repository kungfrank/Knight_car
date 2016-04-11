cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  duckietown_msgs
  std_msgs
  visualization_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
#   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime duckietown_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kinematics_tester_node src/kinematics_tester_node.cpp)
target_link_libraries(kinematics_tester_node ${catkin_LIBRARIES})
add_dependencies(kinematics_tester_node ${catkin_EXPORTED_TARGETS})

add_executable(pose_to_path_node src/pose_to_path_node.cpp)
target_link_libraries(pose_to_path_node ${catkin_LIBRARIES})
add_dependencies(pose_to_path_node ${catkin_EXPORTED_TARGETS})

add_executable(pose2d_to_path_node src/pose2d_to_path_node.cpp)
target_link_libraries(pose2d_to_path_node ${catkin_LIBRARIES})
add_dependencies(pose2d_to_path_node ${catkin_EXPORTED_TARGETS})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/all_library_tests.test)
    add_rostest(tests/forward_kinematics_tester_node.test)
    add_rostest(tests/inverse_kinematics_tester_node.test)
    add_rostest(tests/position_filter_tester_node.test)
    add_rostest(tests/kinematics_learning_tester_node.test)
    add_rostest(tests/vicon_learning_tester_node.test)
endif()
