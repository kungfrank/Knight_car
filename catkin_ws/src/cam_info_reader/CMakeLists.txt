cmake_minimum_required(VERSION 2.8.3)
project(cam_info_reader)

find_package(catkin REQUIRED COMPONENTS
   roscpp  camera_info_manager
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(cam_info_reader src/main.cpp)
target_link_libraries(cam_info_reader 
  ${catkin_LIBRARIES}
)

install(TARGETS
  cam_info_reader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
