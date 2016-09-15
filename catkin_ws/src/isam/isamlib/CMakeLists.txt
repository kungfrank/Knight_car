cmake_minimum_required(VERSION 2.6)

# find all source and header files
file(GLOB SRCS RELATIVE "${PROJECT_SOURCE_DIR}/isamlib" "*.cpp")

# isamlib
add_library(isamlib ${SRCS})
set_property(TARGET isamlib PROPERTY OUTPUT_NAME isam)

# simple way of finding CHOLMOD
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDES} ${catkin_INCLUDE_DIRS})
target_link_libraries(isamlib ${CHOLMOD_LIBRARIES} ${catkin_LIBRARIES})

# install library
install(TARGETS isamlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# install header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/isam/ 
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

