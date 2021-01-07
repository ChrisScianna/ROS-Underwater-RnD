cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  diagnostic_msgs
  roscpp
  roslint
)

## This macro ensures modules and global scripts declared therein get installed
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diagnostic_tools
  CATKIN_DEPENDS diagnostic_updater diagnostic_msgs roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/diagnostic.cpp
  src/periodic_event_status.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

roslint_cpp()

roslint_python()

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
