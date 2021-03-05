cmake_minimum_required(VERSION 2.8.3)
project(ixblue_c3_ins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  auv_interfaces
  diagnostic_tools
  diagnostic_updater
  health_monitor
  message_generation
  roscpp
  roslint
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  NavigationLong.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    auv_interfaces
    diagnostic_tools
    diagnostic_updater
    health_monitor
    message_runtime
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/c3_protocol.cpp
  src/ixblue_c3_ins.cpp
  src/io_helpers.cpp
  src/ros_helpers.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

set(NODE_EXECUTABLE_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_EXECUTABLE_NAME}
   src/ixblue_c3_ins_node.cpp
)

add_dependencies(${NODE_EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(${NODE_EXECUTABLE_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${NODE_EXECUTABLE_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_cpp()

roslint_add_test()
