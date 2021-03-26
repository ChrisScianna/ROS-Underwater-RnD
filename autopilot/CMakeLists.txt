cmake_minimum_required(VERSION 2.8.3)
project(autopilot)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  auv_interfaces
  fin_control
  roscpp
  rospy
  roslint
  std_msgs
  thruster_control
  jaus_ros_bridge
  message_generation
  mission_control
)

add_message_files(
  FILES
  AutoPilotInControl.msg
)

generate_messages(
  DEPENDENCIES
  fin_control
  thruster_control
  jaus_ros_bridge
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS auv_interfaces fin_control roscpp rospy std_msgs thruster_control jaus_ros_bridge mission_control
)

roslint_cpp(
  src/autopilot.cpp
  src/autopilot_main.cpp
  src/pid.cpp
  include/autopilot/autopilot.h
  include/autopilot/pid.h
)

roslint_add_test()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_APP_NAME} 
  src/autopilot.cpp
  src/autopilot_main.cpp
  src/pid.cpp)

add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${NODE_APP_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${NODE_APP_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
