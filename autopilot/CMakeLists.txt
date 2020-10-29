cmake_minimum_required(VERSION 2.8.3)
project(autopilot)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  fin_control
  pose_estimator
  roscpp
  rospy
  std_msgs
  thruster_control
  jaus_ros_bridge
  message_generation
)

add_message_files(
  FILES
  AutoPilotInControl.msg
)

generate_messages(
  DEPENDENCIES
  fin_control
  thruster_control
  pose_estimator
  jaus_ros_bridge
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS fin_control pose_estimator roscpp rospy std_msgs thruster_control jaus_ros_bridge
)

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

install(FILES
  launch/autopilot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
