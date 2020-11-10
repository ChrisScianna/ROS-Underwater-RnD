cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  diagnostic_tools
  health_monitor
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  geographic_msgs
  thruster_control
  message_generation
)

add_message_files(
  FILES
  CorrectedData.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs  
  geometry_msgs
  geographic_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs geographic_msgs thruster_control
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_APP_NAME} 
  src/pose_estimator.cpp
  src/pose_estimator_main.cpp
)

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
  launch/pose_estimator.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############