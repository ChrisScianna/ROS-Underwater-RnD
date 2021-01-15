cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)
add_compile_options(-DTIXML_USE_STL)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  behaviortree_cpp_v3
  health_monitor
  message_generation
  pose_estimator
  payload_manager
  roscpp
  roslint
  rospy
  std_msgs
)

add_message_files(
  FILES
  AttitudeServo.msg
  DepthHeading.msg
  FixedRudder.msg
  Ping.msg
  Waypoint.msg
  AltitudeHeading.msg
  LoadMission.msg
  ExecuteMission.msg
  ReportLoadMissionState.msg
  ReportExecuteMissionState.msg
  AbortMission.msg
  QueryMissions.msg
  ReportMissions.msg
  MissionData.msg
  RemoveMissions.msg
  ReportHeartbeat.msg
)

generate_messages(
  DEPENDENCIES
  pose_estimator
  payload_manager
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS pose_estimator payload_manager roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_APP_NAME} 
   src/mission.cpp
   src/mission_control.cpp
   src/behaviors/attitude_servo.cpp
   src/behaviors/depth_heading.cpp
   src/behaviors/waypoint.cpp
)

add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${NODE_APP_NAME}
  ${catkin_LIBRARIES}
)

roslint_cpp()

#############
## Install ##
#############

install(TARGETS ${NODE_APP_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/mission_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_add_test()
