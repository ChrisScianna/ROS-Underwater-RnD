cmake_minimum_required(VERSION 2.8.3)
project(mission_manager)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)
add_compile_options(-DTIXML_USE_STL)

find_package(catkin REQUIRED COMPONENTS
  health_monitor
  pose_estimator
  payload_manager
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  AttitudeServo.msg
  DepthHeading.msg
  FixedRudder.msg
  Ping.msg
  SetBehavior.msg
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
   src/mission_manager.cpp
   src/mission_parser.cpp
   src/behavior_factory.cpp
   src/behaviors/attitude_servo.cpp
   src/behaviors/depth_heading.cpp
   src/behaviors/fixed_rudder.cpp
   src/behaviors/pinger.cpp
   src/behaviors/podlog.cpp
   src/behaviors/waypoint.cpp
   src/behaviors/altitude_heading.cpp
   src/behaviors/payload_command.cpp
   src/tinyxml/tinystr.cpp
   src/tinyxml/tinyxml.cpp
   src/tinyxml/tinyxmlerror.cpp
   src/tinyxml/tinyxmlparser.cpp
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
  launch/mission_manager.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
