cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  auv_interfaces
  behaviortree_cpp_v3
  health_monitor
  jaus_ros_bridge
  message_generation
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
  Waypoint.msg
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
  payload_manager
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS auv_interfaces payload_manager health_monitor jaus_ros_bridge roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/mission.cpp
  src/mission_control.cpp
  src/behaviors/waypoint.cpp
  src/behaviors/fixed_rudder.cpp
  src/behaviors/depth_heading.cpp
  src/behaviors/attitude_servo.cpp
  src/behaviors/altitude_heading.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set(NODE_APP_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_APP_NAME}
   src/mission_control_main.cpp
)

target_link_libraries(${NODE_APP_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${NODE_APP_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()

  find_package(rostest REQUIRED)
  add_rostest(test/test_if_mission_control_aborts_when_health_monitor_reports_fault.test)
  add_rostest(test/test_jaus_ros_bridge_interface_mission_control.test)
  add_rostest(test/test_mission_control_publishes_heartbeat.test)
  add_rostest(test/test_mission_control_behaviors.test)
  catkin_add_gtest(behavior-object-test test/test_behavior_object.cpp)
  target_link_libraries(behavior-object-test ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
