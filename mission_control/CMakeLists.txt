cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  auv_interfaces
  behaviortree_cpp_v3
  geodesy
  health_monitor
  message_generation
  payload_manager
  rosbag_storage
  roscpp
  roslint
  rospy
  std_msgs
  topic_tools
)

add_message_files(
  FILES
  AltitudeHeading.msg
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
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS auv_interfaces health_monitor roscpp rospy std_msgs
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
  src/behaviors/introspection.cpp
  src/behaviors/go_to_waypoint.cpp
  src/behaviors/fix_rudder.cpp
  src/behaviors/set_depth_heading.cpp
  src/behaviors/attitude_servo.cpp
  src/behaviors/set_altitude_heading.cpp
  src/behaviors/payload_command.cpp
  src/behaviors/abort.cpp
  src/behaviors/delay.cpp
  src/behaviors/log_to_bagfile.cpp
  src/behaviors/internal/async_bag_writer.cpp
  src/behaviors/internal/generic_subscription.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set(NODE_APP_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_APP_NAME}
   src/mission_control_main.cpp
)

target_link_libraries(${NODE_APP_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS
  --filter=-whitespace/braces,-build/c++11,-runtime/references
)
roslint_cpp()
roslint_python()

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

  catkin_add_gtest(test_reactive_action test/test_reactive_action.cpp)
  target_link_libraries(test_reactive_action ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(test_introspectable_node test/test_introspectable_node.cpp)
  target_link_libraries(test_introspectable_node ${PROJECT_NAME} ${catkin_LIBRARIES})
  catkin_add_gtest(test_delay_action test/test_delay_action.cpp)
  target_link_libraries(test_delay_action ${PROJECT_NAME} ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest(test/mission_control_interface.test)
  add_rostest(test/mission_control_failure_modes.test)
  add_rostest(test/fix_rudder_action.test)
  add_rostest(test/attitude_servo_action.test)
  add_rostest(test/go_to_waypoint_action.test)
  add_rostest(test/set_altitude_heading_action.test)
  add_rostest(test/set_depth_heading_action.test)
endif()
