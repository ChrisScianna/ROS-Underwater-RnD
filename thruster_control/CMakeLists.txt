cmake_minimum_required(VERSION 2.8.3)
project(thruster_control )

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  diagnostic_tools
  health_monitor
  message_generation
  roscpp
  roslint
  rospy
  std_msgs
)

add_message_files(
   FILES
   SetRPM.msg
   ReportRPM.msg
   ReportMotorTemperature.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs
)

if (DEFINED ENV{CANFESTIVALHOME})
    message("-- CANFESTIVALHOME = $ENV{CANFESTIVALHOME}")
else()
    message(FATAL_ERROR "CANFESTIVALHOME is not defined.")
endif()

###########
## Build ##
###########

set(CANFESTIVAL_INCLUDES
    $ENV{CANFESTIVALHOME}/include/
)

set(CANFESTIVAL_LIBRARIES
  $ENV{CANFESTIVALHOME}/lib/libcanfestival.a
  $ENV{CANFESTIVALHOME}/lib/libcanfestival_unix.a
  $ENV{CANFESTIVALHOME}/lib/libcanfestival_can_socket.a
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CANFESTIVAL_INCLUDES}
)

add_library(${PROJECT_NAME} STATIC
  src/thruster_control.cpp
  src/thruster_control_ros.cpp
  src/CANIntf.cpp
  src/CO_VehicleSBC.c
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CANFESTIVAL_LIBRARIES}
  rt
  dl
  pthread
)

add_executable(${PROJECT_NAME}_node src/thruster_control_main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/thruster_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gmock(test_thruster_control_ros
    test/thruster_control_ros.test
    test/test_thruster_control_ros.cpp)
  target_link_libraries(test_thruster_control_ros ${PROJECT_NAME})

  roslint_cpp(
    src/thruster_control.cpp
    src/thruster_control_ros.cpp
    src/thruster_control_main.cpp
    include/thruster_control/thruster_control.h
    include/thruster_control/thruster_control_ros.h
  )

   roslint_add_test()
endif()
