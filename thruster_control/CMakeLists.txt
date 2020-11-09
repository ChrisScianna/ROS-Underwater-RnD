cmake_minimum_required(VERSION 2.8.3)
project(thruster_control)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  diagnostic_tools
  health_monitor
  message_generation
  roscpp
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CANFESTIVAL_INCLUDES}
)

add_executable(${NODE_APP_NAME}
   src/thruster_control.cpp
   src/thruster_control_main.cpp
   src/CANIntf.cpp
   src/CO_VehicleSBC.c
)

add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CANFESTIVAL_LIBRARIES
    $ENV{CANFESTIVALHOME}/lib/libcanfestival.a
    $ENV{CANFESTIVALHOME}/lib/libcanfestival_unix.a
    $ENV{CANFESTIVALHOME}/lib/libcanfestival_can_socket.a
)

target_include_directories(${NODE_APP_NAME} PUBLIC
    ${CANFESTIVAL_INCLUDES})

 target_link_libraries(${NODE_APP_NAME}
   ${catkin_LIBRARIES}
   ${CANFESTIVAL_LIBRARIES}
    rt
    dl
    pthread
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
  launch/thruster_control.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)