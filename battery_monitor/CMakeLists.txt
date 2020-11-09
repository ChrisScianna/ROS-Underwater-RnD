cmake_minimum_required(VERSION 2.8.3)
project(battery_monitor)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rospy
  std_msgs
  sensor_msgs
  message_generation
  diagnostic_updater
  diagnostic_tools
  health_monitor
)

add_message_files(
   FILES
   ReportBatteryInfo.msg
   ReportLeakDetected.msg
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
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
  src/battery_monitor_main.cpp
  src/battery_monitor.cpp
  src/CANIntf.cpp
  src/CO_VehicleSBC.c
)

add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(CANFESTIVAL_LIBRARIES
    $ENV{CANFESTIVALHOME}/lib/libcanfestival.a
    $ENV{CANFESTIVALHOME}/lib/libcanfestival_unix.a
    $ENV{CANFESTIVALHOME}/lib/libcanfestival_can_socket.a
)

roslint_cpp(
  src/battery_monitor.cpp
  src/battery_monitor_main.cpp
  include/battery_monitor/battery_monitor.h
  include/battery_monitor/BatteryInfo.h
)

roslint_add_test()

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
  launch/battery_monitor.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
