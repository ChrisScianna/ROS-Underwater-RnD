cmake_minimum_required(VERSION 3.0.2)
project(sea_scan_echo_sensor)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ReportAltimeterRange.msg
  ReportSonarImage.msg
  ReportSonarConfiguration.msg
  SetFilterLength.msg
  SetLockoutRange.msg
  SetMaxListeningRange.msg
  SetSpeedOfSoundInWater.msg
  SetTransmitterEnabled.msg
  SetTriggerMode.msg
  SetBaudrate.msg
  SetDetectionThreshold.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(CANFESTIVAL_INCLUDES
    $ENV{CANFESTIVALHOME}/include/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CANFESTIVAL_INCLUDES}
)

add_executable(${NODE_APP_NAME}
   src/sea_scan_echo_msgs.cpp
   src/sea_scan_echo_sensor.cpp
   src/sea_scan_echo_sensor_main.cpp
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

roslint_cpp(
  src/sea_scan_echo_sensor.cpp
  src/sea_scan_echo_sensor_main.cpp
  src/sea_scan_echo_msgs.cpp
  include/sea_scan_echo_msgs.h
  include/sea_scan_echo_sensor.h
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
  launch/sea_scan_echo_sensor.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_add_test()
