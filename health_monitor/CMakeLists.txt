cmake_minimum_required(VERSION 2.8.3)
project(health_monitor)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rosmon_msgs
  rospy
  std_msgs
  sensor_msgs
  message_generation
  diagnostic_tools
  diagnostic_updater
)

add_message_files(
   FILES
   ReportFault.msg
   ClearFault.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   rosmon_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs rosmon_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_APP_NAME} 
  src/health_monitor_main.cpp
  src/health_monitor.cpp
)

add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

roslint_cpp(
  src/health_monitor.cpp
  src/health_monitor_main.cpp
  include/health_monitor/health_monitor.h
)

roslint_add_test()


target_include_directories(${NODE_APP_NAME} PUBLIC)

 target_link_libraries(${NODE_APP_NAME}
   ${catkin_LIBRARIES}
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
  launch/health_monitor.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
