cmake_minimum_required(VERSION 2.8.3)
project(fin_control)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_controllers
  diagnostic_updater
  diagnostic_tools
  health_monitor
  roscpp
  roslint
  rospy
  std_msgs
  std_srvs
  message_generation
)

add_message_files(
   FILES
   EnableReportAngles.msg
   ReportAngle.msg
   SetAngle.msg
   SetAngles.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamixel_workbench_controllers rospy roscpp std_msgs std_srvs
)

roslint_cpp(
  src/fin_control.cpp
  src/fin_control_main.cpp
  include/fin_control.h
)

roslint_add_test()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  dynamixel_workbench_toolbox/include
)

add_executable(${NODE_APP_NAME} 
   src/fin_control.cpp
   src/fin_control_main.cpp
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)