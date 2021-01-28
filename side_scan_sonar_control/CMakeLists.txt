cmake_minimum_required(VERSION 2.8.3)
project(side_scan_sonar_control)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslint
  message_generation
)

add_message_files(
   FILES
   SideScanSonarCommand.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

roslint_cpp(
  src/side_scan_sonar_control.cpp
)


roslint_add_test()

add_executable(${NODE_APP_NAME} src/side_scan_sonar_control.cpp)
target_link_libraries(${NODE_APP_NAME} ${catkin_LIBRARIES})
add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(${NODE_APP_NAME} PUBLIC)

 target_link_libraries(${NODE_APP_NAME}
   ${catkin_LIBRARIES}
    rt
    dl
    pthread
 )



