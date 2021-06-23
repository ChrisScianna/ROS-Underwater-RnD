cmake_minimum_required(VERSION 2.8.3)
project(cmd_actuators_mux)

set(NODE_APP_NAME ${PROJECT_NAME}_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  fin_control
  jaus_ros_bridge
  message_generation
  mission_control
  roscpp
  roslint
  std_msgs
  thruster_control
  pluginlib 
  nodelet 
  dynamic_reconfigure
)
find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/reload.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS fin_control roscpp std_msgs pluginlib nodelet dynamic_reconfigure thruster_control jaus_ros_bridge mission_control
  DEPENDS yaml-cpp
 )

roslint_cpp(
        src/cmd_actuator_mux_nodelet.cpp
        src/cmd_actuator_subscribers.cpp
        include/cmd_actuators_mux/cmd_actuator_mux_nodelet.cpp
        include/cmd_actuators_mux/cmd_actuator_subscribers.cpp
        include/cmd_actuators_mux/exceptions.h
)

roslint_add_test()

 ###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/cmd_actuator_subscribers.cpp src/cmd_actuator_mux_nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
