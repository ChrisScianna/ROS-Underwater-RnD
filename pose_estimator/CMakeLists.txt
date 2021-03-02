cmake_minimum_required(VERSION 2.8.3)
project(pose_estimator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  auv_interfaces
  diagnostic_updater
  diagnostic_tools
  health_monitor
  roscpp
  roslint
  sensor_msgs
  tf
  thruster_control
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS auv_interfaces
    diagnostic_updater diagnostic_tools
    roscpp sensor_msgs tf thruster_control
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pose_estimator.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set(NODE_EXECUTABLE_NAME ${PROJECT_NAME}_node)
add_executable(${NODE_EXECUTABLE_NAME}
  src/pose_estimator_node.cpp
)

add_dependencies(${NODE_EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${NODE_EXECUTABLE_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})

roslint_cpp()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${NODE_EXECUTABLE_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_add_test()
