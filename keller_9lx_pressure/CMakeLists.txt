cmake_minimum_required(VERSION 2.8.3)
project(keller_9lx_pressure)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_tools
  diagnostic_updater
  health_monitor
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/keller_9lx_pressure_main
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

roslint_python(
  scripts/keller_9lx_pressure_main
)

roslint_add_test()
