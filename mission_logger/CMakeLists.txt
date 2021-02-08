cmake_minimum_required(VERSION 2.8.3)
project(mission_logger)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/mission_bag_recorder
  scripts/mission_text_logger
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslint_python(
  scripts/mission_bag_recorder
  scripts/mission_text_logger
)

roslint_python()  # all .py files

roslint_add_test()
