cmake_minimum_required(VERSION 2.8.3)
project(payload_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roslint
  message_generation
)

add_message_files(
   FILES
   PayloadCommand.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package()

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/payloadManagerMain
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

roslint_python(
  scripts/payloadManagerMain
)

roslint_add_test()
