cmake_minimum_required(VERSION 2.8.3)
project(payload_manager)

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
   PayloadCommand.msg
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

roslint_cpp(
  src/payload_manager.cpp
)


roslint_add_test()

add_executable(${NODE_APP_NAME} src/payload_manager.cpp)
target_link_libraries(${NODE_APP_NAME} ${catkin_LIBRARIES})
add_dependencies(${NODE_APP_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(payload_publisher src/payload_publisher.cpp)
target_link_libraries(payload_publisher ${catkin_LIBRARIES})
add_dependencies(payload_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(${NODE_APP_NAME} PUBLIC)

 target_link_libraries(${NODE_APP_NAME}
   ${catkin_LIBRARIES}
    rt
    dl
    pthread
 )



