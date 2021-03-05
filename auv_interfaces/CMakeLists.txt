cmake_minimum_required(VERSION 3.0.2)
project(auv_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  message_generation
  roslint
  std_msgs
)

#############################
## Generate ROS interfaces ##
#############################

add_message_files(
  FILES
    CartesianPose.msg
    CartesianPoseWithCovariance.msg
    GeneralizedVector.msg
    GeneralizedVectorWithCovariance.msg
    GeoLocation.msg
    ManoeuvringState.msg
    MotionPerturbation.msg
    SeakeepingState.msg
    State.msg
    StateStamped.msg
)

generate_messages(
  DEPENDENCIES
    geographic_msgs
    geometry_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_helpers
  CATKIN_DEPENDS
    geographic_msgs
    geometry_msgs
    message_runtime
    std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_helpers
  src/helpers.cpp
)

add_dependencies(${PROJECT_NAME}_helpers
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_helpers ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_helpers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_helpers test/test_helpers.cpp)
  target_link_libraries(test_helpers
    ${PROJECT_NAME}_helpers ${catkin_LIBRARIES})

  roslint_add_test()
endif()
