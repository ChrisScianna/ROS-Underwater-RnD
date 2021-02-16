cmake_minimum_required(VERSION 3.0.2)
project(auv_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  message_generation
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
  CATKIN_DEPENDS
    geographic_msgs
    geometry_msgs
    message_runtime
    std_msgs
)
