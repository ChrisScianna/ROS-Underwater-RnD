/*
 * sea_scan_echo_sensor_main.cpp
 */


#include <sea_scan_echo_sensor.h>
#include "sea_scan_echo_msgs.h"

#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <vector>


// Version log
// 1.0 Initial version
#define NODE_VERSION "1.0"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sea_scan_echo_sensor_node");
    ros::Time::init();

    ros::NodeHandle nodeHandle;

  	ROS_INFO("Starting Sea Scan Echo Sensor node Version: [%s]",NODE_VERSION);
  	nodeHandle.setParam("/version_numbers/sea_scan_echo_sensor_node", NODE_VERSION);

    qna::sensor::SeaScanEchoSensor sensor(nodeHandle);

    ros::spin();

    return 0;
}


