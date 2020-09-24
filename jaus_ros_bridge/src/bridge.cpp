/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of QinetiQ nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include <iostream>
#include <sstream>

#include "udpserver.h"

#include <pthread.h>
#include "JausDataManager.h"

#define NODE_VERSION "3.3x"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard from thruster control node: [%s]", msg->data.c_str());
}

int main(int argc, char** argv) {
  // pthread_t sendThread;
  ros::init(argc, argv, "bridge");

  ros::NodeHandle nodehandle;
  nodehandle.setParam("/version_numbers/Jaus_ros_bridge", NODE_VERSION);
  ROS_INFO("Starting Jaus_ros_bridge node Version: [%s]", NODE_VERSION);

  string ipAddress = "192.168.192.135";
  int port = 51818;

  string batterypack = "A";

  if (nodehandle.getParam("/jaus_ros_bridge/ipaddress", ipAddress) == 0)
    ROS_ERROR("IP address not found");
  else
    ROS_INFO("IP address is: %s", ipAddress.c_str());

  if (nodehandle.getParam("/jaus_ros_bridge/port", port) == 0)
    ROS_ERROR("Port not found");
  else
    ROS_INFO("Port is: [%d]", port);

  string temp = "";
  if (nodehandle.getParam("/jaus_ros_bridge/batterypack", temp) != 0) {
    if (temp == "A" || temp == "B") batterypack = temp;

    ROS_INFO("batterypack is: [%s]", batterypack.c_str());
  }

  if (nodehandle.getParam("/jaus_ros_bridge/debug_mode", debug_mode) == 0) {
    ROS_ERROR("debug_mode is not found!");
  } else {
    ROS_INFO("debug_mode is %d", debug_mode);
  }

  // test the fake_thruster messaging
  // ros::Publisher chatter_pub = nodehandle.advertise<std_msgs::String>("bridge_topic", 1000);
  // ros::Subscriber sub = nodehandle.subscribe("fake_thruster_topic", 1, chatterCallback);

  ros::Rate loop_rate(20);  // 20 Hz
  udpserver udp(ipAddress.c_str(), port);
  JausDataManager dataMgr(&nodehandle, &udp);
  dataMgr.SetBatteryPack(batterypack);

  while (ros::ok()) {
    if (udp.HasMessageToSend()) {
      udp.SendMessage();
    }

    if (udp.receive() == true) {
      char* data = udp.getData();

      dataMgr.ProcessReceivedData(data);
      std_msgs::String msg;

      msg.data = data;

      // ROS_INFO("Data received from client: %s", msg.data.c_str());
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  if (udp.IsConnected()) {
    // Tell the remote that I am disconnected
    udp.Disconnect();
  }
  // pthread_exit(NULL); // should not do this, and it cause the ending of the program last long
  // time.
  return 0;
}
