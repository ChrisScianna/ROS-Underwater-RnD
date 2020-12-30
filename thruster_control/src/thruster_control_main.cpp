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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

/*
 * thruster_control_main.cpp
 */

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <thruster_control/thruster_control_ros.h>

// Version log
// 1.0 Initial version
// 1.1 Added CAN Node1 and CAN Node2 Id's to launch file to change preoperational state to
// operational state in CANIntf.cpp
#define NODE_VERSION "1.1"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_control");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO("Starting Thruster Control node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/thruster_control_node", NODE_VERSION);

  diagnostic_updater::Updater updater(nh, pnh);
  updater.setHardwareID("thruster");
  ros::Timer diagnostics_timer = nh.createTimer(
      ros::Duration(updater.getPeriod()),
      [&updater](const ros::TimerEvent&) { updater.update(); });

  qna::robot::ThrusterControlROS controller(nh, pnh);
  controller.MonitorUsing(&updater);

  ros::spin();

  return 0;
}
