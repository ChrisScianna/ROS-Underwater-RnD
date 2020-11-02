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
 * autopilot.h
 */

#ifndef AUTOPILOT_AUTOPILOT_H
#define AUTOPILOT_AUTOPILOT_H

#include <autopilot/AutoPilotInControl.h>
#include <math.h>
#include <pose_estimator/CorrectedData.h>
#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>

#include "autopilot/pid.h"
#include "fin_control/EnableReportAngles.h"
#include "fin_control/ReportAngle.h"
#include "fin_control/SetAngles.h"
#include "jaus_ros_bridge/ActivateManualControl.h"
#include "mission_manager/AttitudeServo.h"
#include "mission_manager/DepthHeading.h"
#include "mission_manager/FixedRudder.h"
#include "mission_manager/ReportExecuteMissionState.h"
#include "mission_manager/ReportHeartbeat.h"
#include "mission_manager/SetBehavior.h"
#include "sensor_msgs/Imu.h"
#include "thruster_control/ReportRPM.h"
#include "thruster_control/SetRPM.h"

#define NODE_VERSION "2.02x"

class AutoPilotNode
{
 public:
  explicit AutoPilotNode(ros::NodeHandle& node_handle);
  void Start();
  void Stop();

 private:
  ros::NodeHandle nh;
  ros::Publisher auto_pilot_in_control_pub;
  ros::Publisher thruster_pub;
  ros::Publisher fins_control_pub;
  ros::Publisher fins_enable_reporting_pub;

  ros::Subscriber jaus_ros_sub;
  ros::Subscriber thruster_sub;

  ros::Subscriber desiredRoll_sub_;
  ros::Subscriber desiredYaw_sub_;
  ros::Subscriber desiredPitch_sub_;

  ros::Subscriber correctedData_sub_;
  ros::Subscriber missionStatus_sub_;
  ros::Timer missionMgrHeartbeatTimer;

  ros::Subscriber missionMgrHeartBeat_sub_;
  ros::Subscriber setAttitudeBehavior_sub_;
  ros::Subscriber setDepthHeadingBehavior_sub_;
  ros::Subscriber setFixedRudderBehavior_sub_;

  control_toolbox::Pid roll_pid_controller;
  control_toolbox::Pid pitch_pid_controller;
  control_toolbox::Pid yaw_pid_controller;
  control_toolbox::Pid depth_pid_controller;

  bool auto_pilot_in_control;
  bool mission_mode;

  double roll_pgain;
  double roll_igain;
  double roll_dgain;
  double roll_imax;
  double roll_imin;

  double pitch_pgain;
  double pitch_igain;
  double pitch_dgain;
  double pitch_imax;
  double pitch_imin;

  double yaw_pgain;
  double yaw_igain;
  double yaw_dgain;
  double yaw_imax;
  double yaw_imin;

  double depth_pgain;
  double depth_igain;
  double depth_dgain;
  double depth_imax;
  double depth_imin;

  void correctedDataCallback(const pose_estimator::CorrectedData& data);
  void missionStatusCallback(const mission_manager::ReportExecuteMissionState& data);

  void HandleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data);

  void mixactuators(double roll, double pitch, double yaw);
  double radiansToDegrees(double radians);
  double degreesToRadians(double degrees);
  void missionMgrHeartbeatTimeout(const ros::TimerEvent& timer);

  void workerFunc();

  boost::mutex m_mutex;

  double currentRoll;
  double currentPitch;
  double currentYaw;
  double currentDepth;

  double desiredRoll;
  double desiredPitch;
  double desiredYaw;
  double desiredDepth;
  double desiredRudder;
  double desiredSpeed;

  double maxAllowedThrusterRpm;
  double minimalspeed;  // knots
  double rpmPerKnot;
  int control_loop_rate;  // Hz

  bool thrusterEnabled;      // Thruster can't spin if false
  bool speedControlEnabled;  // Autopilot controls acceleration if true
  bool fixedRudder;          // Robot cordinate system for yaw if true
  bool depthControl;         // Depth if true, pitch if false
  bool autopilotEnabled;     // Autopilot if true, OCU if false

  bool allowReverseThrusterAutopilot;  // allow negative RPM if true

  double maxCtrlPlaneAngle;  // Max fin angle (degrees)
  double maxDepthCommand;    // Max amount the depth affects the fin angle (degrees)

  boost::shared_ptr<boost::thread> m_thread;

  // Behavior callbacks
  void missionMgrHbCallback(const mission_manager::ReportHeartbeat& msg);
  void attitudeServoCallback(const mission_manager::AttitudeServo& msg);
  void depthHeadingCallback(const mission_manager::DepthHeading& msg);
  void fixedRudderCallback(const mission_manager::FixedRudder& msg);

  // Mask that keeps tracks of active behaviors
  boost::mutex m_behavior_mutex;
  boost::mutex mm_hb_mutex;  // misson manager heartbeat lock

  double mmTimeout;
  bool mmIsAlive;
};
#endif  // AUTOPILOT_AUTOPILOT_H
