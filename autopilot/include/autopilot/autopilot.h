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
#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>

#include <algorithm>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>

#include "autopilot/pid.h"
#include "autopilot/value_set.h"
#include "auv_interfaces/StateStamped.h"
#include "fin_control/ReportAngle.h"
#include "fin_control/SetAngles.h"
#include "geodesy/utm.h"
#include "jaus_ros_bridge/ActivateManualControl.h"
#include "mission_control/AttitudeServo.h"
#include "mission_control/DepthHeading.h"
#include "mission_control/AltitudeHeading.h"
#include "mission_control/FixedRudder.h"
#include "mission_control/ReportExecuteMissionState.h"
#include "mission_control/ReportHeartbeat.h"
#include "mission_control/Waypoint.h"
#include "sensor_msgs/Imu.h"
#include "thruster_control/ReportRPM.h"
#include "thruster_control/SetRPM.h"

#define NODE_VERSION "2.02x"

class AutoPilotNode
{
public:
  AutoPilotNode();

  void spin();

private:
  void attitudeServoCallback(const mission_control::AttitudeServo& msg);
  void depthHeadingCallback(const mission_control::DepthHeading& msg);
  void fixedRudderCallback(const mission_control::FixedRudder& msg);
  void altitudeHeadingCallback(const mission_control::AltitudeHeading& msg);
  void waypointCallback(const mission_control::Waypoint& msg);
  void stateCallback(const auv_interfaces::StateStamped& msg);

  void missionHeartbeatTimeout(const ros::TimerEvent& ev);
  void missionHeartbeatCallback(const mission_control::ReportHeartbeat& msg);
  void missionStatusCallback(const mission_control::ReportExecuteMissionState& data);

  void handleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data);

  void mixActuators(double roll, double pitch, double yaw, double thrust);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher thruster_control_pub_;
  ros::Publisher fin_control_pub_;
  ros::Subscriber state_sub_;

  ros::Subscriber manual_control_sub_;

  ros::Subscriber mission_status_sub_;
  ros::Timer mission_heartbeat_timer_;
  ros::Subscriber mission_heartbeat_sub_;
  ros::Time last_heartbeat_stamp_;

  ros::Subscriber attitude_servo_sub_;
  ros::Subscriber depth_heading_sub_;
  ros::Subscriber altitude_heading_sub_;
  ros::Subscriber fixed_rudder_sub_;
  ros::Subscriber waypoint_sub_;

  control_toolbox::Pid roll_pid_controller_;
  control_toolbox::Pid pitch_pid_controller_;
  control_toolbox::Pid yaw_pid_controller_;
  control_toolbox::Pid depth_pid_controller_;
  control_toolbox::Pid altitude_pid_controller_;

  ros::Publisher auto_pilot_in_control_pub_;
  bool auto_pilot_in_control_;

  bool state_up_to_date_;

  double current_roll_;
  double current_pitch_;
  double current_yaw_;
  double current_depth_;
  geodesy::UTMPoint current_position_;

  double desired_roll_;
  double desired_pitch_;
  double desired_yaw_;
  double desired_depth_;
  double desired_rudder_;
  double desired_speed_;
  geodesy::UTMPoint desired_position_;

  double control_loop_rate_;  // Hz

  double minimal_speed_;  // knots
  double max_depth_command_;  // m
  double max_altitude_command_;  // m
  double rpms_per_knot_;  // RPMs
  double max_ctrl_fin_angle_in_radians_;

  enum class Setpoint
  {
    Rudder,
    Speed,
    Roll,
    Pitch,
    Yaw,
    Depth,
    Altitude,
    Position
  };
  ValueSet<Setpoint> active_setpoints_;

  bool thruster_enabled_;      // Thruster can't spin if false
  bool allow_reverse_thrust_;  // allow negative RPM if true
};

#endif  // AUTOPILOT_AUTOPILOT_H
