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

#ifndef MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H
#define MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <string>

#include "auv_interfaces/StateStamped.h"
#include "mission_control/AttitudeServo.h"
#include "mission_control/behavior.h"

namespace mission_control
{
class AttitudeServoBehavior : public Behavior
{
 public:
  AttitudeServoBehavior(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus behaviorRunningProcess();

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("roll", "roll"),  //  NOLINT
            BT::InputPort<double>("pitch", "pitch"),
            BT::InputPort<double>("yaw", "yaw"),
            BT::InputPort<double>("speed_knots", "speed_knots"),
            BT::InputPort<double>("time_out", "time_out"),
            BT::InputPort<double>("roll_tol", 0.0, "roll_tol"),
            BT::InputPort<double>("pitch_tol", 0.0, "pitch_tol"),
            BT::InputPort<double>("yaw_tol", 0.0, "yaw_tol")};
  }

 private:
  void stateDataCallback(const auv_interfaces::StateStamped& data);
  void publishGoalMsg();

  ros::NodeHandle nodeHandle_;
  ros::Publisher attitudeServoBehaviorPub_;
  ros::Subscriber subStateData_;
  ros::Time behaviorStartTime_;

  double roll_;
  double pitch_;
  double yaw_;
  double speedKnots_;
  double timeOut_;

  bool rollEnable_;
  bool pitchEnable_;
  bool yawEnable_;
  bool speedKnotsEnable_;
  bool timeOutEnable_;

  double rollTolerance_;
  double pitchTolerance_;
  double yawTolerance_;

  bool goalHasBeenPublished_;
  bool behaviorComplete_;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H
