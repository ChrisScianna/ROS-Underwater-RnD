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

#include "mission_control/AttitudeServo.h"
#include "mission_control/behavior.h"
#include "pose_estimator/CorrectedData.h"

namespace mission_control
{
class AttitudeServoBehavior : public Behavior
{
 public:
  AttitudeServoBehavior(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus behaviorRunningProcess();

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {BT::InputPort<double>("roll", 0.0, "roll"),
                           BT::InputPort<double>("pitch", 0.0, "pitch"),
                           BT::InputPort<double>("yaw", 0.0, "yaw"),
                           BT::InputPort<double>("speed_knots", 0.0, "speed_knots"),
                           BT::InputPort<double>("time_out", 0.0, "time_out"),
                           BT::InputPort<double>("roll_tol",0.0,"roll_tol"),
                           BT::InputPort<double>("pitch_tol",0.0,"pitch_tol"),
                           BT::InputPort<double>("yaw_tol",0.0,"yaw_tol")};
    return ports;
  }

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher attitudeServoBehaviorPub_;
  ros::Subscriber subCorrectedData_;

  double roll_;
  double pitch_;
  double yaw_;
  double speedKnots_;
  double timeOut_;

  bool rollEnable_;
  bool pitchEnable_;
  bool yawEnable_;
  bool speedKnotsEnable_;

  double rollTolerance_;
  double pitchTolerance_;
  double yawTolerance_;

  void correctedDataCallback(const pose_estimator::CorrectedData& data);
  bool goalHasBeenPublished_;
  void publishGoalMsg();
  ros::Time behaviorStartTime_;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H
