/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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

#ifndef MISSION_CONTROL_BEHAVIORS_ABORT_H
#define MISSION_CONTROL_BEHAVIORS_ABORT_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <string>

#include "auv_interfaces/StateStamped.h"
#include "mission_control/AttitudeServo.h"
#include "thruster_control/ReportRPM.h"

namespace mission_control
{
class Abort : public BT::ActionNodeBase
{
 public:
  Abort(const std::string& name);

  BT::NodeStatus tick() override;

  void halt() override;

 private:
  void stateDataCallback(const auv_interfaces::StateStamped& data);
  void thrusterRPMCallback(const thruster_control::ReportRPM& data);
  void publishAbortMsg();

  ros::NodeHandle nodeHandle_;
  ros::Publisher attitudeServoBehaviorPub_;
  ros::Subscriber subStateData_;
  ros::Subscriber subThrusterRPM_;

  auv_interfaces::StateStamped stateData_;
  thruster_control::ReportRPM velocityData_;

  // fins are set to surface and Thruster velocity is 0
  double roll_{0.0};
  double pitch_;
  double yaw_{0.0};
  double speedKnots_{0.0};
  double rollTolerance_{0.1};
  double pitchTolerance_{0.1};
  double yawTolerance_{0.1};

  bool stateUpToDate_{false};
  bool velocityUpToDate_{false};
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ABORT_BEHAVIOR_H
