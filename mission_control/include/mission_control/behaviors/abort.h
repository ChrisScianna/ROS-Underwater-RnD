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
#include <ros/ros.h>

#include <string>

#include "auv_interfaces/StateStamped.h"

#include "mission_control/behaviors/reactive_action.h"

namespace mission_control
{
class AbortNode : public ReactiveActionNode
{
public:
  AbortNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() { return {}; }

private:
  void stateDataCallback(auv_interfaces::StateStamped::ConstPtr msg);

  BT::NodeStatus setUp() override;
  BT::NodeStatus doWork() override;
  void tearDown() override;

  ros::NodeHandle nh_;
  ros::Publisher attitude_servo_pub_;
  ros::Subscriber state_sub_;

  auv_interfaces::StateStamped::ConstPtr state_;

  // fins are set to surface and Thruster velocity is 0
  double roll_{0.0};
  double pitch_;
  double yaw_{0.0};
  double speed_knots_{0.0};
  double roll_tolerance_{0.1};
  double pitch_tolerance_{0.1};
  double yaw_tolerance_{0.1};
  double speed_tolerance_{0.1};
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ABORT_H
