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

#ifndef MISSION_CONTROL_BEHAVIORS_SET_ALTITUDE_HEADING_H
#define MISSION_CONTROL_BEHAVIORS_SET_ALTITUDE_HEADING_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <ros/ros.h>

#include <string>

#include "auv_interfaces/StateStamped.h"
#include "mission_control/behaviors/helpers.h"
#include "mission_control/behaviors/reactive_action.h"


namespace mission_control
{

class SetAltitudeHeadingNode : public ReactiveActionNode
{
public:
  SetAltitudeHeadingNode(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts()
  {
    return MakePortsList(
      InputPort<double, HasTolerance>(
        "altitude", NAN, "Altitude to reach (positive up), in meters"),
      InputPort<double, HasTolerance, HasAngleUnits>(
        "heading", NAN, "Heading (or yaw) to reach"),
      BT::InputPort<double>("speed_knots", NAN, "Cruise speed to command, in knots"));
  }

private:
  void stateDataCallback(auv_interfaces::StateStamped::ConstPtr msg);

  BT::NodeStatus setUp() override;
  BT::NodeStatus doWork() override;
  void tearDown() override;

  ros::NodeHandle nh_;
  ros::Publisher altitude_heading_pub_;
  ros::Subscriber state_sub_;

  double target_altitude_;
  double altitude_tolerance_;
  double target_heading_;
  double heading_tolerance_;
  double speed_knots_;
  uint8_t enable_mask_;

  auv_interfaces::StateStamped::ConstPtr state_;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_SET_ALTITUDE_HEADING_H
