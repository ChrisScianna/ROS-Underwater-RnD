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

#ifndef MISSION_CONTROL_BEHAVIORS_ALTITUDE_HEADING_H
#define MISSION_CONTROL_BEHAVIORS_ALTITUDE_HEADING_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <string>

#include "auv_interfaces/StateStamped.h"
#include "mission_control/AltitudeHeading.h"
#include "mission_control/behavior.h"

namespace mission_control
{

class AltitudeHeadingBehavior : public Behavior
{
 public:
  AltitudeHeadingBehavior(const std::string &name, const BT::NodeConfiguration &config);

  BT::NodeStatus behaviorRunningProcess();

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {BT::InputPort<double>("altitude", 0.0, "altitude"),  //  NOLINT
                           BT::InputPort<double>("heading", 0.0, "heading"),
                           BT::InputPort<double>("speed_knots", 0.0, "speed_knots"),
                           BT::InputPort<double>("altitude_tol", 0.0, "altitude_tol"),
                           BT::InputPort<double>("heading_tol", 0.0, "heading_tol"),
                           BT::InputPort<double>("time_out", 0.0, "time_out")};
    return ports;
  }

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher altitudeHeadingBehaviorPub_;
  ros::Subscriber subCorrectedData_;

  double altitude_;
  double heading_;
  double speedKnots_;
  double timeOut_;

  bool altitudeEnable_;
  bool headingEnable_;
  bool speedKnotsEnable_;
  bool timeOutEnable_;

  double altitudeTolerance_;
  double headingTolerance_;

  void stateDataCallback(const auv_interfaces::StateStamped &data);
  bool goalHasBeenPublished_;
  void publishGoalMsg();
  ros::Time behaviorStartTime_;
  bool behaviorComplete_;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ALTITUDE_HEADING_H
