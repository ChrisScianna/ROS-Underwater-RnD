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

#ifndef MISSION_CONTROL_BEHAVIORS_WAYPOINT_H
#define MISSION_CONTROL_BEHAVIORS_WAYPOINT_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geodesy/utm.h>
#include <ros/ros.h>
#include <string>

#include "mission_control/Waypoint.h"
#include "mission_control/behavior.h"
#include "auv_interfaces/StateStamped.h"

namespace mission_control
{

class GoToWaypoint : public BT::ActionNodeBase
{
public:
  GoToWaypoint(const std::string &name, const BT::NodeConfiguration &config);

  BT::NodeStatus tick() override;

  void halt() override;

  static BT::PortsList providedPorts()
  {
    return {  // NOLINT
      BT::InputPort<double>("depth", "depth"),
      BT::InputPort<double>("altitude", "altitude"),
      BT::InputPort<double>("latitude", "latitude"),
      BT::InputPort<double>("longitude", "longitude"),
      BT::InputPort<double>("speed_knots", "speed_knots"),
      BT::InputPort<double>("tolerance_radius", 0.0, "tolerance_radius")};  // NOLINT
  }

private:
  void stateCallback(const auv_interfaces::StateStamped &data);
  mission_control::Waypoint makeWaypointMsg();

  ros::NodeHandle nh_;
  ros::Publisher waypoint_pub_;
  ros::Subscriber state_sub_;

  double latitude_;
  double longitude_;
  double altitude_{0.0};
  double depth_{0.0};
  double speed_knots_{0.0};
  double tolerance_radius_;
  uint8_t enable_mask_{0};

  geodesy::UTMPoint current_position_;
  geodesy::UTMPoint target_position_;
  bool state_up_to_date_;
  ros::Time start_time_;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_WAYPOINT_H
