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
/*  Behavioral Waipoint
  <waypoint>
      <description>
          00:00:00 - .
      </description>
      <when unit="sec">0</when>
      <timeout unit="sec">50</timeout>
      <depth unit="m">10.0</depth>
      <latitude>42.656040</latitude>
      <longitude>-70.591213</longitude>
      <radius unit="m">14.0</radius>
      <speed_knots>0.0</speed_knots>
  </waypoint>
*/

#ifndef MISSION_CONTROL_BEHAVIORS_WAYPOINT_H
#define MISSION_CONTROL_BEHAVIORS_WAYPOINT_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include "mission_control/behavior.h"
using namespace BT;

namespace mission_control
{
struct WaypointData
{
  double depth;
  double altitude;
  double latitude;
  double longitude;
  double wp_radius;
  double speed_knots;
};

}  // namespace mission_control

namespace BT
{
template <>
inline mission_control::WaypointData convertFromString<mission_control::WaypointData>(
    StringView str)

{
  // real numbers separated by semicolons
  auto parts = splitString(str, ';');
  if (parts.size() != 6)
  {
    throw RuntimeError("invalid input)");
  }
  else
  {
    mission_control::WaypointData output;
    output.depth = convertFromString<double>(parts[0]);
    output.altitude = convertFromString<double>(parts[1]);
    output.latitude = convertFromString<double>(parts[2]);
    output.longitude = convertFromString<double>(parts[3]);
    output.wp_radius = convertFromString<double>(parts[4]);
    output.speed_knots = convertFromString<double>(parts[5]);
    return output;
  }
}
}  // namespace BT

namespace mission_control
{
class Waypoint : public Behavior
{
 public:
  Waypoint(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus getBehaviorStatus();
  void abortBehavior();
  void publishMsg();

  static PortsList providedPorts()
  {
    const char* description = "depth;altitude;latitude;longitude;wp_radius;speed_knots";
    return {InputPort<WaypointData>("value", description)};
  }

 private:
  ros::NodeHandle nodeHandle;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_WAYPOINT_H
