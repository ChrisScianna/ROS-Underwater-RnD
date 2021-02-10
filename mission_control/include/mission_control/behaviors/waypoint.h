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

#include "mission_control/Waypoint.h"
#include "mission_control/behavior.h"
#include "pose_estimator/CorrectedData.h"

using namespace BT;

// WGS84 Parameters
#define WGS84_A 6378137.0         // major axis
#define WGS84_B 6356752.31424518  // minor axis
#define WGS84_F 0.0033528107      // ellipsoid flattening
#define WGS84_E 0.0818191908      // first eccentricity
#define WGS84_EP 0.0820944379     // second eccentricity

// UTM Parameters
#define UTM_K0 0.9996                    // scale factor
#define UTM_FE 500000.0                  // false easting
#define UTM_FN_N 0.0                     // false northing, northern hemisphere
#define UTM_FN_S 10000000.0              // false northing, southern hemisphere
#define UTM_E2 (WGS84_E * WGS84_E)       // e^2
#define UTM_E4 (UTM_E2 * UTM_E2)         // e^4
#define UTM_E6 (UTM_E4 * UTM_E2)         // e^6
#define UTM_EP2 (UTM_E2 / (1 - UTM_E2))  // e'^2

namespace mission_control
{
class WaypointBehavior : public Behavior
{
 public:
  WaypointBehavior(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus getBehaviorStatus();

  static PortsList providedPorts()
  {
    BT::PortsList ports = {BT::InputPort<double>("depth", 0.0, "depth"),
                           BT::InputPort<double>("altitude", 0.0, "altitude"),
                           BT::InputPort<double>("latitude", 0.0, "latitude"),
                           BT::InputPort<double>("longitude", 0.0, "longitude"),
                           BT::InputPort<double>("wp_radius", 0.0, "wp_radius"),
                           BT::InputPort<double>("speed_knots", 0.0, "speed_knots")};
    return ports;
  }

 private:
  ros::NodeHandle nodeHandle;
  ros::Publisher waypoint_behavior_pub;
  ros::Subscriber sub_corrected_data;

  double m_altitude;
  double m_depth;
  double m_lat;
  double m_long;
  double m_speed_knots;
  double m_wp_radius;

  bool m_altitude_ena;
  bool m_depth_ena;
  bool m_lat_ena;
  bool m_long_ena;
  bool m_speed_knots_ena;
  bool m_wp_radius_ena;

  double m_depth_tol;

  void latLongtoUTM(double latitude, double longitude, double* ptrNorthing, double* ptrEasting);
  double degreesToRadians(double degrees);

  void correctedDataCallback(const pose_estimator::CorrectedData& data);
  bool goalHasBeenPublished;
  void publishGoalMsg();
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_WAYPOINT_H
