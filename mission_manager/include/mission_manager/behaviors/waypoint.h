/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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

#ifndef MISSION_MANAGER_BEHAVIORS_WAYPOINT_H
#define MISSION_MANAGER_BEHAVIORS_WAYPOINT_H

#include "mission_manager/behavior.h"
#include "mission_manager/Waypoint.h"  // this is the ROS Message

namespace mission_manager
{

class WaypointBehavior : public Behavior
{
 public:
  WaypointBehavior();
  virtual ~WaypointBehavior();

  virtual bool parseMissionFileParams();
  bool getParams(ros::NodeHandle nh);

  virtual void publishMsg();
  bool checkState(const auv_interfaces::StateStamped& data);

 private:
  ros::Publisher waypoint_behavior_pub;

  float m_altitude;
  float m_depth;
  float m_lat;
  float m_long;
  float m_speed_knots;
  double m_wp_radius;

  bool m_altitude_ena;
  bool m_depth_ena;
  bool m_lat_ena;
  bool m_long_ena;
  bool m_speed_knots_ena;
  bool m_wp_radius_ena;

  float m_depth_tol;
  void latLongtoUTM(double latitude, double longitude, double* ptrNorthing, double* ptrEasting);
  double degreesToRadians(double degrees);
};

}   //  namespace mission_manager

#endif  //  MISSION_MANAGER_BEHAVIORS_WAYPOINT_H
