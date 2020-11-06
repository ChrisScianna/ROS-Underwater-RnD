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

#include "behaviors/waypoint.h"

#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "mission_manager/Waypoint.h"

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

using namespace mission_manager;

WaypointBehavior::WaypointBehavior() : Behavior("waypoint", BEHAVIOR_TYPE_MSG, "/mngr/waypoint", "")
{
  m_altitude_ena = false;
  m_depth_ena = false;
  m_speed_knots_ena = false;
  m_lat_ena = false;
  m_long_ena = false;
  m_wp_radius_ena = false;

  m_depth = 0.0;
  m_speed_knots = 0.0;
  m_lat = 0;
  m_long = 0;
  m_wp_radius = 0;

  m_depth_tol = 0.0;
  m_duration = -1;

  ros::NodeHandle nh;
  waypoint_behavior_pub = nh.advertise<mission_manager::Waypoint>("/mngr/waypoint", 1);
}

WaypointBehavior::~WaypointBehavior() {}

double WaypointBehavior::degreesToRadians(double degrees) { return ((degrees / 180.0) * M_PI); }
void WaypointBehavior::latLongtoUTM(double latitude, double longitude, double* ptrNorthing,
                                    double* ptrEasting)
{
  int zone;
  double Lat = latitude;
  double Long = longitude;
  double easting;
  double northing;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;
  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  // (JOQ: this is broken for Long < -180, do a real normalize)

  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

  double LatRad = degreesToRadians(Lat);

  double LongRad = degreesToRadians(LongTemp);

  double LongOriginRad;

  zone = int((LongTemp + 180) / 6) + 1;

  if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0) zone = 32;

  // Special zones for Svalbard

  if (Lat >= 72.0 && Lat < 84.0)

  {
    if (LongTemp >= 0.0 && LongTemp < 9.0)
      zone = 31;

    else if (LongTemp >= 9.0 && LongTemp < 21.0)
      zone = 33;

    else if (LongTemp >= 21.0 && LongTemp < 33.0)
      zone = 35;

    else if (LongTemp >= 33.0 && LongTemp < 42.0)
      zone = 37;
  }

  // +3 puts origin in middle of zone

  LongOrigin = (zone - 1) * 6 - 180 + 3;

  LongOriginRad = degreesToRadians(LongOrigin);

#if 0

     if (to.band == ' ')

       throw std::range_error;

#endif

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);
  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);
  M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
            5 * eccSquared * eccSquared * eccSquared / 256) *
               LatRad -
           (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
            45 * eccSquared * eccSquared * eccSquared / 1024) *
               sin(2 * LatRad) +
           (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) *
               sin(4 * LatRad) -
           (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  easting = (double)

      (k0 * N *
           (A + (1 - T + C) * A * A * A / 6 +
            (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) +
       500000.0);
  northing = (double)(k0 * (M + N * tan(LatRad) *
                                    (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                                     (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A *
                                         A * A * A * A * A / 720)));

  if (Lat < 0)
  {
    // 10000000 meter offset for southern hemisphere
    northing += 10000000.0;
  }

  *ptrNorthing = northing;
  *ptrEasting = easting;
}

bool WaypointBehavior::getParams(ros::NodeHandle nh)
{
  double f = 0.0;
  return true;
}

bool WaypointBehavior::parseMissionFileParams()
{
  bool retval = true;
  ROS_INFO("WaypointBehavior::parseMissionFileParams - xmlparams size = %ld",
           m_behaviorXMLParams.size());
  std::list<BehaviorXMLParam>::iterator it;
  for (it = m_behaviorXMLParams.begin(); it != m_behaviorXMLParams.end(); it++)
  {
    std::string xmlParamTag = it->getXMLTag();
    if ((xmlParamTag.compare("when") == 0) || (xmlParamTag.compare("timeout") == 0))
    {
      retval = parseTimeStamps(it);
    }
    else if (xmlParamTag.compare("depth") == 0)
    {
      m_depth = std::atof(it->getXMLTagValue().c_str());
      m_depth_ena = true;
    }
    else if (xmlParamTag.compare("altitude") == 0)
    {
      m_altitude = std::atof(it->getXMLTagValue().c_str());
      m_altitude_ena = true;
    }
    else if (xmlParamTag.compare("latitude") == 0)
    {
      m_lat = std::atof(it->getXMLTagValue().c_str());
      m_lat_ena = true;
    }
    else if (xmlParamTag.compare("longitude") == 0)
    {
      m_long = std::atof(it->getXMLTagValue().c_str());
      m_long_ena = true;
    }
    else if (xmlParamTag.compare("radius") == 0)
    {
      m_wp_radius = std::atof(it->getXMLTagValue().c_str());
      m_wp_radius_ena = true;
    }
    else if (xmlParamTag.compare("speed_knots") == 0)
    {
      m_speed_knots = std::atof(it->getXMLTagValue().c_str());
      m_speed_knots_ena = true;
    }
    else
    {
      ROS_INFO("Waypoint behavior found invalid parameter [%s]", xmlParamTag.c_str());
    }
  }

  return retval;
}

void WaypointBehavior::publishMsg()
{
  Waypoint msg;

  msg.depth = m_depth;
  msg.latitude = m_lat;
  msg.longitude = m_long;
  msg.speed_knots = m_speed_knots;

  msg.ena_mask = 0x0;
  if (m_depth_ena) msg.ena_mask |= Waypoint::DEPTH_ENA;
  if (m_lat_ena) msg.ena_mask |= Waypoint::LAT_ENA;
  if (m_long_ena) msg.ena_mask |= Waypoint::LONG_ENA;
  if (m_speed_knots_ena) msg.ena_mask |= Waypoint::SPEED_KNOTS_ENA;
  // note: radius is not passed to autopilot, mission manger will determined if we have arrived at
  // waypoint then
  // send autopilot the next waypoint.

  while (0 == waypoint_behavior_pub.getNumSubscribers())
  {
    ROS_INFO("Waiting for waypoint subscribers to connect");
    ros::Duration(0.1).sleep();
  }

  msg.header.stamp = ros::Time::now();

  waypoint_behavior_pub.publish(msg);
}

bool WaypointBehavior::checkCorrectedData(const pose_estimator::CorrectedData& data)
{
  if (m_behavior_done)
  {
    return true;
  }
  double dist_to_wp;
  double currentNorthing;
  double currentEasting;

  double desiredNorthing;
  double desiredEasting;

  latLongtoUTM(data.position.latitude, data.position.longitude, &currentNorthing, &currentEasting);
  latLongtoUTM(m_lat, m_long, &desiredNorthing, &desiredEasting);

  // formula for distance between two 3D points is d=sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
  // investigating hypot it looks like it does the squaring and square root to figure out the return
  // value. Need to add depth or altitude (whatever we are using).
  //   dist_to_wp = std::hypot((currentNorthing-desiredNorthing, currentEasting-desiredEasting,
  //   data.depth-m_depth);
  // dist_to_wp = hypot(abs(currentNorthing-desiredNorthing), abs(currentEasting-desiredEasting));

  dist_to_wp = sqrt(pow((currentNorthing - desiredNorthing), 2) +
                    pow((currentEasting - desiredEasting), 2) + pow((data.depth - m_depth), 2));

  if (dist_to_wp < m_wp_radius)
  {
    m_behavior_done = true;
    ROS_INFO("We have arrived at waypoint distance to wp [%f] , wp radius [%f]", dist_to_wp,
             m_wp_radius);
    return true;
  }

  // TODO: check shaft speed?
  return false;
}
