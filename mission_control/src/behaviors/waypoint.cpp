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
#include <string>
#include "mission_control/behaviors/waypoint.h"

using mission_control::GoToWaypoint;

GoToWaypoint::GoToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  getInput<double>("depth", depth_);
  if (depth_ != 0.0) depthEnable_ = true;

  getInput<double>("altitude", altitude_);
  if (altitude_ != 0.0) altitudeEnable_ = true;

  getInput<double>("latitude", lat_);
  if (lat_ != 0.0) latEnable_ = true;

  getInput<double>("longitude", long_);
  if (long_ != 0.0) longEnable_ = true;

  getInput<double>("wp_radius", wpRadius_);
  if (wpRadius_ != 0.0) wpRadiusEnable_ = true;

  getInput<double>("speed_knots", speedKnots_);
  if (speedKnots_ != 0.0) speedKnotsEnable_ = true;

  getInput<double>("time_out", timeOut_);
  subCorrectedData_ =
      nodeHandle_.subscribe("/pose/corrected_data", 1, &GoToWaypoint::correctedDataCallback, this);

  goalHasBeenPublished_ = false;
  waypointBehaviorPub_ = nodeHandle_.advertise<mission_control::Waypoint>("/mngr/waypoint", 1);
}

BT::NodeStatus GoToWaypoint::behaviorRunningProcess()
{
  if (!goalHasBeenPublished_)
  {
    publishGoalMsg();
    goalHasBeenPublished_ = true;
  }
  else
  {
    ros::Duration delta_t = ros::Time::now() - behaviorStartTime_;
    if (delta_t.toSec() > timeOut_) setStatus(BT::NodeStatus::FAILURE);
  }
  return (status());
}

void GoToWaypoint::publishGoalMsg()
{  // Altitude is not used.
  Waypoint msg;
  msg.depth = depth_;
  msg.latitude = lat_;
  msg.longitude = long_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 0x0;
  if (depthEnable_) msg.ena_mask |= Waypoint::DEPTH_ENA;
  if (latEnable_) msg.ena_mask |= Waypoint::LAT_ENA;
  if (longEnable_) msg.ena_mask |= Waypoint::LONG_ENA;
  if (speedKnotsEnable_) msg.ena_mask |= Waypoint::SPEED_KNOTS_ENA;

  // note: radius is not passed to autopilot, mission manger will determined if we have arrived at
  // waypoint then
  // send autopilot the next waypoint.

  msg.header.stamp = ros::Time::now();
  waypointBehaviorPub_.publish(msg);
}

void GoToWaypoint::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  if (status() == BT::NodeStatus::RUNNING)
  {
    double distToWaypoint;
    double currentNorthing;
    double currentEasting;

    double desiredNorthing;
    double desiredEasting;

    latLongtoUTM(data.position.latitude, data.position.longitude, &currentNorthing,
                 &currentEasting);
    latLongtoUTM(lat_, long_, &desiredNorthing, &desiredEasting);

    // formula for distance between two 3D points is d=sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
    // investigating hypot it looks like it does the squaring and square root to figure out the
    // return value. Need to add depth or altitude (whatever we are using).
    //   distToWaypoint = std::hypot((currentNorthing-desiredNorthing,
    //   currentEasting-desiredEasting, data.depth-depth_);
    // distToWaypoint = hypot(abs(currentNorthing-desiredNorthing),
    // abs(currentEasting-desiredEasting));

    distToWaypoint =
        sqrt(pow((currentNorthing - desiredNorthing), 2) +
             pow((currentEasting - desiredEasting), 2) + pow((data.depth - depth_), 2));

    if (distToWaypoint < wpRadius_)
    {
      ROS_INFO("We have arrived at waypoint distance to wp [%f] , wp radius [%f]", distToWaypoint,
               wpRadius_);
      setStatus(BT::NodeStatus::SUCCESS);
    }
    else
    {
      setStatus(BT::NodeStatus::RUNNING);
    }
  }
  // TODO(QNA): check shaft speed?
}

void mission_control::latLongtoUTM(double latitude, double longitude, double* ptrNorthing,
                                   double* ptrEasting)
{
  // WGS84 Parameters
  static constexpr double WGS84_A = 6378137.0;         // major axis
  static constexpr double WGS84_B = 6356752.31424518;  // minor axis
  static constexpr double WGS84_F = 0.0033528107;      // ellipsoid flattening
  static constexpr double WGS84_E = 0.0818191908;      // first eccentricity
  static constexpr double WGS84_EP = 0.0820944379;     // second eccentricity

  // UTM Parameters
  static constexpr double UTM_K0 = 0.9996;                    // scale factor
  static constexpr double UTM_FE = 500000.0;                  // false easting
  static constexpr double UTM_FN_N = 0.0;                     // false northing, northern hemisphere
  static constexpr double UTM_FN_S = 10000000.0;              // false northing, southern hemisphere
  static constexpr double UTM_E2 = (WGS84_E * WGS84_E);       // e^2
  static constexpr double UTM_E4 = (UTM_E2 * UTM_E2);         // e^4
  static constexpr double UTM_E6 = (UTM_E4 * UTM_E2);         // e^6
  static constexpr double UTM_EP2 = (UTM_E2 / (1 - UTM_E2));  // e'^2

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

  double LongTemp = (Long + 180) - static_cast<int>((Long + 180) / 360) * 360 - 180;

  double LatRad = degreesToRadians(Lat);

  double LongRad = degreesToRadians(LongTemp);

  double LongOriginRad;

  zone = static_cast<int>((LongTemp + 180) / 6) + 1;

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

  easting = static_cast<double>(

      (k0 * N *
           (A + (1 - T + C) * A * A * A / 6 +
            (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) +
       500000.0));
  northing = static_cast<double>(
      k0 * (M + N * tan(LatRad) *
                    (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                     (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A *
                         A / 720)));

  if (Lat < 0)
  {
    // 10000000 meter offset for southern hemisphere
    northing += 10000000.0;
  }

  *ptrNorthing = northing;
  *ptrEasting = easting;
}
