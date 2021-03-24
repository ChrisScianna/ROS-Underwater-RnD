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

#include <geodesy/utm.h>

#include "mission_control/behaviors/waypoint.h"

namespace mission_control
{

GoToWaypoint::GoToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
  if (!getInput<double>("latitude", latitude_) || !getInput<double>("longitude", longitude_))
  {
    throw std::runtime_error("Either latitude or longitude were left unspecified");
  }
  geodesy::fromMsg(geodesy::toMsg(latitude_, longitude_), target_position_);
  enable_mask_ = Waypoint::LAT_ENA | Waypoint::LONG_ENA;

  if (getInput<double>("altitude", altitude_))
  {
    enable_mask_ |= Waypoint::ALTITUDE_ENA;
    target_position_.altitude = altitude_;
  }
  else if (getInput<double>("depth", depth_))
  {
    enable_mask_ |= Waypoint::DEPTH_ENA;
    target_position_.altitude = depth_;
  }
  else
  {
    throw std::runtime_error("Neither altitude nor depth were specified");
  }

  if (getInput<double>("speed_knots", speed_knots_))
  {
    enable_mask_ |= Waypoint::SPEED_KNOTS_ENA;
  }

  getInput<double>("tolerance_radius", tolerance_radius_);

  waypoint_pub_ = nh_.advertise<mission_control::Waypoint>("/mngr/waypoint", 1);
}

BT::NodeStatus GoToWaypoint::tick()
{
  BT::NodeStatus current_status = status();

  switch (current_status)
  {
    case BT::NodeStatus::IDLE:
      state_up_to_date_ = false;
      state_sub_ = nh_.subscribe(
        "/state", 1, &GoToWaypoint::stateCallback, this);
      waypoint_pub_.publish(makeWaypointMsg());
      current_status = BT::NodeStatus::RUNNING;
      break;
    case BT::NodeStatus::RUNNING:
      if (state_up_to_date_)
      {
        double distance_to_waypoint =
          std::sqrt(std::pow(target_position_.northing - current_position_.northing, 2) +
                    std::pow(target_position_.easting - current_position_.easting, 2) +
                    std::pow(target_position_.altitude - current_position_.altitude, 2));
        ROS_INFO("distance to wp [%f], tolerance radius [%f]",
                 distance_to_waypoint, tolerance_radius_);
        if (distance_to_waypoint < tolerance_radius_)
        {
          ROS_INFO("We have arrived at waypoint: "
                   "distance to wp [%f], tolerance radius [%f]",
                   distance_to_waypoint, tolerance_radius_);
          current_status = BT::NodeStatus::SUCCESS;
          state_sub_.shutdown();
        }
      }
      break;
    default:
      break;
  }
  return current_status;
}

void GoToWaypoint::halt()
{
  if (status() == BT::NodeStatus::RUNNING)
  {
    state_sub_.shutdown();
  }
  setStatus(BT::NodeStatus::IDLE);
}

mission_control::Waypoint GoToWaypoint::makeWaypointMsg()
{
  mission_control::Waypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.latitude = latitude_;
  msg.longitude = longitude_;
  msg.altitude = altitude_;
  msg.depth = depth_;
  msg.speed_knots = speed_knots_;
  msg.ena_mask = enable_mask_;
  return msg;
}

void GoToWaypoint::stateCallback(const auv_interfaces::StateStamped& msg)
{
  geodesy::fromMsg(msg.state.geolocation.position, current_position_);
  if (enable_mask_ & mission_control::Waypoint::DEPTH_ENA)  // Use depth instead
  {
    current_position_.altitude = msg.state.manoeuvring.pose.mean.position.z;
  }
  state_up_to_date_ = true;
  // TODO(QNA): check shaft speed?
}

}  // namespace mission_control
