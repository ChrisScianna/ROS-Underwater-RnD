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
#include "mission_control/behaviors/go_to_waypoint.h"

#include "mission_control/Waypoint.h"

#include <string>

namespace mission_control
{

GoToWaypointNode::GoToWaypointNode(const std::string& name, const BT::NodeConfiguration& config)
  : ReactiveActionNode(name, config)
{
  waypoint_pub_ = nh_.advertise<mission_control::Waypoint>("/mngr/waypoint", 1);
}

BT::NodeStatus GoToWaypointNode::setUp()
{
  // Update action parameters
  if (!getInput<double>("latitude", latitude_) || !getInput<double>("longitude", longitude_))
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "', action needs latitude and longitude");
    return BT::NodeStatus::FAILURE;
  }
  geodesy::fromMsg(geodesy::toMsg(latitude_, longitude_), target_position_);
  enable_mask_ = mission_control::Waypoint::LAT_ENA | mission_control::Waypoint::LONG_ENA;

  altitude_ = depth_ = 0.0;
  if (getInput<double>("altitude", altitude_))
  {
    enable_mask_ |= mission_control::Waypoint::ALTITUDE_ENA;
    target_position_.altitude = altitude_;
  }
  else if (getInput<double>("depth", depth_))
  {
    enable_mask_ |= mission_control::Waypoint::DEPTH_ENA;
    target_position_.altitude = depth_;
  }
  else
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "', action needs altitude or depth");
    return BT::NodeStatus::FAILURE;
  }

  if (getInput<double>("speed_knots", speed_knots_))
  {
    enable_mask_ |= mission_control::Waypoint::SPEED_KNOTS_ENA;
  }
  else
  {
    speed_knots_ = 0.0;
  }

  getInput<double>("tolerance_radius", tolerance_radius_);

  // Setup state subscriber
  state_.reset();
  state_sub_ = nh_.subscribe(
      "/state", 1, &GoToWaypointNode::stateCallback, this);

  // Publish position setpoint
  mission_control::Waypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.latitude = latitude_;
  msg.longitude = longitude_;
  msg.altitude = altitude_;
  msg.depth = depth_;
  msg.speed_knots = speed_knots_;
  msg.ena_mask = enable_mask_;
  waypoint_pub_.publish(msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToWaypointNode::doWork()
{
  if (state_)
  {
    geodesy::UTMPoint current_position(state_->state.geolocation.position);
    if (enable_mask_ & mission_control::Waypoint::DEPTH_ENA)  // Use depth instead
    {
      current_position.altitude = state_->state.manoeuvring.pose.mean.position.z;
    }
    double distance_to_waypoint =
        std::sqrt(std::pow(target_position_.northing - current_position.northing, 2) +
                  std::pow(target_position_.easting - current_position.easting, 2) +
                  std::pow(target_position_.altitude - current_position.altitude, 2));
    ROS_DEBUG("Distance to (%f, %f) waypoint is %f m",
              latitude_, longitude_, distance_to_waypoint);
    // TODO(QNA): check shaft speed?
    if (distance_to_waypoint < tolerance_radius_)
    {
      ROS_INFO("Arrived to the (%f, %f) waypoint!", latitude_, longitude_);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void GoToWaypointNode::tearDown()
{
  state_sub_.shutdown();
}

void GoToWaypointNode::stateCallback(auv_interfaces::StateStamped::ConstPtr msg)
{
  state_ = msg;
}

}  // namespace mission_control
