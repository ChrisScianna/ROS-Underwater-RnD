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
#include "mission_control/behaviors/set_altitude_heading.h"

#include "mission_control/AltitudeHeading.h"

#include <string>

namespace mission_control
{

SetAltitudeHeadingNode::SetAltitudeHeadingNode(
    const std::string &name, const BT::NodeConfiguration &config)
  : ReactiveActionNode(name, config)
{
  altitude_heading_pub_ =
    nh_.advertise<mission_control::AltitudeHeading>("/mngr/altitude_heading", 1, true);
}

BT::NodeStatus SetAltitudeHeadingNode::setUp()
{
  // Update action parameters
  enable_mask_ = 0u;

  if (getInput<double>("altitude", target_altitude_))
  {
    getInput<double>("altitude_tol", altitude_tolerance_);
    enable_mask_ |= mission_control::AltitudeHeading::ALTITUDE_ENA;
  }
  else
  {
    target_altitude_ = 0.0;
  }

  if (getInput<double>("heading", target_heading_))
  {
    getInput<double>("heading_tol", heading_tolerance_);
    enable_mask_ |= mission_control::AltitudeHeading::HEADING_ENA;
  }
  else
  {
    target_heading_ = 0.0;
  }

  if (getInput<double>("speed_knots", speed_knots_))
  {
    enable_mask_ |= mission_control::AltitudeHeading::SPEED_KNOTS_ENA;
  }
  else
  {
    speed_knots_ = 0.0;
  }

  // Setup state subscriber
  state_.reset();
  state_sub_ = nh_.subscribe(
      "/state", 1, &SetAltitudeHeadingNode::stateDataCallback, this);

  // Publish altitude+heading setpoint
  mission_control::AltitudeHeading msg;
  msg.header.stamp = ros::Time::now();
  msg.altitude = target_altitude_;
  msg.heading = target_heading_;
  msg.speed_knots = speed_knots_;
  msg.ena_mask = enable_mask_;
  altitude_heading_pub_.publish(msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetAltitudeHeadingNode::doWork()
{
  if (!state_)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (enable_mask_ & mission_control::AltitudeHeading::HEADING_ENA)
  {
    double current_heading = state_->state.manoeuvring.pose.mean.orientation.z;

    if (std::abs(target_heading_ - current_heading) > heading_tolerance_)
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  if (enable_mask_ & mission_control::AltitudeHeading::ALTITUDE_ENA)
  {
    double current_altitude = state_->state.geolocation.position.altitude;

    if (std::abs(target_altitude_ - current_altitude) > altitude_tolerance_)
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void SetAltitudeHeadingNode::tearDown()
{
  state_sub_.shutdown();
}

void SetAltitudeHeadingNode::stateDataCallback(auv_interfaces::StateStamped::ConstPtr msg)
{
  state_ = msg;
}

}  // namespace mission_control
