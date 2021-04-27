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
#include "mission_control/behaviors/fix_rudder.h"

#include "mission_control/FixedRudder.h"
#include "mission_control/behaviors/helpers.h"

#include <string>

namespace mission_control
{

FixRudderNode::FixRudderNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  fixedRudderBehaviorPub_ =
    nodeHandle_.advertise<mission_control::FixedRudder>("/mngr/fixed_rudder", 1, true);
}

BT::NodeStatus FixRudderNode::tick()
{
  mission_control::FixedRudder msg;
  msg.header.stamp = ros::Time::now();

  msg.ena_mask = 0u;

  double depth;
  auto result = getInputValue<double, HasAngleUnits>(this, "depth", depth);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(depth))
  {
    msg.ena_mask |= mission_control::FixedRudder::DEPTH_ENA;
    msg.depth = depth;
  }

  double rudder;
  result = getInputValue<double, HasAngleUnits>(this, "rudder", rudder);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(rudder))
  {
    msg.ena_mask |= mission_control::FixedRudder::RUDDER_ENA;
    msg.rudder = rudder;
  }

  double speed_knots;
  result = getInput<double>("speed_knots", speed_knots);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(speed_knots))
  {
    msg.ena_mask |= mission_control::FixedRudder::SPEED_KNOTS_ENA;
    msg.speed_knots = speed_knots;
  }

  fixedRudderBehaviorPub_.publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mission_control
