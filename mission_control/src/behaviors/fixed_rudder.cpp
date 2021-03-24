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
#include "mission_control/behaviors/fixed_rudder.h"

#include <string>

using mission_control::MoveWithFixedRudder;

MoveWithFixedRudder::MoveWithFixedRudder(const std::string& name,
                                         const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  depthEnable_ = false;
  rudderEnable_ = altitudeEnable_ = false;
  speedKnotsEnable_ = false;

  if (getInput<double>("depth", depth_))
  {
    depthEnable_ = true;
    getInput<double>("depth_tol", depthTolerance_);
  }

  if (getInput<double>("altitude", altitude_))
  {
    altitudeEnable_ = true;
    getInput<double>("altitude_tol", altitudeTolerance_);
  }

  if (getInput<double>("rudder", rudder_))
  {
    rudderEnable_ = true;
    getInput<double>("rudder_tol", rudderTolerance_);
  }
  if (getInput<double>("speed_knots", speedKnots_)) speedKnotsEnable_ = true;

  subStateData_ = nodeHandle_.subscribe("/state", 1, &MoveWithFixedRudder::stateDataCallback, this);

  goalHasBeenPublished_ = false;
  fixedRudderBehaviorPub_ =
      nodeHandle_.advertise<mission_control::FixedRudder>("/mngr/fixed_rudder", 1);

  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus MoveWithFixedRudder::behaviorRunningProcess()
{
  if (!goalHasBeenPublished_)
  {
    auto res = getInput<std::string>("behavior_time");
    if (!res)
    {
      ROS_ERROR_STREAM("error reading port [behavior_time]:" << res.error());
      setStatus(BT::NodeStatus::FAILURE);
    }
    else
    {
      getInput<double>("behavior_time", behaviorTime_);
      behaviorStartTime_ = ros::Time::now();
      publishGoalMsg();
      goalHasBeenPublished_ = true;
    }
  }
  else
  {
    ros::Duration delta_t = ros::Time::now() - behaviorStartTime_;
    if (delta_t.toSec() > behaviorTime_)
    {
      setStatus(BT::NodeStatus::SUCCESS);
      goalHasBeenPublished_ = false;
    }
  }
  return (status());
}

void MoveWithFixedRudder::publishGoalMsg()
{
  FixedRudder msg;

  msg.ena_mask = 0x0;

  if (depthEnable_)
  {
    msg.depth = depth_;
    msg.ena_mask |= FixedRudder::DEPTH_ENA;
  }

  if (rudderEnable_)
  {
    msg.ena_mask |= FixedRudder::RUDDER_ENA;
    msg.rudder = rudder_;
  }

  if (speedKnotsEnable_)
  {
    msg.speed_knots = speedKnots_;
    msg.ena_mask |= FixedRudder::SPEED_KNOTS_ENA;
  }

  msg.header.stamp = ros::Time::now();

  fixedRudderBehaviorPub_.publish(msg);
}

void MoveWithFixedRudder::stateDataCallback(const auv_interfaces::StateStamped& data)
{
  // A quick check to see if our depth matches, note: rudder is not part of corrected data.
  // TODO(QNA): put back in when depth working  if (depthEnable_ && (abs(depth_ - data.depth) >
  // depth_Tolerance_)) return false;
  // TODO(QNA): check shaft speed?
}
