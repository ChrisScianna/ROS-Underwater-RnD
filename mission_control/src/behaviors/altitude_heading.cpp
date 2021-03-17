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
#include "mission_control/behaviors/altitude_heading.h"

#include <string>

using mission_control::AltitudeHeadingBehavior;

AltitudeHeadingBehavior::AltitudeHeadingBehavior(const std::string &name,
                                                 const BT::NodeConfiguration &config)
    : Behavior(name, config)
{
  altitudeEnable_ = false;
  headingEnable_ = false;
  speedKnotsEnable_ = false;
  timeOutEnable_ = false;

  getInput<double>("altitude", altitude_);
  if (altitude_)
  {
    altitudeEnable_ = true;
    getInput<double>("altitude_tol", altitudeTolerance_);
  }

  getInput<double>("heading", heading_);
  if (heading_)
  {
    headingEnable_ = true;
    getInput<double>("heading_tol", headingTolerance_);
  }

  getInput<double>("speed_knots", speedKnots_);
  if (speedKnots_) speedKnotsEnable_ = true;

  getInput<double>("time_out", timeOut_);
  if (timeOut_) timeOutEnable_ = true;

  subCorrectedData_ =
      nodeHandle_.subscribe("/state", 1, &AltitudeHeadingBehavior::stateDataCallback, this);

  altitudeHeadingBehaviorPub_ =
      nodeHandle_.advertise<mission_control::AltitudeHeading>("/mngr/altitude_heading", 100);

  goalHasBeenPublished_ = false;
  behaviorComplete_ = false;
  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus AltitudeHeadingBehavior::behaviorRunningProcess()
{
  if (!goalHasBeenPublished_)
  {
    behaviorComplete_ = false;
    publishGoalMsg();
    behaviorStartTime_ = ros::Time::now();
    goalHasBeenPublished_ = true;
  }
  else
  {
    ros::Duration delta_t = ros::Time::now() - behaviorStartTime_;
    if (delta_t.toSec() > timeOut_ && timeOutEnable_)
    {
      goalHasBeenPublished_ = false;
      setStatus(BT::NodeStatus::FAILURE);
    }
    else
    {
      if (behaviorComplete_)
      {
        setStatus(BT::NodeStatus::SUCCESS);
        goalHasBeenPublished_ = false;
      }
    }
  }
  return status();
}

void AltitudeHeadingBehavior::publishGoalMsg()
{
  AltitudeHeading msg;

  msg.altitude = altitude_;
  msg.heading = heading_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 0x0;
  if (altitudeEnable_) msg.ena_mask |= AltitudeHeading::ALTITUDE_ENA;
  if (headingEnable_) msg.ena_mask |= AltitudeHeading::HEADING_ENA;
  if (speedKnotsEnable_) msg.ena_mask |= AltitudeHeading::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();
  altitudeHeadingBehaviorPub_.publish(msg);
}

void AltitudeHeadingBehavior::stateDataCallback(const auv_interfaces::StateStamped &data)
{
  double heading = data.state.manoeuvring.pose.mean.orientation.z;
  // A quick check to see if our RPY angles match
  if (headingEnable_ && (abs(heading_ - heading) < headingTolerance_)) behaviorComplete_ = true;
}
