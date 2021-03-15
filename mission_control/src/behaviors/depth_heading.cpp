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
#include "mission_control/behaviors/depth_heading.h"

#include <string>

using mission_control::DepthHeadingBehavior;

DepthHeadingBehavior::DepthHeadingBehavior(const std::string &name,
                                           const BT::NodeConfiguration &config)
    : Behavior(name, config)
{
  depthEnable_ = false;
  headingEnable_ = false;
  speedKnotsEnable_ = false;
  depthTolerance_ = 0.0;
  headingTolerance_ = 0.0;
  depth_ = 0.0;
  heading_ = 0.0;
  speedKnots_ = 0.0;

  getInput<double>("depth", depth_);
  if (depth_ != 0.0)
    depthEnable_ = true;

  getInput<double>("heading", heading_);
  if (heading_ != 0.0)
    headingEnable_ = true;

  getInput<double>("speed_knots", speedKnots_);
  if (speedKnots_ != 0.0)
    speedKnotsEnable_ = true;

  getInput<double>("depth_tol", depthTolerance_);
  getInput<double>("heading_tol", headingTolerance_);
  getInput<double>("time_out", timeOut_);

  subCorrectedData_ = nodeHandle_.subscribe("/pose/corrected_data", 1,
                                            &DepthHeadingBehavior::correctedDataCallback, this);

  goalHasBeenPublished_ = false;
  depthHeadingBehaviorPub =
      nodeHandle_.advertise<mission_control::DepthHeading>("/mngr/depth_heading", 100);

  behaviorComplete_ = false;
  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus DepthHeadingBehavior::behaviorRunningProcess()
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
    if (delta_t.toSec() > timeOut_)
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

void DepthHeadingBehavior::publishGoalMsg()
{
  mission_control::DepthHeading msg;

  msg.depth = depth_;
  msg.heading = heading_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 0x0;
  if (depthEnable_) msg.ena_mask |= DepthHeading::DEPTH_ENA;
  if (headingEnable_) msg.ena_mask |= DepthHeading::HEADING_ENA;
  if (speedKnotsEnable_) msg.ena_mask |= DepthHeading::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  depthHeadingBehaviorPub.publish(msg);
}

void DepthHeadingBehavior::void stateDataCallback(const auv_interfaces::StateStamped& data)
{
  double heading = data.state.manoeuvring.pose.mean.orientation.z;
  // A quick check to see if our RPY angles match
  if (headingEnable_ && (fabs(heading_ - heading) < headingTolerance_))
    behaviorComplete_ = true;
}
