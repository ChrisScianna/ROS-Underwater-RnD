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

using namespace mission_control;

DepthHeadingBehavior::DepthHeadingBehavior(const std::string& name,
                                           const BT::NodeConfiguration& config)
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
  if (depth_ != 0.0) depthEnable_ = true;

  getInput<double>("heading", heading_);
  if (heading_ != 0.0) headingEnable_ = true;

  getInput<double>("speedKnots", speedKnots_);
  if (speedKnots_ != 0.0) speedKnotsEnable_ = true;

  getInput<double>("depth_tol", depthTolerance_);
  getInput<double>("heading_tol", headingTolerance_);
  getInput<double>("time_out", timeOut_);

  sub_corrected_data_ = nodeHandle_.subscribe("/pose/corrected_data", 1,
                                              &DepthHeadingBehavior::correctedDataCallback, this);

  goalHasBeenPublished_ = false;
  depth_heading_behavior_pub =
      nodeHandle_.advertise<mission_control::DepthHeading>("/mngr/depth_heading", 100);

  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus DepthHeadingBehavior::behaviorRunningProcess()
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

  depth_heading_behavior_pub.publish(msg);
}

void DepthHeadingBehavior::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  // A quick check to see if our RPY angles match
  // tjw debug  if (m_depth_ena && (abs(m_depth - data.depth) > m_depth_tol)) return false;
  if (headingEnable_ && (abs(heading_ - data.rpy_ang.z) > headingTolerance_))
  {
    setStatus(BT::NodeStatus::RUNNING);
  }
  else
    setStatus(BT::NodeStatus::SUCCESS);
}