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
#include "mission_control/behaviors/fixed_rudder.h"

using mission_control::MoveWithFixedRudder;

MoveWithFixedRudder::MoveWithFixedRudder(const std::string& name,
                                         const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  depthEnable_ = rudderEnable_ = altitudeEnable_ = false;
  speedKnotsEnable_ = false;
  depthTolerance_ = rudderTolerance_ = altitudeTolerance_ = 0.0;
  depth_ = rudder_ = altitude_ = 0.0;
  speedKnots_ = 0.0;

  getInput<double>("depth", depth_);
  if (depth_ != 0.0) depthEnable_ = true;

  getInput<double>("altitude", altitude_);
  if (altitude_ != 0.0) altitudeEnable_ = true;

  getInput<double>("rudder", rudder_);
  if (rudder_ != 0.0) rudderEnable_ = true;

  getInput<double>("speedKnots", speedKnots_);
  if (speedKnots_ != 0.0) speedKnotsEnable_ = true;

  getInput<double>("behavior_time", behaviorTime_);
  getInput<double>("rudder_tol", rudderTolerance_);
  getInput<double>("depth_tol", depthTolerance_);
  getInput<double>("altitude_tol", altitudeTolerance_);

  subCorrectedData_ = nodeHandle_.subscribe("/pose/corrected_data", 1,
                                            &MoveWithFixedRudder::correctedDataCallback, this);

  goalHasBeenPublished_ = false;
  fixedRudderBehaviorPub_ =
      nodeHandle_.advertise<mission_control::FixedRudder>("/mngr/fixed_rudder", 1);

  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus MoveWithFixedRudder::behaviorRunningProcess()
{
  if (!goalHasBeenPublished_)
  {
    publishGoalMsg();
    goalHasBeenPublished_ = true;
  }
  else
  {
    ros::Duration delta_t = ros::Time::now() - behaviorStartTime_;
    if (delta_t.toSec() > behaviorTime_) setStatus(BT::NodeStatus::SUCCESS);
  }
  return (status());
}

void MoveWithFixedRudder::publishGoalMsg()
{
  FixedRudder msg;
  msg.depth = depth_;
  msg.rudder = rudder_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 0x0;
  if (depthEnable_) msg.ena_mask |= FixedRudder::DEPTH_ENA;
  if (rudderEnable_) msg.ena_mask |= FixedRudder::RUDDER_ENA;
  if (speedKnotsEnable_) msg.ena_mask |= FixedRudder::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  fixedRudderBehaviorPub_.publish(msg);
}

void MoveWithFixedRudder::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  // A quick check to see if our depth matches, note: rudder is not part of corrected data.
  // TODO(QNA): put back in when depth working  if (depthEnable_ && (abs(depth_ - data.depth) >
  // depth_Tolerance_)) return false;
  // TODO(QNA): check shaft speed?
}
