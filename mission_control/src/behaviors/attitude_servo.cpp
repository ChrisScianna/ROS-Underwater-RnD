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
#include "mission_control/behaviors/attitude_servo.h"

#include <string>

using mission_control::AttitudeServoBehavior;

AttitudeServoBehavior::AttitudeServoBehavior(const std::string& name,
                                             const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;
  speedKnots_ = 0.0;

  rollEnable_ = false;
  pitchEnable_ = false;
  yawEnable_ = false;
  speedKnotsEnable_ = false;

  rollTolerance_ = 0;
  pitchTolerance_ = 0;
  yawTolerance_ = 0;

  getInput<double>("roll", roll_);
  if (roll_ != 0.0) rollEnable_ = true;

  getInput<double>("pitch", pitch_);
  if (pitch_ != 0.0) pitchEnable_ = true;

  getInput<double>("yaw", yaw_);
  if (yaw_ != 0.0) yawEnable_ = true;

  getInput<double>("speed_knots", speedKnots_);
  if (speedKnots_ != 0.0) speedKnotsEnable_ = true;

  getInput<double>("time_out", timeOut_);
  getInput<double>("roll_tol", rollTolerance_);
  getInput<double>("pitch_tol", pitchTolerance_);
  getInput<double>("yaw_tol", yawTolerance_);

  subCorrectedData_ = nodeHandle_.subscribe("/pose/corrected_data", 1,
                                            &AttitudeServoBehavior::correctedDataCallback, this);

  goalHasBeenPublished_ = false;
  attitudeServoBehaviorPub_ =
      nodeHandle_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1);

  behaviorComplete_ = false;
  behaviorStartTime_ = ros::Time::now();
}

BT::NodeStatus AttitudeServoBehavior::behaviorRunningProcess()
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
    if (delta_t.toSec() > timeOut_) setStatus(BT::NodeStatus::FAILURE);
    else
    {
      if (behaviorComplete_) setStatus(BT::NodeStatus::SUCCESS);
    }
  }
  return status();
}

void AttitudeServoBehavior::publishGoalMsg()
{
  AttitudeServo msg;

  msg.roll = roll_;
  msg.pitch = pitch_;
  msg.yaw = yaw_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 0x0;
  if (rollEnable_) msg.ena_mask |= AttitudeServo::ROLL_ENA;
  if (pitchEnable_) msg.ena_mask |= AttitudeServo::PITCH_ENA;
  if (yawEnable_) msg.ena_mask |= AttitudeServo::YAW_ENA;
  if (speedKnotsEnable_) msg.ena_mask |= AttitudeServo::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  attitudeServoBehaviorPub_.publish(msg);
}

void AttitudeServoBehavior::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  behaviorComplete_ = true;

  if (rollEnable_ && fabs(roll_ - data.rpy_ang.x) > rollTolerance_) behaviorComplete_ = false;
  if (pitchEnable_ && fabs(pitch_ - data.rpy_ang.y) > pitchTolerance_) behaviorComplete_ = false;
  if (yawEnable_ && fabs(yaw_ - data.rpy_ang.z) > yawTolerance_) behaviorComplete_ = false;

  // TODO(QNA): check shaft speed and/or battery position?
  // TODO(QNA): make sure our RPY rates are close to zero?
}
