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
  rollEnable_ = false;
  pitchEnable_ = false;
  yawEnable_ = false;
  speedKnotsEnable_ = false;
  timeOutEnable_ = false;

  if (getInput<double>("roll", roll_))
  {
    rollEnable_ = true;
    getInput<double>("roll_tol", rollTolerance_);
  }

  if (getInput<double>("pitch", pitch_))
  {
    pitchEnable_ = true;
    getInput<double>("pitch_tol", pitchTolerance_);
  }

  if (getInput<double>("yaw", yaw_))
  {
    yawEnable_ = true;
    getInput<double>("yaw_tol", yawTolerance_);
  }

  if (getInput<double>("speed_knots", speedKnots_)) speedKnotsEnable_ = true;
  if (getInput<double>("time_out", timeOut_)) timeOutEnable_ = true;

  subStateData_ =
      nodeHandle_.subscribe("/state", 1, &AttitudeServoBehavior::stateDataCallback, this);

  attitudeServoBehaviorPub_ =
      nodeHandle_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1);

  goalHasBeenPublished_ = false;
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
    if (timeOutEnable_)
    {
      ros::Duration delta_t = ros::Time::now() - behaviorStartTime_;
      if (delta_t.toSec() > timeOut_ && timeOutEnable_)
      {
        goalHasBeenPublished_ = false;
        setStatus(BT::NodeStatus::FAILURE);
        return status();
      }
    }
    if (behaviorComplete_)
    {
      setStatus(BT::NodeStatus::SUCCESS);
      goalHasBeenPublished_ = false;
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

void AttitudeServoBehavior::stateDataCallback(const auv_interfaces::StateStamped& data)
{
  behaviorComplete_ = true;
  double roll = data.state.manoeuvring.pose.mean.orientation.x;
  double pitch = data.state.manoeuvring.pose.mean.orientation.y;
  double yaw = data.state.manoeuvring.pose.mean.orientation.z;

  if (rollEnable_ && abs(roll_ - roll) > rollTolerance_) behaviorComplete_ = false;
  if (pitchEnable_ && abs(pitch_ - pitch) > pitchTolerance_) behaviorComplete_ = false;
  if (yawEnable_ && abs(yaw_ - yaw) > yawTolerance_) behaviorComplete_ = false;

  // TODO(QNA): check shaft speed and/or battery position?
  // TODO(QNA): make sure our RPY rates are close to zero?
}
