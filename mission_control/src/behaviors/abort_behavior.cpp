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

#include "mission_control/behaviors/abort_behavior.h"

using mission_control::AbortBehavior;

AbortBehavior::AbortBehavior(const std::string& name,
                                             const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  subStateData_ =
      nodeHandle_.subscribe("/state", 1, &AbortBehavior::stateDataCallback, this);

  subThrusterRPM_ = nodeHandle_.subscribe("/thruster_control/report_rpm", 1,
                                          &AbortBehavior::thrusterRPMCallback, this);

  attitudeServoBehaviorPub_ =
      nodeHandle_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1);

}

BT::NodeStatus AbortBehavior::behaviorRunningProcess()
{
  if (!abortMsgHasBeenPublished_)
  {
    getInput<double>("roll", roll_);
    getInput<double>("pitch", pitch_);
    getInput<double>("yaw", yaw_);
    getInput<double>("speed_knots", speedKnots_);
    getInput<double>("roll_tol", rollTolerance_);
    getInput<double>("pitch_tol", pitchTolerance_);
    getInput<double>("yaw_tol", yawTolerance_);

    publishAbortMsg();
    abortMsgHasBeenPublished_ = true;
  }
  else
  {
    if (orientationReached_ && speedReached_)
    {
      setStatus(BT::NodeStatus::SUCCESS);
      abortMsgHasBeenPublished_ = false;
    }
  }
  return status();
}

void AbortBehavior::publishAbortMsg()
{
  AttitudeServo msg;

  msg.roll = roll_;
  msg.pitch = pitch_;
  msg.yaw = yaw_;
  msg.speed_knots = speedKnots_;

  msg.ena_mask = 15;  //  We are sending all values
  
  msg.header.stamp = ros::Time::now();
  attitudeServoBehaviorPub_.publish(msg);
}

void AbortBehavior::stateDataCallback(const auv_interfaces::StateStamped& data)
{
  orientationReached_ = true;
  double roll = data.state.manoeuvring.pose.mean.orientation.x;
  double pitch = data.state.manoeuvring.pose.mean.orientation.y;
  double yaw = data.state.manoeuvring.pose.mean.orientation.z;

  if (abs(roll_ - roll) > rollTolerance_) orientationReached_ = false;
  if (abs(pitch_ - pitch) > pitchTolerance_) orientationReached_ = false;
  if (abs(yaw_ - yaw) > yawTolerance_) orientationReached_ = false;
}

void AbortBehavior::thrusterRPMCallback(const thruster_control::ReportRPM& data)
{
  if (data.rpms <= speedKnots_) speedReached_ = true;
}