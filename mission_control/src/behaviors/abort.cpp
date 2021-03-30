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
#include "mission_control/behaviors/abort.h"

#include "mission_control/AttitudeServo.h"

#include <string>

namespace mission_control
{

AbortNode::AbortNode(const std::string& name, const BT::NodeConfiguration& config)
  : ReactiveActionNode(name, config)
{
  attitude_servo_pub_ =
      nh_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1);

  nh_.param<double>("/fin_control/max_ctrl_fin_angle", pitch_, 0.3490658503988659);
}

BT::NodeStatus AbortNode::setUp()
{
  state_.reset();
  state_sub_ =
    nh_.subscribe("/state", 1, &AbortNode::stateDataCallback, this);

  mission_control::AttitudeServo msg;
  msg.header.stamp = ros::Time::now();
  msg.roll = roll_;
  msg.pitch = pitch_;
  msg.yaw = yaw_;
  msg.speed_knots = speed_knots_;
  msg.ena_mask =
    mission_control::AttitudeServo::ROLL_ENA |
    mission_control::AttitudeServo::PITCH_ENA |
    mission_control::AttitudeServo::YAW_ENA |
    mission_control::AttitudeServo::SPEED_KNOTS_ENA;
  attitude_servo_pub_.publish(msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AbortNode::doWork()
{
  if (state_)
  {
    double roll = state_->state.manoeuvring.pose.mean.orientation.x;
    double pitch = state_->state.manoeuvring.pose.mean.orientation.y;
    double yaw = state_->state.manoeuvring.pose.mean.orientation.z;
    double speed =
      std::sqrt(std::pow(state_->state.manoeuvring.velocity.mean.linear.x, 2) +
                std::pow(state_->state.manoeuvring.velocity.mean.linear.y, 2) +
                std::pow(state_->state.manoeuvring.velocity.mean.linear.z, 2));

    // NOTE(aschapiro): This ignores angle wraparound and the fact that
    // you can represent the same rotation with different RPY triplets
    if ((std::abs(roll_ - roll) < roll_tolerance_) &&
        (std::abs(pitch_ - pitch) < pitch_tolerance_) &&
        (std::abs(yaw_ - yaw) < yaw_tolerance_) &&
        (speed < speed_tolerance_))
    {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void AbortNode::tearDown()
{
  state_sub_.shutdown();
}

void AbortNode::stateDataCallback(auv_interfaces::StateStamped::ConstPtr msg)
{
  state_ = msg;
}

}  // namespace mission_control
