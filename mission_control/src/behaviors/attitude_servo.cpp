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

#include "mission_control/AttitudeServo.h"
#include "mission_control/behaviors/helpers.h"

#include <string>

namespace mission_control
{

AttitudeServoNode::AttitudeServoNode(const std::string& name,
                                     const BT::NodeConfiguration& config)
  : ReactiveActionNode(name, config)
{
  attitude_servo_pub_ =
    nh_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1, true);
}

BT::NodeStatus AttitudeServoNode::setUp()
{
  // Update action parameters
  enable_mask_ = 0u;

  auto result = getInputValue<double, HasAngleUnits>(this, "roll", target_roll_);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(target_roll_))
  {
    result = getInputTolerance<double, HasAngleUnits>(this, "roll", roll_tolerance_);
    if (!result)
    {
      ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
      return BT::NodeStatus::FAILURE;
    }
    enable_mask_ |= mission_control::AttitudeServo::ROLL_ENA;
  }

  result = getInputValue<double, HasAngleUnits>(this, "pitch", target_pitch_);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(target_pitch_))
  {
    result = getInputTolerance<double, HasAngleUnits>(this, "pitch", pitch_tolerance_);
    if (!result)
    {
      ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
      return BT::NodeStatus::FAILURE;
    }
    enable_mask_ |= mission_control::AttitudeServo::PITCH_ENA;
  }

  result = getInputValue<double, HasAngleUnits>(this, "yaw", target_yaw_);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(target_yaw_))
  {
    result = getInputTolerance<double, HasAngleUnits>(this, "yaw", yaw_tolerance_);
    if (!result)
    {
      ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
      return BT::NodeStatus::FAILURE;
    }
    enable_mask_ |= mission_control::AttitudeServo::YAW_ENA;
  }

  result = getInput<double>("speed_knots", speed_knots_);
  if (!result)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isnan(speed_knots_))
  {
    enable_mask_ |= mission_control::AttitudeServo::SPEED_KNOTS_ENA;
  }

  // Setup state subscriber
  state_.reset();
  state_sub_ = nh_.subscribe(
    "/state", 1, &AttitudeServoNode::stateDataCallback, this);

  // Publish attitude setpoint
  mission_control::AttitudeServo msg;
  msg.header.stamp = ros::Time::now();
  msg.roll = target_roll_;
  msg.pitch = target_pitch_;
  msg.yaw = target_yaw_;
  msg.speed_knots = speed_knots_;
  msg.ena_mask = enable_mask_;
  attitude_servo_pub_.publish(msg);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AttitudeServoNode::doWork()
{
  if (!state_)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (enable_mask_ & mission_control::AttitudeServo::ROLL_ENA)
  {
    double current_roll = state_->state.manoeuvring.pose.mean.orientation.x;

    if (std::abs(target_roll_ - current_roll) > roll_tolerance_)
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  if (enable_mask_ & mission_control::AttitudeServo::PITCH_ENA)
  {
    double current_pitch = state_->state.manoeuvring.pose.mean.orientation.y;

    if (std::abs(target_pitch_ - current_pitch) > pitch_tolerance_)
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  if (enable_mask_ & mission_control::AttitudeServo::YAW_ENA)
  {
    double current_yaw = state_->state.manoeuvring.pose.mean.orientation.z;

    if (std::abs(target_yaw_ - current_yaw) > yaw_tolerance_)
    {
      return BT::NodeStatus::RUNNING;
    }
  }

  // TODO(QNA): check shaft speed and/or battery position?
  // TODO(QNA): make sure our RPY rates are close to zero?

  return BT::NodeStatus::SUCCESS;
}

void AttitudeServoNode::tearDown()
{
  state_sub_.shutdown();
}

void AttitudeServoNode::stateDataCallback(auv_interfaces::StateStamped::ConstPtr msg)
{
  state_ = msg;
}

}  // namespace mission_control
