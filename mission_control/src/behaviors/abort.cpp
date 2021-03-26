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

#include <string>

using mission_control::Abort;

Abort::Abort(const std::string& name) : BT::ActionNodeBase(name, {})
{
  attitudeServoBehaviorPub_ =
      nodeHandle_.advertise<mission_control::AttitudeServo>("/mngr/attitude_servo", 1);
  
  nodeHandle_.param<double>("/fin_control/max_ctrl_fin_angle", pitch_, 0.3490658503988659);

}

BT::NodeStatus Abort::tick()
{
  BT::NodeStatus current_status = status();

  switch (current_status)
  {
    case BT::NodeStatus::IDLE:
      subStateData_ = nodeHandle_.subscribe("/state", 1, &Abort::stateDataCallback, this);

      subThrusterRPM_ = nodeHandle_.subscribe("/thruster_control/report_rpm", 1,
                                              &Abort::thrusterRPMCallback, this);
      stateUpToDate_ = false;
      velocityUpToDate_ = false;
      publishAbortMsg();
      current_status = BT::NodeStatus::RUNNING;
      break;
    case BT::NodeStatus::RUNNING:
      if (stateUpToDate_ && velocityUpToDate_)
      {
        double roll = stateData_.state.manoeuvring.pose.mean.orientation.x;
        double pitch = stateData_.state.manoeuvring.pose.mean.orientation.y;
        double yaw = stateData_.state.manoeuvring.pose.mean.orientation.z;

        if ((abs(roll_ - roll) < rollTolerance_) && (abs(pitch_ - pitch) < pitchTolerance_) &&
            (abs(yaw_ - yaw) < yawTolerance_) && (velocityData_.rpms <= speedKnots_))
        {
          current_status = BT::NodeStatus::SUCCESS;
        }
      }
      break;
    default:
      break;
  }
  return current_status;
}

void Abort::halt()
{
  if (status() == BT::NodeStatus::RUNNING)
  {
    subStateData_.shutdown();
    subThrusterRPM_.shutdown();
  }
  setStatus(BT::NodeStatus::IDLE);
}

void Abort::publishAbortMsg()
{
  AttitudeServo msg;

  msg.roll = roll_;
  msg.pitch = pitch_;
  msg.yaw = yaw_;
  msg.speed_knots = speedKnots_;
  msg.ena_mask = AttitudeServo::ROLL_ENA | AttitudeServo::PITCH_ENA | AttitudeServo::YAW_ENA |
                 AttitudeServo::SPEED_KNOTS_ENA;
  msg.header.stamp = ros::Time::now();
  attitudeServoBehaviorPub_.publish(msg);
}

void Abort::stateDataCallback(const auv_interfaces::StateStamped& data)
{
  // TODO(aschapiro): This ignores angle wraparound and the fact that you can represent the same
  // rotation with different RPY triplets

  stateUpToDate_ = true;
  stateData_ = data;
}

void Abort::thrusterRPMCallback(const thruster_control::ReportRPM& data)
{
  velocityUpToDate_ = true;
  velocityData_ = data;
}
