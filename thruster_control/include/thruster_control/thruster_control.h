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

/*
 * thruster_control.h
 */

#ifndef THRUSTER_CONTROL_THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_THRUSTER_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <thruster_control/ReportMotorTemperature.h>
#include <thruster_control/ReportRPM.h>
#include <thruster_control/SetRPM.h>

#include "thruster_control/CANIntf.h"

namespace qna
{
namespace robot
{

class ThrusterControlInterface
{
public:
  virtual ~ThrusterControlInterface() = default;

  virtual thruster_control::ReportRPM GetMotorRPM() = 0;
  virtual void SetMotorRPM(const thruster_control::SetRPM& message) = 0;
  virtual thruster_control::ReportMotorTemperature GetMotorTemperature() = 0;
  virtual sensor_msgs::BatteryState GetBatteryState() = 0;
};

class ThrusterControl : public ThrusterControlInterface
{
 public:
  explicit ThrusterControl(std::unique_ptr<CANIntf> can_interface);

  thruster_control::ReportRPM GetMotorRPM() override;
  void SetMotorRPM(const thruster_control::SetRPM& msg) override;
  thruster_control::ReportMotorTemperature GetMotorTemperature() override;
  sensor_msgs::BatteryState GetBatteryState() override;

 private:
  std::unique_ptr<CANIntf> can_interface_;
};
}  // namespace robot
}  // namespace qna

#endif  // THRUSTER_CONTROL_THRUSTER_CONTROL_H
