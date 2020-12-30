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
 * thruster_control.cpp
 *
 */

#include "thruster_control/thruster_control.h"

#include <cmath>
#include <string>

namespace qna
{
namespace robot
{
namespace
{

constexpr double RPM_TO_RADSEC = M_PI / 30.0;
constexpr double RADSEC_TO_RPM = 30.0 / M_PI;

double convertRPMToRadSec(double rpms) { return rpms * RPM_TO_RADSEC; }
double convertRadSecToRPM(double radsec) { return radsec * RADSEC_TO_RPM; }

}  // namespace

ThrusterControl::ThrusterControl(std::unique_ptr<CANIntf> can_interface)
  : can_interface_(std::move(can_interface))
{
  can_interface_->Init();
}

thruster_control::ReportRPM ThrusterControl::GetMotorRPM()
{
  thruster_control::ReportRPM msg;
  msg.header.stamp = ros::Time::now();
  msg.rpms = convertRadSecToRPM(can_interface_->velocity_feedback_radsec.Get());
  return msg;
}

void ThrusterControl::SetMotorRPM(const thruster_control::SetRPM& message)
{
  double motor_radsec = convertRPMToRadSec(message.commanded_rpms);
  can_interface_->velocity_radsec.Set(motor_radsec);
  can_interface_->last_set_rpm_time.Set(ros::Time::now().toSec());
}

thruster_control::ReportMotorTemperature
ThrusterControl::GetMotorTemperature()
{
  thruster_control::ReportMotorTemperature message;
  message.header.stamp = ros::Time::now();
  message.motor_temp = can_interface_->motor_tempC.Get();
  return message;
}

sensor_msgs::BatteryState
ThrusterControl::GetBatteryState()
{
  sensor_msgs::BatteryState message;

  message.header.stamp = ros::Time::now();
  message.voltage = can_interface_->battery_voltage.Get() / 1000.0;
  //  message.temperature = can_interface_->motor_tempC.Get();
  message.current = can_interface_->motor_current.Get() / -1000.0;
  message.power_supply_status = 0;
  message.power_supply_health = 0;
  message.power_supply_technology = 0;
  message.present = 1;
  message.location = "A";
  message.serial_number = "1";

  return message;
}

}  // namespace robot
}  // namespace qna
