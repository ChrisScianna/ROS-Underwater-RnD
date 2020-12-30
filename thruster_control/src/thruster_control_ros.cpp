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
 * thruster_control_ros.cpp
 *
 */

#include "thruster_control/thruster_control_ros.h"

#include <string>

namespace qna
{
namespace robot
{

std::unique_ptr<CANIntf> ConfigureCANInterfaceUsingROSParams(ros::NodeHandle nh)
{
  std::unique_ptr<CANIntf> can_interface(new CANIntf());

  double rpm_timeout = nh.param("set_rpm_timeout_seconds", 0.5);
  ROS_DEBUG_STREAM("set_rpm_timeout_seconds set to  " << rpm_timeout);
  can_interface->SetMotorTimeoutSeconds(rpm_timeout);

  bool current_logging_enabled = nh.param("current_logging_enabled", false);
  ROS_DEBUG_STREAM("current_logging_enabled set to  " <<
                   current_logging_enabled ? "true" : "false");
  can_interface->SetEnableCANLogging(current_logging_enabled);

  // TODO(QNA) what should the default be??
  std::string can_node_id = nh.param("can_node_id_1", std::string(""));
  ROS_DEBUG_STREAM("CAN Node 1 Id =  " << can_node_id);
  can_interface->AddCanNodeIdToList(can_node_id);

  can_node_id = nh.param("can_node_id_2", std::string(""));
  ROS_DEBUG_STREAM("CAN Node 2 Id =  " << can_node_id);
  can_interface->AddCanNodeIdToList(can_node_id);

  return std::move(can_interface);
}

ThrusterControlROS::ThrusterControlROS()
  : ThrusterControlROS(ros::NodeHandle(), ros::NodeHandle("~"))
{
}

ThrusterControlROS::ThrusterControlROS(std::unique_ptr<ThrusterControlInterface> thruster_control)
  : ThrusterControlROS(ros::NodeHandle(), ros::NodeHandle("~"), std::move(thruster_control))
{
}

ThrusterControlROS::ThrusterControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : ThrusterControlROS(nh, pnh, std::unique_ptr<ThrusterControlInterface>(
      new ThrusterControl(ConfigureCANInterfaceUsingROSParams(pnh))))
{
}

ThrusterControlROS::ThrusterControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                                       std::unique_ptr<ThrusterControlInterface> thruster_control)
  : nh_(nh), pnh_(pnh), thruster_control_(std::move(thruster_control))
{
  motor_rpm_subscriber_ =
      nh_.subscribe("/thruster_control/set_rpm", 1, &ThrusterControlROS::OnMotorRPMSet, this);

  motor_rpm_publisher_ =
      diagnostic_tools::create_publisher<thruster_control::ReportRPM>(
          nh_, "/thruster_control/report_rpm", 1);

  motor_temperature_publisher_ =
      diagnostic_tools::create_publisher<thruster_control::ReportMotorTemperature>(
          nh_, "/thruster_control/report_motor_temperature", 1);

  battery_health_publisher_ =
      diagnostic_tools::create_publisher<sensor_msgs::BatteryState>(
          nh_, "/thruster_control/report_battery_health", 1);

  report_motor_rpm_period_ = pnh_.param("report_motor_rpm_period", 0.1);
  report_motor_rpm_timer_ =
      nh_.createTimer(ros::Duration(report_motor_rpm_period_),
                      &ThrusterControlROS::OnReportMotorRPMTimeout, this);

  report_motor_temperature_period_ = pnh_.param("report_motor_temperature_period", 0.5);
  report_motor_temperature_timer_ =
      nh_.createTimer(ros::Duration(report_motor_temperature_period_),
                      &ThrusterControlROS::OnReportMotorTemperatureTimeout, this);

  report_battery_health_period_ = pnh_.param("report_battery_health_period", 1.0);
  report_battery_health_timer_ =
      nh_.createTimer(ros::Duration(report_battery_health_period_),
                      &ThrusterControlROS::OnReportBatteryHealthTimeout, this);
}

void ThrusterControlROS::MonitorMotorRPMUsing(diagnostic_updater::Updater* updater)
{
  if (!report_motor_rpm_rate_check_) {
    diagnostic_tools::PeriodicMessageStatusParams params;
    double min_report_motor_rpm_period =
        pnh_.param("min_report_rpm_period", report_motor_rpm_period_ / 2.0);
    params.min_acceptable_period(min_report_motor_rpm_period);
    double max_report_motor_rpm_period =
        pnh_.param("max_report_rpm_period", report_motor_rpm_period_ / 2.0);
    params.max_acceptable_period(max_report_motor_rpm_period);
    params.abnormal_diagnostic({
        diagnostic_tools::Diagnostic::WARN,
        health_monitor::ReportFault::THRUSTER_RPM_STALE});
    using diagnostic_tools::PeriodicMessageStatus;
    report_motor_rpm_rate_check_ = &(
        motor_rpm_publisher_.add_check<PeriodicMessageStatus>("rate check", params));
  }
  updater->add(*report_motor_rpm_rate_check_);

  if (!motor_rpm_check_) {
    double max_allowed_motor_rpm = pnh_.param("max_allowed_rpm", 0.);
    motor_rpm_check_ = diagnostic_tools::create_health_check<double>(
        "Motor RPM check",
        [max_allowed_motor_rpm](double rpms) -> diagnostic_tools::Diagnostic
        {
          if (std::abs(rpms) > max_allowed_motor_rpm)
          {
            return diagnostic_tools::Diagnostic(diagnostic_tools::Diagnostic::WARN)
                .code(health_monitor::ReportFault::THRUSTER_RPM_THRESHOLD_REACHED)
                .description("Absolute RPMs above threshold: |%f| > %f",
                             rpms, max_allowed_motor_rpm);
          }
          return diagnostic_tools::Diagnostic::OK;
        }
    );
  }
  updater->add(motor_rpm_check_);
}

void ThrusterControlROS::MonitorMotorTemperatureUsing(diagnostic_updater::Updater* updater)
{
  if (!motor_temperature_rate_check_)
  {
    diagnostic_tools::PeriodicMessageStatusParams params;
    double min_report_motor_temperature_period =
        pnh_.param("min_report_motor_temperature_period",
                    report_motor_temperature_period_ / 2.0);
    params.min_acceptable_period(min_report_motor_temperature_period);
    double max_report_motor_temperature_period =
        pnh_.param("max_report_motor_temperature_period",
                    report_motor_temperature_period_ * 2.0);
    params.max_acceptable_period(max_report_motor_temperature_period);
    params.abnormal_diagnostic({
        diagnostic_tools::Diagnostic::WARN,
        health_monitor::ReportFault::THRUSTER_TEMP_STALE});
    using diagnostic_tools::PeriodicMessageStatus;
    motor_temperature_rate_check_ = &(
        motor_temperature_publisher_.add_check<PeriodicMessageStatus>("rate check", params));
  }
  updater->add(*motor_temperature_rate_check_);

  if (!motor_temperature_stagnation_check_)
  {
    diagnostic_tools::MessageStagnationCheckParams params;
    params.stagnation_diagnostic({
        diagnostic_tools::Diagnostic::WARN,
        health_monitor::ReportFault::THRUSTER_TEMP_STAGNATED});
    double motor_temperature_steady_band = pnh_.param("motor_temperature_steady_band", 0.);
    auto areTemperaturesEqual = [motor_temperature_steady_band](
        const thruster_control::ReportMotorTemperature& a,
        const thruster_control::ReportMotorTemperature& b)
    {
      return std::fabs(a.motor_temp - b.motor_temp < motor_temperature_steady_band);
    };
    using diagnostic_tools::MessageStagnationCheck;
    motor_temperature_stagnation_check_ = &(
        motor_temperature_publisher_.add_check<MessageStagnationCheck>(
            "stagnation check", areTemperaturesEqual, params));
  }
  updater->add(*motor_temperature_stagnation_check_);

  if (!motor_temperature_check_)
  {
    double motor_temperature_threshold = pnh_.param("motor_temperature_threshold", 50.);
    motor_temperature_check_ = diagnostic_tools::create_health_check<double>(
        "Motor temperature check",
        [motor_temperature_threshold](double temperature) -> diagnostic_tools::Diagnostic
        {
          if (temperature > motor_temperature_threshold)
          {
            return diagnostic_tools::Diagnostic(diagnostic_tools::Diagnostic::ERROR)
                .code(health_monitor::ReportFault::THRUSTER_TEMP_THRESHOLD_REACHED)
                .description("Temperature above threshold: %f degC > %f degC",
                             temperature, motor_temperature_threshold);
          }
          return diagnostic_tools::Diagnostic::OK;
        }
    );
  }
  updater->add(motor_temperature_check_);
}

void ThrusterControlROS::MonitorBatteryHealthUsing(diagnostic_updater::Updater* updater)
{
  if (!battery_health_rate_check_)
  {
    diagnostic_tools::PeriodicMessageStatusParams params;
    double min_report_battery_health_period =
        pnh_.param("min_report_battery_health_period", report_battery_health_period_ / 2.0);
    params.min_acceptable_period(min_report_battery_health_period);
    double max_report_battery_health_period =
        pnh_.param("max_report_battery_health_period", report_battery_health_period_ * 2.0);
    params.max_acceptable_period(max_report_battery_health_period);
    params.abnormal_diagnostic({
        diagnostic_tools::Diagnostic::WARN,
        health_monitor::ReportFault::BATTERY_INFO_STALE});
    using diagnostic_tools::PeriodicMessageStatus;
    battery_health_rate_check_ =
        &(battery_health_publisher_.add_check<PeriodicMessageStatus>("rate check", params));
  }
  updater->add(*battery_health_rate_check_);

  if (!battery_health_stagnation_check_)
  {
    diagnostic_tools::MessageStagnationCheckParams params;
    params.stagnation_diagnostic({
        diagnostic_tools::Diagnostic::WARN,
        health_monitor::ReportFault::BATTERY_INFO_STAGNATED});
    double battery_current_steady_band = pnh_.param("battery_current_steady_band", 0.0);
    double battery_voltage_steady_band = pnh_.param("battery_voltage_steady_band", 0.0);
    auto areBatteryStatesEqual =
        [battery_current_steady_band, battery_voltage_steady_band](
            const sensor_msgs::BatteryState& a, const sensor_msgs::BatteryState& b)
        {
          return (std::fabs(a.current - b.current) < battery_current_steady_band) &&
              (std::fabs(a.voltage - b.voltage) < battery_voltage_steady_band);
        };
    using diagnostic_tools::MessageStagnationCheck;
    battery_health_stagnation_check_ = &(
        battery_health_publisher_.add_check<MessageStagnationCheck>(
            "stagnation check", areBatteryStatesEqual, params));
  }
  updater->add(*battery_health_stagnation_check_);

  if (!battery_total_current_check_) {
    double min_battery_total_current = pnh_.param("min_battery_total_current", 5000.0);
    battery_total_current_check_ = diagnostic_tools::create_health_check<double>(
        "Battery total current check",
        [min_battery_total_current](double total_current) -> diagnostic_tools::Diagnostic
        {
          if (total_current < min_battery_total_current)
          {
            return diagnostic_tools::Diagnostic(diagnostic_tools::Diagnostic::WARN)
                .code(health_monitor::ReportFault::BATTERY_CURRENT_THRESHOLD_REACHED)
                .description("Battery total current - NOT OK (%f mA < %f mA)",
                             total_current, min_battery_total_current);
          }
          return diagnostic_tools::Diagnostic::OK;
        });
  }
  updater->add(battery_total_current_check_);

  if (!battery_cell_voltage_check_)
  {
    double min_battery_cell_voltage = pnh_.param("min_battery_cell_voltage", 30.0);
    battery_cell_voltage_check_ = diagnostic_tools::create_health_check<double>(
        "Battery cell voltage check",
        [min_battery_cell_voltage](double cell_voltage) -> diagnostic_tools::Diagnostic
        {
          if (cell_voltage < min_battery_cell_voltage)
          {
            return diagnostic_tools::Diagnostic(diagnostic_tools::Diagnostic::WARN)
                .code(health_monitor::ReportFault::BATTERY_VOLTAGE_THRESHOLD_REACHED)
                .description("Battery cell voltage - NOT OK (%f V < %f V)",
                             cell_voltage, min_battery_cell_voltage);
          }
          return diagnostic_tools::Diagnostic::OK;
        });
  }
  updater->add(battery_cell_voltage_check_);
}

void ThrusterControlROS::MonitorUsing(diagnostic_updater::Updater* updater)
{
  MonitorMotorRPMUsing(updater);
  MonitorMotorTemperatureUsing(updater);
  MonitorBatteryHealthUsing(updater);
}

void ThrusterControlROS::OnReportMotorRPMTimeout(const ros::TimerEvent&)
{
  thruster_control::ReportRPM message = thruster_control_->GetMotorRPM();
  motor_rpm_check_.test(message.rpms);
  motor_rpm_publisher_.publish(message);
}

void ThrusterControlROS::OnReportMotorTemperatureTimeout(const ros::TimerEvent&)
{
  thruster_control::ReportMotorTemperature message = thruster_control_->GetMotorTemperature();
  motor_temperature_check_.test(message.motor_temp);
  motor_temperature_publisher_.publish(message);
}

void ThrusterControlROS::OnReportBatteryHealthTimeout(const ros::TimerEvent&)
{
  sensor_msgs::BatteryState message = thruster_control_->GetBatteryState();
  battery_total_current_check_.test(message.voltage);
  battery_cell_voltage_check_.test(message.current);
  battery_health_publisher_.publish(message);
}

void ThrusterControlROS::OnMotorRPMSet(const thruster_control::SetRPM::ConstPtr& message)
{
  thruster_control_->SetMotorRPM(*message);
}

}  // namespace robot
}  // namespace qna
