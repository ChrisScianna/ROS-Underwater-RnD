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

// Original version: Christopher Scianna <Christopher.Scianna@us.QinetiQ.com>

/*
 * health_monitor.cpp
 *
 */

#include "health_monitor/health_monitor.h"

#include <string>
#include <utility>


namespace qna
{
namespace robot
{

HealthMonitor::HealthMonitor() :
  pnh_("~"), diagnostics_updater_(nh_)
{
  diagnostics_updater_.setHardwareID("health_monitor");

  clear_faults_request_sub_ = nh_.subscribe(
    "/health_monitor/clear_fault", 1, &HealthMonitor::handleClearFaultRequest, this);
  diagnostics_sub_ = nh_.subscribe("/diagnostics", 1, &HealthMonitor::monitorDiagnostics, this);
  node_states_sub_ = nh_.subscribe("/rosmon/state", 1, &HealthMonitor::monitorNodeStates, this);

  faults_pub_ = diagnostic_tools::create_publisher<health_monitor::ReportFault>(
    nh_, "/health_monitor/report_fault", 1);

  const double report_rate = pnh_.param("report_faults_rate", 1.0);
  const double min_report_rate = pnh_.param("min_report_faults_rate", report_rate / 2.);
  const double max_report_rate = pnh_.param("max_report_faults_rate", report_rate * 2.);
  ROS_INFO("Report rate for faults: %lf Hz", report_rate);

  diagnostics_updater_.add(
    faults_pub_.add_check<diagnostic_tools::PeriodicMessageStatus>(
        "rate check", diagnostic_tools::PeriodicMessageStatusParams{}
        .min_acceptable_period(1. / max_report_rate)
        .max_acceptable_period(1. / min_report_rate)));

  report_faults_timer_ = nh_.createTimer(
    ros::Duration(1. / report_rate), &HealthMonitor::reportFaults, this);
}

void HealthMonitor::handleClearFaultRequest(const health_monitor::ClearFault::ConstPtr &msg)
{
  faults_ &= msg->fault_id != health_monitor::ClearFault::ALL_FAULTS ? ~msg->fault_id : 0u;
}

void HealthMonitor::monitorDiagnostics(const diagnostic_msgs::DiagnosticArrayPtr &msg)
{
  for (int i = 0; i < msg->status.size(); ++i)
  {
    if ((msg->status[i].level != diagnostic_msgs::DiagnosticStatus::OK) &&
        (msg->status[i].values.size() > 0) &&
        (msg->status[i].values[0].key == "Code"))
    {
      faults_ |= stoll(msg->status[i].values[0].value);
    }
  }
}

void HealthMonitor::monitorNodeStates(const rosmon_msgs::State &msg)
{
  std::unordered_map<std::string, uint64_t> monitored_nodes{monitored_nodes_};

  for (const rosmon_msgs::NodeState & node : msg.nodes)
  {
    const std::string fqn = node.ns + node.name;
    auto it = monitored_nodes.find(fqn);
    if (it != monitored_nodes.end())
    {
      if (node.state == rosmon_msgs::NodeState::CRASHED)
      {
        faults_ |= it->second;
      }
      monitored_nodes.erase(it);
    }
    else
    {
      if (node.state == rosmon_msgs::NodeState::CRASHED)
      {
        ROS_WARN_STREAM("Not monitored node " << fqn << " crashed.");
      }
    }
  }

  if (!monitored_nodes.empty())
  {
    for (const std::pair<std::string, uint64_t> & kv : monitored_nodes)
    {
      ROS_ERROR_STREAM("Monitored node " << kv.first << " state not reported.");
    }
  }
}

void HealthMonitor::reportFaults(const ros::TimerEvent &)
{
  health_monitor::ReportFault message;
  message.header.stamp = ros::Time::now();
  message.fault_id = faults_;
  faults_pub_.publish(message);
  diagnostics_updater_.update();
}

}   // namespace robot
}   // namespace qna
