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

namespace qna
{
namespace robot
{

HealthMonitor::HealthMonitor(ros::NodeHandle &nodeHandle) : nodeHandle(nodeHandle),
                                                            faults(0),
                                                            diagnosticsUpdater(nodeHandle)
{
    diagnosticsUpdater.setHardwareID("health_monitor");

    subscriber_clearFault = nodeHandle.subscribe("/health_monitor/clear_fault", 1,
                                                 &HealthMonitor::handle_ClearFault, this);
    subscriber_diagnostics = nodeHandle.subscribe("/diagnostics", 1,
                                                  &HealthMonitor::handle_diagnostics, this);
    subscriber_rosmonFaults = nodeHandle.subscribe("/rosmon/state", 1,
                                                   &HealthMonitor::handle_rosmonFaults, this);

    publisher_reportFault = diagnostic_tools::create_publisher<health_monitor::ReportFault>(
        nodeHandle, "/health_monitor/report_fault", 1);

    double reportFaultsRate = 1.0;
    double minReportFaultRate = minReportFaultRate / 2.0;
    double maxReportFaultRate = minReportFaultRate * 2.0;
    nodeHandle.getParam("/health_monitor_node/report_faults_rate", reportFaultsRate);
    nodeHandle.getParam("/health_monitor_node/min_report_faults_rate", minReportFaultRate);
    nodeHandle.getParam("/health_monitor_node/max_report_faults_rate", maxReportFaultRate);

    ROS_INFO("Report Faults Rate:[%lf]", reportFaultsRate);
    reportFaultsTimer = nodeHandle.createTimer(ros::Duration(reportFaultsRate),
                                               &HealthMonitor::reportFaultsTimeout, this);

    diagnosticsUpdater.add(publisher_reportFault.add_check<diagnostic_tools::PeriodicMessageStatus>(
        "rate check", diagnostic_tools::PeriodicMessageStatusParams{}
                          .min_acceptable_period(minReportFaultRate)
                          .max_acceptable_period(maxReportFaultRate)));
}

HealthMonitor::~HealthMonitor()
{
}

void HealthMonitor::handle_ClearFault(const health_monitor::ClearFault::ConstPtr &msg)
{
    if (msg->fault_id == health_monitor::ClearFault::ALL_FAULTS)
    {
        faultArrayMutex.lock();
        faults = 0;
        faultArrayMutex.unlock();
    }
    else
    {
        faultArrayMutex.lock();
        faults &= ~msg->fault_id;
        faultArrayMutex.unlock();
    }
}

void HealthMonitor::handle_diagnostics(const diagnostic_msgs::DiagnosticArrayPtr &msg)
{
    uint64_t error_values = 0;
    for (int i = 0; i < msg->status.size(); i++)
    {
        if ((msg->status[i].level != diagnostic_msgs::DiagnosticStatus::OK) &&
            (msg->status[i].values.size() > 0) &&
            (msg->status[i].values[0].key == "Code"))
        {
            error_values |= stoi(msg->status[i].values[0].value);
        }
    }
    if (error_values != 0)
    {
        setFault(error_values);
        sendFaults();
    }
}

void HealthMonitor::handle_rosmonFaults(const rosmon_msgs::State &msg)
{
    uint64_t error_values = 0;
    for (int i = 0; i < msg.nodes.size(); i++)
    {
        if (msg.nodes[i].state == rosmon_msgs::NodeState::CRASHED)
        {
            if (uErrorMap.find(msg.nodes[i].name) == uErrorMap.end())
            {
                ROS_DEBUG_STREAM("Rosmon - Node " << msg.nodes[i].name
                                                  << " crashed but no fault is defined for it."
                                                  << "Ignoring.");
            }
            else
            {
                error_values |= uErrorMap[msg.nodes[i].name];
            }
        }
    }
    if (error_values != 0)
    {
        setFault(error_values);
        sendFaults();
    }
}

void HealthMonitor::setFault(uint64_t fault_id)
{
    faultArrayMutex.lock();
    faults |= fault_id;
    faultArrayMutex.unlock();
}

void HealthMonitor::sendFaults()
{
    health_monitor::ReportFault message;

    faultArrayMutex.lock();
    uint64_t temp_fault_id = faults;
    faultArrayMutex.unlock();

    ROS_DEBUG_STREAM("Error Code: " << faults);
    message.header.stamp = ros::Time::now();
    message.fault_id = temp_fault_id;

    publisher_reportFault.publish(message);
    diagnosticsUpdater.update();
}

void HealthMonitor::reportFaultsTimeout(const ros::TimerEvent &timer)
{
    sendFaults();
}

}   // namespace robot
}   // namespace qna
