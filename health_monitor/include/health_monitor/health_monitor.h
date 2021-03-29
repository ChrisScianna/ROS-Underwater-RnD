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
 * health_monitor.h
 */

#ifndef HEALTH_MONITOR_HEALTH_MONITOR_H
#define HEALTH_MONITOR_HEALTH_MONITOR_H

#include <ros/ros.h>

#include <unordered_map>
#include <string>

#include <health_monitor/ReportFault.h>
#include <health_monitor/ClearFault.h>
#include <rosmon_msgs/State.h>

#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>

namespace qna
{
namespace robot
{

class HealthMonitor
{
public:
    explicit HealthMonitor(ros::NodeHandle &nodeHandle);
    virtual ~HealthMonitor();

    ros::Timer reportFaultsTimer;
    void reportFaultsTimeout(const ros::TimerEvent &timer);

    void sendFaults();
    void setFault(uint64_t fault_id);

private:
    uint64_t faults;
    std::unordered_map<std::string, uint64_t> uErrorMap
        {
        {"payload_manager", health_monitor::ReportFault::PAYLOAD_NODE_DIED},
        {"vectornav", health_monitor::ReportFault::AHRS_NODE_DIED},
        {"ixblue_c3_ins_node", health_monitor::ReportFault::AHRS_NODE_DIED},
        {"pressure_sensor", health_monitor::ReportFault::PRESSURE_NODE_DIED},
        {"mission_control_node", health_monitor::ReportFault::MISSION_NODE_DIED},
        {"pose_estimator_node", health_monitor::ReportFault::POSE_NODE_DIED},
        {"thruster_control_node", health_monitor::ReportFault::THRUSTER_NODE_DIED},
        {"autopilot_node", health_monitor::ReportFault::AUTOPILOT_NODE_DIED},
        {"battery_monitor_node", health_monitor::ReportFault::BATTERY_NODE_DIED},
        {"jaus_ros_bridge", health_monitor::ReportFault::JAUS_NODE_DIED},
        {"fin_control", health_monitor::ReportFault::FIN_NODE_DIED}
        };

    void handle_ClearFault(const health_monitor::ClearFault::ConstPtr &msg);
    void handle_diagnostics(const diagnostic_msgs::DiagnosticArrayPtr &msg);
    void handle_rosmonFaults(const rosmon_msgs::State &msg);

    ros::NodeHandle &nodeHandle;
    ros::Subscriber subscriber_clearFault;
    ros::Subscriber subscriber_diagnostics;
    ros::Subscriber subscriber_rosmonFaults;

    diagnostic_tools::DiagnosedPublisher<health_monitor::ReportFault> publisher_reportFault;
    diagnostic_updater::Updater diagnosticsUpdater;
};

}   // namespace robot
}   // namespace qna

#endif  // HEALTH_MONITOR_HEALTH_MONITOR_H
