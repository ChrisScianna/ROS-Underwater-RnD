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

#include "mission_control/mission_control.h"

#include <string>

namespace mission_control
{

MissionControlNode::MissionControlNode() : pnh_("~")
{
  // Get runtime parameters
  double heartbeat_rate;
  pnh_.param("heartbeat_rate", heartbeat_rate, 1.0);  // Hz
  ROS_DEBUG_STREAM("heartbeat rate: " << heartbeat_rate);

  double update_rate;
  pnh_.param("update_rate", update_rate, 10.0);  // Hz
  ROS_DEBUG_STREAM("update rate: " << update_rate);

  // TODO(hidmic): give behaviors access to parameters via blackboard
  // pnh.param<double>("/fin_control/max_ctrl_fin_angle", maxCtrlFinAngle, 10.0);

  // Subscribe to all topics
  fault_sub_ = pnh_.subscribe("faults", 1, &MissionControlNode::faultCallback, this);

  load_mission_sub_ =
      pnh_.subscribe("load_mission", 1, &MissionControlNode::loadMissionCallback, this);
  execute_mission_sub_ =
      pnh_.subscribe("execute_mission", 1, &MissionControlNode::executeMissionCallback, this);
  abort_mission_sub_ =
      pnh_.subscribe("abort_mission", 1, &MissionControlNode::abortMissionCallback, this);
  query_mission_sub_ =
      pnh_.subscribe("query_missions", 1, &MissionControlNode::queryMissionsCallback, this);
  remove_mission_sub_ =
      pnh_.subscribe("remove_missions", 1, &MissionControlNode::removeMissionsCallback, this);

  // Advertise all topics and services
  report_mission_load_state_pub_ =
      pnh_.advertise<ReportLoadMissionState>("report_mission_load_state", 100);
  report_mission_execute_state_pub_ =
      pnh_.advertise<ReportExecuteMissionState>("report_mission_execute_state", 100);
  report_missions_pub_ = pnh_.advertise<ReportMissions>("report_missions", 100);
  report_heartbeat_pub_ = pnh_.advertise<ReportHeartbeat>("report_heartbeat", 100);

  heartbeat_timer_ = pnh_.createTimer(
      ros::Duration(1.0 / heartbeat_rate),
      &MissionControlNode::reportHeartbeat, this);

  update_timer_ = pnh_.createTimer(
      ros::Duration(1.0 / update_rate),
      &MissionControlNode::update, this);
}

void MissionControlNode::reportHeartbeat(const ros::TimerEvent&)
{
  ReportHeartbeat msg;
  msg.header.stamp = ros::Time::now();
  msg.seq_id = heartbeat_sequence_id_++;
  report_heartbeat_pub_.publish(msg);
}

namespace
{

using ReportExecuteMissionStateType =
    ReportExecuteMissionState::_execute_mission_state_type;

const char *to_string(ReportExecuteMissionStateType state)
{
  switch (state)
  {
    case ReportExecuteMissionState::ERROR:
      return "ERROR";
    case ReportExecuteMissionState::ABORTING:
      return "ABORTING";
    case ReportExecuteMissionState::COMPLETE:
      return "COMPLETE";
    case ReportExecuteMissionState::PAUSED:
      return "PAUSED";
    case ReportExecuteMissionState::EXECUTING:
      return "EXECUTING";
  }
}

}  // namespace

void MissionControlNode::reportOn(const Mission& mission)
{
  ReportExecuteMissionState msg;
  msg.header.stamp = ros::Time::now();
  msg.mission_id = mission.id();

  // TODO(hidmic): revisit state enum in mission control ROS interface
  switch (mission.status())
  {
    case Mission::Status::PENDING:
    case Mission::Status::EXECUTING:
      msg.execute_mission_state = ReportExecuteMissionState::EXECUTING;
      msg.current_behavior_name = "";  // TODO(hidmic): populate
      break;
    case Mission::Status::ABORTING:
      msg.execute_mission_state = ReportExecuteMissionState::ABORTING;
      msg.current_behavior_name = "";  // TODO(hidmic): populate
      break;
    case Mission::Status::COMPLETED:
    case Mission::Status::PREEMPTED:
    case Mission::Status::ABORTED:
      msg.execute_mission_state = ReportExecuteMissionState::COMPLETE;
      break;
    default:
      break;
  }

  if (last_mission_state_report_.mission_id != msg.mission_id ||
      last_mission_state_report_.execute_mission_state != msg.execute_mission_state)
  {
    ROS_INFO_STREAM("Mission [" << msg.mission_id << "] " <<
                    to_string(msg.execute_mission_state));
    report_mission_execute_state_pub_.publish(msg);
    last_mission_state_report_ = msg;
  }
}

void MissionControlNode::spin()
{
  ros::spin();
}

void MissionControlNode::update(const ros::TimerEvent&)
{
  if (current_mission_)
  {
    reportOn(current_mission_->resume());

    if (!current_mission_->active())
    {
      current_mission_.reset();
    }
  }
}

void MissionControlNode::loadMissionCallback(const mission_control::LoadMission& msg)
{
  ROS_DEBUG_STREAM(
      "loadMissionCallback - just received mission file " <<
      msg.mission_file_full_path);

  std::unique_ptr<Mission> mission =
      Mission::fromFile(msg.mission_file_full_path);

  ReportLoadMissionState outmsg;
  outmsg.header.stamp = ros::Time::now();
  if (mission)
  {
    ROS_DEBUG_STREAM(
        "loadMissionCallback - SUCCESSFULLY parsed mission file " <<
        msg.mission_file_full_path);
    int mission_id = mission->id();
    mission_map_[mission_id] = std::move(mission);

    outmsg.mission_id = mission_id;
    outmsg.load_state = ReportLoadMissionState::SUCCESS;
  }
  else
  {
    ROS_ERROR_STREAM(
        "loadMissionCallback - FAILED to parse mission file " <<
        msg.mission_file_full_path);
    outmsg.load_state = ReportLoadMissionState::FAILED;
  }
  report_mission_load_state_pub_.publish(outmsg);
}

void MissionControlNode::executeMissionCallback(const mission_control::ExecuteMission& msg)
{
  if (system_fault_ids_)
  {
    ROS_WARN("No mission can be executed in a faulty system, ignoring execute request");
    return;
  }

  if (mission_map_.count(msg.mission_id) == 0)
  {
    ROS_ERROR_STREAM("No mission [" << msg.mission_id << "] was found, ignoring execute request");
    return;
  }

  if (current_mission_)
  {
    ROS_DEBUG_STREAM(
        "executeMissionCallback - Preempting mission [" << current_mission_->id() << "]");

    reportOn(current_mission_->preempt());
  }

  current_mission_ = mission_map_[msg.mission_id];

  ROS_DEBUG_STREAM("executeMissionCallback - Executing mission [" << current_mission_->id() << "]");

  reportOn(current_mission_->start());
}

void MissionControlNode::abortMissionCallback(const mission_control::AbortMission& msg)
{
  if (!current_mission_)
  {
    ROS_WARN("No mission to abort, ignoring abort request");
    return;
  }

  if (current_mission_->id() != msg.mission_id)
  {
    ROS_WARN_STREAM(
        "Mission [" << msg.mission_id << "] is not currently executing, ignoring request");
    return;
  }

  ROS_DEBUG_STREAM("abortMissionCallback - Stopping mission [" << current_mission_->id() << "]");

  reportOn(current_mission_->abort());
}

void MissionControlNode::queryMissionsCallback(const mission_control::QueryMissions&)
{
  mission_control::ReportMissions msg;
  msg.header.stamp = ros::Time::now();

  ROS_DEBUG_STREAM("queryMissionsCallback - Reporting these missions");
  for (const auto& entry : mission_map_)
  {
    const Mission* mission = entry.second.get();
    mission_control::MissionData data;
    data.mission_id = mission->id();
    data.mission_description = mission->description();
    ROS_DEBUG_STREAM("Mission Id: " << data.mission_id << " " <<
                     "Description: " << data.mission_description);
    msg.missions.push_back(data);
  }

  report_missions_pub_.publish(msg);
}

void MissionControlNode::removeMissionsCallback(const mission_control::RemoveMissions&)
{
  ROS_DEBUG_STREAM("removeMissionsCallback - Removing missions");
  mission_map_.clear();
}

void MissionControlNode::faultCallback(const health_monitor::ReportFault& msg)
{
  if (system_fault_ids_ != msg.fault_id)
  {
    // TODO(hidmic): handle non critical faults
    if (msg.fault_id != 0)
    {
      ROS_ERROR_STREAM("Got system faults " << msg.fault_id);

      if (current_mission_)
      {
        ROS_WARN_STREAM("Aborting mission [" << current_mission_->id() << "]");
        reportOn(current_mission_->abort());
      }
    }
    else
    {
      ROS_INFO("System faults cleared!");
    }
    system_fault_ids_ = msg.fault_id;
  }
}

}  // namespace mission_control
