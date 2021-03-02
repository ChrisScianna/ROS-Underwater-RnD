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

MissionControlNode::MissionControlNode(ros::NodeHandle& h) : pnh(h)
{
  heartbeat_sequence_id = 0;
  last_state = Mission::State::READY;
  reportExecuteMissionStateRate = 1.0;  // every 1 second
  reportHeartbeatRate = 1.0;            // every 1 second
  currentMissionId = 0;
  MissionIdCounter = 0;

  reportExecuteMissionStateRate = 1.0;
  reportHeartbeatRate = 1.0;

  // Get runtime parameters
  pnh.getParam("report_execute_mission_state_rate", reportExecuteMissionStateRate);
  pnh.getParam("report_heart_beat_rate", reportHeartbeatRate);
  pnh.getParam("execute_mission_asynchronous_rate", executeMissionAsynchronousRate);

  ROS_DEBUG_STREAM("report executemissionstate rate: " << reportExecuteMissionStateRate);
  ROS_DEBUG_STREAM("report heartbeat rate: " << reportHeartbeatRate);

  // Subscribe to all topics
  report_fault_sub = pnh.subscribe("/health_monitor/report_fault", 1,
                                   &MissionControlNode::reportFaultCallback, this);
  load_mission_sub =
      pnh.subscribe("load_mission", 1, &MissionControlNode::loadMissionCallback, this);
  execute_mission_sub =
      pnh.subscribe("execute_mission", 1, &MissionControlNode::executeMissionCallback, this);
  abort_mission_sub =
      pnh.subscribe("abort_mission", 1, &MissionControlNode::abortMissionCallback, this);
  query_mission_sub =
      pnh.subscribe("query_missions", 1, &MissionControlNode::queryMissionsCallback, this);
  remove_mission_sub =
      pnh.subscribe("remove_missions", 1, &MissionControlNode::removeMissionsCallback, this);

  // Advertise all topics and services
  pub_report_mission_load_state =
      pnh.advertise<mission_control::ReportLoadMissionState>("report_mission_load_state", 100);
  pub_report_mission_execute_state = pnh.advertise<mission_control::ReportExecuteMissionState>(
      "report_mission_execute_state", 100);
  pub_report_missions = pnh.advertise<mission_control::ReportMissions>("report_missions", 100);
  pub_report_heartbeat = pnh.advertise<mission_control::ReportHeartbeat>("report_heartbeat", 100);
  pub_activate_manual_control = pnh.advertise<jaus_ros_bridge::ActivateManualControl>(
      "/jaus_ros_bridge/activate_manual_control", 1);

  reportExecuteMissionStateTimer =
      pnh.createTimer(ros::Duration(reportExecuteMissionStateRate),
                      &MissionControlNode::reportExecuteMissionState, this);
  reportHeartbeatTimer = pnh.createTimer(ros::Duration(reportHeartbeatRate),
                                         &MissionControlNode::reportHeartbeat, this);

  executeMissionTimer = pnh.createTimer(ros::Duration(executeMissionAsynchronousRate),
                                        &MissionControlNode::executeMissionT, this);
  executeMissionTimer.stop();

  registerBehaviorActions();
}

MissionControlNode::~MissionControlNode() {}

int MissionControlNode::loadMissionFile(std::string mission_full_path)
{
  std::unique_ptr<Mission> newMission =
      Mission::FromMissionDefinition(mission_full_path, missionFactory_);

  if (newMission)
  {
    MissionIdCounter++;
    m_mission_map[MissionIdCounter] = std::move(newMission);
    return 0;
  }
  else
    return -1;
}

int MissionControlNode::abortMission(int missionId)
{
  if (currentMissionId == missionId)
  {
    executeMissionTimer.stop();
    m_mission_map[currentMissionId]->stop();

    processAbort();
    ROS_DEBUG_STREAM("Aborting Mission " << currentMissionId);
  }
  else
  {
    ROS_ERROR_STREAM("The mission currently running is different from request");
    ROS_ERROR_STREAM("Current mission: " << currentMissionId
                                         << " - Request mission to abort: " << missionId);
  }
}

void MissionControlNode::reportHeartbeat(const ros::TimerEvent& timer)
{
  ReportHeartbeat outmsg;
  outmsg.header.stamp = ros::Time::now();
  outmsg.seq_id = heartbeat_sequence_id++;
  pub_report_heartbeat.publish(outmsg);
}

void MissionControlNode::executeMissionT(const ros::TimerEvent& timer)
{
  NodeStatus missionStatus = m_mission_map[currentMissionId]->getStatus();
  if (missionStatus != NodeStatus::SUCCESS)
  {
    missionStatus = m_mission_map[currentMissionId]->Continue();
  }
  else
    executeMissionTimer.stop();
    ROS_DEBUG_STREAM("Mission successfully completed");
}

void MissionControlNode::reportExecuteMissionState(const ros::TimerEvent& timer)
{
  if ((currentMissionId != 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(currentMissionId) > 0))
  {
    BT::NodeStatus state = m_mission_map[currentMissionId]->getStatus();
    ReportExecuteMissionState outmsg;

    // we implement a transformation between BehavioralTree NodeStatus
    // and Jaus Ros Bridge Report msg
    switch (state)
    {
      case BT::NodeStatus::IDLE:
        outmsg.execute_mission_state = mission_control::ReportExecuteMissionState::PAUSED;
        break;
      case BT::NodeStatus::RUNNING:
        outmsg.execute_mission_state = mission_control::ReportExecuteMissionState::EXECUTING;
        break;
      case BT::NodeStatus::SUCCESS:
        outmsg.execute_mission_state = mission_control::ReportExecuteMissionState::COMPLETE;
        break;
      case BT::NodeStatus::FAILURE:
        outmsg.execute_mission_state = mission_control::ReportExecuteMissionState::ABORTING;
        abortMission(currentMissionId);
        break;
    }

    outmsg.current_behavior_name = m_mission_map[currentMissionId]->getCurrentMissionDescription();
    outmsg.mission_id = currentMissionId;
    outmsg.header.stamp = ros::Time::now();
    pub_report_mission_execute_state.publish(outmsg);
  }
}

void MissionControlNode::loadMissionCallback(const mission_control::LoadMission::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("loadMissionCallback - just received mission file "
                   << msg->mission_file_full_path.c_str());

  int retval = loadMissionFile(msg->mission_file_full_path);

  ReportLoadMissionState outmsg;
  if (retval == -1)
  {
    outmsg.load_state = ReportLoadMissionState::FAILED;
    outmsg.mission_id = 0;
    ROS_ERROR_STREAM("loadMissionCallback - FAILED to parse mission file "
                     << msg->mission_file_full_path.c_str());
  }
  else
  {
    outmsg.load_state = ReportLoadMissionState::SUCCESS;
    outmsg.mission_id = MissionIdCounter;
    ROS_DEBUG_STREAM("loadMissionCallback - SUCCESSFULLY parsed mission file "
                     << msg->mission_file_full_path.c_str());
  }

  outmsg.header.stamp = ros::Time::now();
  pub_report_mission_load_state.publish(outmsg);
}

void MissionControlNode::executeMissionCallback(
    const mission_control::ExecuteMission::ConstPtr& msg)
{
  if (currentMissionId > 0)
  {
    BT::NodeStatus missionStatus = m_mission_map[currentMissionId]->getStatus();
    if ((missionStatus != BT::NodeStatus::IDLE) && (missionStatus != BT::NodeStatus::SUCCESS))
    {
      ROS_WARN_STREAM("There is a mission being executed - mission id["
                      << currentMissionId
                      << "] - Mission Status: " << m_mission_map[currentMissionId]->getStatus());
      return;
    }
  }

  if ((msg->mission_id > 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(msg->mission_id) > 0))
  {
    currentMissionId = msg->mission_id;
    executeMissionTimer.start();
    ROS_DEBUG_STREAM("executeMissionCallback - Executing mission id[" << currentMissionId << "]");
  }
  else
  {
    ROS_ERROR_STREAM("Error - Wrong Mission Number. "
                     << "Mission Number: " << msg->mission_id);
  }
}

void MissionControlNode::abortMissionCallback(const mission_control::AbortMission::ConstPtr& msg)
{
  abortMission(msg->mission_id);
  ROS_DEBUG_STREAM("abortMissionCallback - Aborting mission id " << msg->mission_id);
}

void MissionControlNode::queryMissionsCallback(const mission_control::QueryMissions::ConstPtr& msg)
{
  ReportMissions outmsg;
  MissionData data;

  ROS_DEBUG_STREAM("queryMissionsCallback - Reporting these missions");
  for (const auto& entry : m_mission_map)
  {
    data.mission_id = entry.first;
    data.mission_description = entry.second->getCurrentMissionDescription();
    outmsg.missions.push_back(data);
    ROS_DEBUG_STREAM("Mission Id: " << data.mission_id
                                    << " Description: " << data.mission_description.c_str());
  }
  outmsg.header.stamp = ros::Time::now();
  pub_report_missions.publish(outmsg);
}

void MissionControlNode::removeMissionsCallback(
    const mission_control::RemoveMissions::ConstPtr& msg)
{
  if (m_mission_map.size() > 0)
  {
    if (currentMissionId != 0)
    {
      executeMissionTimer.stop();
      m_mission_map[currentMissionId]->stop();
      currentMissionId = 0;
      MissionIdCounter = 0;
    }
    ROS_DEBUG_STREAM("removeMissionsCallback - Removing missions");
    m_mission_map.clear();
  }
}

void MissionControlNode::reportFaultCallback(const health_monitor::ReportFault::ConstPtr& msg)
{
  if (msg->fault_id != 0)
  {
    ROS_ERROR_STREAM("Fault Detected by health monitor[" << msg->fault_id << "] aborting mission");
    abortMission(currentMissionId);
  }
}

void MissionControlNode::processAbort()
{
  jaus_ros_bridge::ActivateManualControl msg;
  msg.activate_manual_control = true;
  pub_activate_manual_control.publish(msg);
}

void MissionControlNode::registerBehaviorActions()
{
  missionFactory_.registerNodeType<mission_control::GoToWaypoint>("GoToWaypoint");
  missionFactory_.registerNodeType<mission_control::MoveWithFixedRudder>("MoveWithFixedRudder");
  missionFactory_.registerNodeType<mission_control::DepthHeadingBehavior>("DepthHeadingBehavior");
  missionFactory_.registerNodeType<mission_control::AttitudeServoBehavior>("AttitudeServoBehavior");
}
