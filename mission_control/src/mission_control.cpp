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

#include <map>
#include <string>

MissionControlNode::MissionControlNode(ros::NodeHandle& h) : node_handle(h)
{
  heartbeat_sequence_id = 0;
  last_id = -1;
  last_state = Mission::MissionState::READY;
  reportExecuteMissionStateRate = 1.0;  // every 1 second
  reportHeartbeatRate = 1.0;            // every 1 second
  m_current_mission_id = 0;
  m_mission_id_counter = 0;

  disable_abort = false;
  reportExecuteMissionStateRate = 1.0;
  reportHeartbeatRate = 1.0;

  // Get runtime parameters
  node_handle.getParam("/mission_control_node/mission_path", mission_path);
  node_handle.getParam("/mission_control_node/disable_abort", disable_abort);

  node_handle.getParam("/mission_control_node/report_execute_mission_state_rate",
                       reportExecuteMissionStateRate);
  node_handle.getParam("/mission_control_node/report_heart_beat_rate", reportHeartbeatRate);

  ROS_INFO("mission path:[%s]", mission_path.c_str());
  ROS_INFO("disable abort:[%s]", disable_abort ? "true" : "false");
  ROS_INFO("report executemissionstate rate:[%lf]", reportExecuteMissionStateRate);
  ROS_INFO("report heartbeat rate:[%lf]", reportHeartbeatRate);

  // Subscribe to all topics
  sub_corrected_data = node_handle.subscribe("/pose/corrected_data", 1,
                                             &MissionControlNode::correctedDataCallback, this);
  report_fault_sub = node_handle.subscribe("/health_monitor/report_fault", 1,
                                           &MissionControlNode::reportFaultCallback, this);
  load_mission_sub = node_handle.subscribe("/mission_control_node/load_mission", 1,
                                           &MissionControlNode::loadMissionCallback, this);
  execute_mission_sub = node_handle.subscribe("/mission_control_node/execute_mission", 1,
                                              &MissionControlNode::executeMissionCallback, this);
  abort_mission_sub = node_handle.subscribe("/mission_control_node/abort_mission", 1,
                                            &MissionControlNode::abortMissionCallback, this);
  query_mission_sub = node_handle.subscribe("/mission_control_node/query_missions", 1,
                                            &MissionControlNode::queryMissionsCallback, this);
  remove_mission_sub = node_handle.subscribe("/mission_control_node/remove_missions", 1,
                                             &MissionControlNode::removeMissionsCallback, this);

  // Advertise all topics and services
  pub_report_mission_load_state = node_handle.advertise<mission_control::ReportLoadMissionState>(
      "/mission_control_node/report_mission_load_state", 100);
  pub_report_mission_execute_state =
      node_handle.advertise<mission_control::ReportExecuteMissionState>(
          "/mission_control_node/report_mission_execute_state", 100);
  pub_report_missions = node_handle.advertise<mission_control::ReportMissions>(
      "/mission_control_node/report_missions", 100);
  pub_report_heartbeat = node_handle.advertise<mission_control::ReportHeartbeat>(
      "/mission_control_node/report_heartbeat", 100);

  reportExecuteMissionStateTimer =
      node_handle.createTimer(ros::Duration(reportExecuteMissionStateRate),
                              &MissionControlNode::reportExecuteMissionState, this);
  reportHeartbeatTimer = node_handle.createTimer(ros::Duration(reportHeartbeatRate),
                                                 &MissionControlNode::reportHeartbeat, this);
}

MissionControlNode::~MissionControlNode()
{
  stop();

  std::map<int, Mission*>::iterator it;

  for (it = m_mission_map.begin(); it != m_mission_map.end(); it++)
  {
    if (it->second != NULL)
    {
      delete it->second;
    }
  }

  m_mission_map.clear();
}

int MissionControlNode::loadMissionFile(std::string mission_full_path)
{
  Mission* newMission = new Mission();
  if (newMission->loadBehavior(mission_full_path) == true)
  {
    m_mission_id_counter++;
    m_mission_map[m_mission_id_counter] = newMission;
    return 0;
  }
  else
  {
    delete newMission;
    return -1;
  }
}

int MissionControlNode::executeMission(int missionId)
{
  if ((missionId > 0) && (m_mission_map.size() > 0) && (m_mission_map.count(missionId) > 0))
  {
    last_state = Mission::MissionState::READY;
    last_id = -1;
    m_current_mission_id = missionId;
    m_mission_map[m_current_mission_id]->executeMission();
  }
}

int MissionControlNode::abortMission(int missionId) {}

int MissionControlNode::start() { return 0; }

int MissionControlNode::stop() { return 0; }

bool MissionControlNode::spin() {}

void MissionControlNode::reportHeartbeat(const ros::TimerEvent& timer)
{
  ReportHeartbeat outmsg;
  outmsg.header.stamp = ros::Time::now();
  outmsg.seq_id = heartbeat_sequence_id++;
  pub_report_heartbeat.publish(outmsg);
}

void MissionControlNode::reportExecuteMissionState(const ros::TimerEvent& timer)
{
  if ((m_current_mission_id != 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(m_current_mission_id) > 0))
  {
    // TODO GetState
    // Mission::MissionState state = m_mission_map[m_current_mission_id]->GetState();
    ReportExecuteMissionState outmsg;

    outmsg.current_behavior_name = "name";
    outmsg.current_behavior_id = 0;
    outmsg.execute_mission_state = ReportExecuteMissionState::PAUSED;
    outmsg.mission_id = m_current_mission_id;
    outmsg.header.stamp = ros::Time::now();
    pub_report_mission_execute_state.publish(outmsg);
    ROS_INFO("In reportExecuteMissionState, mission id[%d] behavior name:[%s] status is [%s]",
             m_current_mission_id, "name", "RUNNING");
  }
}

void MissionControlNode::loadMissionCallback(const mission_control::LoadMission::ConstPtr& msg)
{
  ROS_INFO("loadMissionCallback - just received mission file '%s'",
           (msg->mission_file_full_path).c_str());

  int retval = loadMissionFile(msg->mission_file_full_path);

  ReportLoadMissionState outmsg;
  if (retval == -1)
  {
    outmsg.load_state = ReportLoadMissionState::FAILED;
    outmsg.mission_id = 0;
    ROS_INFO("loadMissionCallback - FAILED to parse mission file '%s'",
             (msg->mission_file_full_path).c_str());
  }
  else
  {
    outmsg.load_state = ReportLoadMissionState::SUCCESS;
    outmsg.mission_id = m_mission_id_counter;
    ROS_INFO("loadMissionCallback - SUCCESSFULLY parsed mission file '%s'",
             (msg->mission_file_full_path).c_str());
  }

  outmsg.header.stamp = ros::Time::now();

  pub_report_mission_load_state.publish(outmsg);
}

void MissionControlNode::executeMissionCallback(
    const mission_control::ExecuteMission::ConstPtr& msg)
{
  // TODO(qna): Need to shutdown any mission that is currently running.
  ROS_INFO("executeMissionCallback - Executing mission id[%d]", msg->mission_id);
  executeMission(msg->mission_id);
}

void MissionControlNode::abortMissionCallback(const mission_control::AbortMission::ConstPtr& msg)
{
  ROS_INFO("abortMissionCallback - Aborting mission id[%d]", msg->mission_id);
}

void MissionControlNode::queryMissionsCallback(const mission_control::QueryMissions::ConstPtr& msg)
{
  ReportMissions outmsg;
  MissionData data;
  std::map<int, Mission*>::iterator it;

  for (it = m_mission_map.begin(); it != m_mission_map.end(); it++)
  {
    data.mission_id = it->first;
    data.mission_description = it->second->getCurrentMissionDescription();
    outmsg.missions.push_back(data);
  }

  outmsg.header.stamp = ros::Time::now();
  pub_report_missions.publish(outmsg);

  ROS_INFO("queryMissionsCallback - Reporting these missions");
  for (int i = 0; i < outmsg.missions.size(); ++i)
  {
    const mission_control::MissionData& mdata = outmsg.missions[i];
    ROS_INFO("Mission Id:[%d] Description:[%s]", mdata.mission_id,
             mdata.mission_description.c_str());
  }
}

void MissionControlNode::removeMissionsCallback(
    const mission_control::RemoveMissions::ConstPtr& msg)
{
}

void MissionControlNode::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  // TODO This should be inside each node.
  // mission_map.count checks to see if a key is in the map
  if ((m_current_mission_id > 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(m_current_mission_id) > 0))
  {
    // m_mission_map[m_current_mission_id]->ProcessCorrectedPoseData(data);
  }
}

void MissionControlNode::reportFaultCallback(const health_monitor::ReportFault::ConstPtr& msg)
{
  if (msg->fault_id != 0)
  {
    ROS_WARN_STREAM("Fault Detected [" << msg->fault_id << "] aborting mission");
    abortMission(m_current_mission_id);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_control_node");

  ros::NodeHandle nh;
  ROS_INFO("Starting Mission mgr node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/mission_control_node", NODE_VERSION);

  MissionControlNode mcn(nh);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Mission control shutting down");

  return 0;
}
