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

#include <map>
#include <string>

#include "mission_control/mission_control.h"

MissionManagerNode::MissionManagerNode(ros::NodeHandle& h) : node_handle(h)
{
  heartbeat_sequence_id = 0;
  last_id = -1;
  last_state = Mission::MissionState::READY;
  reportExecuteMissionStateRate = 1.0;  // every 1 second
  reportHeartbeatRate = 1.0;            // every 1 second
  m_current_mission_id = 0;
  m_cur_mission = NULL;
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
                                             &MissionManagerNode::correctedDataCallback, this);
  report_fault_sub = node_handle.subscribe("/health_monitor/report_fault", 1,
                                           &MissionManagerNode::reportFaultCallback, this);

  load_mission_sub = node_handle.subscribe("/mngr/load_mission", 1,
                                           &MissionManagerNode::loadMissionCallback, this);
  execute_mission_sub = node_handle.subscribe("/mngr/execute_mission", 1,
                                              &MissionManagerNode::executeMissionCallback, this);
  abort_mission_sub = node_handle.subscribe("/mngr/abort_mission", 1,
                                            &MissionManagerNode::abortMissionCallback, this);
  query_mission_sub = node_handle.subscribe("/mngr/query_missions", 1,
                                            &MissionManagerNode::queryMissionsCallback, this);
  remove_mission_sub = node_handle.subscribe("/mngr/remove_missions", 1,
                                             &MissionManagerNode::removeMissionsCallback, this);

  // Advertise all topics and services
  pub_report_mission_load_state = node_handle.advertise<mission_control::ReportLoadMissionState>(
      "/mngr/report_mission_load_state", 100);
  pub_report_mission_execute_state =
      node_handle.advertise<mission_control::ReportExecuteMissionState>(
          "/mngr/report_mission_execute_state", 100);
  pub_report_missions =
      node_handle.advertise<mission_control::ReportMissions>("/mngr/report_missions", 100);
  pub_report_heartbeat =
      node_handle.advertise<mission_control::ReportHeartbeat>("/mngr/report_heartbeat", 100);

  reportExecuteMissionStateTimer =
      node_handle.createTimer(ros::Duration(reportExecuteMissionStateRate),
                              &MissionManagerNode::reportExecuteMissionState, this);
  reportHeartbeatTimer = node_handle.createTimer(ros::Duration(reportHeartbeatRate),
                                                 &MissionManagerNode::reportHeartbeat, this);
}

MissionManagerNode::~MissionManagerNode()
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

int MissionManagerNode::loadMissionFile(std::string mission_full_path)
{
  MissionParser parser(node_handle);

  Mission* newMission = new Mission();
  // Parse the specified mission
  if (parser.parseMissionFile(*newMission, mission_full_path) == false)
  {
    delete newMission;
    return -1;
  }

  m_mission_id_counter++;

  m_mission_map[m_mission_id_counter] = newMission;

  return 0;
}

int MissionManagerNode::executeMission(int missionId)
{
  if ((missionId > 0) && (m_mission_map.size() > 0) && (m_mission_map.count(missionId) > 0))
  {
    last_state = Mission::MissionState::READY;
    last_id = -1;
    m_current_mission_id = missionId;
    m_mission_map[m_current_mission_id]->ExecuteMission(node_handle);
  }
}

int MissionManagerNode::abortMission(int missionId)
{
  if ((missionId > 0) && (m_mission_map.size() > 0) && (m_mission_map.count(missionId) > 0))
  {
    last_state = Mission::MissionState::READY;
    last_id = -1;
    m_current_mission_id = missionId;
    m_mission_map[m_current_mission_id]->AbortMissionWrapper(node_handle);
  }
}

int MissionManagerNode::start()
{
  stop();

  int retval = loadMissionFile(mission_path);

  if (retval != -1) m_mission_map[m_current_mission_id]->ExecuteMission(node_handle);

  return retval;
}

int MissionManagerNode::stop()
{
  if ((m_current_mission_id != 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(m_current_mission_id) > 0))
    m_mission_map[m_current_mission_id]->Stop();

  return 0;
}

bool MissionManagerNode::spin()
{
  if (start() == 0)
  {
    while (node_handle.ok())
    {
      ros::spin();
    }
  }

  stop();

  return true;
}

void MissionManagerNode::reportHeartbeat(const ros::TimerEvent& timer)
{
  ReportHeartbeat outmsg;
  outmsg.header.stamp = ros::Time::now();
  outmsg.seq_id = heartbeat_sequence_id++;
  pub_report_heartbeat.publish(outmsg);
}

void MissionManagerNode::reportExecuteMissionState(const ros::TimerEvent& timer)
{
  if ((m_current_mission_id != 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(m_current_mission_id) > 0))
  {
    Mission::MissionState state = m_mission_map[m_current_mission_id]->GetState();

    int id = -1;
    std::string name = "";
    if (state == Mission::MissionState::ABORTING)
    {
      id = m_mission_map[m_current_mission_id]->getCurrentAbortBehaviorId();
      name = m_mission_map[m_current_mission_id]->getCurrentAbortBehaviorsName();
    }
    else
    {
      id = m_mission_map[m_current_mission_id]->getCurrentBehaviorId();
      name = m_mission_map[m_current_mission_id]->getCurrentBehaviorsName();
    }

    if ((state != last_state) || (id != last_id))
    {
      ReportExecuteMissionState outmsg;

      outmsg.current_behavior_name = name;
      outmsg.current_behavior_id = id;
      outmsg.execute_mission_state = ReportExecuteMissionState::PAUSED;
      std::string stateStr = "PAUSED";
      if (state == Mission::MissionState::EXECUTING)
      {
        outmsg.execute_mission_state = ReportExecuteMissionState::EXECUTING;
        stateStr = "EXECUTING";
      }
      if (state == Mission::MissionState::ABORTING)
      {
        outmsg.execute_mission_state = ReportExecuteMissionState::ABORTING;
        stateStr = "ABORTING";
      }
      if (state == Mission::MissionState::COMPLETE)
      {
        outmsg.execute_mission_state = ReportExecuteMissionState::COMPLETE;
        stateStr = "COMPLETE";
      }

      outmsg.mission_id = m_current_mission_id;
      outmsg.header.stamp = ros::Time::now();
      pub_report_mission_execute_state.publish(outmsg);
      ROS_INFO("In reportExecuteMissionState, mission id[%d] behavior name:[%s] status is [%s]",
               m_current_mission_id, name.c_str(), stateStr.c_str());
    }

    last_state = state;
    last_id = id;
  }
}

void MissionManagerNode::loadMissionCallback(const mission_control::LoadMission::ConstPtr& msg)
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

void MissionManagerNode::executeMissionCallback(
    const mission_control::ExecuteMission::ConstPtr& msg)
{
  // TODO(qna): Need to shutdown any mission that is currently running.
  ROS_INFO("executeMissionCallback - Executing mission id[%d]", msg->mission_id);
  executeMission(msg->mission_id);
}

void MissionManagerNode::abortMissionCallback(const mission_control::AbortMission::ConstPtr& msg)
{
  // TODO(qna): Need to shutdown any mission that is currently running.
  // TODO(qna): should probably also check the timestamp from the message
  ROS_INFO("abortMissionCallback - Aborting mission id[%d]", msg->mission_id);
  abortMission(msg->mission_id);  // assume mission ID is the current mission
}

void MissionManagerNode::queryMissionsCallback(const mission_control::QueryMissions::ConstPtr& msg)
{
  ReportMissions outmsg;
  MissionData data;
  std::map<int, Mission*>::iterator it;

  for (it = m_mission_map.begin(); it != m_mission_map.end(); it++)
  {
    data.mission_id = it->first;
    data.mission_description = it->second->getMissionDescription();
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

void MissionManagerNode::removeMissionsCallback(
    const mission_control::RemoveMissions::ConstPtr& msg)
{
  MissionData data;
  std::map<int, Mission*>::iterator it;

  ROS_INFO("removeMissionsCallback - Removing missions");

  for (it = m_mission_map.begin(); it != m_mission_map.end(); it++)
  {
    if ((it->second->GetState() != Mission::MissionState::ABORTING) &&
        (it->second->GetState() != Mission::MissionState::EXECUTING))
    {
      ROS_INFO("Deleting mission id[%d] with description[%s]", it->first,
               (it->second->getMissionDescription()).c_str());
      delete it->second;
      m_mission_map.erase(it);
    }
  }
}

void MissionManagerNode::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  // mission_map.count checks to see if a key is in the map
  if ((m_current_mission_id > 0) && (m_mission_map.size() > 0) &&
      (m_mission_map.count(m_current_mission_id) > 0))
  {
    m_mission_map[m_current_mission_id]->ProcessCorrectedPoseData(data);
  }
}

void MissionManagerNode::reportFaultCallback(const health_monitor::ReportFault::ConstPtr& msg)
{
  if (msg->fault_id != 0)
  {
    ROS_WARN_STREAM("Fault Detected [" << msg->fault_id << "] aborting mission");
    abortMission(m_current_mission_id);
  }
}

bool MissionManagerNode::endsWith(const std::string& str, const char* suffix)
{
  unsigned suffixLen = std::string::traits_type::length(suffix);
  return str.size() >= suffixLen &&
         0 == str.compare(str.size() - suffixLen, suffixLen, suffix, suffixLen);
}

bool MissionManagerNode::FoundMissionFile()
{
  ROS_INFO("Mission path is [%s]", mission_path.c_str());

  bool retval = false;
  DIR* d;
  struct dirent* dir;
  d = opendir(mission_path.c_str());

  if (d)  // if the directory is valid
  {
    if (endsWith(mission_path, ".xml")) return true;

    while (retval == false)
    {
      while ((dir = readdir(d)) != NULL)  // read file in directory
      {
        // Condition to check regular file.
        if (dir->d_type == DT_REG)
        {
          mission_path.append("/");
          mission_path.append(dir->d_name);

          ROS_INFO("Found mission file. full path is: %s\n", mission_path.c_str());
          retval = true;
          break;
        }
      }

      if (retval == false)
      {
        ROS_INFO("sleeping");
        usleep(250000);
      }
    }
  }
  else
  {
    ROS_ERROR("The directory path to the mission file is invalid: %s", mission_path.c_str());
  }

  closedir(d);
  return retval;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_control_node");

  ros::NodeHandle nh;
  ROS_INFO("Starting Mission mgr node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/mission_control_node", NODE_VERSION);

  MissionManagerNode mmn(nh);
  ros::spin();

  ROS_INFO("Mission mgr shutting down");

  return 0;
}
