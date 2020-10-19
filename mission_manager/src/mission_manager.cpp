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

#include <dirent.h>

#include <ros/ros.h>
#include <stdint.h>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <map>
#include <string>

#include "health_monitor/ReportFault.h"
#include "pose_estimator/CorrectedData.h"

#include "mission_manager/AbortMission.h"
#include "mission_manager/ExecuteMission.h"
#include "mission_manager/LoadMission.h"
#include "mission_manager/QueryMissions.h"
#include "mission_manager/RemoveMissions.h"
#include "mission_manager/ReportExecuteMissionState.h"
#include "mission_manager/ReportHeartbeat.h"
#include "mission_manager/ReportLoadMissionState.h"
#include "mission_manager/ReportMissions.h"

#include "behavior.h"
#include "mission.h"
#include "mission_parser.h"

#define LOGGING (1)

// Version 2.0 rewritten in 2019 for SeaScout mkIII
// Version 2.1 removed the battery handling for faults. There is now a
//	    HeatlhMonitor ROS Node that publishes a ReportFault message.
//      The mission manager now subscribes to the ReportFault message.
#define NODE_VERSION "2.1x"

using namespace std;
using namespace pose_estimator;
using namespace mission_manager;

class MissionManagerNode {
 public:
  ros::NodeHandle node_handle;
  ros::Subscriber sub_corrected_data;

  ros::Subscriber report_fault_sub;

  ros::Subscriber load_mission_sub;
  ros::Subscriber execute_mission_sub;
  ros::Subscriber abort_mission_sub;
  ros::Subscriber query_mission_sub;
  ros::Subscriber remove_mission_sub;

  ros::Publisher pub_report_mission_load_state;
  ros::Publisher pub_report_mission_execute_state;
  ros::Publisher pub_report_missions;
  ros::Publisher pub_report_heartbeat;

  ros::Timer reportExecuteMissionStateTimer;
  Mission::MissionState last_state;
  int last_id;

  ros::Timer reportHeartbeatTimer;

 private:
  // Vars holding runtime params
  std::string mission_path, behavior_dir;
  bool disable_abort;
  double reportExecuteMissionStateRate;
  double reportHeartbeatRate;

  pose_estimator::CorrectedData m_correctedData;

  int m_current_mission_id;
  int m_mission_id_counter;
  Mission* m_cur_mission;

  std::map<int, Mission*> m_mission_map;

  unsigned long heartbeat_sequence_id;

 public:
  MissionManagerNode(ros::NodeHandle& h) : node_handle(h) {
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
    node_handle.getParam("/mission_manager_node/mission_path", mission_path);
    node_handle.getParam("/mission_manager_node/disable_abort", disable_abort);

    node_handle.getParam("/mission_manager_node/report_execute_mission_state_rate",
                         reportExecuteMissionStateRate);
    node_handle.getParam("/mission_manager_node/report_heart_beat_rate", reportHeartbeatRate);

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
    pub_report_mission_load_state = node_handle.advertise<mission_manager::ReportLoadMissionState>(
        "/mngr/report_mission_load_state", 100);
    pub_report_mission_execute_state =
        node_handle.advertise<mission_manager::ReportExecuteMissionState>(
            "/mngr/report_mission_execute_state", 100);
    pub_report_missions =
        node_handle.advertise<mission_manager::ReportMissions>("/mngr/report_missions", 100);
    pub_report_heartbeat =
        node_handle.advertise<mission_manager::ReportHeartbeat>("/mngr/report_heartbeat", 100);

    reportExecuteMissionStateTimer =
        node_handle.createTimer(ros::Duration(reportExecuteMissionStateRate),
                                &MissionManagerNode::reportExecuteMissionState, this);
    reportHeartbeatTimer = node_handle.createTimer(ros::Duration(reportHeartbeatRate),
                                                   &MissionManagerNode::reportHeartbeat, this);
  }

  ~MissionManagerNode() {
    stop();

    std::map<int, Mission*>::iterator it;

    for (it = m_mission_map.begin(); it != m_mission_map.end(); it++) {
      if (it->second != NULL) {
        delete it->second;
      }
    }

    m_mission_map.clear();
  }

  int loadMissionFile(std::string mission_full_path) {
    MissionParser parser(node_handle);

    Mission* newMission = new Mission();
    // Parse the specified mission
    if (parser.parseMissionFile(*newMission, mission_full_path) == false) {
      delete newMission;
      return -1;
    }

    m_mission_id_counter++;

    m_mission_map[m_mission_id_counter] = newMission;

    return 0;
  }

  int executeMission(int missionId) {
    if ((missionId > 0) && (m_mission_map.size() > 0) && (m_mission_map.count(missionId) > 0)) {
      last_state = Mission::MissionState::READY;
      last_id = -1;
      m_current_mission_id = missionId;
      m_mission_map[m_current_mission_id]->ExecuteMission(node_handle);
    }
  }

  int abortMission(int missionId) {
    if ((missionId > 0) && (m_mission_map.size() > 0) && (m_mission_map.count(missionId) > 0)) {
      last_state = Mission::MissionState::READY;
      last_id = -1;
      m_current_mission_id = missionId;
      m_mission_map[m_current_mission_id]->AbortMissionWrapper(node_handle);
    }
  }

  int start() {
    stop();

    int retval = loadMissionFile(mission_path);

    if (retval != -1) m_mission_map[m_current_mission_id]->ExecuteMission(node_handle);

    return retval;
  }

  int stop() {
    if ((m_current_mission_id != 0) && (m_mission_map.size() > 0) &&
        (m_mission_map.count(m_current_mission_id) > 0))
      m_mission_map[m_current_mission_id]->Stop();

    return 0;
  }

  bool spin() {
    if (start() == 0) {
      while (node_handle.ok()) {
        ros::spin();
      }
    }

    stop();

    return true;
  }

  void reportHeartbeat(const ros::TimerEvent& timer) {
    ReportHeartbeat outmsg;
    outmsg.header.stamp = ros::Time::now();
    outmsg.seq_id = heartbeat_sequence_id++;
    pub_report_heartbeat.publish(outmsg);
  }

  void reportExecuteMissionState(const ros::TimerEvent& timer) {
    if ((m_current_mission_id != 0) && (m_mission_map.size() > 0) &&
        (m_mission_map.count(m_current_mission_id) > 0)) {
      Mission::MissionState state = m_mission_map[m_current_mission_id]->GetState();

      int id = -1;
      std::string name = "";
      if (state == Mission::MissionState::ABORTING) {
        id = m_mission_map[m_current_mission_id]->getCurrentAbortBehaviorId();
        name = m_mission_map[m_current_mission_id]->getCurrentAbortBehaviorsName();
      } else {
        id = m_mission_map[m_current_mission_id]->getCurrentBehaviorId();
        name = m_mission_map[m_current_mission_id]->getCurrentBehaviorsName();
      }

      if ((state != last_state) || (id != last_id)) {
        ReportExecuteMissionState outmsg;

        outmsg.current_behavior_name = name;
        outmsg.current_behavior_id = id;
        outmsg.execute_mission_state = ReportExecuteMissionState::PAUSED;
        std::string stateStr = "PAUSED";
        if (state == Mission::MissionState::EXECUTING) {
          outmsg.execute_mission_state = ReportExecuteMissionState::EXECUTING;
          stateStr = "EXECUTING";
        }
        if (state == Mission::MissionState::ABORTING) {
          outmsg.execute_mission_state = ReportExecuteMissionState::ABORTING;
          stateStr = "ABORTING";
        }
        if (state == Mission::MissionState::COMPLETE) {
          outmsg.execute_mission_state = ReportExecuteMissionState::COMPLETE;
          stateStr = "COMPLETE";
        }
        // if (state == Mission::MissionState::PAUSED) outmsg.execute_mission_state =
        // ReportExecuteMissionState::PAUSED;

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

  void loadMissionCallback(const mission_manager::LoadMission::ConstPtr& msg) {
    ROS_INFO("loadMissionCallback - just received mission file '%s'",
             (msg->mission_file_full_path).c_str());

    int retval = loadMissionFile(msg->mission_file_full_path);

    ReportLoadMissionState outmsg;
    if (retval == -1) {
      outmsg.load_state = ReportLoadMissionState::FAILED;
      outmsg.mission_id = 0;
      ROS_INFO("loadMissionCallback - FAILED to parse mission file '%s'",
               (msg->mission_file_full_path).c_str());
    } else {
      outmsg.load_state = ReportLoadMissionState::SUCCESS;
      outmsg.mission_id = m_mission_id_counter;
      ROS_INFO("loadMissionCallback - SUCCESSFULLY parsed mission file '%s'",
               (msg->mission_file_full_path).c_str());
    }

    outmsg.header.stamp = ros::Time::now();

    pub_report_mission_load_state.publish(outmsg);
  }

  void executeMissionCallback(const mission_manager::ExecuteMission::ConstPtr& msg) {
    // TODO: Need to shutdown any mission that is currently running.
    ROS_INFO("executeMissionCallback - Executing mission id[%d]", msg->mission_id);
    executeMission(msg->mission_id);
  }

  void abortMissionCallback(const mission_manager::AbortMission::ConstPtr& msg) {
    // TODO: Need to shutdown any mission that is currently running.
    // TODO should probably also check the timestamp from the message
    ROS_INFO("abortMissionCallback - Aborting mission id[%d]", msg->mission_id);
    abortMission(msg->mission_id);  // assume mission ID is the current mission
  }

  void queryMissionsCallback(const mission_manager::QueryMissions::ConstPtr& msg) {
    ReportMissions outmsg;
    MissionData data;
    std::map<int, Mission*>::iterator it;

    for (it = m_mission_map.begin(); it != m_mission_map.end(); it++) {
      data.mission_id = it->first;
      data.mission_description = it->second->getMissionDescription();
      outmsg.missions.push_back(data);
    }

    outmsg.header.stamp = ros::Time::now();
    pub_report_missions.publish(outmsg);

    ROS_INFO("queryMissionsCallback - Reporting these missions");
    for (int i = 0; i < outmsg.missions.size(); ++i) {
      const mission_manager::MissionData& mdata = outmsg.missions[i];
      ROS_INFO("Mission Id:[%d] Description:[%s]", mdata.mission_id,
               mdata.mission_description.c_str());
    }
  }

  void removeMissionsCallback(const mission_manager::RemoveMissions::ConstPtr& msg) {
    MissionData data;
    std::map<int, Mission*>::iterator it;

    ROS_INFO("removeMissionsCallback - Removing missions");

    for (it = m_mission_map.begin(); it != m_mission_map.end(); it++) {
      if ((it->second->GetState() != Mission::MissionState::ABORTING) &&
          (it->second->GetState() != Mission::MissionState::EXECUTING)) {
        ROS_INFO("Deleting mission id[%d] with description[%s]", it->first,
                 (it->second->getMissionDescription()).c_str());
        delete it->second;
        m_mission_map.erase(it);
      }
    }
  }

  void correctedDataCallback(const pose_estimator::CorrectedData& data) {
    // mission_map.count checks to see if a key is in the map
    if ((m_current_mission_id > 0) && (m_mission_map.size() > 0) &&
        (m_mission_map.count(m_current_mission_id) > 0)) {
      m_mission_map[m_current_mission_id]->ProcessCorrectedPoseData(data);
    }
  }

  void reportFaultCallback(const health_monitor::ReportFault::ConstPtr &msg)
  {
    if (msg->fault_id != 0)
    {
      ROS_WARN_STREAM("Fault Detected [" << msg->fault_id << "] aborting mission");
      abortMission(m_current_mission_id);
    }
  }

  bool endsWith(const std::string& str, const char* suffix) {
    unsigned suffixLen = std::string::traits_type::length(suffix);
    return str.size() >= suffixLen &&
           0 == str.compare(str.size() - suffixLen, suffixLen, suffix, suffixLen);
  }

  bool FoundMissionFile() {
    ROS_INFO("Mission path is [%s]", mission_path.c_str());

    bool retval = false;
    DIR* d;
    struct dirent* dir;
    // char path[1000]="/home/joy/Downloads";
    d = opendir(mission_path.c_str());

    if (d)  // if the directory is valid
    {
      if (endsWith(mission_path, ".xml")) return true;

      while (retval == false) {
        while ((dir = readdir(d)) != NULL)  // read file in directory
        {
          // Condition to check regular file.
          if (dir->d_type == DT_REG) {
            mission_path.append("/");
            mission_path.append(dir->d_name);

            ROS_INFO("Found mission file. full path is: %s\n", mission_path.c_str());
            retval = true;
            break;
          }
        }

        if (retval == false) {
          ROS_INFO("sleeping");
          usleep(250000);
        }
      }
    } else {
      ROS_ERROR("The directory path to the mission file is invalid: %s", mission_path.c_str());
    }

    closedir(d);
    return retval;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_manager_node");

  ros::NodeHandle nh;
  ROS_INFO("Starting Mission mgr node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/mission_manager_node", NODE_VERSION);

  MissionManagerNode mmn(nh);
  //	if ( mmn.FoundMissionFile() )
  //		mmn.spin();
  ros::spin();

  ROS_INFO("Mission mgr shutting down");

  return 0;
}
