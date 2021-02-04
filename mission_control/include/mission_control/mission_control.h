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
 * mission_control.h
 */

#ifndef MISSION_CONTROL_MISSION_CONTROL_H
#define MISSION_CONTROL_MISSION_CONTROL_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <dirent.h>
#include <ros/ros.h>
#include <stdint.h>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <map>
#include <string>
#include <memory>

#include "health_monitor/ReportFault.h"
#include "mission_control/AbortMission.h"
#include "mission_control/ExecuteMission.h"
#include "mission_control/LoadMission.h"
#include "mission_control/QueryMissions.h"
#include "mission_control/RemoveMissions.h"
#include "mission_control/ReportExecuteMissionState.h"
#include "mission_control/ReportHeartbeat.h"
#include "mission_control/ReportLoadMissionState.h"
#include "mission_control/ReportMissions.h"
#include "mission_control/behavior.h"
#include "mission_control/mission.h"

#define LOGGING (1)

// Version 2.0 rewritten in 2019 for SeaScout mkIII
// Version 2.1 removed the battery handling for faults. There is now a
//      HeatlhMonitor ROS Node that publishes a ReportFault message.
//      The mission manager now subscribes to the ReportFault message.
#define NODE_VERSION "2.1x"

using namespace BT;

using mission_control::LoadMission;
using mission_control::Mission;
using mission_control::MissionData;
using mission_control::ReportExecuteMissionState;
using mission_control::ReportHeartbeat;
using mission_control::ReportLoadMissionState;
using mission_control::ReportMissions;

class MissionControlNode
{
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

  ros::Timer reportHeartbeatTimer;
  ros::Timer executeMissionTimer;

 private:
  // Vars holding runtime params
  double reportExecuteMissionStateRate;
  double reportHeartbeatRate;

  int m_current_mission_id;
  int m_mission_id_counter;

  std::unordered_map<int, std::shared_ptr<Mission>> m_mission_map;

  uint64_t heartbeat_sequence_id;

 public:
  explicit MissionControlNode(ros::NodeHandle& h);
  ~MissionControlNode();

  int loadMissionFile(std::string mission_full_path);
  int executeMission(int missionId);
  int abortMission(int missionId);
  int stopMission();

  void reportHeartbeat(const ros::TimerEvent& timer);
  void executeMissionT(const ros::TimerEvent& timer);
  void reportExecuteMissionState(const ros::TimerEvent& timer);
  void loadMissionCallback(const mission_control::LoadMission::ConstPtr& msg);
  void executeMissionCallback(const mission_control::ExecuteMission::ConstPtr& msg);
  void abortMissionCallback(const mission_control::AbortMission::ConstPtr& msg);
  void queryMissionsCallback(const mission_control::QueryMissions::ConstPtr& msg);
  void removeMissionsCallback(const mission_control::RemoveMissions::ConstPtr& msg);
  void reportFaultCallback(const health_monitor::ReportFault::ConstPtr& msg);
};

#endif  // MISSION_CONTROL_MISSION_CONTROL_H
