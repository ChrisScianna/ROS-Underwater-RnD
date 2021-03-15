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
#include "jaus_ros_bridge/ActivateManualControl.h"
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

#include "mission_control/behaviors/waypoint.h"
#include "mission_control/behaviors/fixed_rudder.h"
#include "mission_control/behaviors/depth_heading.h"
#include "mission_control/behaviors/attitude_servo.h"
#include "mission_control/behaviors/payload_command.h"

#define LOGGING (1)

#define NODE_VERSION "1.0x"

using BT::NodeStatus;
using BT::Tree;
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
  ros::Publisher pub_activate_manual_control;

  ros::Timer reportExecuteMissionStateTimer;
  Mission::State last_state;

  ros::Timer reportHeartbeatTimer;
  ros::Timer executeMissionTimer;

  explicit MissionControlNode(ros::NodeHandle& h);
  ~MissionControlNode();

  int loadMissionFile(std::string mission_full_path);
  int executeMission(int missionId);
  int abortMission(int missionId);
  void processAbort();
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

private:
  void registerBehaviorActions();

  ros::NodeHandle pnh;
  // Vars holding runtime params
  double reportExecuteMissionStateRate;
  double reportHeartbeatRate;
  double executeMissionAsynchronousRate;

  int currentMissionId;
  int MissionIdCounter;

  int m_current_mission_id;
  int m_mission_id_counter;

  std::unordered_map<int, std::unique_ptr<Mission>> m_mission_map;
  uint64_t heartbeat_sequence_id;
  BT::BehaviorTreeFactory missionFactory_;
};

#endif  // MISSION_CONTROL_MISSION_CONTROL_H
