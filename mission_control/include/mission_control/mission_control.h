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

#define NODE_VERSION "1.0x"


namespace mission_control
{

class MissionControlNode
{
public:
  MissionControlNode();

  void spin();

private:
  void reportOn(const Mission& mission);
  void update(const ros::TimerEvent& ev);

  void reportHeartbeat(const ros::TimerEvent& ev);

  void loadMissionCallback(const LoadMission& msg);
  void executeMissionCallback(const ExecuteMission& msg);
  void abortMissionCallback(const AbortMission& msg);
  void queryMissionsCallback(const QueryMissions& msg);
  void removeMissionsCallback(const RemoveMissions& msg);

  void faultCallback(const health_monitor::ReportFault& msg);

  ros::NodeHandle pnh_;

  ros::Subscriber fault_sub_;

  ros::Subscriber load_mission_sub_;
  ros::Subscriber execute_mission_sub_;
  ros::Subscriber abort_mission_sub_;
  ros::Subscriber query_mission_sub_;
  ros::Subscriber remove_mission_sub_;

  ros::Publisher report_mission_load_state_pub_;
  ros::Publisher report_mission_execute_state_pub_;
  ros::Publisher report_missions_pub_;
  ros::Publisher report_heartbeat_pub_;

  ros::Timer heartbeat_timer_;
  ros::Timer update_timer_;

  uint64_t system_fault_ids_{0};
  uint64_t heartbeat_sequence_id_{0};

  std::shared_ptr<Mission> current_mission_;
  std::unordered_map<int, std::shared_ptr<Mission>> mission_map_;
  ReportExecuteMissionState last_mission_state_report_;
};

}  // namespace mission_control

#endif  // MISSION_CONTROL_MISSION_CONTROL_H
