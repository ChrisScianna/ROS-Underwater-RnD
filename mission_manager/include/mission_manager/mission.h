/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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

#ifndef MISSION_MANAGER_MISSION_H
#define MISSION_MANAGER_MISSION_H

#include <vector>
#include <string>
#include "mission_manager/behavior.h"

namespace mission_manager
{

class Mission
{
 public:
  Mission();
  virtual ~Mission();

  void addBehavior(Behavior *behavior);
  Behavior *getNextBehavior(bool reset = false);

  void addAbortBehavior(Behavior *behavior);
  Behavior *getNextAbortBehavior(bool reset = false);

  enum MissionState
  {
    READY,
    EXECUTING,
    ABORTING,
    STOPPED,
    PAUSED,
    COMPLETE
  };
  void SetState(MissionState state);
  MissionState GetState();
  boost::mutex m_MissionStateLock;

  void ExecuteMission(ros::NodeHandle nh);
  void AbortMissionWrapper(ros::NodeHandle nh);

  void Stop();

  boost::shared_ptr<boost::thread> m_threadProcMission, m_threadAbortMission;

  void ProcessState(const auv_interfaces::StateStamped &data);

  void setMissionDescription(std::string descStr) { m_mission_description = descStr; }
  std::string getMissionDescription() { return m_mission_description; }

  std::string getCurrentBehaviorsName() { return current_behavior_name; }
  std::string getCurrentAbortBehaviorsName() { return current_abort_behavior_name; }

  int getCurrentBehaviorId() { return current_behavior_id; }
  int getCurrentAbortBehaviorId() { return current_abort_behavior_id; }

 private:
  void AbortMission();

  ros::NodeHandle node_handle;
  ros::Timer callBackTmr;

  MissionState missionState;
  void processAbort();
  void processMission();

  Behavior *m_current_behavior;  // this is a the current behavior which can be accessed from ROS
                                 // callbacks
  boost::mutex m_mutCallbacks;   // this is mutex to give safe access in callbacks

  std::vector<Behavior *> m_behaviors;
  std::vector<Behavior *>::iterator m_behavior_iterator;

  std::vector<Behavior *> m_aborts;
  std::vector<Behavior *>::iterator m_abort_behavior_iterator;

  int mission_status;  // loaded, running, complete, aborted, error
  double elasped_time;

  std::string m_mission_description;

  std::string current_behavior_name;
  int current_behavior_id;

  std::string current_abort_behavior_name;
  int current_abort_behavior_id;
};

}   //  namespace mission_manager

#endif  //  MISSION_MANAGER_MISSION_H
