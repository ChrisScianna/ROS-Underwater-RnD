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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

#ifndef REPORTMISSIONINFO_H
#define REPORTMISSIONINFO_H

#include "JausMessageOut.h"

#include <mission_manager/ReportExecuteMissionState.h>
#include <mission_manager/ReportLoadMissionState.h>
#include <mission_manager/ReportMissions.h>

struct MissionState {
  int timestamp_seconds;
  ushort missionID;
  ushort execute_mission_state;
  ushort current_behavior_id;
  string current_behavior_name;
};

struct UploadState {
  // float timestamp_seconds;
  ushort missionID;
  ushort uploadState;
};

struct Mission {
  ushort missionID;
  string description;
};

class ReportMissionInfo : public JausMessageOut {
 private:
  ros::Subscriber _subscriber_reportMissionState;
  ros::Subscriber _subscriber_reportUploadState;
  ros::Subscriber _subscriber_reportMissions;

 public:
  void init(ros::NodeHandle* nodeHandle, udpserver* udp);

  void handleReportMissionStateData(
      const mission_manager::ReportExecuteMissionState::ConstPtr& msg);
  void handleReportUploadData(const mission_manager::ReportLoadMissionState::ConstPtr& msg);
  void handleReportMissions(const mission_manager::ReportMissions& msg);

  virtual DataInfo GetPackedMessage(void* data) {}
  DataInfo GetPackedMessageMissionState(void* data);
  DataInfo GetPackedMessageUploadState(void* data);
  DataInfo GetPackedMessageMissions(void* data);
  virtual void Reset(){};
};

#endif  // REPORTMISSIONINFO_H
