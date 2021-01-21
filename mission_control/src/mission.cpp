
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

#include "mission_control/mission.h"

#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <string>

using mission_control::Mission;
using namespace mission_control;

Mission::Mission()
{
  currentBehaviorID = 0;

  _missionFactory.registerNodeType<Waypoint1>("Waypoint1");
  _missionFactory.registerNodeType<Attitude_Servo>("Attitude_Servo");
  _missionFactory.registerNodeType<Depth_Heading>("Depth_Heading");
}

Mission::~Mission() {}

bool Mission::loadBehavior(const std::string &missionFullPath)
{
  //  TODO Check for erros and return -1 if were.
  if (access(missionFullPath.c_str(), F_OK) != -1)
  {
    _missionFullPath = missionFullPath;
    _missionTree = _missionFactory.createTreeFromFile(_missionFullPath);
    _missionDescription = _missionTree.rootNode()->name();
    printTreeRecursively(_missionTree.rootNode());
    return true;
  }

  return false;
}

void Mission::Stop() { _missionTree.haltTree(); }

int Mission::getMissionStatus() { return 0; }

std::string Mission::getCurrentMissionDescription() { return _missionDescription; }

std::string Mission::getCurrentBehavioralName() { return "Current Behavioral Name"; }

int Mission::getCurrentBehaviorID() { return 0; }

void Mission::executeMission() { _missionStatus = _missionTree.tickRoot(); }

void Mission::processMission() {}