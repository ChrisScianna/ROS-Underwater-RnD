
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

#include <string>

using BT::NodeStatus;
using BT::Tree;
using mission_control::Mission;

Mission::Mission(BT::Tree&& missionTree) : tree_(std::move(missionTree))
{
  description_ = tree_.rootNode()->name();
}

Mission::~Mission() {}

std::unique_ptr<Mission> Mission::FromMissionDefinition(const std::string& missionFullPath,
                                                        BT::BehaviorTreeFactory& missionFactory)
{
  if (access(missionFullPath.c_str(), F_OK) != -1)
  {
    return std::unique_ptr<Mission>(
        new Mission(missionFactory.createTreeFromFile(missionFullPath)));
  }
  else
    return nullptr;
}

void Mission::stop() { tree_.haltTree(); }

BT::NodeStatus Mission::getStatus() { return behaviorStatus_; }

std::string Mission::getCurrentMissionDescription() { return description_; }

BT::NodeStatus Mission::Continue()
{
  behaviorStatus_ = tree_.tickRoot();
  return behaviorStatus_;
}
