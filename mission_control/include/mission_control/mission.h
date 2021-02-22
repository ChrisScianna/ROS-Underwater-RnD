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

#ifndef MISSION_CONTROL_MISSION_H
#define MISSION_CONTROL_MISSION_H

#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>

// Behaviors
#include "mission_control/behaviors/waypoint.h"

using namespace BT;

namespace mission_control
{
class Mission
{
 public:
  static std::unique_ptr<Mission> FromMissionDefinition(const std::string& missionFullPath);
  static std::unique_ptr<Mission> FromMissionDefinition(const std::string& missionFullPath,
                                                        BT::BehaviorTreeFactory& missionFactory);

  ~Mission();

  enum class State
  {
    READY,
    EXECUTING,
    ABORTING,
    STOPPED,
    PAUSED,
    COMPLETE
  };

  NodeStatus Continue();
  void stop();

  NodeStatus getStatus();
  std::string getCurrentMissionDescription();

 private:
  explicit Mission(BT::Tree&& missionTree);

  BT::Tree tree_;
  std::string description_;   //  Description of the mission
  std::string behaviorName_;  //  Name of the action (behavior) being executed.
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_MISSION_H
