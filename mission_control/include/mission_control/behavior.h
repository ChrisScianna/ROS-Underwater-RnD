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

#ifndef MISSION_CONTROL_BEHAVIOR_H
#define MISSION_CONTROL_BEHAVIOR_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <string>

namespace mission_control
{
class Behavior : public BT::AsyncActionNode
{
 public:
  Behavior(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
  {
    idleToRunning = true;
  }

  ~Behavior() {}

  // You must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    switch (status())
    {
      case BT::NodeStatus::IDLE:
        setStatus(BT::NodeStatus::RUNNING);
        break;
      case BT::NodeStatus::RUNNING:
        setStatus(behaviorRunningProcess());
        break;
      case BT::NodeStatus::SUCCESS:
        halt();
        break;
      case BT::NodeStatus::FAILURE:
        setStatus(BT::NodeStatus::FAILURE);
        break;
      default:
        throw std::logic_error("Unexpected state in ::tick()");
        break;
    }
    return status();
  }
  virtual void halt() override { setStatus(BT::NodeStatus::IDLE); }
  
  /// Method (to be implemented by the user) to implement the function when the satus is RUNNING
  /// User should return the NodeStatus of the action (RUNNING, SUCCESS or FAILURE).
  virtual BT::NodeStatus behaviorRunningProcess() = 0;

 private:
  bool idleToRunning;
  BT::NodeStatus behaviorStatus;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIOR_H
