/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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

#ifndef MISSION_CONTROL_BEHAVIORS_TIMEOUT_H_
#define MISSION_CONTROL_BEHAVIORS_TIMEOUT_H_

#include <behaviortree_cpp_v3/decorator_node.h>

#include <atomic>
#include <chrono>
#include <string>

#include "mission_control/behaviors/internal/timer_queue.h"

namespace mission_control
{
class TimeoutNode : public BT::DecoratorNode
{
 public:
  TimeoutNode(const std::string& name, std::chrono::milliseconds timeout);

  TimeoutNode(const std::string& name, const BT::NodeConfiguration& config);

  ~TimeoutNode() override { halt(); }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<unsigned>("msec", "Timeout to halt child if still running, in milliseconds")};
  }

  void halt() override
  {
    timer_.cancelAll();
    DecoratorNode::halt();
  }

 private:
  internal::TimerQueue timer_;

  BT::NodeStatus tick() override;

  std::chrono::milliseconds timeout_;

  enum class Status
  {
    PENDING,
    RUNNING,
    EXPIRED
  };
  std::atomic<Status> timeout_status_;

  bool read_parameter_from_ports_;
};

}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_TIMEOUT_H_
