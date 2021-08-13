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

#include <ros/ros.h>

#include <string>

#include "mission_control/behaviors/delay_for.h"

namespace mission_control
{
DelayForNode::DelayForNode(const std::string& name, std::chrono::milliseconds delay)
    : DecoratorNode(name, {}),
      delay_(delay),
      delay_status_(Status::PENDING),
      read_parameter_from_ports_(false)
{
  setRegistrationID("DelayFor");
}

DelayForNode::DelayForNode(const std::string& name, const BT::NodeConfiguration& config)
    : DecoratorNode(name, config),
      delay_(0u),
      delay_status_(Status::PENDING),
      read_parameter_from_ports_(true)
{
}

BT::NodeStatus DelayForNode::tick()
{
  if (Status::PENDING == delay_status_)
  {
    if (read_parameter_from_ports_)
    {
      unsigned msec_;
      auto result = getInput("delay_msec", msec_);
      if (!result)
      {
        ROS_ERROR_STREAM("Cannot '" << name() << "': " << result.error());
        return BT::NodeStatus::FAILURE;
      }
      delay_ = std::chrono::milliseconds(msec_);
    }
    delay_status_ = Status::RUNNING;

    timer_.add(delay_, [this](bool aborted) {
      if (!aborted)
      {
        delay_status_ = Status::COMPLETE;
      }
      else
      {
        delay_status_ = Status::PENDING;
      }
    });
  }

  if (Status::COMPLETE != delay_status_)
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus child_status = child()->executeTick();
  if (BT::NodeStatus::RUNNING != child_status)
  {
    delay_status_ = Status::PENDING;
  }
  return child_status;
}

}  // namespace mission_control
