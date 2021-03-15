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
#include "mission_control/behaviors/payload_command.h"

#include <string>

using mission_control::PayloadCommandBehavior;

PayloadCommandBehavior::PayloadCommandBehavior(const std::string& name,
                                               const BT::NodeConfiguration& config)
    : Behavior(name, config)
{
  getInput<std::string>("command", payloadCommand_);

  payloadCommandHasBeenPublished_ = false;
  payloadCommandPub_ =
      nodeHandle_.advertise<payload_manager::PayloadCommand>("/payload_manager/command", 1);
}

BT::NodeStatus PayloadCommandBehavior::behaviorRunningProcess()
{
  if (!payloadCommandHasBeenPublished_)
  {
    publishPayloadCommandMsg();
    payloadCommandHasBeenPublished_ = true;
  }
  else
  {
    setStatus(BT::NodeStatus::SUCCESS);
    payloadCommandHasBeenPublished_ = false;
  }
  return (status());
}

void PayloadCommandBehavior::publishPayloadCommandMsg()
{
  payload_manager::PayloadCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.command = payloadCommand_;
  payloadCommandPub_.publish(msg);
}
