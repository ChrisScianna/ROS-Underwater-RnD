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

#include "mission_manager/behaviors/payload_command.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <list>

using mission_manager::PayloadCommandBehavior;

PayloadCommandBehavior::PayloadCommandBehavior()
    : Behavior("payload_command", BEHAVIOR_TYPE_MSG, "/payload_manager/command", "")
{
  m_command_str_ena = false;
  m_command_str = "";

  ros::NodeHandle nh;
  payload_command_behavior_pub =
      nh.advertise<payload_manager::PayloadCommand>("/payload_manager/command", 1);
}

PayloadCommandBehavior::~PayloadCommandBehavior() {}

bool PayloadCommandBehavior::getParams(ros::NodeHandle nh) { return true; }

bool PayloadCommandBehavior::parseMissionFileParams()
{
  bool retval = true;
  std::list<BehaviorXMLParam>::iterator it;
  for (it = m_behaviorXMLParams.begin(); it != m_behaviorXMLParams.end(); it++)
  {
    std::string xmlParamTag = it->getXMLTag();
    if ((xmlParamTag.compare("when") == 0) || (xmlParamTag.compare("timeout") == 0))
    {
      retval = parseTimeStamps(it);
    }
    else if (xmlParamTag.compare("command") == 0)
    {
      m_command_str = it->getXMLTagValue();
      m_command_str_ena = true;
    }
    else
    {
      std::cout << "Payload Command behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void PayloadCommandBehavior::publishMsg()
{
  payload_manager::PayloadCommand msg;

  msg.header.stamp = ros::Time::now();
  msg.command = m_command_str;

  payload_command_behavior_pub.publish(msg);
}

bool PayloadCommandBehavior::checkState(const auv_interfaces::StateStamped& data)
{
  return true;
}
