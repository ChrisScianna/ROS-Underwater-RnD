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

#include "behaviors/podlog.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>

#include <string>

using namespace mission_manager;

/*  Behavioral podlog

        <podlog>
            <description>
                Turn on logging for the servocntl.
            </description>
            <when unit="sec">0</when>
            <log_id state="on">servocntl</log_id>
        </podlog>
*/

PodlogBehavior::PodlogBehavior()
    : Behavior("podlog", BEHAVIOR_TYPE_SRV, "/podlog/set_log_state", "")
{
  m_logging = false;
  m_logid = "";
}

PodlogBehavior::~PodlogBehavior() {}

bool PodlogBehavior::parseMissionFileParams()
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
    else if (xmlParamTag.compare("log_id") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        m_logging = false;
        if (it->second.compare("on") == 0)
        {
          m_logging = true;
        }
      }
      else
      {
        std::cout << "No attributes for log_id tag" << std::endl;
      }

      m_logid = it->getXMLTagValue();
    }
    else
    {
      std::cout << "Podlog behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

// TODO - commented out for now 9/9/2019 - Not sure if we are using logging node or service.
void PodlogBehavior::callService(ros::NodeHandle node_handle) {}
