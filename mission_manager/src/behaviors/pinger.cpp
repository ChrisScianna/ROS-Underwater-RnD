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

#include "mission_manager/behaviors/pinger.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <list>

#include "mission_manager/Ping.h"
using mission_manager::Ping;
using mission_manager::PingerBehavior;

PingerBehavior::PingerBehavior() : Behavior("pinger", BEHAVIOR_TYPE_MSG, "/mngr/pinger", "")
{
  m_pingenabled = false;

  ros::NodeHandle node_handle;
  fixed_rudder_behavior_pub = node_handle.advertise<mission_manager::Ping>("/mngr/pinger", 100);
}

PingerBehavior::~PingerBehavior() {}

bool PingerBehavior::getParams(ros::NodeHandle nh) { return true; }

bool PingerBehavior::parseMissionFileParams()
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
    else if (xmlParamTag.compare("ping_enabled") == 0)
    {
      m_pingenabled = false;
      int pingEnabled = std::atoi(it->getXMLTagValue().c_str());
      if (pingEnabled == 1)
      {
        m_pingenabled = true;
        m_pingenabled_ena = true;
      }
    }
    else if (xmlParamTag.compare("waveform_select") == 0)
    {
      m_waveform_select = std::atoi(it->getXMLTagValue().c_str());
      m_waveform_select_ena = true;
    }
    else if (xmlParamTag.compare("waveform_amplitude") == 0)
    {
      m_waveform_amp = std::atof(it->getXMLTagValue().c_str());
      m_waveform_amp_ena = true;
    }
    else if (xmlParamTag.compare("waveform_frequency") == 0)
    {
      m_waveform_freq = std::atof(it->getXMLTagValue().c_str());
      m_waveform_freq_ena = true;
    }
    else if (xmlParamTag.compare("waveform_ON_time_sec") == 0)
    {
      m_waveform_on = std::atof(it->getXMLTagValue().c_str());
      m_waveform_on_ena = true;
    }
    else if (xmlParamTag.compare("waveform_OFF_time_sec") == 0)
    {
      m_waveform_off = std::atof(it->getXMLTagValue().c_str());
      m_waveform_off_ena = true;
    }
    else
    {
      std::cout << "Pinger behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void PingerBehavior::publishMsg()
{
  Ping msg;

  msg.ping = m_pingenabled;
  msg.waveform_select_byte = m_waveform_select;
  msg.waveform_amplitude_cmd = m_waveform_amp;
  msg.waveform_frequency_cmd = m_waveform_freq;
  msg.waveform_ON_time_sec_cmd = m_waveform_on;
  msg.waveform_OFF_time_sec_cmd = m_waveform_off;

  msg.header.stamp = ros::Time::now();

  fixed_rudder_behavior_pub.publish(msg);
}
