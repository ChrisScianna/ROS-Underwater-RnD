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

#include "mission_manager/behaviors/fixed_rudder.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <map>
#include <list>
#include <string>

#include "mission_manager/FixedRudder.h"
using mission_manager::FixedRudder;
using mission_manager::FixedRudderBehavior;

FixedRudderBehavior::FixedRudderBehavior()
    : Behavior("fixed_rudder", BEHAVIOR_TYPE_MSG, "/mngr/fixed_rudder", "")
{
  m_depth_ena = m_rudder_ena = m_altitude_ena = false;
  m_speed_knots_ena = false;
  m_depth_tol = m_rudder_tol = m_altitude_tol = 0.0;
  m_depth = m_rudder = m_altitude = 0.0;
  m_speed_knots = 0.0;

  ros::NodeHandle node_handle;
  fixed_rudder_behavior_pub =
      node_handle.advertise<mission_manager::FixedRudder>("/mngr/fixed_rudder", 100);
}

FixedRudderBehavior::~FixedRudderBehavior() {}

bool FixedRudderBehavior::getParams(ros::NodeHandle nh)
{
  double f = 0.0;

  nh.getParam("/mission_manager_node/fixed_rudder_depth_tol", f);
  if (f != 0.0) m_depth_tol = static_cast<float>(f);
  f = 0.0;
  nh.getParam("/mission_manager_node/fixed_rudder_rudder_tol", f);
  if (f != 0.0) m_rudder_tol = static_cast<float>(f);

  return true;
}

bool FixedRudderBehavior::parseMissionFileParams()
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
    else if (xmlParamTag.compare("depth") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0)
        {
          m_depth_unit = it->second;
        }
      }

      m_depth = std::atof(it->getXMLTagValue().c_str());
      m_depth_ena = true;
    }
    else if (xmlParamTag.compare("altitude") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0)
        {
          m_altitude_unit = it->second;
        }
      }

      m_altitude = std::atof(it->getXMLTagValue().c_str());
      m_altitude_ena = true;
    }
    else if (xmlParamTag.compare("rudder") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0)
        {
          m_rudder_unit = it->second;
        }
      }

      m_rudder = std::atof(it->getXMLTagValue().c_str());
      m_rudder_ena = true;
    }
    else if (xmlParamTag.compare("speed_knots") == 0)
    {
      m_speed_knots = std::atof(it->getXMLTagValue().c_str());
      m_speed_knots_ena = true;
    }
    else
    {
      std::cout << "Fixed Rudder behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void FixedRudderBehavior::publishMsg()
{
  FixedRudder msg;

  msg.depth = m_depth;
  msg.rudder = m_rudder;
  msg.speed_knots = m_speed_knots;

  msg.ena_mask = 0x0;
  if (m_depth_ena) msg.ena_mask |= FixedRudder::DEPTH_ENA;
  if (m_rudder_ena) msg.ena_mask |= FixedRudder::RUDDER_ENA;
  if (m_speed_knots_ena) msg.ena_mask |= FixedRudder::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  fixed_rudder_behavior_pub.publish(msg);
}

bool FixedRudderBehavior::checkState(const auv_interfaces::StateStamped& data)
{
  // A quick check to see if our depth matches, note: rudder is not part of corrected data.
  // TODO(QNA): put back in when depth working  if (m_depth_ena && (abs(m_depth - data.depth) >
  // m_depth_tol)) return false;
  // TODO(QNA): check shaft speed?
  ROS_INFO("corrected data returning true");

  return true;
}
