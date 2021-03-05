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

#include "mission_manager/behaviors/altitude_heading.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <map>
#include <list>

#include "mission_manager/AltitudeHeading.h"
using mission_manager::AltitudeHeading;
using mission_manager::AltitudeHeadingBehavior;

AltitudeHeadingBehavior::AltitudeHeadingBehavior()
    : Behavior("altitude_heading", BEHAVIOR_TYPE_MSG, "/mngr/altitude_heading", "")
{
  m_altitude_ena = m_heading_ena = false;
  m_speed_knots_ena = false;
  m_altitude_tol = m_heading_tol = 0.0;
  m_altitude = m_heading = 0.0;
  m_speed_knots = 0.0;

  ros::NodeHandle node_handle;
  altitude_heading_behavior_pub =
      node_handle.advertise<mission_manager::AltitudeHeading>("/mngr/altitude_heading", 100);
}

AltitudeHeadingBehavior::~AltitudeHeadingBehavior() {}

bool AltitudeHeadingBehavior::getParams(ros::NodeHandle nh)
{
  double f = 0.0;

  nh.getParam("/mission_manager_node/altitude_heading_altitude_tol", f);
  if (f != 0.0) m_altitude_tol = static_cast<float>(f);
  f = 0.0;
  nh.getParam("/mission_manager_node/altitude_heading_heading_tol", f);
  if (f != 0.0) m_heading_tol = static_cast<float>(f);

  return true;
}

bool AltitudeHeadingBehavior::parseMissionFileParams()
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
    else if (xmlParamTag.compare("heading") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0)
        {
          m_heading_unit = it->second;
        }
      }

      m_heading = std::atof(it->getXMLTagValue().c_str());
      m_heading_ena = true;
    }
    else if (xmlParamTag.compare("speed_knots") == 0)
    {
      m_speed_knots = std::atof(it->getXMLTagValue().c_str());
      m_speed_knots_ena = true;
    }
    else
    {
      std::cout << "Altitude Heading behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void AltitudeHeadingBehavior::publishMsg()
{
  AltitudeHeading msg;

  msg.altitude = m_altitude;
  msg.heading = m_heading;
  msg.speed_knots = m_speed_knots;

  msg.ena_mask = 0x0;
  if (m_altitude_ena) msg.ena_mask |= AltitudeHeading::ALTITUDE_ENA;
  if (m_heading_ena) msg.ena_mask |= AltitudeHeading::HEADING_ENA;
  if (m_speed_knots_ena) msg.ena_mask |= AltitudeHeading::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  altitude_heading_behavior_pub.publish(msg);
}

bool AltitudeHeadingBehavior::checkState(const auv_interfaces::StateStamped& data)
{
  // A quick check to see if our RPY angles match
  // tjw debug  if (m_depth_ena && (abs(m_depth - data.depth) > m_depth_tol)) return false;
  double heading = data.state.manoeuvring.pose.mean.orientation.z;
  if (m_heading_ena && (abs(m_heading - heading) > m_heading_tol))
  {
    ROS_INFO("heading corrected data returning false");
    return false;
  }
  // TODO(QNA): check shaft speed?
  ROS_INFO("corrected data returning true");

  return true;
}
