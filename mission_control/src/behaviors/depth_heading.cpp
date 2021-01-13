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

#include "mission_control/behaviors/depth_heading.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <map>
#include <list>

#include "mission_control/DepthHeading.h"
using mission_control::DepthHeading;
using mission_control::DepthHeadingBehavior;

DepthHeadingBehavior::DepthHeadingBehavior()
    : Behavior("depth_heading", BEHAVIOR_TYPE_MSG, "/mngr/depth_heading", "")
{
  m_depth_ena = m_heading_ena = false;
  m_speed_knots_ena = false;
  m_depth_tol = m_heading_tol = 0.0;
  m_depth = m_heading = 0.0;
  m_speed_knots = 0.0;

  ros::NodeHandle node_handle;
  depth_heading_behavior_pub =
      node_handle.advertise<mission_control::DepthHeading>("/mngr/depth_heading", 100);
}

DepthHeadingBehavior::~DepthHeadingBehavior() {}

bool DepthHeadingBehavior::getParams(ros::NodeHandle nh)
{
  double f = 0.0;

  nh.getParam("/mission_control_node/depth_heading_depth_tol", f);
  if (f != 0.0) m_depth_tol = static_cast<float>(f);
  f = 0.0;
  nh.getParam("/mission_control_node/depth_heading_heading_tol", f);
  if (f != 0.0) m_heading_tol = static_cast<float>(f);

  return true;
}

bool DepthHeadingBehavior::parseMissionFileParams()
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
      std::cout << "Depth Heading behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void DepthHeadingBehavior::publishMsg()
{
  DepthHeading msg;

  msg.depth = m_depth;
  msg.heading = m_heading;
  msg.speed_knots = m_speed_knots;

  msg.ena_mask = 0x0;
  if (m_depth_ena) msg.ena_mask |= DepthHeading::DEPTH_ENA;
  if (m_heading_ena) msg.ena_mask |= DepthHeading::HEADING_ENA;
  if (m_speed_knots_ena) msg.ena_mask |= DepthHeading::SPEED_KNOTS_ENA;

  msg.header.stamp = ros::Time::now();

  depth_heading_behavior_pub.publish(msg);
}

bool DepthHeadingBehavior::checkCorrectedData(const pose_estimator::CorrectedData& data)
{
  // A quick check to see if our RPY angles match
  // tjw debug  if (m_depth_ena && (abs(m_depth - data.depth) > m_depth_tol)) return false;
  if (m_heading_ena && (abs(m_heading - data.rpy_ang.z) > m_heading_tol))
  {
    ROS_INFO("heading corrected data returning false");
    return false;
  }
  // TODO(QNA): check shaft speed?
  ROS_INFO("corrected data returning true");

  return true;
}