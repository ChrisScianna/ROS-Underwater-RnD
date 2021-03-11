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

#include "mission_manager/mission_parser.h"
#include <ros/console.h>
#include <stdarg.h>
#include <list>
#include <string>

#include "mission_manager/behavior.h"
#include "mission_manager/behavior_factory.h"
#include "mission_manager/mission.h"

using mission_manager::MissionParser;

static char *create_msg(const char *msg, va_list orig)
{
  int n, size = 1024;
  char *p, *np;
  va_list ap;

  if ((p = static_cast<char *>(malloc(size))) == NULL) return NULL;

  while (1)
  {
    va_copy(ap, orig);
    n = vsnprintf(p, size, msg, ap);
    va_end(ap);

    if ((n > -1) && (n < size)) return p;
    if (n > -1)
      size += n + 1;
    else
      size *= 2;
    if ((np = static_cast<char *>(realloc(p, size))) == NULL)
    {
      free(p);
      return NULL;
    }
    else
      p = np;
  }
}

MissionParser::MissionParser() {}

MissionParser::MissionParser(ros::NodeHandle nh) : node_handle(nh) {}

MissionParser::~MissionParser() {}

bool MissionParser::parseMissionFile(Mission &mission, const std::string &mission_file)
{
  int rc;

  TiXmlDocument doc(mission_file);

  try
  {
    if (!doc.LoadFile())
    {
      ROS_ERROR("%s(): Could not read mission file.", __FUNCTION__);
      return false;
    }

    TiXmlElement *pMissionElement = doc.FirstChildElement("mission");
    if (pMissionElement == NULL)
    {
      ROS_WARN("Could not get mission element");
      return false;
    }

    // look for mission description
    TiXmlElement *pDesc = pMissionElement->FirstChildElement("description");
    if (pDesc != NULL)
    {
      ROS_INFO("Parsing Mission Description");
      const char *str = pDesc->GetText();
      if (str != NULL)
      {
        std::string descStr = str;
        mission.setMissionDescription(descStr);
      }
      else
      {
        std::string descStr = "NONE<>";
        mission.setMissionDescription(descStr);
        ROS_WARN("Mission Description not found. Setting to 'NONE<>'!");
      }
    }
    else
    {
      ROS_WARN("Could not get mission description");
    }

    TiXmlElement *pBehaviors = pMissionElement->FirstChildElement("behaviors");
    if (pBehaviors != NULL)
    {
      ROS_INFO("Parsing Behaviors");
      TiXmlElement *pABehavior = pBehaviors->FirstChildElement();
      // A behavior
      while (pABehavior != NULL)
      {
        std::string nameStr = pABehavior->Value();

        if (nameStr.compare("description") == 0)
        {
          pABehavior = pABehavior->NextSiblingElement();
          continue;
        }

        ROS_INFO("Creating %s behavior", nameStr.c_str());

        Behavior *new_behavior = m_factory.createBehavior((const char *)nameStr.c_str());
        if (new_behavior == NULL)
        {
          ROS_WARN("Behavior node does not have an object factory [%s].", nameStr.c_str());
          return false;
        }

        if (!new_behavior->getParams(node_handle))
        {
          ROS_ERROR("Failed to get configuration parameters for behavior [%s].", nameStr.c_str());
          return false;
        }

        std::list<BehaviorXMLParam> paramXmlTags;  // = new_behavior->getBehaviorXmlParams();

        TiXmlElement *pBehaviorParam = pABehavior->FirstChildElement();
        while (pBehaviorParam != NULL)
        {
          ROS_INFO("Element Name[%s], Value[%s]", pBehaviorParam->Value(),
                   pBehaviorParam->GetText());

          BehaviorXMLParam behParam;

          behParam.setXMLTag(pBehaviorParam->Value());  // this should return a string value like
                                                        // "waypoint", "attitude_servo", etc.
          const char *str = pBehaviorParam->GetText();
          if (str != NULL)
          {
            behParam.setXMLTagValue(str);
          }

          TiXmlAttribute *pAttrib = pBehaviorParam->FirstAttribute();

          while (pAttrib != NULL)
          {
            ROS_INFO("\t\t\t Attirbute Name[%s], Value[%s]", pAttrib->Name(), pAttrib->Value());
            behParam.addXMLTagAttribute(pAttrib->Name(), pAttrib->Value());
            pAttrib = pAttrib->Next();
          }

          paramXmlTags.push_back(behParam);

          pBehaviorParam = pBehaviorParam->NextSiblingElement();
        }

        new_behavior->setBehaviorXmlParams(paramXmlTags);
        if (new_behavior->parseMissionFileParams() == false)
        {
          ROS_INFO("Error parsing mission file parameters");
          return false;
        }

        // Add the node to the current mission
        mission.addBehavior(new_behavior);

        pABehavior = pABehavior->NextSiblingElement();
      }
    }
    else
    {
      ROS_WARN("Could not get behavoirs");
      return false;
    }

    TiXmlElement *pAbortBehaviors = pMissionElement->FirstChildElement("abort");
    if (pAbortBehaviors != NULL)
    {
      ROS_INFO("Parsing Abort Behaviors");
      TiXmlElement *pABehavior = pAbortBehaviors->FirstChildElement();
      while (pABehavior != NULL)
      {
        std::string nameStr = pABehavior->Value();

        if (nameStr.compare("description") == 0)
        {
          pABehavior = pABehavior->NextSiblingElement();
          continue;
        }

        ROS_INFO("Creating %s behavior", nameStr.c_str());

        Behavior *new_behavior = m_factory.createBehavior((const char *)nameStr.c_str());
        if (new_behavior == NULL)
        {
          ROS_WARN("Abort Behavior node does not have an object factory [%s].", nameStr.c_str());
          return false;
        }

        if (!new_behavior->getParams(node_handle))
        {
          ROS_ERROR("Failed to get configuration parameters for abort behavior [%s].",
                    nameStr.c_str());
          return false;
        }

        std::list<BehaviorXMLParam> paramXmlTags;

        TiXmlElement *pBehaviorParam = pABehavior->FirstChildElement();
        while (pBehaviorParam != NULL)
        {
          ROS_INFO("Element Name[%s], Value[%s]", pBehaviorParam->Value(),
                   pBehaviorParam->GetText());
          BehaviorXMLParam behParam;

          behParam.setXMLTag(pBehaviorParam->Value());  // this should return a string value like
                                                        // "waypoint", "attitude_servo", etc.
          const char *str = pBehaviorParam->GetText();
          if (str != NULL)
          {
            behParam.setXMLTagValue(str);
          }

          TiXmlAttribute *pAttrib = pBehaviorParam->FirstAttribute();

          while (pAttrib != NULL)
          {
            ROS_INFO("\t\t\t Attirbute Name[%s], Value[%s]", pAttrib->Name(), pAttrib->Value());
            behParam.addXMLTagAttribute(pAttrib->Name(), pAttrib->Value());
            pAttrib = pAttrib->Next();
          }

          paramXmlTags.push_back(behParam);

          pBehaviorParam = pBehaviorParam->NextSiblingElement();
        }

        new_behavior->setBehaviorXmlParams(paramXmlTags);
        if (new_behavior->parseMissionFileParams() == false)
        {
          ROS_INFO("Error parsing mission file parameters. Skipping abort behavior");
        }
        else
        {
          // Add the abort node to the current mission
          mission.addAbortBehavior(new_behavior);
        }

        pABehavior = pABehavior->NextSiblingElement();
      }
    }
    else
    {
      ROS_WARN("Could not get abort behaviors");
      return false;
    }
  }
  catch (...)  // catch all exceptions
  {
    ROS_WARN("Parsing mission failed");
    return false;
  }

  return true;
}

void MissionParser::cleanupMission(Mission &mission) {}
