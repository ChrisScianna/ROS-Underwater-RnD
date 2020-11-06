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

#ifndef __BEHAVIOR_H
#define __BEHAVIOR_H

#include <ros/ros.h>
#include <ros/time.h>
#include <string.h>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <string>

#include "pose_estimator/CorrectedData.h"
#include "std_msgs/Header.h"
#include "tinyxml/tinyxml.h"

namespace mission_manager
{
typedef enum
{
  BEHAVIOR_TYPE_MSG = 0,
  BEHAVIOR_TYPE_SRV
} behavior_type_t;

class TimeStamp
{
 public:
  TimeStamp() {}
  virtual ~TimeStamp() {}

  typedef enum
  {
    REL_TIME = 0,
    ABS_TIME,
    ABS_DATETIME
  } ts_type_t;

  ts_type_t type;
  boost::posix_time::time_duration duration;
  boost::posix_time::ptime time;
};

// class IFactory;

class BehaviorXMLParam
{
 public:
  BehaviorXMLParam(){};
  virtual ~BehaviorXMLParam(){};

  void setXMLTag(std::string val) { xmlTag = val; }
  std::string getXMLTag() { return xmlTag; }

  void setXMLTagValue(std::string val) { xmlTagValue = val; }
  std::string getXMLTagValue() { return xmlTagValue; }

  void addXMLTagAttribute(std::string name, std::string val) { xmltagAttributes[name] = val; }
  std::map<std::string, std::string> getXMLTagAttribute() { return xmltagAttributes; }

 private:
  std::string xmlTag;
  std::map<std::string, std::string> xmltagAttributes;  // name and value
  std::string xmlTagValue;
};

class Behavior
{
 public:
  Behavior(const std::string& xml_tag, behavior_type_t type, const std::string& topic,
           const std::string& service)
  {
    m_xml_tag = xml_tag;
    m_type = type;
    switch (type)
    {
      case BEHAVIOR_TYPE_MSG:
        m_topic = topic;
        break;
      case BEHAVIOR_TYPE_SRV:
        m_service = service;
        break;
    }
    m_timeout_ena = false;
    m_behavior_done = false;
    m_duration = 0;
  }

  Behavior()
  {
    m_timeout_ena = false;
    m_behavior_done = false;
    m_duration = 0;
  };
  virtual ~Behavior() {}

  // Behavior meta-data retrieval functions
  behavior_type_t getType() { return m_type; }
  const char* getXmlTag() { return m_xml_tag.c_str(); }

  // Message-type behavior implementation overrides
  virtual void publishMsg(){};
  virtual bool checkCorrectedData(const pose_estimator::CorrectedData& data) { return true; }

  // Service-type behavior implementation overrides
  virtual void callService(){};

  // Common behavior overrides
  virtual bool getParams(ros::NodeHandle nh) { return true; }
  std::list<BehaviorXMLParam>& getBehaviorXmlParams() { return m_behaviorXMLParams; }
  void setBehaviorXmlParams(std::list<BehaviorXMLParam> aList) { m_behaviorXMLParams = aList; }
  virtual bool parseMissionFileParams() = 0;

  TimeStamp getWhen() { return m_when; }
  TimeStamp getTimeout() { return m_timeout; }
  bool getTimeoutEna() { return m_timeout_ena; }
  bool getBehaviorDone() { return m_behavior_done; }
  float getBehaviorDuration() { return m_duration; }

  int ExecuteBehavior(ros::NodeHandle nh)
  {
    switch (getType())
    {
      case BEHAVIOR_TYPE_MSG:
        // Publish the appropriate behavior message
        ROS_INFO("Publishing behavior [%s]", m_xml_tag.c_str());
        publishMsg();
        break;

      case BEHAVIOR_TYPE_SRV:

        ROS_INFO("Behavior [%s] calling service", m_xml_tag.c_str());
        callService();
        break;
    }
  }

  int WaitForExecutionTimeSlot()
  {
    int retval = 0;  // 0 means success
    boost::posix_time::ptime cur_ptime;
    boost::posix_time::ptime calc_ptime;
    cur_ptime = boost::posix_time::second_clock::universal_time();

    TimeStamp when = getWhen();

    switch (when.type)
    {
      case TimeStamp::REL_TIME:
        if (when.duration.total_seconds() == 0) break;  // now
        boost::this_thread::sleep(when.duration);
        break;

      case TimeStamp::ABS_TIME:
        // Construct the appropriate absolute datetime
        calc_ptime = boost::posix_time::ptime(cur_ptime.date(), when.time.time_of_day());
        // Make sure to handle if the time is less than the current time (occurs next day)
        if (calc_ptime.time_of_day() < cur_ptime.time_of_day())
          calc_ptime += boost::gregorian::days(1);
        boost::this_thread::sleep(boost::system_time(calc_ptime));
        break;

      case TimeStamp::ABS_DATETIME:
        if (when.time < cur_ptime)
        {
          ROS_ERROR("Behavior occurs in the past; aborting mission");
          retval = -1;
          break;
        }
        boost::this_thread::sleep(boost::system_time(when.time));
        break;
    };

    return retval;
  }

  int computeExecutionTimeForBehavior()
  {
    int nsec = -1;

    TimeStamp tout = getTimeout();
    boost::posix_time::ptime calc_ptime,
        cur_ptime = boost::posix_time::second_clock::universal_time();

    switch (tout.type)
    {
      case TimeStamp::REL_TIME:
        if (tout.duration.total_seconds() == 0) return 0;  // execution time is 0 seconds
        nsec = tout.duration.total_seconds();
        break;

      case TimeStamp::ABS_TIME:
        // Construct the appropriate absolute datetime
        calc_ptime = boost::posix_time::ptime(cur_ptime.date(), tout.time.time_of_day());
        // Make sure to handle if the time is less than the current time (occurs next day)
        if (calc_ptime.time_of_day() < cur_ptime.time_of_day())
          calc_ptime += boost::gregorian::days(1);
        nsec = (calc_ptime - cur_ptime).total_seconds();
        break;

      case TimeStamp::ABS_DATETIME:
        if (tout.time < cur_ptime)
        {
          ROS_ERROR("Timeout occurs in the past; aborting mission");
          break;
        }
        nsec = (tout.time - cur_ptime).total_seconds();
        break;
    };

    return nsec;
  }

  bool startBehavior()
  {
    std::string xml_tag = getXmlTag();

    switch (getType())
    {
      case BEHAVIOR_TYPE_MSG:

        break;

      case BEHAVIOR_TYPE_SRV:

        break;
    }

    return true;
  }

  bool stopBehavior()
  {
    std::string xml_tag = getXmlTag();

    switch (getType())
    {
      case BEHAVIOR_TYPE_MSG:
        break;

      case BEHAVIOR_TYPE_SRV:
        break;
    }

    return true;
  }

 protected:
  bool parseTimeStamps(std::list<BehaviorXMLParam>::iterator it)
  {
    bool retval = true;
    bool pnts1 = true;
    bool pnts2 = true;
    std::string xmlParamTag = it->getXMLTag();
    if (xmlParamTag.compare("when") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator attrib_it;
        attrib_it = attribMap.begin();
        pnts1 = parseNodeTimeStamp(attrib_it->second, it->getXMLTagValue(), m_when);
      }
      else
      {
        std::cout << "No attributes for when tag" << std::endl;
      }
    }
    else if (xmlParamTag.compare("timeout") == 0)
    {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0)
      {
        std::map<std::string, std::string>::iterator attrib_it;
        attrib_it = attribMap.begin();
        pnts2 = parseNodeTimeStamp(attrib_it->second, it->getXMLTagValue(), m_timeout);
        m_timeout_ena = true;
      }
      else
      {
        std::cout << "No attributes for timeout tag" << std::endl;
      }
    }

    if ((pnts1 == false) || (pnts2 == false)) retval = false;

    return retval;
  }

  bool parseNodeTimeStamp(std::string unit, std::string val, TimeStamp& t)
  {
    using namespace boost::gregorian;
    using namespace boost::posix_time;

    int hrs, min, sec;
    bool ret = true;

    if (unit.compare("abs") == 0)
    {
      // Parse an absolute time in HH:MM:SS format
      if (sscanf((const char*)val.c_str(), "%d:%d:%d", &hrs, &min, &sec) != 3)
      {
        std::cout << "Could not parse \"abs\" time." << std::endl;
        ret = false;
      }
      else
      {
        t.type = TimeStamp::ABS_TIME;
        t.time = ptime(date(0, 0, 0), time_duration(hrs, min, sec));
      }
    }
    else if (unit.compare("hms") == 0)
    {
      // Parse a relative time in HH:MM:SS format
      if (sscanf((const char*)val.c_str(), "%d:%d:%d", &hrs, &min, &sec) != 3)
      {
        std::cout << "Could not parse \"hms\" time." << std::endl;
        ret = false;
      }
      else
      {
        t.type = TimeStamp::REL_TIME;
        t.duration = time_duration(hrs, min, sec);
      }
    }
    else if (unit.compare("iso") == 0)
    {
      // Parse an absolute ISO 8601 date/time string
      t.type = TimeStamp::ABS_DATETIME;
      try
      {
        t.time = from_iso_string(val);
      }
      catch (std::exception)
      {
        std::cout << "Could not parse \"iso\" time." << std::endl;
        ret = false;
      }
    }
    else if (unit.compare("sec") == 0)
    {
      // Parse a relative time in seconds
      if (sscanf((const char*)val.c_str(), "%d", &sec) != 1)
      {
        std::cout << "Could not parse \"sec\" time." << std::endl;
        ret = false;
      }
      else
      {
        t.type = TimeStamp::REL_TIME;
        t.duration = time_duration(0, 0, sec);
      }
    }
    else
    {
      ROS_INFO("parseNodeTimeStamp - unsupported time unit [%s]", unit.c_str());

      ret = false;
    }

    return ret;
  }

  TimeStamp m_when;
  TimeStamp m_timeout;
  bool m_timeout_ena;

  std::string m_xml_tag;
  std::string m_topic;
  std::string m_service;
  behavior_type_t m_type;

  bool m_behavior_done;
  float m_duration;
  std::list<BehaviorXMLParam> m_behaviorXMLParams;
};

}  // namespace mission_manager

#endif
