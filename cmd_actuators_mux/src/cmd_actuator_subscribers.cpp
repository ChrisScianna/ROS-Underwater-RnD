/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
 *  Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos
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

#include "cmd_actuators_mux/cmd_actuator_subscribers.h"

#include <fstream>
#include <vector>
#include <string>

#include "cmd_actuators_mux/exceptions.h"

namespace cmd_actuator_mux
{
void CmdActuatorSubscribers::CmdActuatorSubs::operator<<(const YAML::Node& node)
{
  // Fill attributes with a YAML node content
  double new_timeout;
  std::string new_topic;
  node["name"] >> name;
  node["topic"] >> new_topic;
  node["timeout"] >> new_timeout;
  node["priority"] >> priority;
#ifdef HAVE_NEW_YAMLCPP
  if (node["short_desc"])
  {
#else
  if (node.FindValue("short_desc") != NULL)
  {
#endif
    node["short_desc"] >> short_desc;
  }

  if (new_topic != topic)
  {
    // Shutdown the topic if the name has changed so it gets recreated on configuration reload
    // In the case of new subscribers, topic is empty and shutdown has just no effect
    topic = new_topic;
    subs.shutdown();
  }

  if (new_timeout != timeout)
  {
    // Change timer period if the timeout changed
    timeout = new_timeout;
    timer.setPeriod(ros::Duration(timeout));
  }
}

void CmdActuatorSubscribers::configure(const YAML::Node& node)
{
  try
  {
    if (node.size() == 0)
    {
      throw EmptyCfgException("Configuration is empty");
    }

    std::vector<std::shared_ptr<CmdActuatorSubs>> new_list(node.size());
    for (unsigned int i = 0; i < node.size(); i++)
    {
      // Parse entries on YAML
      std::string new_subs_name = node[i]["name"].Scalar();
      auto old_subs = std::find_if(list.begin(), list.end(),
                                   [&new_subs_name](const std::shared_ptr<CmdActuatorSubs>& subs) {   //NOLINT
                                     return subs->name == new_subs_name;
                                   });  //NOLINT
      if (old_subs != list.end())
      {
        // For names already in the subscribers list, retain current object so we don't re-subscribe
        // to the topic
        new_list[i] = *old_subs;
      }
      else
      {
        new_list[i] = std::make_shared<CmdActuatorSubs>(i);
      }
      // update existing or new object with the new configuration
      *new_list[i] << node[i];
    }

    list = new_list;
  }
  catch (EmptyCfgException& e)
  {
    throw e;
  }
  catch (YAML::ParserException& e)
  {
    throw YamlException(e.what());
  }
  catch (YAML::RepresentationException& e)
  {
    throw YamlException(e.what());
  }
}

}  // namespace cmd_actuator_mux
