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


#include "cmd_actuators_mux/cmd_actuator_mux_nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include <fstream>
#include "cmd_actuators_mux/exceptions.h"

namespace cmd_actuator_mux
{
/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void CmdActuatorMuxNodelet::cmdActuatorCallback(const fin_control::SetAngles::ConstPtr& msg,
                                                unsigned int idx)
{
  // Reset general timer
  common_timer.stop();
  common_timer.start();

  // Reset timer for this source
  cmd_actuator_subs[idx]->timer.stop();
  cmd_actuator_subs[idx]->timer.start();

  cmd_actuator_subs[idx]->active = true;  // obviously his source is sending commands, so active

  // Give permit to publish to this source if it's the only active or is
  // already allowed or has higher priority that the currently allowed
  if ((cmd_actuator_subs.allowed == VACANT) || (cmd_actuator_subs.allowed == idx) ||
      (cmd_actuator_subs[idx]->priority > cmd_actuator_subs[cmd_actuator_subs.allowed]->priority))
  {
    if (cmd_actuator_subs.allowed != idx)
    {
      cmd_actuator_subs.allowed = idx;

      // Notify the world that a new cmd_actuator source took the control
      std_msgs::StringPtr acv_msg(new std_msgs::String);
      acv_msg->data = cmd_actuator_subs[idx]->name;
      active_subscriber.publish(acv_msg);
    }

    output_topic_pub.publish(msg);
  }
}

void CmdActuatorMuxNodelet::timerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  if (cmd_actuator_subs.allowed == idx ||
      (idx == GLOBAL_TIMER && cmd_actuator_subs.allowed != VACANT))
  {
    if (idx == GLOBAL_TIMER)
    {
      // No messages timeout happened for ANYONE, so last active source got stuck without further
      // messages; not a big problem, just dislodge it; but possibly reflect a problem in the
      // controller
      NODELET_WARN("CmdActuatorMux : No messages from ANY input received in the last %fs",
                   common_timer_period);
      NODELET_WARN("CmdActuatorMux : %s dislodged due to general timeout",
                   cmd_actuator_subs[cmd_actuator_subs.allowed]->name.c_str());
    }

    // No messages timeout happened to currently active source, so...
    cmd_actuator_subs.allowed = VACANT;

    // ...notify the world that nobody is publishing; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "idle";
    active_subscriber.publish(acv_msg);
  }

  if (idx != GLOBAL_TIMER) cmd_actuator_subs[idx]->active = false;
}

void CmdActuatorMuxNodelet::onInit()
{
  ros::NodeHandle& nh = this->getPrivateNodeHandle();

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure_cb = boost::bind(&CmdActuatorMuxNodelet::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server = new dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

  active_subscriber = nh.advertise<std_msgs::String>("active", 1, true);  // latched topic

  // Notify the world that by now nobody is publishing on yet
  std_msgs::StringPtr active_msg(new std_msgs::String);
  active_msg->data = "idle";
  active_subscriber.publish(active_msg);

  // could use a call to reloadConfiguration here, but it seems to automatically call it once with
  // defaults anyway.
  NODELET_DEBUG("CmdActuatorMux : successfully initialized");
}

void CmdActuatorMuxNodelet::reloadConfiguration(cmd_actuators_mux::reloadConfig& config,
                                                uint32_t unused_level)
{
  ros::NodeHandle& pnh = this->getPrivateNodeHandle();

  std::unique_ptr<std::istream> is;

  // Configuration can come directly as a yaml-formatted string or as a file path,
  // but not both, so we give priority to the first option
  if (config.yaml_cfg_data.size() > 0)
  {
    is.reset(new std::istringstream(config.yaml_cfg_data));
  }
  else
  {
    std::string yaml_cfg_file;
    if (config.yaml_cfg_file == "")
    {
      // typically fired on startup, so look for a parameter to set a default
      pnh.getParam("yaml_cfg_file", yaml_cfg_file);
    }
    else
    {
      yaml_cfg_file = config.yaml_cfg_file;
    }

    is.reset(new std::ifstream(yaml_cfg_file.c_str(), std::ifstream::in));
    if (is->good() == false)
    {
      NODELET_ERROR_STREAM("CmdActuatorMux : configuration file not found [" << yaml_cfg_file
                                                                             << "]");
      return;
    }
  }

  /*********************
  ** Yaml File Parsing
  **********************/

  YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(*is);
#else
  YAML::Parser parser(*is);
  parser.GetNextDocument(doc);
#endif

  /*********************
  ** Output Publisher
  **********************/
  std::string output_name("output");
#ifdef HAVE_NEW_YAMLCPP
  if (doc["publisher"])
  {
    doc["publisher"] >> output_name;
  }
#else
  const YAML::Node* node = doc.FindValue("publisher");
  if (node != NULL)
  {
    *node >> output_name;
  }
#endif

  if (output_topic_name != output_name)
  {
    output_topic_name = output_name;
    output_topic_pub = pnh.advertise<fin_control::SetAngles>(output_topic_name, 10);
    NODELET_DEBUG_STREAM("CmdActuatorMux : subscribe to output topic '" << output_name << "'");
  }
  else
  {
    NODELET_DEBUG_STREAM("CmdActuatorMux : no need to re-subscribe to output topic '" << output_name
                                                                                      << "'");
  }

  /*********************
  ** Input Subscribers
  **********************/
  try
  {
    cmd_actuator_subs.configure(doc["subscribers"]);
  }
  catch (EmptyCfgException& e)
  {
    NODELET_WARN_STREAM("CmdActuatorMux : yaml configured zero subscribers, check yaml content");
  }
  catch (YamlException& e)
  {
    NODELET_ERROR_STREAM("CmdActuatorMux : yaml parsing problem [" << std::string(e.what()) << "]");
  }

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  for (unsigned int i = 0; i < cmd_actuator_subs.size(); i++)
  {
    if (!cmd_actuator_subs[i]->subs)
    {
      cmd_actuator_subs[i]->subs = pnh.subscribe<fin_control::SetAngles>(
          cmd_actuator_subs[i]->topic, 10, CmdActuatorFunctor(i, this));
      NODELET_DEBUG("CmdActuatorMux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                    cmd_actuator_subs[i]->name.c_str(), cmd_actuator_subs[i]->topic.c_str(),
                    cmd_actuator_subs[i]->priority, cmd_actuator_subs[i]->timeout);
    }
    else
    {
      NODELET_DEBUG_STREAM("CmdActuatorMux : no need to re-subscribe to input topic '"
                           << cmd_actuator_subs[i]->topic << "'");
    }

    if (!cmd_actuator_subs[i]->timer)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      cmd_actuator_subs[i]->timer = pnh.createTimer(ros::Duration(cmd_actuator_subs[i]->timeout),
                                                    TimerFunctor(i, this), true, false);
    }

    if (cmd_actuator_subs[i]->timeout > longest_timeout)
      longest_timeout = cmd_actuator_subs[i]->timeout;
  }

  if (!common_timer)
  {
    // Create another timer for  messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    common_timer_period = longest_timeout * 2.0;
    common_timer = pnh.createTimer(ros::Duration(common_timer_period),
                                   TimerFunctor(GLOBAL_TIMER, this), true, false);
  }
  else if (longest_timeout != (common_timer_period / 2.0))
  {
    // Longest timeout changed; just update existing timer period
    common_timer_period = longest_timeout * 2.0;
    common_timer.setPeriod(ros::Duration(common_timer_period));
  }

  NODELET_INFO_STREAM("CmdActuatorMux : (re)configured");
}

}  // namespace cmd_actuator_mux

PLUGINLIB_EXPORT_CLASS(cmd_actuator_mux::CmdActuatorMuxNodelet, nodelet::Nodelet);
