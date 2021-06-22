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
#include <string>

#include "cmd_actuators_mux/exceptions.h"

namespace cmd_actuator_mux
{
/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void CmdActuatorMuxNodelet::cmdFinAngleCallback(const fin_control::SetAngles::ConstPtr& msg,
                                                unsigned int idx)
{
  // Reset general timer
  fin_angles_common_timer.stop();
  fin_angles_common_timer.start();

  if (cmdActuatorCallbackProcess(cmd_fin_angles_subs, idx))
  {
    // The fin angle input has changed from an input to another
    fin_angles_acv_msg.data = cmd_fin_angles_subs[idx]->name;

    // Wait until both RPM and Fin commands come from the same source
    if (set_rpm_acv_msg.data == fin_angles_acv_msg.data)
    {
      if (publish_new_subscriber)
      {
        active_subscriber_pub.publish(fin_angles_acv_msg);
        publish_new_subscriber = false;
      }
      fin_angles_output_topic_pub.publish(msg);
    }
    else
      ROS_DEBUG_STREAM("RPM and fin angles values come from different source"
                       << " - set RPM source: " << set_rpm_acv_msg.data
                       << " - set Fin Angles source: " << fin_angles_acv_msg.data);
  }
}

void CmdActuatorMuxNodelet::cmdSetRPMCallback(const thruster_control::SetRPM::ConstPtr& msg,
                                              unsigned int idx)
{
  set_rpm_common_timer.stop();
  set_rpm_common_timer.start();

  if (cmdActuatorCallbackProcess(cmd_set_rpm_subs, idx))
  {
    set_rpm_acv_msg.data = cmd_set_rpm_subs[idx]->name;

    // waint until both, RPM and Fin commands come from the same source
    if (set_rpm_acv_msg.data == fin_angles_acv_msg.data)
    {
      if (publish_new_subscriber)
      {
        active_subscriber_pub.publish(set_rpm_acv_msg);
        publish_new_subscriber = false;
      }
      set_rpm_output_topic_pub.publish(msg);
    }
    else
      ROS_DEBUG_STREAM("RPM and fin angles values come from different source"
                       << " - set RPM source: " << cmd_set_rpm_subs[idx]->name
                       << " - set Fin Angles source: " << cmd_fin_angles_subs[idx]->name);
  }
}

bool CmdActuatorMuxNodelet::cmdActuatorCallbackProcess(CmdActuatorSubscribers& cmd_actuator_subs,
                                                       const unsigned int& idx)
{
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
      publish_new_subscriber = true;
    }
    return true;
  }
  return false;
}

void CmdActuatorMuxNodelet::finAnglesTimerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  timerCallbackProcess(cmd_fin_angles_subs, idx);
}

void CmdActuatorMuxNodelet::setRPMTimerCallback(const ros::TimerEvent& event, unsigned int idx)
{
  timerCallbackProcess(cmd_set_rpm_subs, idx);
}

void CmdActuatorMuxNodelet::timerCallbackProcess(CmdActuatorSubscribers& cmd_actuator_subs,
                                                 const unsigned int& idx)
{
  if (cmd_actuator_subs.allowed == idx ||
      (idx == GLOBAL_TIMER && cmd_actuator_subs.allowed != VACANT))
  {
    if (idx == GLOBAL_TIMER)
    {
      // No messages timeout happened for ANYONE, so last active source got stuck without further
      // messages; not a big problem, just dislodge it; but possibly reflect a problem in the
      // controller
      NODELET_WARN("Actuator : No messages from ANY input received");
      NODELET_WARN("Actuator: %s dislodged due to general timeout",
                   cmd_actuator_subs[cmd_actuator_subs.allowed]->name.c_str());
    }

    // No messages timeout happened to currently active source, so...
    cmd_actuator_subs.allowed = VACANT;

    // ...notify the world that nobody is publishing; its vacant
    std_msgs::StringPtr acv_msg(new std_msgs::String);
    acv_msg->data = "idle";
    active_subscriber_pub.publish(acv_msg);
  }

  if (idx != GLOBAL_TIMER) cmd_actuator_subs[idx]->active = false;
}

void CmdActuatorMuxNodelet::onInit()
{
  nh = this->getNodeHandle();
  pnh = this->getPrivateNodeHandle();

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure_cb = boost::bind(&CmdActuatorMuxNodelet::reloadConfiguration, this, _1, _2);
  dynamic_reconfigure_server =
      new dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>(pnh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_cb);

  active_subscriber_pub = pnh.advertise<std_msgs::String>("active_subscriber", 1, true);

  // Notify the world that by now nobody is publishing on yet
  std_msgs::StringPtr active_msg(new std_msgs::String);
  active_msg->data = "idle";
  active_subscriber_pub.publish(active_msg);

  // could use a call to reloadConfiguration here, but it seems to automatically call it once with
  // defaults anyway.
  NODELET_DEBUG("Command Actuator Mux : successfully initialized");
}

void CmdActuatorMuxNodelet::reloadConfiguration(cmd_actuators_mux::reloadConfig& config,
                                                uint32_t unused_level)
{
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
      NODELET_ERROR_STREAM("Fin Angle Mux : configuration file not found [" << yaml_cfg_file
                                                                            << "]");
      return;
    }
  }

  /*********************
  ** Yaml File Parsing
  **********************/

#ifdef HAVE_NEW_YAMLCPP
  doc = YAML::Load(*is);
#else
  YAML::Parser parser(*is);
  parser.GetNextDocument(doc);
#endif

  /*********************
  ** Fin Angles
  **********************/

  /******** Publisher  *******************************/
  std::string output_name = getOutputTopicName("fin_angles_publisher");
  if (fin_angles_output_topic_name != output_name)
  {
    fin_angles_output_topic_name = output_name;
    fin_angles_output_topic_pub =
        nh.advertise<fin_control::SetAngles>(fin_angles_output_topic_name, 10);
    NODELET_DEBUG_STREAM("Subscribe to output topic '" << output_name << "'");
  }
  else
  {
    NODELET_DEBUG_STREAM("No need to re-subscribe to output topic '" << output_name << "'");
  }

  /******** Input Subscribers *******************************/

  try
  {
    cmd_fin_angles_subs.configure(doc["fin_angles_subscribers"]);
  }
  catch (EmptyCfgException& e)
  {
    NODELET_WARN_STREAM(
        "yaml configured zero subscribers, check yaml content: fin_angles_subscribers");
  }
  catch (YamlException& e)
  {
    NODELET_ERROR_STREAM("fin_angles_subscribers: yaml parsing problem [" << std::string(e.what())
                                                                          << "]");
  }

  // (Re)create subscribers whose topic is invalid: new ones and those with changed names
  double longest_timeout = 0.0;
  for (unsigned int i = 0; i < cmd_fin_angles_subs.size(); i++)
  {
    if (!cmd_fin_angles_subs[i]->subs)
    {
      cmd_fin_angles_subs[i]->subs = nh.subscribe<fin_control::SetAngles>(
          cmd_fin_angles_subs[i]->topic, 10, CmdFinAngleFunctor(i, this));
      NODELET_DEBUG("Fin Angle Mux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                    cmd_fin_angles_subs[i]->name.c_str(), cmd_fin_angles_subs[i]->topic.c_str(),
                    cmd_fin_angles_subs[i]->priority, cmd_fin_angles_subs[i]->timeout);
    }
    else
    {
      NODELET_DEBUG_STREAM("Fin Angle Mux : no need to re-subscribe to input topic '"
                           << cmd_fin_angles_subs[i]->topic << "'");
    }

    if (!cmd_fin_angles_subs[i]->timer)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      cmd_fin_angles_subs[i]->timer =
          pnh.createTimer(ros::Duration(cmd_fin_angles_subs[i]->timeout),
                          FinAngleTimerFunctor(i, this), true, false);
    }

    if (cmd_fin_angles_subs[i]->timeout > longest_timeout)
      longest_timeout = cmd_fin_angles_subs[i]->timeout;
  }

  if (!fin_angles_common_timer)
  {
    // Create another timer for  messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    fin_angles_common_timer_period = longest_timeout * 2.0;
    fin_angles_common_timer =
        pnh.createTimer(ros::Duration(fin_angles_common_timer_period),
                        FinAngleTimerFunctor(GLOBAL_TIMER, this), true, false);
  }
  else if (longest_timeout != (fin_angles_common_timer_period / 2.0))
  {
    // Longest timeout changed; just update existing timer period
    fin_angles_common_timer_period = longest_timeout * 2.0;
    fin_angles_common_timer.setPeriod(ros::Duration(fin_angles_common_timer_period));
  }

  NODELET_INFO_STREAM("Fin Angle Mux : (re)configured");

  /*********************
  ** Thruster / Set RPM
  **********************/

  /******** Publisher  *******************************/
  output_name = getOutputTopicName("set_rpm_publisher");
  if (set_rpm_output_topic_name != output_name)
  {
    set_rpm_output_topic_name = output_name;
    set_rpm_output_topic_pub =
        nh.advertise<thruster_control::SetRPM>(set_rpm_output_topic_name, 10);
    NODELET_DEBUG_STREAM("Subscribe to output topic '" << output_name << "'");
  }
  else
  {
    NODELET_DEBUG_STREAM("No need to re-subscribe to output topic '" << output_name << "'");
  }

  /******** Input Subscribers *******************************/
  try
  {
    cmd_set_rpm_subs.configure(doc["set_rpm_subscribers"]);
  }
  catch (EmptyCfgException& e)
  {
    NODELET_WARN_STREAM(
        "yaml configured zero subscribers, check yaml content: set_rpm_subscribers");
  }
  catch (YamlException& e)
  {
    NODELET_ERROR_STREAM("set_rpm_subscribers: yaml parsing problem [" << std::string(e.what())
                                                                       << "]");
  }

  double set_rpm_longest_timeout = 0.0;
  for (unsigned int i = 0; i < cmd_set_rpm_subs.size(); i++)
  {
    if (!cmd_set_rpm_subs[i]->subs)
    {
      cmd_set_rpm_subs[i]->subs = nh.subscribe<thruster_control::SetRPM>(
          cmd_set_rpm_subs[i]->topic, 10, CmdSetRPMFunctor(i, this));
      NODELET_DEBUG("Thruster Set RPM Mux : subscribed to '%s' on topic '%s'. pr: %d, to: %.2f",
                    cmd_set_rpm_subs[i]->name.c_str(), cmd_set_rpm_subs[i]->topic.c_str(),
                    cmd_set_rpm_subs[i]->priority, cmd_set_rpm_subs[i]->timeout);
    }
    else
    {
      NODELET_DEBUG_STREAM("Thruster Set RPM Mux : no need to re-subscribe to input topic '"
                           << cmd_set_rpm_subs[i]->topic << "'");
    }

    if (!cmd_set_rpm_subs[i]->timer)
    {
      // Create (stopped by now) a one-shot timer for every subscriber, if it doesn't exist yet
      cmd_set_rpm_subs[i]->timer = pnh.createTimer(ros::Duration(cmd_set_rpm_subs[i]->timeout),
                                                   SetRPMTimerFunctor(i, this), true, false);
    }

    if (cmd_set_rpm_subs[i]->timeout > set_rpm_longest_timeout)
      set_rpm_longest_timeout = cmd_set_rpm_subs[i]->timeout;
  }

  if (!set_rpm_common_timer)
  {
    // Create another timer for  messages from any source, so we can
    // dislodge last active source if it gets stuck without further messages
    set_rpm_common_timer_period = longest_timeout * 2.0;
    set_rpm_common_timer = pnh.createTimer(ros::Duration(set_rpm_common_timer_period),
                                           SetRPMTimerFunctor(GLOBAL_TIMER, this), true, false);
  }
  else if (longest_timeout != (set_rpm_common_timer_period / 2.0))
  {
    // Longest timeout changed; just update existing timer period
    set_rpm_common_timer_period = longest_timeout * 2.0;
    set_rpm_common_timer.setPeriod(ros::Duration(set_rpm_common_timer_period));
  }

  // check that the both arrays have the same length 
  ROS_ASSERT_MSG(cmd_set_rpm_subs.size()==(cmd_fin_angles_subs.size()),
                   "Different amount of inputs between SetRPM Subscriber (%d) and "
                   "Fin_Angles Subscribers (%d) in YAML file:",
                   static_cast<int>(cmd_set_rpm_subs.size()),
                   static_cast<int>(cmd_fin_angles_subs.size()));

  // check that the subscribers input have the same name in YAML
  for (int i = 0; i < cmd_set_rpm_subs.size(); i++)
  {
    ROS_ASSERT_MSG(!cmd_set_rpm_subs[i]->name.compare(cmd_fin_angles_subs[i]->name),
                   "Subscribers from same input must have equal names in YAML file:"
                   "\nID: %d - Subscriber 1 name: %s - Subscriber 2 name: %s",
                   i, cmd_set_rpm_subs[i]->name.c_str(), cmd_fin_angles_subs[i]->name.c_str());
  }
}

std::string CmdActuatorMuxNodelet::getOutputTopicName(const std::string& label)
{
  std::string output_name("output");
#ifdef HAVE_NEW_YAMLCPP
  if (doc[label])
  {
    doc[label] >> output_name;
  }
#else
  const YAML::Node* node = doc.FindValue(label);
  if (node != NULL)
  {
    *node >> output_name;
  }
#endif

  return output_name;
}

}  // namespace cmd_actuator_mux

PLUGINLIB_EXPORT_CLASS(cmd_actuator_mux::CmdActuatorMuxNodelet, nodelet::Nodelet);
