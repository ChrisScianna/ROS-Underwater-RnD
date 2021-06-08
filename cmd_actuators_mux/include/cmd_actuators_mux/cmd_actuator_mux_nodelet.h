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

#ifndef CMD_ACTUATORS_MUX_CMD_ACTUATOR_MUX_NODELET_H
#define CMD_ACTUATORS_MUX_CMD_ACTUATOR_MUX_NODELET_H

#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <string>

#include "cmd_actuators_mux/cmd_actuator_subscribers.h"
#include "cmd_actuators_mux/reloadConfig.h"
#include "fin_control/SetAngles.h"
#include "thruster_control/SetRPM.h"

namespace cmd_actuator_mux
{
class CmdActuatorMuxNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();

  CmdActuatorMuxNodelet()
  {
    cmd_fin_angles_subs.allowed = VACANT;
    cmd_set_rpm_subs.allowed = VACANT;

    dynamic_reconfigure_server = NULL;
  }

  ~CmdActuatorMuxNodelet()
  {
    if (dynamic_reconfigure_server != NULL) delete dynamic_reconfigure_server;
  }

 private:
  static const unsigned int VACANT =
      666666; /**< ID for "nobody" active input; anything big is ok */
  static const unsigned int GLOBAL_TIMER =
      888888; /**< ID for the global timer functor; anything big is ok */

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  /**< Pool of topics subscribers */
  CmdActuatorSubscribers cmd_fin_angles_subs;
  CmdActuatorSubscribers cmd_set_rpm_subs;

  /**< Multiplexed command Fin Angle topic */
  ros::Publisher fin_angles_output_topic_pub;
  ros::Publisher set_rpm_output_topic_pub;

  /**< Multiplexed command Fin Angle topic name */
  std::string fin_angles_output_topic_name;
  std::string set_rpm_output_topic_name;

  /**< Currently allowed subscriber */
  ros::Publisher fin_angles_active_subscriber;
  ros::Publisher set_rpm_active_subscriber;

  /**< No messages from any subscriber timeout */
  ros::Timer fin_angles_common_timer;
  ros::Timer set_rpm_common_timer;

  /**< No messages from any subscriber timeout period */
  double fin_angles_common_timer_period;
  double set_rpm_common_timer_period;

  YAML::Node doc;

  void finAnglesTimerCallback(const ros::TimerEvent& event, unsigned int idx);
  void setRPMtimerCallback(const ros::TimerEvent& event, unsigned int idx);
  void timerCallbackProcess(CmdActuatorSubscribers& cmd_actuator_subs, const unsigned int& idx,
                            const ros::Publisher& active_subscriber);

  void cmdFinAngleCallback(const fin_control::SetAngles::ConstPtr& msg, unsigned int idx);
  void cmdSetRPMCallback(const thruster_control::SetRPM::ConstPtr& msg, unsigned int idx);
  bool cmdActuatorCallbackProcess(CmdActuatorSubscribers& cmd_actuator_subs,
                                  const unsigned int& idx, ros::Publisher& active_subscriber);

  std::string getOutputTopicName(const std::string& label);
  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>* dynamic_reconfigure_server;
  dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(cmd_actuators_mux::reloadConfig& config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming Fin Angle topic to bind it to callback
  class CmdFinAngleFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    CmdFinAngleFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const fin_control::SetAngles::ConstPtr& msg)
    {
      node->cmdFinAngleCallback(msg, idx);
    }
  };

  // Functor assigned to each Fin Angle messages source to bind it to timer callback
  class FinAngleTimerFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    FinAngleTimerFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const ros::TimerEvent& event) { node->finAnglesTimerCallback(event, idx); }
  };

  // Functor assigned to each incoming Set RPM topic to bind it to callback
  class CmdSetRPMFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    CmdSetRPMFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const thruster_control::SetRPM::ConstPtr& msg)
    {
      node->cmdSetRPMCallback(msg, idx);
    }
  };

  // Functor assigned to each Set RPM Angle messages source to bind it to timer callback
  class SetRPMTimerFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    SetRPMTimerFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const ros::TimerEvent& event) { node->setRPMtimerCallback(event, idx); }
  };
};

}  // namespace cmd_actuator_mux

#endif  //  CMD_ACTUATORS_MUX_CMD_ACTUATOR_MUX_NODELET_H
