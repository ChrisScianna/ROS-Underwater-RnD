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

namespace cmd_actuator_mux
{
class CmdActuatorMuxNodelet : public nodelet::Nodelet
{
 public:
  virtual void onInit();

  CmdActuatorMuxNodelet()
  {
    cmd_actuator_subs.allowed = VACANT;
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

  CmdActuatorSubscribers cmd_actuator_subs; /**< Pool of topics subscribers */
  ros::Publisher output_topic_pub;          /**< Multiplexed command velocity topic */
  std::string output_topic_name;            /**< Multiplexed command velocity topic name */
  ros::Publisher active_subscriber;         /**< Currently allowed subscriber */
  ros::Timer common_timer;                  /**< No messages from any subscriber timeout */
  double common_timer_period;               /**< No messages from any subscriber timeout period */

  void timerCallback(const ros::TimerEvent& event, unsigned int idx);
  void cmdActuatorCallback(const fin_control::SetAngles::ConstPtr& msg, unsigned int idx);

  /*********************
  ** Dynamic Reconfigure
  **********************/
  dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>* dynamic_reconfigure_server;
  dynamic_reconfigure::Server<cmd_actuators_mux::reloadConfig>::CallbackType dynamic_reconfigure_cb;
  void reloadConfiguration(cmd_actuators_mux::reloadConfig& config, uint32_t unused_level);

  /*********************
   ** Private Classes
   **********************/
  // Functor assigned to each incoming velocity topic to bind it to callback
  class CmdActuatorFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    CmdActuatorFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const fin_control::SetAngles::ConstPtr& msg)
    {
      node->cmdActuatorCallback(msg, idx);
    }
  };

  // Functor assigned to each velocity messages source to bind it to timer callback
  class TimerFunctor
  {
   private:
    unsigned int idx;
    CmdActuatorMuxNodelet* node;

   public:
    TimerFunctor(unsigned int idx, CmdActuatorMuxNodelet* node) : idx(idx), node(node) {}

    void operator()(const ros::TimerEvent& event) { node->timerCallback(event, idx); }
  };
};

}  // namespace cmd_actuator_mux

#endif  //  CMD_ACTUATORS_MUX_CMD_ACTUATOR_MUX_NODELET_H
