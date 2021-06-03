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

#ifndef CMD_ACTUATORS_MUX_CMD_ACTUATOR_SUBSCRIBERS_H
#define CMD_ACTUATORS_MUX_CMD_ACTUATOR_SUBSCRIBERS_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

namespace cmd_actuator_mux
{
class CmdActuatorSubscribers
{
 public:
  class CmdActuatorSubs
  {
   public:
    unsigned int idx;       /**< Index; assigned according to the order on YAML file */
    std::string name;       /**< Descriptive name; must be unique to this subscriber */
    std::string topic;      /**< The name of the topic */
    ros::Subscriber subs;   /**< The subscriber itself */
    ros::Timer timer;       /**< No incoming messages timeout */
    double timeout;         /**< Timer's timeout, in seconds  */
    unsigned int priority;  /**< UNIQUE integer from 0 (lowest priority) to MAX_INT */
    std::string short_desc; /**< Short description (optional) */
    bool active;            /**< Whether this source is active */

    explicit CmdActuatorSubs(unsigned int idx) : idx(idx), active(false) {}
    ~CmdActuatorSubs() {}

    /* Fill attributes with a YAML node content */
    void operator<<(const YAML::Node& node);
  };

  CmdActuatorSubscribers() {}
  ~CmdActuatorSubscribers() {}

  std::vector<std::shared_ptr<CmdActuatorSubs>>::size_type size() { return list.size(); }
  std::shared_ptr<CmdActuatorSubs>& operator[](unsigned int idx) { return list[idx]; }

  /**
   * @brief Configures the subscribers from a yaml file.
   *
   * @exception FileNotFoundException : yaml file not found
   * @exception YamlException : problem parsing the yaml
   * @exception EmptyCfgException : empty configuration file
   * @param node : node holding all the subscriber configuration
   */
  void configure(const YAML::Node& node);

  unsigned int allowed;

 private:
  std::vector<std::shared_ptr<CmdActuatorSubs>> list;
};

}  // namespace cmd_actuator_mux

#endif  //  CMD_ACTUATORS_MUX_CMD_ACTUATOR_SUBSCRIBERS_H
