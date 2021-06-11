/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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
#ifndef MISSION_CONTROL_BEHAVIORS_LOG_TO_BAGFILE_H
#define MISSION_CONTROL_BEHAVIORS_LOG_TO_BAGFILE_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/decorator_node.h>

#include <ros/ros.h>

#include <string>
#include <vector>

#include "mission_control/behaviors/internal/async_bag_writer.h"
#include "mission_control/behaviors/internal/generic_subscription.h"


namespace mission_control
{

class LogToBagfileNode : public BT::DecoratorNode
{
 public:
  struct Options
  {
    static BT::PortsList portsList()
    {
      static const Options options;
      return {
        BT::InputPort<std::string>(
          "prefix", options.prefix, "Prefix for bag filename"),
        BT::InputPort<std::string>(
          "topics", "all", "Topics to log, 'all' by default"),
        BT::InputPort<rosbag::CompressionType>(
          "compression", options.writer_options.compression,
          "Compression type for bag")};
    }

    static Options populateFromPorts(BT::TreeNode * node);

    std::string prefix{"mission_bag_"};
    std::vector<std::string> topics{};
    internal::AsyncBagWriter::Options writer_options{};
  };

  static BT::PortsList providedPorts() { return Options::portsList(); }

  LogToBagfileNode(const std::string& name, Options options);

  LogToBagfileNode(const std::string& name, const BT::NodeConfiguration& config);

  void halt() override;

 private:
  BT::NodeStatus tick() override;

  bool startLogging();
  bool stopLogging();

  ros::NodeHandle nh_;
  Options options_;
  bool read_parameter_from_ports_;

  internal::AsyncBagWriter bag_writer_;
  internal::GenericSubscription subscription_;
};

}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_LOG_TO_BAGFILE_H
