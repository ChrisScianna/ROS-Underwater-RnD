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

#include <boost/date_time/local_time/local_time.hpp>
#include <ros/file_log.h>

#include <cassert>
#include <sstream>
#include <string>

#include "mission_control/behaviors/log_to_bagfile.h"


namespace BT
{

template<>
std::string
toStr<rosbag::CompressionType>(rosbag::CompressionType compression)
{
  switch (compression)
  {
    case rosbag::compression::Uncompressed:
      return "none";
    case rosbag::compression::BZ2:
      return "bz2";
    case rosbag::compression::LZ4:
      return "lz4";
    default:
      throw RuntimeError(
        "unknown compression type: " + std::to_string(compression));
  }
}

template<>
rosbag::CompressionType
convertFromString<rosbag::CompressionType>(StringView str)
{
  if ("none" == str)
  {
    return rosbag::compression::Uncompressed;
  }
  if ("bz2" == str)
  {
    return rosbag::compression::BZ2;
  }
  if ("lz4" == str)
  {
    return rosbag::compression::LZ4;
  }
  throw RuntimeError(
    "unknown compression type: " + str.to_string());
}
}  // namespace BT

namespace mission_control
{

namespace {

BT::StringView
strip(const BT::StringView& str)
{
  auto start = str.begin();
  while (std::isspace(*start)) ++start;
  auto end = str.rbegin();
  while (std::isspace(*end)) ++end;
  return BT::StringView(
    start, std::distance(start, end.base()));
}

}  // namespace

LogToBagfileNode::Options
LogToBagfileNode::Options::populateFromPorts(BT::TreeNode * node)
{
  assert(node != nullptr);

  Options options;

  auto result = node->getInput<std::string>("prefix", options.prefix);
  if (!result)
  {
    ROS_WARN_STREAM(
      "Cannot fetch bag prefix for '" << node->name() << "': " << result.error());
    ROS_WARN_STREAM("Falling back to default bag prefix");
  }

  std::string topics;
  result = node->getInput<std::string>("topics", topics);
  if (result)
  {
    if (topics != "all")
    {
      for (const auto& topic : BT::splitString(topics, ','))
      {
        options.topics.push_back(strip(topic).to_string());
      }
    }
  }
  else
  {
    ROS_WARN_STREAM(
      "Cannot fetch topics to log for '" << node->name() << "': " << result.error());
    ROS_WARN_STREAM("Falling back to default topics to log");
  }

  result = node->getInput<rosbag::CompressionType>(
    "compression", options.writer_options.compression);
  if (!result)
  {
    ROS_WARN_STREAM(
      "Cannot fetch bag compression for '" << node->name() << "': " << result.error());
    ROS_WARN_STREAM("Falling back to default bag compression");
  }

  return options;
}

LogToBagfileNode::LogToBagfileNode(const std::string& name, Options options)
  : BT::DecoratorNode(name, {}),
    options_(options),
    read_parameter_from_ports_(false)
{
    setRegistrationID("LogToBagfile");
}

LogToBagfileNode::LogToBagfileNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::DecoratorNode(name, config),
    read_parameter_from_ports_(true)
{
}

namespace
{

std::string
generateBagfilePath(const std::string& prefix)
{
  std::stringstream path;
  auto prefix_view = nonstd::to_string_view(prefix);
  if (!prefix_view.starts_with("/"))
  {
    path << ros::file_log::getLogDirectory() << "/";
  }
  path << prefix;
  if (!prefix_view.ends_with(".bag"))
  {
    const boost::posix_time::ptime now =
      boost::posix_time::second_clock::local_time();
    auto * const facet =
      new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    path.imbue(std::locale(path.getloc(), facet));
    path << now << ".bag";
  }
  return path.str();
}

}  // namespace

bool
LogToBagfileNode::startLogging()
{
  try
  {
    bag_writer_.start(
      generateBagfilePath(options_.prefix),
      options_.writer_options);
    using MessageEventT =
      ros::MessageEvent<const topic_tools::ShapeShifter>;
    auto callback =
      [this](const std::string& topic, const MessageEventT& event)
      {
        if (!bag_writer_.write(topic, event))
        {
          ROS_WARN_STREAM("'" << name() << "' stopped logging due to errors");
          subscription_.shutdown();
        }
      };
    subscription_ =
      !options_.topics.empty()?
      internal::GenericSubscription::create(nh_, options_.topics, callback) :
      internal::GenericSubscription::create(nh_, callback);
    return true;
  }
  catch (const rosbag::BagIOException & e)
  {
    ROS_ERROR_STREAM("Cannot '" << name() << "': " << e.what());
    return false;
  }
}

bool
LogToBagfileNode::stopLogging()
{
  try
  {
    subscription_.shutdown();
    bag_writer_.stop();
    return true;
  }
  catch(const rosbag::BagIOException & e)
  {
    ROS_ERROR_STREAM("Failed to '" << name() << "': " << e.what());
    return false;
  }
}

void
LogToBagfileNode::halt()
{
  if (BT::NodeStatus::RUNNING == status())
  {
    (void)stopLogging();
  }
  DecoratorNode::halt();
}

BT::NodeStatus
LogToBagfileNode::tick()
{
  if (BT::NodeStatus::IDLE == status())
  {
    if (read_parameter_from_ports_)
    {
      options_ = Options::populateFromPorts(this);
    }

    if (!startLogging())
    {
      return BT::NodeStatus::FAILURE;
    }

    setStatus(BT::NodeStatus::RUNNING);
  }

  if (BT::NodeStatus::RUNNING == status())
  {
    BT::NodeStatus status = child()->executeTick();
    if (BT::NodeStatus::RUNNING != status)
    {
      if (!stopLogging())
      {
        status = BT::NodeStatus::FAILURE;
      }
      setStatus(status);
    }
  }

  return status();
}

}   // namespace mission_control
