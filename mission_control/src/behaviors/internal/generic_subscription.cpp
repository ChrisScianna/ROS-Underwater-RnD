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

#include "mission_control/behaviors/internal/generic_subscription.h"

#include <string>
#include <unordered_map>
#include <vector>


namespace mission_control
{
namespace internal
{

class GenericSubscriptionImpl
{
 public:
  explicit GenericSubscriptionImpl(const ros::NodeHandle& nh)
    : nh_(nh)
  {
  }

  virtual ~GenericSubscriptionImpl() = default;

  virtual void shutdown()
  {
    for (auto& kv : active_subscribers_)
    {
      kv.second.shutdown();
    }
    active_subscribers_.clear();
  }

 protected:
  bool isSubscribed(const std::string& topic) const
  {
    return active_subscribers_.count(topic) > 0;
  }

  void subscribe(
    const std::string& topic,
    const GenericSubscription::CallbackT& callback)
  {
    ros::SubscribeOptions ops;
    ops.topic = topic;
    ops.queue_size = 100;
    ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
    ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
    ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
      const ros::MessageEvent<const topic_tools::ShapeShifter> &> >(
        boost::bind(callback, topic, _1));
    active_subscribers_[topic] = nh_.subscribe(ops);
  }

 private:
  ros::NodeHandle nh_;
  std::unordered_map<std::string, ros::Subscriber> active_subscribers_;
};

class NonTrackingImpl : public GenericSubscriptionImpl
{
 public:
  NonTrackingImpl(
    const ros::NodeHandle& nh,
    const std::vector<std::string>& topics,
    const GenericSubscription::CallbackT& callback)
  : GenericSubscriptionImpl(nh)
  {
    for (const auto& topic : topics)
    {
      subscribe(topic, callback);
    }
  }
};

class TrackingImpl : public GenericSubscriptionImpl
{
 public:
  TrackingImpl(
    const ros::NodeHandle& nh,
    const GenericSubscription::CallbackT& callback)
  : GenericSubscriptionImpl(nh), callback_(callback)
  {
    timer_ = nh.createTimer(
      ros::Duration(1.0),
      boost::bind(&TrackingImpl::trackTopics, this));
  }

  void shutdown() override
  {
    timer_.stop();
    GenericSubscriptionImpl::shutdown();
  }

 private:
  void trackTopics()
  {
    ros::master::V_TopicInfo topics;
    if (ros::master::getTopics(topics))
    {
      for (const ros::master::TopicInfo& topic : topics)
      {
        if (!isSubscribed(topic.name))
        {
          subscribe(topic.name, callback_);
        }
      }
    }
  }

  ros::Timer timer_;
  GenericSubscription::CallbackT callback_;
};

GenericSubscription
GenericSubscription::create(
  const ros::NodeHandle& nh,
  const CallbackT& callback)
{
  std::unique_ptr<GenericSubscriptionImpl> impl{
    new TrackingImpl(nh, callback)};
  return GenericSubscription{std::move(impl)};
}

GenericSubscription
GenericSubscription::create(
  const ros::NodeHandle& nh,
  const std::vector<std::string>& topics,
  const CallbackT& callback)
{
  std::unique_ptr<GenericSubscriptionImpl> impl{
    new NonTrackingImpl(nh, topics, callback)};
  return GenericSubscription{std::move(impl)};
}

GenericSubscription::GenericSubscription(
  std::unique_ptr<GenericSubscriptionImpl> impl)
  : impl_(std::move(impl))
{
}

GenericSubscription::GenericSubscription() = default;

GenericSubscription::GenericSubscription(GenericSubscription&&) = default;

GenericSubscription&
GenericSubscription::operator=(GenericSubscription&&) = default;

GenericSubscription::~GenericSubscription() = default;

void
GenericSubscription::shutdown()
{
  impl_->shutdown();
}

}  // namespace internal
}  // namespace mission_control
