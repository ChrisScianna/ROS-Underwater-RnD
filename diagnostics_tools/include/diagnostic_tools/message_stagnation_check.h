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

/*
 * diagnostic_tools/message_stagnation_check.h
 */

#ifndef DIAGNOSTIC_TOOLS_MESSAGE_STAGNATION_CHECK_H
#define DIAGNOSTIC_TOOLS_MESSAGE_STAGNATION_CHECK_H

#include <cinttypes>
#include <deque>
#include <mutex>  // NOLINT(build/c++11)
#include <string>
#include <utility>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/diagnostic.h"
#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna
{
namespace diagnostic_tools
{

class MessageStagnationCheckParams final
{
public:
  MessageStagnationCheckParams() = default;

  MessageStagnationCheckParams &window_size(size_t size)
  {
    window_size_ = size;
    return *this;
  }

  size_t window_size() const
  {
    return window_size_;
  }

  MessageStagnationCheckParams &stagnation_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Messages have stagnated");
    }
    stagnation_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stagnation_diagnostic() const
  {
    return stagnation_diagnostic_;
  }

  MessageStagnationCheckParams &stale_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Not enough messages since last update");
    }
    stale_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stale_diagnostic() const
  {
    return stale_diagnostic_;
  }

  MessageStagnationCheckParams &normal_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Messages fluctuate normally");
    }
    normal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &normal_diagnostic() const
  {
    return normal_diagnostic_;
  }

private:
  size_t window_size_{3};
  Diagnostic normal_diagnostic_{Diagnostic::OK, "Messages fluctuate normally"};
  Diagnostic stagnation_diagnostic_{Diagnostic::WARN, "Messages have stagnated"};
  Diagnostic stale_diagnostic_{Diagnostic::STALE, "Not enough messages since last update"};
};

template <typename MessageT>
class MessageStagnationCheck : public TopicDiagnosticTask<MessageT>
{
public:
  using MessageEqualOp = boost::function<bool(const MessageT &, const MessageT &)>;

  MessageStagnationCheck(const std::string &name, const MessageEqualOp &equal_op)
    : MessageStagnationCheck(name, equal_op, MessageStagnationCheckParams{}) {}

  MessageStagnationCheck(const std::string &name, const MessageEqualOp &equal_op,
                         MessageStagnationCheckParams params)
    : TopicDiagnosticTask<MessageT>(name), equal_op_(equal_op), params_(std::move(params)) {}

  void tick(const ros::Time &stamp, const MessageT &message) override
  {
    tick(stamp, boost::make_shared<MessageT const>(message));
  }

  void tick(const ros::Time &stamp, const boost::shared_ptr<MessageT const> &message) override
  {
    if (!message)
    {
      ROS_WARN_NAMED("diagnostic_tools", "No stagnation checks possible on null message");
      return;
    }

    std::lock_guard<std::mutex> guard(mutex_);
    message_window_.push_back(message);
    if (message_window_.size() > params_.window_size())
    {
      message_window_.pop_front();
    }
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const Diagnostic &diagnostic = diagnose();
    stat.summary(diagnostic.status(), diagnostic.description());
    if (diagnostic.has_code())
    {
      stat.addf("Code", "%" PRIu64, diagnostic.code());
    }
  }

private:
  const Diagnostic &diagnose()
  {
    if (message_window_.size() < params_.window_size())
    {
      return params_.stale_diagnostic();
    }
    auto predicate = [this](const boost::shared_ptr<MessageT const> &a,
                            const boost::shared_ptr<MessageT const> &b)
    {
      return equal_op_(*a, *b);
    };
    if (std::adjacent_find(message_window_.begin(),
                           message_window_.end(),
                           predicate) != message_window_.end())
    {
      return params_.stagnation_diagnostic();
    }
    return params_.normal_diagnostic();
  }

  MessageEqualOp equal_op_;
  MessageStagnationCheckParams params_;
  std::deque<boost::shared_ptr<MessageT const>> message_window_;
  std::mutex mutex_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_MESSAGE_STAGNATION_CHECK_H
