/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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
 * diagnostic_tools/periodic_message_status.h
 */

#ifndef DIAGNOSTIC_TOOLS_PERIODIC_MESSAGE_STATUS_H
#define DIAGNOSTIC_TOOLS_PERIODIC_MESSAGE_STATUS_H

#include <diagnostic_updater/diagnostic_updater.h>

#include <string>

#include <ros/ros.h>

#include "diagnostic_tools/periodic_event_status.h"
#include "diagnostic_tools/topic_diagnostic_task.h"

namespace qna
{
namespace diagnostic_tools
{

using PeriodicMessageStatusParams = PeriodicEventStatusParams;

template <typename MessageT>
class PeriodicMessageStatus : public TopicDiagnosticTask<MessageT>
{
public:
  explicit PeriodicMessageStatus(const std::string &name)
    : PeriodicMessageStatus(name, PeriodicMessageStatusParams{}) {}

  PeriodicMessageStatus(const std::string &name, PeriodicMessageStatusParams params)
    : TopicDiagnosticTask<MessageT>(name),
      impl_(name, std::move(params)) {}

  void tick(const ros::Time &stamp, const MessageT &) override
  {
    this->tick(stamp);
  }

  void tick(const ros::Time &stamp, const boost::shared_ptr<MessageT const> &) override
  {
    this->tick(stamp);
  }

  void tick(const ros::Time &stamp) override
  {
    impl_.tick(stamp);
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
  {
    impl_.run(stat);
  }

private:
  PeriodicEventStatus impl_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_PERIODIC_MESSAGE_STATUS_H
