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
 * diagnostic_tools/topic_diagnostic_task.h
 */

#ifndef DIAGNOSTIC_TOOLS_TOPIC_DIAGNOSTIC_TASK_H
#define DIAGNOSTIC_TOOLS_TOPIC_DIAGNOSTIC_TASK_H

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/periodic_diagnostic_task.h"

namespace qna
{
namespace diagnostic_tools
{

template <typename MessageT>
class TopicDiagnosticTask : public PeriodicDiagnosticTask
{
public:
  using PeriodicDiagnosticTask::PeriodicDiagnosticTask;

  void tick(const MessageT& message)
  {
    tick(ros::Time::now(), message);
  }

  void tick(const boost::shared_ptr<MessageT const>& message)
  {
    tick(ros::Time::now(), message);
  }

  virtual void tick(const ros::Time& stamp, const MessageT& message) = 0;

  virtual void tick(const ros::Time& stamp, const boost::shared_ptr<MessageT const>& message) = 0;

  void tick(const ros::Time& stamp) override
  {
    this->tick(stamp, nullptr);
  }
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_TOPIC_DIAGNOSTIC_TASK_H
