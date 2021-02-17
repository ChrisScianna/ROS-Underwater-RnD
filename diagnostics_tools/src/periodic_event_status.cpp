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

#include "diagnostic_tools/periodic_event_status.h"

#include <cinttypes>
#include <string>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace qna
{
namespace diagnostic_tools
{
PeriodicEventStatus::PeriodicEventStatus(const std::string& name)
  : PeriodicEventStatus(name, PeriodicEventStatusParams{}) {}

PeriodicEventStatus::PeriodicEventStatus(const std::string& name, PeriodicEventStatusParams params)
  : PeriodicDiagnosticTask(name),
    params_(std::move(params)),
    short_term_period_(params.short_term_avg_window()),
    long_term_period_(params.long_term_avg_window()),
    start_stamp_(ros::Time::now()) {}

void PeriodicEventStatus::tick(const ros::Time& stamp)
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!last_stamp_.isZero())
  {
    if (last_stamp_ <= stamp)
    {
      const double time_delta = (stamp - last_stamp_).toSec();
      if (time_delta <= params_.max_reasonable_period())
      {
        short_term_period_.update(time_delta);
        long_term_period_.update(time_delta);
      }
      else
      {
        ROS_DEBUG_NAMED(
          "diagnostics_tools",
          "Time delta %f too long, ignoring",
          time_delta);
      }
    }
    else
    {
      ROS_DEBUG_NAMED(
        "diagnostics_tools",
        "Time went backwards from %f to %f, ignoring",
        last_stamp_.toSec(), stamp.toSec());
    }
  }
  last_stamp_ = stamp;
}

void PeriodicEventStatus::run(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const ros::Time current_stamp = ros::Time::now();
  const double time_delta = (current_stamp - last_stamp_).toSec();
  if (short_term_period_.sample_count() > 0 && time_delta < params_.max_reasonable_period())
  {
    if (short_term_period_.average() < params_.min_acceptable_period() ||
        short_term_period_.average() > params_.max_acceptable_period())
    {
      stat.summary(params_.abnormal_diagnostic().status(),
                   params_.abnormal_diagnostic().description());
      if (params_.abnormal_diagnostic().has_code())
      {
        stat.addf("Code", "%" PRIu64, params_.abnormal_diagnostic().code());
      }
    }
    else
    {
      stat.summary(params_.normal_diagnostic().status(),
                   params_.normal_diagnostic().description());
      if (params_.normal_diagnostic().has_code())
      {
        stat.addf("Code", "%" PRIu64, params_.normal_diagnostic().code());
      }
    }
    stat.addf("Average period (short term)", "%f", short_term_period_.average());
    stat.addf("Minimum period (short term)", "%f", short_term_period_.minimum());
    stat.addf("Maximum period (short term)", "%f", short_term_period_.maximum());
  }
  else
  {
    const double time_since_init = (current_stamp - start_stamp_).toSec();

    if (time_since_init <= params_.max_reasonable_period())
    {
      // Likely still initializing, skip diagnostic checks
      return;
    }

    // Event is likely stale, reset estimate
    short_term_period_.reset();

    stat.summary(params_.stale_diagnostic().status(),
                 params_.stale_diagnostic().description());
    if (params_.stale_diagnostic().has_code())
    {
      stat.addf("Code", "%" PRIu64, params_.stale_diagnostic().code());
    }
  }
  if (long_term_period_.sample_count() > 0)
  {
    stat.addf("Average period (long term)", "%f", long_term_period_.average());
    stat.addf("Minimum period (long term)", "%f", long_term_period_.minimum());
    stat.addf("Maximum period (long term)", "%f", long_term_period_.maximum());
  }
  stat.addf("Minimum acceptable period", "%f", params_.min_acceptable_period());
  stat.addf("Maximum acceptable period", "%f", params_.max_acceptable_period());
}

}  // namespace diagnostic_tools
}  // namespace qna
