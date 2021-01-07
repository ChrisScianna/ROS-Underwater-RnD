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
 * diagnostic_tools/periodic_event_status.h
 */

#ifndef DIAGNOSTIC_TOOLS_PERIODIC_EVENT_STATUS_H
#define DIAGNOSTIC_TOOLS_PERIODIC_EVENT_STATUS_H

#include <mutex>  // NOLINT(build/c++11)
#include <string>

#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include "diagnostic_tools/diagnostic.h"
#include "diagnostic_tools/periodic_diagnostic_task.h"
#include "diagnostic_tools/sampled_statistics.h"

namespace qna
{
namespace diagnostic_tools
{

class PeriodicEventStatusParams final
{
public:
  PeriodicEventStatusParams() = default;

  PeriodicEventStatusParams &min_acceptable_period(double min_acceptable_period)
  {
    min_acceptable_period_ = min_acceptable_period;
    return *this;
  }

  double min_acceptable_period() const
  {
    return min_acceptable_period_;
  }

  PeriodicEventStatusParams &max_acceptable_period(double max_acceptable_period)
  {
    max_acceptable_period_ = max_acceptable_period;
    return *this;
  }

  double max_acceptable_period() const
  {
    return max_acceptable_period_;
  }

  PeriodicEventStatusParams &max_reasonable_period(double max_reasonable_period)
  {
    max_reasonable_period_ = max_reasonable_period;
    has_max_reasonable_period_ = true;
    return *this;
  }

  double max_reasonable_period() const
  {
    if (!has_max_reasonable_period_)
    {
      return 5 * max_acceptable_period_;
    }
    return max_reasonable_period_;
  }

  PeriodicEventStatusParams &normal_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Rate within tolerance");
    }
    normal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &normal_diagnostic() const
  {
    return normal_diagnostic_;
  }

  PeriodicEventStatusParams &abnormal_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Rate too high or too low");
    }
    abnormal_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &abnormal_diagnostic() const
  {
    return abnormal_diagnostic_;
  }

  PeriodicEventStatusParams &stale_diagnostic(Diagnostic diagnostic)
  {
    if (diagnostic.description().empty())
    {
      diagnostic.description("Not enough data since last update");
    }
    stale_diagnostic_ = std::move(diagnostic);
    return *this;
  }

  const Diagnostic &stale_diagnostic() const
  {
    return stale_diagnostic_;
  }

  PeriodicEventStatusParams &short_term_avg_window(size_t short_term_avg_window)
  {
    short_term_avg_window_ = short_term_avg_window;
    return *this;
  }

  size_t short_term_avg_window() const
  {
    return short_term_avg_window_;
  }

  PeriodicEventStatusParams &long_term_avg_window(size_t long_term_avg_window)
  {
    long_term_avg_window_ = long_term_avg_window;
    return *this;
  }

  size_t long_term_avg_window() const
  {
    return long_term_avg_window_;
  }

private:
  double min_acceptable_period_{ -1};
  double max_acceptable_period_{5};
  bool has_max_reasonable_period_{false};
  double max_reasonable_period_{};
  size_t short_term_avg_window_{1};
  size_t long_term_avg_window_{10000};
  Diagnostic normal_diagnostic_{Diagnostic::OK, "Rate within tolerance"};
  Diagnostic abnormal_diagnostic_{Diagnostic::WARN, "Rate too high or too low"};
  Diagnostic stale_diagnostic_{Diagnostic::STALE, "Not enough data since last update"};
};

class PeriodicEventStatus : public PeriodicDiagnosticTask
{
public:
  explicit PeriodicEventStatus(const std::string &name);

  PeriodicEventStatus(const std::string &name, PeriodicEventStatusParams params);

  void tick(const ros::Time &stamp) override;

  void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

private:
  PeriodicEventStatusParams params_;
  SampledStatistics<double> short_term_period_;
  SampledStatistics<double> long_term_period_;
  ros::Time last_stamp_;
  std::mutex mutex_;
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_PERIODIC_EVENT_STATUS_H
