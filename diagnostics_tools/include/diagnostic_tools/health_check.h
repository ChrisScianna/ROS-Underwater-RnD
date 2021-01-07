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
 * diagnostic_tools/health_check.h
 */

#ifndef DIAGNOSTIC_TOOLS_HEALTH_CHECK_H
#define DIAGNOSTIC_TOOLS_HEALTH_CHECK_H

#include <map>
#include <mutex>  // NOLINT(build/c++11)
#include <string>

#include <boost/make_unique.hpp>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "diagnostic_tools/diagnostic.h"

namespace qna
{
namespace diagnostic_tools
{

template <typename... Ts>
class HealthCheck
{
public:
  using TestFunctionType = boost::function<Diagnostic(Ts...)>;

  HealthCheck() : impl_(boost::make_unique<Implementation>("Some health check")) {}

  HealthCheck(const std::string &name, TestFunctionType test_function)
    : impl_(boost::make_unique<Implementation>(name, test_function)) {}

  bool test(const Ts &... args)
  {
    return impl_->test(args...);
  }

  operator diagnostic_updater::DiagnosticTask &()
  {
    return *impl_;
  }

private:
  class Implementation : public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit Implementation(const std::string &name)
      : diagnostic_updater::DiagnosticTask(name) {}

    Implementation(const std::string &name, TestFunctionType test_function)
      : diagnostic_updater::DiagnosticTask(name), test_function_(test_function) {}

    bool test(const Ts &... args)
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (test_function_)
      {
        diagnostic_ = test_function_(args...);
        if (diagnostic_.description().empty())
        {
          diagnostic_.description(description_for(diagnostic_.status()));
        }
      }
      return diagnostic_.status() == Diagnostic::OK;
    }

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
    {
      std::lock_guard<std::mutex> guard(mutex_);
      stat.summary(diagnostic_.status(), diagnostic_.description());
      if (diagnostic_.has_code())
      {
        stat.add("Code", diagnostic_.code());
      }
      for (const auto &kv : diagnostic_.data())
      {
        stat.add(kv.first, kv.second);
      }
    }

  private:
    static const char *description_for(Diagnostic::status_type status)
    {
      switch (status)
      {
      case Diagnostic::OK:
        return "Everything OK";
      case Diagnostic::WARN:
        return "Something doesn't seem right";
      case Diagnostic::ERROR:
        return "Something's wrong";
      case Diagnostic::STALE:
        return "Not enough tests conducted";
      default:
        throw std::logic_error("Invalid diagnostic status");
      }
    }

    Diagnostic diagnostic_;
    TestFunctionType test_function_;
    std::mutex mutex_;
  };

  std::unique_ptr<Implementation> impl_;
};

template <typename... Ts>
HealthCheck<Ts...> create_health_check(
  const std::string &name, typename HealthCheck<Ts...>::TestFunctionType test_function)
{
  return HealthCheck<Ts...>(name, test_function);
}

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_HEALTH_CHECK_H
