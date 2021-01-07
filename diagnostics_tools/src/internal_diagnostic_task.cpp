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

#include "diagnostic_tools/internal_diagnostic_task.h"

#include <cstdarg>
#include <memory>
#include <string>

namespace qna
{
namespace diagnostic_tools
{

using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;

InternalDiagnosticTask::InternalDiagnosticTask(const std::string &name)
  : InternalDiagnosticTask(name, false) {}

InternalDiagnosticTask::InternalDiagnosticTask(const std::string &name, bool critical)
  : diagnostic_updater::DiagnosticTask(name), critical_(critical) {}

bool InternalDiagnosticTask::check(const std::string &name, bool condition)
{
  return check(name, condition, "NOT OK");
}

bool InternalDiagnosticTask::check(const std::string &name, bool condition, const char *format,
                                   ...)
{
  va_list va;
  va_start(va, format);
  try
  {
    std::lock_guard<std::mutex> guard(mutex_);
    status_[name].ok &= condition;
    if (!condition)
    {
      va_list other_va;
      va_copy(other_va, va);
      int size = vsnprintf(NULL, 0, format, other_va);
      if (size < 0)
      {
        throw std::runtime_error("Bad format");
      }
      std::unique_ptr<char> buffer(new char[size + 1]);
      if (vsnprintf(buffer.get(), size, format, va) < 0)
      {
        throw std::runtime_error("Bad format");
      }
      status_[name].error = buffer.get();
    }
  }
  catch (...)
  {
    va_end(va);
    throw;
  }
  return condition;
}

void InternalDiagnosticTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  std::lock_guard<std::mutex> guard(mutex_);
  bool ok = true;
  for (const auto &kv : status_)
  {
    ok &= kv.second.ok;
    stat.add(kv.first, kv.second.ok ? "OK" : kv.second.error);
  }
  if (!ok)
  {
    auto level = critical_ ? DiagnosticStatus::ERROR : DiagnosticStatus::WARN;
    stat.summary(level, "Some checks failed");
  }
  else
  {
    stat.summary(DiagnosticStatus::OK, "Everything OK");
  }
  status_.clear();
}

}  // namespace diagnostic_tools
}  // namespace qna
