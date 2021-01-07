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

#ifndef DIAGNOSTIC_TOOLS_DIAGNOSTIC_H
#define DIAGNOSTIC_TOOLS_DIAGNOSTIC_H

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/DiagnosticStatus.h"

namespace qna
{
namespace diagnostic_tools
{

class Diagnostic
{
public:
  using status_type = diagnostic_msgs::DiagnosticStatus::_level_type;
  enum : status_type
  {
    OK = diagnostic_msgs::DiagnosticStatus::OK,
    WARN = diagnostic_msgs::DiagnosticStatus::WARN,
    ERROR = diagnostic_msgs::DiagnosticStatus::ERROR,
    STALE = diagnostic_msgs::DiagnosticStatus::STALE
  };

  Diagnostic() = default;

  Diagnostic(status_type status);  // NOLINT(runtime/explicit)

  Diagnostic(status_type status, uint64_t code);

  Diagnostic(status_type status, std::string description);

  Diagnostic(status_type status, std::string description, uint64_t code);

  status_type status() const
  {
    return status_;
  }

  Diagnostic& status(status_type status)
  {
    status_ = status;
    return *this;
  }

  const std::string& description() const
  {
    return description_;
  }

  Diagnostic& description(std::string description)
  {
    description_ = std::move(description);
    return *this;
  }

  Diagnostic& description(const char* format, ...);

  bool has_code() const
  {
    return code_ != 0;
  }

  uint64_t code() const
  {
    if (code_ == 0)
    {
      throw std::logic_error("diagnostic has no code");
    }
    return code_;
  }

  Diagnostic& code(uint64_t code)
  {
    code_ = code;
    return *this;
  }

  const std::map<std::string, std::string>& data() const
  {
    return data_;
  }

  Diagnostic& data(const std::string& key, const char* format, ...);

private:
  status_type status_{OK};
  std::string description_{""};
  uint64_t code_{0};
  std::map<std::string, std::string> data_{};
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  //  DIAGNOSTIC_TOOLS_DIAGNOSTIC_H
