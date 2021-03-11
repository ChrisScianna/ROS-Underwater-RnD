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

#include "diagnostic_tools/diagnostic.h"

#include <cstdarg>
#include <cstring>
#include <memory>
#include <string>

namespace qna
{
namespace diagnostic_tools
{

Diagnostic::Diagnostic(Diagnostic::status_type status)
  : status_(status), description_(""), code_(0) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, std::string description)
  : status_(status), description_(std::move(description)), code_(0) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, uint64_t code)
  : status_(status), description_(""), code_(code) {}

Diagnostic::Diagnostic(Diagnostic::status_type status, std::string description, uint64_t code)
  : status_(status), description_(std::move(description)), code_(code)
{
  if (description.empty())
  {
    throw std::logic_error("Diagnostic description is empty");
  }
}

namespace
{

std::string vaformat(const char* format, va_list va)
{
  va_list other_va;
  va_copy(other_va, va);
  int size = vsnprintf(NULL, 0, format, other_va);
  va_end(other_va);
  if (size < 0)
  {
    return "Failed to format";
  }
  std::unique_ptr<char> buffer(new(std::nothrow) char[size + 1]);
  if (!buffer)
  {
    return "Failed to format";
  }
  if (vsnprintf(buffer.get(), size + 1, format, va) < 0)
  {
    return "Failed to format";
  }
  return buffer.get();
}

}  // namespace

Diagnostic& Diagnostic::description(const char* format, ...)
{
  va_list va;
  va_start(va, format);
  description_ = vaformat(format, va);
  va_end(va);
  return *this;
}

Diagnostic& Diagnostic::data(const std::string& key, const char* format, ...)
{
  va_list va;
  va_start(va, format);
  data_[key] = vaformat(format, va);
  va_end(va);
  return *this;
}

}  // namespace diagnostic_tools
}  // namespace qna
