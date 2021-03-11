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
 * diagnostic_tools/sampled_statistics.h
 */

#ifndef DIAGNOSTIC_TOOLS_SAMPLED_STATISTICS_H
#define DIAGNOSTIC_TOOLS_SAMPLED_STATISTICS_H

#include <algorithm>
#include <queue>

namespace qna
{
namespace diagnostic_tools
{
template <typename T>
class SampledStatistics
{
public:
  explicit SampledStatistics(T windows_size)
  {
    windows_size_ = windows_size;
  }

  void reset()
  {
    average_ = T{};
    minimum_ = T{};
    maximum_ = T{};
    accumulate_ = T{};

    std::queue<T> empty;
    std::swap(buffer_, empty);
  }

  void update(const T &sample)
  {
    if (buffer_.size() > 0)
    {
      if (maximum_ < sample)
      {
        maximum_ = sample;
      }
      else if (minimum_ > sample)
      {
        minimum_ = sample;
      }
    }
    else
    {
      minimum_ = maximum_ = sample;
    }

    if (buffer_.size() == windows_size_)
    {
      accumulate_ -= buffer_.front();
      buffer_.pop();
    }
    accumulate_ += sample;
    buffer_.push(sample);

    average_ = (accumulate_ / buffer_.size());
  }

  size_t sample_count() const
  {
    return buffer_.size();
  }

  T average() const
  {
    return average_;
  }

  T maximum() const
  {
    return maximum_;
  }

  T minimum() const
  {
    return minimum_;
  }

private:
  T average_{};
  T minimum_{};
  T maximum_{};
  T accumulate_{};
  std::queue<T> buffer_;
  T windows_size_{};
};

}  // namespace diagnostic_tools
}  // namespace qna

#endif  // DIAGNOSTIC_TOOLS_SAMPLED_STATISTICS_H
