/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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
#include "mission_control/behaviors/internal/async_bag_writer.h"

#include <string>

namespace mission_control
{
namespace internal
{

void AsyncBagWriter::start(const std::string& path, const Options& options)
{
  // Open temporary bag for writing
  path_ = path;
  std::string tmp_path = path_ + ".active";
  bag_.open(tmp_path, rosbag::bagmode::Write);
  bag_.setChunkThreshold(1024 * options.chunk_size);
  bag_.setCompression(options.compression);
  max_buffer_size_ = 1024 * 1024 * options.buffer_size;

  // Start asynchronous writing
  exception_ = nullptr;
  stop_requested_ = false;
  worker_thread_ = std::thread(
    boost::bind(&AsyncBagWriter::doWrite, this));
}

void AsyncBagWriter::stop()
{
  // Stop asynchronous writing
  stop_requested_ = true;
  event_.notify_one();
  worker_thread_.join();
  buffer_.clear();
  buffer_size_ = 0u;

  // Move temporary bag to its final location
  std::string tmp_path = bag_.getFileName();
  bag_.close();  // Close bagfile before moving
  if (std::rename(tmp_path.c_str(), path_.c_str()))
  {
    throw std::system_error(
      std::error_code(errno, std::generic_category()),
      strerror(errno));
  }
  path_.clear();

  // Propagate errors, if any
  if (exception_)
  {
    std::rethrow_exception(exception_);
  }
}

bool AsyncBagWriter::write(
  const std::string& topic,
  const ros::MessageEvent<const topic_tools::ShapeShifter>& event)
{
  if (exception_)
  {
    // Skip write
    return false;
  }

  // Write to buffer and notify asynchronous writer
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.push_back(Entry{topic, event});
    buffer_size_ += event.getMessage()->size();
    if (max_buffer_size_ > 0)
    {
      while (buffer_size_ > max_buffer_size_)
      {
        Entry& entry = buffer_.front();
        buffer_size_ -= entry.event.getMessage()->size();
        buffer_.pop_front();
      }
    }
  }
  event_.notify_one();

  return true;
}

void
AsyncBagWriter::doWrite()
{
  try
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true)
    {
      while (!buffer_.empty())
      {
        const Entry& entry = buffer_.front();
        bag_.write(entry.topic, entry.event);
        buffer_.pop_front();
      }
      if (stop_requested_)
      {
        break;
      }
      event_.wait(lock);
    }
  }
  catch(...)
  {
    exception_ = std::current_exception();
  }
}

}  // namespace internal
}  // namespace mission_control
