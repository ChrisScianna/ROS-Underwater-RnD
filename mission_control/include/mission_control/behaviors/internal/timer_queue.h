/*******************************************************************************
 * The MIT License (MIT)
 *
 * Copyright (c) 2021, QinetiQ, Inc.
 * Copyright (c) 2015 - 2018 Michele Colledanchise
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/
#ifndef MISSION_CONTROL_BEHAVIORS_INTERNAL_TIMER_QUEUE_H
#define MISSION_CONTROL_BEHAVIORS_INTERNAL_TIMER_QUEUE_H

#include <assert.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

namespace mission_control
{
namespace internal
{
class Semaphore
{
 public:
  explicit Semaphore(unsigned int count = 0) : m_count(count) {}

  void notify()
  {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_count++;
    m_cv.notify_one();
  }

  void wait()
  {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_cv.wait(lock, [this]() { return m_count > 0; });
    m_count--;
  }

  bool waitUntil(const ros::Time& time)
  {
    std::unique_lock<std::mutex> lock(m_mtx);
    auto predicate = [this]() { return m_count > 0; };
    if (ros::Time::isSimTime())
    {
      ros::NodeHandle nh;
      boost::function<void(const rosgraph_msgs::Clock::ConstPtr&)> callback{
          [this, &time](const rosgraph_msgs::Clock::ConstPtr&) {
            if (ros::Time::now() >= time) m_cv.notify_one();
          }};
      auto sub = nh.subscribe("/clock", 1, callback);
      while (!predicate())
      {
        if (ros::Time::now() >= time)
        {
          return false;
        }
        m_cv.wait(lock);
      }
    }
    else
    {
      const std::chrono::nanoseconds time_since_epoch{
          static_cast<int64_t>(time.sec * 1e9 + time.nsec)};
      const std::chrono::system_clock::time_point time_point{time_since_epoch};
      if (!m_cv.wait_until(lock, time_point, predicate)) return false;
    }
    m_count--;
    return true;
  }

 private:
  std::mutex m_mtx;
  std::condition_variable m_cv;
  unsigned int m_count;
};

// Timer Queue
//
// Allows execution of handlers at a specified time in the future
// Guarantees:
//  - All handlers are executed ONCE, even if canceled (aborted parameter will
// be set to true)
//      - If TimerQueue is destroyed, it will cancel all handlers.
//  - Handlers are ALWAYS executed in the Timer Queue worker thread.
//  - Handlers execution order is NOT guaranteed
//
class TimerQueue
{
 public:
  TimerQueue()
  {
    if (!ros::isInitialized())
    {
      throw std::runtime_error("TimerQueue needs ROS initialized");
    }
    m_th = std::thread([this] { run(); });
  }

  ~TimerQueue()
  {
    cancelAll();
    // Abusing the timer queue to trigger the shutdown.
    add(std::chrono::milliseconds(0), [this](bool) { m_finish = true; });  // NOLINT
    m_th.join();
  }

  //! Adds a new timer
  // \return
  //  Returns the ID of the new timer. You can use this ID to cancel the
  // timer
  uint64_t add(std::chrono::milliseconds milliseconds, std::function<void(bool)> handler)
  {
    WorkItem item;

    item.end =
        ros::Time::now() + ros::Duration().fromNSec(static_cast<int64_t>(milliseconds.count()));
    item.handler = std::move(handler);

    std::unique_lock<std::mutex> lk(m_mtx);
    uint64_t id = ++m_idcounter;
    item.id = id;
    m_items.push(std::move(item));
    lk.unlock();

    // Something changed, so wake up timer thread
    m_checkWork.notify();
    return id;
  }

  //! Cancels the specified timer
  // \return
  //  1 if the timer was cancelled.
  //  0 if you were too late to cancel (or the timer ID was never valid to
  // start with)
  size_t cancel(uint64_t id)
  {
    // Instead of removing the item from the container (thus breaking the
    // heap integrity), we set the item as having no handler, and put
    // that handler on a new item at the top for immediate execution
    // The timer thread will then ignore the original item, since it has no
    // handler.
    std::unique_lock<std::mutex> lk(m_mtx);
    for (auto&& item : m_items.getContainer())
    {
      if (item.id == id && item.handler)
      {
        WorkItem newItem;
        // Zero time, so it stays at the top for immediate execution
        newItem.end = ros::Time();
        newItem.id = 0;  // Means it is a canceled item
        // Move the handler from item to newitem.
        // Also, we need to manually set the handler to nullptr, since
        // the standard does not guarantee moving an std::function will
        // empty it. Some STL implementation will empty it, others will
        // not.
        newItem.handler = std::move(item.handler);
        item.handler = nullptr;
        m_items.push(std::move(newItem));

        lk.unlock();
        // Something changed, so wake up timer thread
        m_checkWork.notify();
        return 1;
      }
    }
    return 0;
  }

  //! Cancels all timers
  // \return
  //  The number of timers cancelled
  size_t cancelAll()
  {
    // Setting all "end" to 0 (for immediate execution) is ok,
    // since it maintains the heap integrity
    std::unique_lock<std::mutex> lk(m_mtx);
    for (auto&& item : m_items.getContainer())
    {
      if (item.id)
      {
        item.end = ros::Time();
        item.id = 0;
      }
    }
    auto ret = m_items.size();

    lk.unlock();
    m_checkWork.notify();
    return ret;
  }

 private:
  TimerQueue(const TimerQueue&) = delete;
  TimerQueue& operator=(const TimerQueue&) = delete;

  void run()
  {
    while (!m_finish)
    {
      auto end = calcWaitTime();
      if (end.first)
      {
        // Timers found, so wait until it expires (or something else
        // changes)
        m_checkWork.waitUntil(end.second);
      }
      else
      {
        // No timers exist, so wait forever until something changes
        m_checkWork.wait();
      }

      // Check and execute as much work as possible, such as, all expired
      // timers
      checkWork();
    }

    // If we are shutting down, we should not have any items left,
    // since the shutdown cancels all items
    assert(m_items.size() == 0);
  }

  std::pair<bool, ros::Time> calcWaitTime()
  {
    std::lock_guard<std::mutex> lk(m_mtx);
    while (m_items.size())
    {
      if (m_items.top().handler)
      {
        // Item present, so return the new wait time
        return std::make_pair(true, m_items.top().end);
      }
      else
      {
        // Discard empty handlers (they were cancelled)
        m_items.pop();
      }
    }

    // No items found, so return no wait time (causes the thread to wait
    // indefinitely)
    return std::make_pair(false, ros::Time());
  }

  void checkWork()
  {
    std::unique_lock<std::mutex> lk(m_mtx);
    while (m_items.size() && m_items.top().end <= ros::Time::now())
    {
      WorkItem item(std::move(m_items.top()));
      m_items.pop();

      lk.unlock();
      if (item.handler) item.handler(item.id == 0);
      lk.lock();
    }
  }

  Semaphore m_checkWork;
  std::thread m_th;
  bool m_finish = false;
  uint64_t m_idcounter = 0;

  struct WorkItem
  {
    ros::Time end;
    uint64_t id;  // id==0 means it was cancelled
    std::function<void(bool)> handler;
    bool operator>(const WorkItem& other) const { return end > other.end; }
  };

  std::mutex m_mtx;
  // Inheriting from priority_queue, so we can access the internal container
  class Queue : public std::priority_queue<WorkItem, std::vector<WorkItem>, std::greater<WorkItem>>
  {
   public:
    std::vector<WorkItem>& getContainer() { return this->c; }
  } m_items;
};

}  // namespace internal
}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_INTERNAL_TIMER_QUEUE_H
