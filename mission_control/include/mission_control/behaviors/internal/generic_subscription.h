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
#ifndef MISSION_CONTROL_BEHAVIORS_INTERNAL_GENERIC_SUBSCRIPTION_H
#define MISSION_CONTROL_BEHAVIORS_INTERNAL_GENERIC_SUBSCRIPTION_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <memory>
#include <string>
#include <vector>

namespace mission_control
{
namespace internal
{

class GenericSubscriptionImpl;

class GenericSubscription final
{
 public:
  // Uses boost because ROS does
  using CallbackT = boost::function<void(
    const std::string&,
    const ros::MessageEvent<
      const topic_tools::ShapeShifter
    > &)>;

  GenericSubscription();
  ~GenericSubscription();

  GenericSubscription(GenericSubscription&&);

  GenericSubscription&
  operator=(GenericSubscription&&);

  static
  GenericSubscription
  create(const ros::NodeHandle& nh,
         const CallbackT& callback);

  static
  GenericSubscription create(
    const ros::NodeHandle& nh,
    const std::vector<std::string>& topics,
    const CallbackT& callback);

  void shutdown();

 private:
  explicit GenericSubscription(
    std::unique_ptr<GenericSubscriptionImpl> impl);

  std::unique_ptr<GenericSubscriptionImpl> impl_;
};

}  // namespace internal
}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_INTERNAL_GENERIC_SUBSCRIPTION_H
