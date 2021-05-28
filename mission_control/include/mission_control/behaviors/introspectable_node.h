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

#ifndef MISSION_CONTROL_BEHAVIORS_INTROSPECTABLE_NODE_H
#define MISSION_CONTROL_BEHAVIORS_INTROSPECTABLE_NODE_H

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include <string>

#include "mission_control/behaviors/introspection.h"


namespace mission_control
{

// NOTE(hidmic): this is insane, but BT library support for
// various constructor forms left little margin.
template<class BaseNodeT, bool default_constructible_only =
         BT::has_default_constructor<BaseNodeT>::value &&
         !BT::has_params_constructor<BaseNodeT>::value>
class IntrospectableNode;

template<class BaseNodeT>
class IntrospectableNode<BaseNodeT, false> : public BaseNodeT
{
public:
  using BaseNodeT::BaseNodeT;

  BT::NodeStatus executeTick() override
  {
    if (BT::NodeStatus::IDLE == this->status())
    {
      parent_path_ = introspection::extendActivePath(
          this->config().blackboard.get(), this->name());
    }

    // NOTE(hidmic): this will not work for async and coroutine-based action types.
    // The BT library does not lend itself well to built-in node types extension.
    BT::NodeStatus current_status = BaseNodeT::executeTick();

    if (BT::NodeStatus::RUNNING != current_status)
    {
      introspection::setActivePath(this->config().blackboard.get(), parent_path_);
    }
    return current_status;
  }

  void halt() override
  {
    introspection::setActivePath(this->config().blackboard.get(), parent_path_);

    BaseNodeT::halt();
  }

 private:
  std::string parent_path_{};
};

template<class BaseNodeT>
class IntrospectableNode<BaseNodeT, true> : public BaseNodeT
{
 public:
  IntrospectableNode(const std::string& name, const BT::NodeConfiguration& config)
      : BaseNodeT(name), blackboard_(config.blackboard)
  {
  }

  BT::NodeStatus executeTick() override
  {
    if (BT::NodeStatus::IDLE == this->status())
    {
      parent_path_ =
          introspection::extendActivePath(blackboard_.get(), this->name());
    }
    // NOTE(hidmic): this will not work for async and coroutine-based action types.
    // The BT library does not lend itself well to built-in node types extension.
    BT::NodeStatus current_status = BaseNodeT::executeTick();

    if (BT::NodeStatus::RUNNING != current_status)
    {
      introspection::setActivePath(blackboard_.get(), parent_path_);
    }
    return current_status;
  }

  void halt() override
  {
    introspection::setActivePath(blackboard_.get(), parent_path_);

    BaseNodeT::halt();
  }

 private:
  BT::Blackboard::Ptr blackboard_;
  std::string parent_path_{};
};

}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_INTROSPECTABLE_NODE_H
