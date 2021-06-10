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

#include <gtest/gtest.h>

#include <string>

#include "mission_control/behaviors/introspectable_node.h"
#include "mission_control/behaviors/introspection.h"


namespace mission_control
{
namespace
{

class SimpleActionNode : public BT::ActionNodeBase
{
public:
  using TickFunction =
      std::function<BT::NodeStatus(BT::TreeNode&)>;

  SimpleActionNode(
      const std::string& name,
      const TickFunction& tick_function,
      const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config),
      tick_function_(tick_function)
  {
  }

  void halt() override
  {
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  BT::NodeStatus tick() override
  {
    return tick_function_(*this);
  }

private:
  TickFunction tick_function_;
};

using SimpleIntrospectableActionNode = IntrospectableNode<SimpleActionNode>;

TEST(TestIntrospectableActionNode, nominal)
{
  BT::Tree tree;
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  tree.blackboard_stack.push_back(bb);
  BT::NodeConfiguration config;
  config.blackboard = bb;
  auto chat =
    std::make_shared<IntrospectableNode<BT::SequenceNode>>("chat", config);
  tree.nodes.push_back(chat);
  auto say_hi =
    std::make_shared<SimpleIntrospectableActionNode>(
      "say_hi",
      [](BT::TreeNode& node)
      {
        if (node.status() == BT::NodeStatus::RUNNING)
        {
          return BT::NodeStatus::SUCCESS;
        }
        std::cout << "hi" << std::endl;
        return BT::NodeStatus::RUNNING;
      }, config);
  chat->addChild(say_hi.get());
  tree.nodes.push_back(say_hi);
  auto say_bye =
    std::make_shared<SimpleIntrospectableActionNode>(
      "say_bye",
      [](BT::TreeNode& node)
      {
        if (node.status() == BT::NodeStatus::RUNNING)
        {
          return BT::NodeStatus::SUCCESS;
        }
        std::cout << "bye" << std::endl;
        return BT::NodeStatus::RUNNING;
      }, config);
  chat->addChild(say_bye.get());
  tree.nodes.push_back(say_bye);

  EXPECT_EQ(introspection::getActivePath(tree), "");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(introspection::getActivePath(tree), "/chat/say_hi");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(introspection::getActivePath(tree), "/chat/say_bye");
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(introspection::getActivePath(tree), "");
}

}  // namespace
}  // namespace mission_control


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
