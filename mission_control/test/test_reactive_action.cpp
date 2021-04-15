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

#include "mission_control/behaviors/reactive_action.h"


namespace mission_control
{
namespace
{
class ReactiveCountDownNode : public ReactiveActionNode
{
public:
  explicit ReactiveCountDownNode(size_t start_count)
    : ReactiveActionNode("countdown", {}), // NOLINT
      start_count_(start_count)
  {
  }

  bool wasSetUp() const { return setUp_; }

  bool wasTornDown() const { return tornDown_; }

private:
  BT::NodeStatus setUp() override
  {
    if (start_count_ == 0u)
    {
      return BT::NodeStatus::SUCCESS;
    }
    setUp_ = true;
    current_count_ = start_count_;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus doWork() override
  {
    return --current_count_ == 0u ?
      BT::NodeStatus::SUCCESS :
      BT::NodeStatus::RUNNING;
  }

  void tearDown() override
  {
    tornDown_ = true;
  }

  const size_t start_count_;
  size_t current_count_;
  bool tornDown_{false};
  bool setUp_{false};
};

TEST(TestReactiveActionNode, immediate_success)
{
  ReactiveCountDownNode root(0);
  BT::NodeStatus status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(root.wasSetUp());
  EXPECT_FALSE(root.wasTornDown());
}

TEST(TestReactiveActionNode, nominal)
{
  ReactiveCountDownNode root(1);
  BT::NodeStatus status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_TRUE(root.wasSetUp());
  EXPECT_FALSE(root.wasTornDown());
  status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(root.wasTornDown());
}

TEST(TestReactiveActionNode, halt_while_idle)
{
  ReactiveCountDownNode root(1u);
  root.halt();
  EXPECT_EQ(root.status(), BT::NodeStatus::IDLE);
  EXPECT_FALSE(root.wasSetUp());
  EXPECT_FALSE(root.wasTornDown());
}

TEST(TestReactiveActionNode, halt)
{
  ReactiveCountDownNode root(1u);
  BT::NodeStatus status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  EXPECT_TRUE(root.wasSetUp());
  EXPECT_FALSE(root.wasTornDown());
  root.halt();
  EXPECT_EQ(root.status(), BT::NodeStatus::IDLE);
  EXPECT_TRUE(root.wasTornDown());
}

}  // namespace
}  // namespace mission_control

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
