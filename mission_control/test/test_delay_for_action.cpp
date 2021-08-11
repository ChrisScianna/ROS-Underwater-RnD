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

#include <behaviortree_cpp_v3/actions/always_success_node.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <thread>

#include "mission_control/behaviors/delay_for.h"

namespace mission_control
{
namespace
{
TEST(TestDelayForNode, nominal)
{
  DelayForNode root("delay some time", std::chrono::milliseconds(500));
  BT::AlwaysSuccessNode child("just succeed");
  root.setChild(&child);

  BT::NodeStatus status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  do
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status = root.executeTick();
  } while (BT::NodeStatus::RUNNING == status);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  status = root.executeTick();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  root.halt();
  EXPECT_EQ(root.status(), BT::NodeStatus::IDLE);
}

}  // namespace
}  // namespace mission_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_delay_for_action");
  ros::start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
