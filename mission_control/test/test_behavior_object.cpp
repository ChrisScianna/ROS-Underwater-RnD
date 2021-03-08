
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>
#include <stdlib.h>

#include <iostream>

#include "../include/mission_control/behavior.h"

using namespace mission_control;

class DummyBehavior : public Behavior
{
 public:
  DummyBehavior(const std::string& name, const BT::NodeConfiguration& config)
      : Behavior(name, config)
  {
    tickCount = 0;
  }
  BT::NodeStatus behaviorRunningProcess()
  {
    tickCount++;
    return status();
  }

  int tickCount;
  void changeStatus(const BT::NodeStatus& newStatus) { setStatus(newStatus); }
};

GTEST_TEST(BehaviorTest, TestIfBehaviorStartsInIDLE)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  ASSERT_EQ(dummyBehavior.status(), BT::NodeStatus::IDLE);
}

GTEST_TEST(BehaviorTest, TestIfBehaviorChangeToRunningAfterTick)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  ASSERT_EQ(dummyBehavior.tick(), BT::NodeStatus::RUNNING);
}

GTEST_TEST(BehaviorTest, TestIfBehaviorStopsAfterHalt)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  dummyBehavior.tick();
  dummyBehavior.halt();
  ASSERT_TRUE(dummyBehavior.isHalted());
}

GTEST_TEST(BehaviorTest, TestIfBehaviorDoesntChangeStatusToIdleAfterSuccess)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  dummyBehavior.changeStatus(BT::NodeStatus::SUCCESS);
  dummyBehavior.tick();
  ASSERT_EQ(dummyBehavior.tick(), BT::NodeStatus::SUCCESS);
}

GTEST_TEST(BehaviorTest, TestIfBehaviorDontChangeStatusOnTickIfFailure)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  dummyBehavior.changeStatus(BT::NodeStatus::FAILURE);
  ASSERT_EQ(dummyBehavior.tick(), BT::NodeStatus::FAILURE);
}

GTEST_TEST(BehaviorTest, TestIfBehaviorTickMultipleTimes)
{
  BT::NodeConfiguration nodeConfig;
  DummyBehavior dummyBehavior("dummy", nodeConfig);
  dummyBehavior.tick();
  dummyBehavior.tick();
  dummyBehavior.tick();
  ASSERT_EQ(dummyBehavior.tickCount, 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}