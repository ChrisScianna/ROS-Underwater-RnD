/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, QinetiQ, Inc.
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
 * test_thruster_control_ros.cpp
 *
 */

#include "thruster_control/thruster_control_ros.h"

#include <memory>

#include <gmock/gmock.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include "thruster_control/thruster_control.h"


namespace qna
{
namespace robot
{
namespace testing
{

class TestingUpdater : public diagnostic_updater::Updater
{
 public:
  using diagnostic_updater::Updater::Updater;

  using DiagnosticTaskInternal = diagnostic_updater::Updater::DiagnosticTaskInternal;

  const std::vector<DiagnosticTaskInternal> &getTasks()
  {
    return diagnostic_updater::Updater::getTasks();
  }
};

class MockThrusterControl : public ThrusterControlInterface
{
 public:
  MOCK_METHOD0(GetMotorRPM, thruster_control::ReportRPM() override);
  MOCK_METHOD1(SetMotorRPM, void(const thruster_control::SetRPM&) override);
  MOCK_METHOD0(GetMotorTemperature, thruster_control::ReportMotorTemperature() override);
  MOCK_METHOD0(GetBatteryState, sensor_msgs::BatteryState() override);
};

class ProxyThrusterControl : public ThrusterControlInterface
{
 public:
  explicit ProxyThrusterControl(std::shared_ptr<ThrusterControlInterface> internal)
    : internal_(internal) {}

  thruster_control::ReportRPM GetMotorRPM() override { return internal_->GetMotorRPM(); }
  void SetMotorRPM(const thruster_control::SetRPM& message) override { internal_->SetMotorRPM(message); };
  thruster_control::ReportMotorTemperature GetMotorTemperature() override { return internal_->GetMotorTemperature(); }
  sensor_msgs::BatteryState GetBatteryState() override { return internal_->GetBatteryState(); }
 private:
  std::shared_ptr<ThrusterControlInterface> internal_;
};

using ::testing::NiceMock;
using ::testing::AtLeast;
using ::testing::Return;

TEST(TestThrusterControlROS, checks_motor_rpms)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("report_motor_rpm_period", 1.0);
  pnh.setParam("max_allowed_motor_rpm", 100);
  ros::Time::setNow(ros::Time());

  std::shared_ptr<MockThrusterControl> mock(new MockThrusterControl());
  std::unique_ptr<ThrusterControlInterface> proxy(new ProxyThrusterControl(mock));
  ThrusterControlROS controller(nh, pnh, std::move(proxy));

  TestingUpdater updater(nh, pnh);
  controller.MonitorMotorRPMUsing(&updater);
  EXPECT_GE(updater.getTasks().size(), 2);

  thruster_control::ReportRPM rpm_message;
  rpm_message.rpms = 200;
  EXPECT_CALL(*mock, GetMotorRPM())
    .Times(AtLeast(1))
    .WillRepeatedly(Return(rpm_message));

  thruster_control::ReportMotorTemperature temp_message;
  temp_message.motor_temp = 10;
  EXPECT_CALL(*mock, GetMotorTemperature())
    .Times(AtLeast(1))
    .WillRepeatedly(Return(temp_message));

  sensor_msgs::BatteryState battery_message;
  EXPECT_CALL(*mock, GetBatteryState())
    .Times(AtLeast(1))
    .WillRepeatedly(Return(battery_message));

  ros::Time::setNow(ros::Time(1.5));
  ros::CallbackQueue* queue = ros::getGlobalCallbackQueue();
  queue->callAvailable(ros::WallDuration(1.0));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_thruster_control_ros");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace testing
}  // namespace robot
}  // namespace qna

int main(int argc, char ** argv)
{
  return qna::robot::testing::main(argc, argv);
}
