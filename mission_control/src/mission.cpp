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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com
#include "mission_control/mission.h"

#include <sys/types.h>
#include <sys/stat.h>

#include "mission_control/behaviors/abort.h"
#include "mission_control/behaviors/attitude_servo.h"
#include "mission_control/behaviors/delay.h"
#include "mission_control/behaviors/fix_rudder.h"
#include "mission_control/behaviors/go_to_waypoint.h"
#include "mission_control/behaviors/log_to_bagfile.h"
#include "mission_control/behaviors/payload_command.h"
#include "mission_control/behaviors/set_altitude_heading.h"
#include "mission_control/behaviors/set_depth_heading.h"

#include "mission_control/behaviors/introspectable_node.h"

#include <string>


namespace mission_control
{

namespace
{

class MissionBehaviorTreeFactory : public BT::BehaviorTreeFactory
{
 public:
  static MissionBehaviorTreeFactory& instance()
  {
    static MissionBehaviorTreeFactory factory;
    return factory;
  }
 private:
  MissionBehaviorTreeFactory()
  {
    // TODO(hidmic): load behavior classes from ROS plugins
    this->registerNodeType<IntrospectableNode<AbortNode>>("Abort");
    this->registerNodeType<IntrospectableNode<AttitudeServoNode>>("AttitudeServo");
    this->registerNodeType<IntrospectableNode<DelayNode>>("Delay");
    this->registerNodeType<IntrospectableNode<FixRudderNode>>("FixRudder");
    this->registerNodeType<IntrospectableNode<GoToWaypointNode>>("GoToWaypoint");
    this->registerNodeType<IntrospectableNode<LogToBagfileNode>>("LogToBagfile");
    this->registerNodeType<IntrospectableNode<PayloadCommandNode>>("PayloadCommand");
    this->registerNodeType<IntrospectableNode<SetAltitudeHeadingNode>>("SetAltitudeHeading");
    this->registerNodeType<IntrospectableNode<SetDepthHeadingNode>>("SetDepthHeading");
  }
};

}  // namespace

int Mission::id_sequence_ = 0;

Mission::Mission(BT::Tree&& behavior_tree)  // NOLINT
    : main_behavior_tree_(std::move(behavior_tree)),
      status_(Mission::Status::READY),
      id_(++id_sequence_)
{
  // TODO(hidmic): fetch pitch angle from blackboard
  static constexpr char text[] = R"(
<root main_tree_to_execute="Go to the surface" >
    <BehaviorTree ID="Go to the surface">
       <Abort name="Command thruster and fins" />
    </BehaviorTree>
</root>
)";

  MissionBehaviorTreeFactory& factory =
      MissionBehaviorTreeFactory::instance();
  abort_behavior_tree_ = factory.createTreeFromText(text);
}

namespace filesystem
{

bool exists(const std::string& path)
{
  struct stat tmp;
  return stat(path.c_str(), &tmp) == 0;
}

}  // namespace filesystem

std::unique_ptr<Mission> Mission::fromFile(const std::string& path)
{
  if (!filesystem::exists(path))
  {
    ROS_ERROR_STREAM("Cannot read mission definition from " << path);
    return nullptr;
  }

  MissionBehaviorTreeFactory& factory = MissionBehaviorTreeFactory::instance();
  return std::unique_ptr<Mission>(new Mission(factory.createTreeFromFile(path)));
}

const std::string& Mission::description() const
{
  return main_behavior_tree_.rootNode()->name();
}

std::string Mission::active_path() const
{
  switch (status_)
  {
    case Mission::Status::EXECUTING:
      return introspection::getActivePath(main_behavior_tree_);
    case Mission::Status::ABORTING:
      return introspection::getActivePath(abort_behavior_tree_);
    default:
      return "";
  }
}

bool Mission::active() const
{
  return (status_ != Mission::Status::READY &&
          status_ != Mission::Status::PREEMPTED &&
          status_ != Mission::Status::COMPLETED &&
          status_ != Mission::Status::ABORTED);
}

Mission& Mission::start()
{
  if (!active())
  {
    status_ = Mission::Status::PENDING;
  }
  return *this;
}

Mission& Mission::resume()
{
  switch (status_)
  {
    case Mission::Status::PENDING:
    case Mission::Status::EXECUTING:
      switch (main_behavior_tree_.tickRoot())
      {
        case BT::NodeStatus::RUNNING:
          status_ = Mission::Status::EXECUTING;
          break;
        case BT::NodeStatus::SUCCESS:
          status_ = Mission::Status::COMPLETED;
          break;
        case BT::NodeStatus::FAILURE:
          status_ = Mission::Status::ABORTING;
          break;
      }
      break;
    case Mission::Status::ABORTING:
      switch (abort_behavior_tree_.tickRoot())
      {
        case BT::NodeStatus::RUNNING:
          status_ = Mission::Status::ABORTING;
          break;
        case BT::NodeStatus::SUCCESS:
        case BT::NodeStatus::FAILURE:
          status_ = Mission::Status::ABORTED;
          break;
      }
      break;
    default:
      break;
  }
  return *this;
}

Mission& Mission::preempt()
{
  switch (status_)
  {
    case Mission::Status::PENDING:
      status_ = Mission::Status::PREEMPTED;
      break;
    case Mission::Status::EXECUTING:
      status_ = Mission::Status::PREEMPTED;
      main_behavior_tree_.haltTree();
      break;
    case Mission::Status::ABORTING:
      status_ = Mission::Status::PREEMPTED;
      abort_behavior_tree_.haltTree();
    default:
      break;
  }
  return *this;
}

Mission& Mission::abort()
{
  switch (status_)
  {
    case Mission::Status::PENDING:
      status_ = Mission::Status::ABORTED;
      break;
    case Mission::Status::EXECUTING:
      status_ = Mission::Status::ABORTING;
      main_behavior_tree_.haltTree();
      break;
    default:
      break;
  }
  return *this;
}

}  // namespace mission_control
