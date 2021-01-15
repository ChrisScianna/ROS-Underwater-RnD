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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

/* Behavioral Attitude Servo
        <attitude_servo>
            <description>
                00:00:00 - At the beginning of the mission, set everything to initial conditions.
            </description>
            <when unit="sec">0</when>
            <timeout unit="sec">5</timeout>
            <roll unit="deg">0.0</roll>
            <pitch unit="deg">0.0</pitch>
            <yaw unit="deg">0.0</yaw>
            <speed_knots>0.0</speed_knots>
        </attitude_servo>
*/

#ifndef MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H
#define MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H

#include <string>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace mission_control
{
class Attitude_Servo : public BT::SyncActionNode
{
  public:
    Attitude_Servo(const std::string& name) :
        BT::SyncActionNode(name, {}), _behavioralStatus(BT::NodeStatus::IDLE)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    BT::NodeStatus getStatus();

  private:
    BT::NodeStatus _behavioralStatus;
};

}   //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_ATTITUDE_SERVO_H
