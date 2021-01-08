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

/* Behavioral payload
  <payload>
    <description>Payload Msg</description>
    <when unit="sec">30</when>
    <timeout unit="sec">35</timeout>
    <command>Go,500</command>
  </payload>
*/

#ifndef MISSION_CONTROL_BEHAVIORS_PAYLOAD_COMMAND_H
#define MISSION_CONTROL_BEHAVIORS_PAYLOAD_COMMAND_H

#include <string>
#include "mission_control/behavior.h"
#include "payload_manager/PayloadCommand.h"  // this is the ROS Message

namespace mission_control
{

class PayloadCommandBehavior : public Behavior
{
 public:
  PayloadCommandBehavior();
  virtual ~PayloadCommandBehavior();

  virtual bool parseMissionFileParams();

  bool getParams(ros::NodeHandle nh);

  virtual void publishMsg();
  bool checkCorrectedData(const pose_estimator::CorrectedData& data);

 private:
  ros::Publisher payload_command_behavior_pub;

  std::string m_command_str;

  bool m_command_str_ena;
};

}   //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_PAYLOAD_COMMAND_H
