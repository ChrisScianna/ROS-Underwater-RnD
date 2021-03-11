/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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

#ifndef MISSION_MANAGER_BEHAVIORS_ATTITUDE_SERVO_H
#define MISSION_MANAGER_BEHAVIORS_ATTITUDE_SERVO_H

#include <string>
#include "mission_manager/behavior.h"
#include "mission_manager/AttitudeServo.h"

namespace mission_manager
{

class AttitudeServoBehavior : public Behavior
{
 public:
  AttitudeServoBehavior();
  virtual ~AttitudeServoBehavior();

  virtual bool parseMissionFileParams();
  bool getParams(ros::NodeHandle nh);
  virtual void publishMsg();
  bool checkState(const auv_interfaces::StateStamped& data);

 private:
  ros::Publisher attitude_servo_behavior_pub;

  float m_roll;
  float m_pitch;
  float m_yaw;
  float m_speed_knots;

  std::string m_roll_unit;
  std::string m_pitch_unit;
  std::string m_yaw_unit;

  bool m_roll_ena;
  bool m_pitch_ena;
  bool m_yaw_ena;
  bool m_speed_knots_ena;

  float m_roll_tol;
  float m_pitch_tol;
  float m_yaw_tol;
};

}   //  namespace mission_manager

#endif  //  MISSION_MANAGER_BEHAVIORS_ATTITUDE_SERVO_H
