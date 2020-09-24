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

#include "behaviors/attitude_servo.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>
#include <string>

#include "mission_manager/AttitudeServo.h"

using namespace mission_manager;

// DECLARE_COMMON_HELPERS(AttitudeServoBehavior)

// DECLARE_CONSTRUCTOR_MSG(AttitudeServoBehavior, "attitude_servo", "/mngr/attitude_servo")
AttitudeServoBehavior::AttitudeServoBehavior()
    : Behavior("attitude_servo", BEHAVIOR_TYPE_MSG, "/mngr/attitude_servo", "") {
  m_roll_ena = m_pitch_ena = m_yaw_ena = false;
  m_speed_knots_ena = false;
  m_roll_tol = m_pitch_tol = m_yaw_tol = 0.0;
  m_roll = m_pitch = m_yaw = 0.0;
  m_speed_knots = 0.0;

  ros::NodeHandle node_handle;
  attitude_servo_behavior_pub =
      node_handle.advertise<mission_manager::AttitudeServo>("/mngr/attitude_servo", 1);
}

AttitudeServoBehavior::~AttitudeServoBehavior() {}

bool AttitudeServoBehavior::getParams(ros::NodeHandle nh) {
  double f = 0.0;

  nh.getParam("/mission_manager_node/attitude_servo_roll_tol", f);
  if (f != 0.0) m_roll_tol = (float)f;
  f = 0.0;
  nh.getParam("/mission_manager_node/attitude_servo_pitch_tol", f);
  if (f != 0.0) m_pitch_tol = (float)f;
  f = 0.0;
  nh.getParam("/mission_manager_node/attitude_servo_yaw_tol", f);
  if (f != 0.0) m_yaw_tol = (float)f;

  return true;
}

/*
bool AttitudeServoBehavior::parseXml(xmlNodePtr node)
{
        parseCommonElements(node);

        for (xmlNodePtr cur = xmlFirstElementChild(node); cur; cur = cur->next) {
                if (!strcmp((const char *)cur->name, "roll")) {
                        if (!parseNodeAngle(cur, m_roll)) return false;
                        m_roll_ena = true;
                } else if (!strcmp((const char *)cur->name, "pitch")) {
                        if (!parseNodeAngle(cur, m_pitch)) return false;
                        m_pitch_ena = true;
                } else if (!strcmp((const char *)cur->name, "yaw")) {
                        if (!parseNodeAngle(cur, m_yaw)) return false;
                        m_yaw_ena = true;
                } else if (!strcmp((const char *)cur->name, "shaft_speed")) {
                        if (!parseNodeText(cur, m_shaft_speed)) return false;
                        m_shaft_speed_ena = true;
                } else if (!strcmp((const char *)cur->name, "battery_pos")) {
                        if (!parseNodeText(cur, m_batt_pos)) return false;
                        m_batt_pos_ena = true;
                }
        }

        return true;
}
*/
/*
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

bool AttitudeServoBehavior::parseMissionFileParams() {
  bool retval = true;
  std::list<BehaviorXMLParam>::iterator it;
  for (it = m_behaviorXMLParams.begin(); it != m_behaviorXMLParams.end(); it++) {
    std::string xmlParamTag = it->getXMLTag();
    if ((xmlParamTag.compare("when") == 0) || (xmlParamTag.compare("timeout") == 0)) {
      retval = parseTimeStamps(it);
    } else if (xmlParamTag.compare("roll") == 0) {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0) {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0) {
          m_roll_unit = it->second;
        }
      }

      m_roll = std::atof(it->getXMLTagValue().c_str());
      m_roll_ena = true;
    } else if (xmlParamTag.compare("pitch") == 0) {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0) {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0) {
          m_pitch_unit = it->second;
        }
      }

      m_pitch = std::atof(it->getXMLTagValue().c_str());
      m_pitch_ena = true;
    } else if (xmlParamTag.compare("yaw") == 0) {
      std::map<std::string, std::string> attribMap = it->getXMLTagAttribute();
      if (attribMap.size() > 0) {
        std::map<std::string, std::string>::iterator it;
        it = attribMap.begin();
        if (it->first.compare("unit") == 0) {
          m_yaw_unit = it->second;
        }
      }

      m_yaw = std::atof(it->getXMLTagValue().c_str());
      m_yaw_ena = true;
    } else if (xmlParamTag.compare("speed_knots") == 0) {
      m_speed_knots = std::atof(it->getXMLTagValue().c_str());
      m_speed_knots_ena = true;
    } else {
      std::cout << "Attitude Servo behavior found invalid parameter " << xmlParamTag << std::endl;
    }
  }

  return retval;
}

void AttitudeServoBehavior::publishMsg() {
  AttitudeServo msg;

  msg.roll = m_roll;
  msg.pitch = m_pitch;
  msg.yaw = m_yaw;
  msg.speed_knots = m_speed_knots;

  msg.ena_mask = 0x0;
  if (m_roll_ena) msg.ena_mask |= AttitudeServo::ROLL_ENA;
  if (m_pitch_ena) msg.ena_mask |= AttitudeServo::PITCH_ENA;
  if (m_yaw_ena) msg.ena_mask |= AttitudeServo::YAW_ENA;
  if (m_speed_knots_ena) msg.ena_mask |= AttitudeServo::SPEED_KNOTS_ENA;

  while (0 == attitude_servo_behavior_pub.getNumSubscribers()) {
    ROS_INFO("Waiting for attitude_servo subscribers to connect");
    ros::Duration(0.1).sleep();
  }

  msg.header.stamp = ros::Time::now();

  attitude_servo_behavior_pub.publish(msg);
}

/*
void AttitudeServoBehavior::populateMsg(ros::Message *msg)
{
        AttitudeServo *pmsg = dynamic_cast<AttitudeServo *>(msg);

        pmsg->roll = m_roll;
        pmsg->pitch = m_pitch;
        pmsg->yaw = m_yaw;
        pmsg->battery_pos = m_batt_pos;
        pmsg->shaft_speed = m_shaft_speed;

        pmsg->ena_mask = 0x0;
        if (m_roll_ena) pmsg->ena_mask |= AttitudeServo::ROLL_ENA;
        if (m_pitch_ena) pmsg->ena_mask |= AttitudeServo::PITCH_ENA;
        if (m_yaw_ena) pmsg->ena_mask |= AttitudeServo::YAW_ENA;
        if (m_batt_pos_ena) pmsg->ena_mask |= AttitudeServo::BATTERY_POS_ENA;
        if (m_shaft_speed_ena) pmsg->ena_mask |= AttitudeServo::SHAFT_SPEED_ENA;

        pmsg->header.stamp = ros::Time::now();
}
*/
bool AttitudeServoBehavior::checkCorrectedData(const pose_estimator::CorrectedData& data) {
  // A quick check to see if our RPY angles match
  if (m_roll_ena && (abs(m_roll - data.rpy_ang.x) > m_roll_tol)) return false;
  if (m_pitch_ena && (abs(m_pitch - data.rpy_ang.y) > m_pitch_tol)) return false;
  if (m_yaw_ena && (abs(m_yaw - data.rpy_ang.z) > m_yaw_tol)) return false;

  // TODO: check shaft speed and/or battery position?
  // TODO: make sure our RPY rates are close to zero?

  return true;
}
