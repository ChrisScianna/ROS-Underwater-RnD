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

/*  Behavioral Pinger
  <pinger>
      <description>
          00:00:00 - .
      </description>
      <when unit="sec">0</when>
      <timeout unit="sec">20</timeout>
      <ping_enabled>1</ping_enabled>
      <waveform_select>0</waveform_select>
      <waveform_amplitude>100.0</waveform_amplitude>
      <waveform_frequency>2.0</waveform_frequency>
      <waveform_ON_time_sec>10.0</waveform_ON_time_sec>
      <waveform_OFF_time_sec>10.0</waveform_OFF_time_sec>
  </pinger>
*/

#ifndef MISSION_CONTROL_BEHAVIORS_PINGER_H
#define MISSION_CONTROL_BEHAVIORS_PINGER_H

#include <string>
#include "mission_control/behavior.h"
#include "mission_control/Ping.h"

namespace mission_control
{

class PingerBehavior : public Behavior
{
 public:
  PingerBehavior();
  virtual ~PingerBehavior();
  virtual bool parseMissionFileParams();

  bool getParams(ros::NodeHandle nh);

  virtual void publishMsg();

 private:
  ros::Publisher fixed_rudder_behavior_pub;

  bool m_pingenabled;
  uint8_t m_waveform_select;
  float m_waveform_amp;
  float m_waveform_freq;
  float m_waveform_on;
  float m_waveform_off;

  bool m_pingenabled_ena;
  bool m_waveform_select_ena;
  bool m_waveform_amp_ena;
  bool m_waveform_freq_ena;
  bool m_waveform_on_ena;
  bool m_waveform_off_ena;
};

}  //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_PINGER_H
