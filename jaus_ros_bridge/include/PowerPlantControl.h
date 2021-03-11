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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>


/*
 * PowerPlantControl.h
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#ifndef POWERPLANTCONTROL_H_
#define POWERPLANTCONTROL_H_

#include <stdint.h>

#include <ros/ros.h>
#include "JausMessageIn.h"

class PowerPlantControl : public JausMessageIn {
 public:
  // PowerPlantControl();
  void init(ros::NodeHandle* nodeHandle);

  void ProcessData(char* message);

  enum PresenceBits { RPMBit = 0, ThrottleBit = 1, GlowPlugStateBit = 2 };

  int8_t GetPowerPlantID();

  int8_t GetPowerCMD();

  int8_t GetEnginId();

  int GetRpm();
  void StopThruster();

 private:
  int8_t _powerPlantId;  // 1 byte
  int8_t _powerCmd;      // 1 byte
  int8_t _engineId;      // 1 byte
  int _rpm;              // 4 bytes - optional
  ros::Publisher _publisher_setRPM;
  int _maxRPM;

  bool _isSetToZero;
  ros::Timer ThrusterControl_timer;
};

#endif /* POWERPLANTCONTROL_H_ */
