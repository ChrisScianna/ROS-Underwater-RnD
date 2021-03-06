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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

/*
 * SetFinDeflection.h
 *
 *  Created on: Jun 11, 2019
 *      Author: jsun
 */

#ifndef SETFINDEFLECTION_H_
#define SETFINDEFLECTION_H_

#include <ros/ros.h>
#include <stdint.h>
#include "JausMessageIn.h"
#include <fin_control/SetAngles.h>

class SetFinDeflection : public JausMessageIn {
 private:
  void commandFinAngles(const ros::TimerEvent& ev);

  int8_t _finId;              // 1 byte
  int8_t _deflectionAngle;    // 1 byte
  int8_t _deflectionRateCmd;  // 1 byte but not used
  ros::Publisher _publisher_setAngles;
  ros::Timer _commandFinAnglesTimer;
  std::vector<float> _finAngles{0.,0.,0.,0.};

 public:
  void init(ros::NodeHandle* nodeHandle);
  void PublishFinAngles(const bool enable);

  void ProcessData(char* message);

  int8_t GetFinID();
  int8_t GetDeflectionAngle();
  int8_t GetDeflectionRateCmd();
};

#endif /* SETFINDEFLECTION_H_ */
