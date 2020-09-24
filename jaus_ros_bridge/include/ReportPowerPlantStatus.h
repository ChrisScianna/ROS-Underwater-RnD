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
 * ReportPowerPlantStatus.h
 *
 *  Created on: Jun 18, 2019
 *      Author: jsun
 */

#ifndef REPORTPOWERPLANTSTATUS_H_
#define REPORTPOWERPLANTSTATUS_H_

#include <thruster_control/ReportMotorTemperature.h>
#include <thruster_control/ReportRPM.h>
#include "JausMessageOut.h"

struct EngineData {
  int8_t _engineId = 0;     // 1 byte
  int8_t _engineState = 0;  // 1 byte but we don't care for now
  // Set to some invalid numbers so OCU ignore them, will work on presence vector.
  int16_t _rpm = 30000;         // 2 bytes
  int16_t _engineTemp = 30000;  // 2 bytes
};

class ReportPowerPlantStatus : public JausMessageOut {
 private:
  uint8_t* Int16ToByteArray(int16_t inVal);
  ros::Subscriber _subscriber_reportRPM;
  ros::Subscriber _subscriber_reportMotorTemp;
  EngineData _engineData;

 public:
  void init(ros::NodeHandle* nodeHandle, udpserver* udp);

  void handleReportRPM(const thruster_control::ReportRPM::ConstPtr& msg);
  void handleReportMotorTemp(const thruster_control::ReportMotorTemperature::ConstPtr& msg);

  virtual DataInfo GetPackedMessage(void* data);
  virtual void Reset();

  //    void SetEnginId(int8_t id) {
  //        _engineId = id;};

  //    void SetEngineState(int8_t state){
  //        _engineState = state;};

  //    void SetRPM(int rpm){
  //        _rpm = rpm;};

  //    void SetTemp(int temp){
  //        _engineTemp = (int16_t)temp;};
};

#endif /* REPORTPOWERPLANTSTATUS_H_ */
