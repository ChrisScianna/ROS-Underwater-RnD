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
 * ReportFinDeflection.h
 *
 *  Created on: Jun 13, 2019
 *      Author: jsun
 */

#ifndef REPORTFINDEFLECTION_H_
#define REPORTFINDEFLECTION_H_

#include <fin_control/ReportAngle.h>
#include <fin_control/SetAngle.h>
#include "JausMessageOut.h"

// struct FinData{
//    int8_t _deflectionAngle; 		// 1 byte  - DeflectionCMD in document
//};

class ReportFinDeflection : public JausMessageOut {
 private:
  int8_t _deflectionAngle;  // 1 byte  - DeflectionCMD in document
  int8_t _finId;            // 1 byte
  ros::Subscriber _subscriber_reportAngle;

 public:
  void init(ros::NodeHandle* nodeHandle, int8_t finId, udpserver* udp);
  virtual DataInfo GetPackedMessage(void* data);
  void SetFinId(int8_t finId) { _finId = finId; };
  virtual void Reset() { _deflectionAngle = -100; }
  //    void SetdeflectionAngle(int8_t angle){
  //        _deflectionAngle = angle;};
  void handleReportAngle(const fin_control::ReportAngle::ConstPtr& msg);
};

#endif /* REPORTFINDEFLECTION_H_ */
