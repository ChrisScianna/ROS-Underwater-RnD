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

#ifndef REPORTINSDATA_H_
#define REPORTINSDATA_H_

#include "JausMessageOut.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"

struct INSData {
  float _heading;
  float _roll;
  float _pitch;
  float _north_speed;
  float _east_speed;
  float _vertical_speed;
  float _latitude;
  float _longitude;
  float _altitude;
  unsigned long _timestamp;
};

class ReportINSData : public JausMessageOut {
 private:
  INSData _insData;

  ros::Subscriber _subscriber_reportINS;

  bool HasNewData(const sensor_msgs::Imu data);

 public:
  void init(ros::NodeHandle* nodeHandle, udpserver* udp);
  virtual DataInfo GetPackedMessage(void* data);
  INSData getInsData() { return _insData; };
  virtual void Reset();

  void handleReportINS(const sensor_msgs::Imu data);
};

#endif /* REPORTINSDATA_H_ */
