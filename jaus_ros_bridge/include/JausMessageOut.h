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
 * JausMessageOut.h
 *
 *  Created on: Jun 13, 2019
 *      Author: jsun
 */

#ifndef JAUSMESSAGEOUT_H_
#define JAUSMESSAGEOUT_H_

#include <ros/ros.h>
#include <ctime>
#include "JausMessageHeader.h"
#include "JausPresenceVector.h"
#include "udpserver.h"

struct DataInfo {
  char* _data;
  int _size;
};

class JausMessageOut {
 protected:
  // JausMessageHeader* _header;
  // JausPresenceVector* _presenceVector;
  ros::NodeHandle* _nodeHandle;
  udpserver* _udpserver;
  // int _sizeOfData;
  bool _newRefresh;

  string _myID;

  // provide 3 timers for each classes in case more messages are handled in same class
  clock_t _beginTime;
  clock_t _beginTime1;
  clock_t _beginTime2;
  int _sendInterval = 100;  // 10 Hz - wait for this number of ms between sending this message.

  bool TimeToSend(clock_t beginTime, clock_t endTime) {
    int difference = (((int)endTime) - ((int)beginTime));
    return difference >= _sendInterval;
  }

 public:
  JausMessageOut();

  virtual ~JausMessageOut();

  virtual DataInfo GetPackedMessage(void* data) = 0;
  virtual void Reset() = 0;

  // Implementation must be in the body for children to access???
  void Refresh();  // { _newRefresh = true; };
};

#endif /* JAUSMESSAGEOUT_H_ */
