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
 * JausMessageHeader.cpp
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */
#include <stdio.h>

#include "JausMessageHeader.h"

// bool debug_mode = false;

// Build header from commandID and data size
JausMessageHeader::JausMessageHeader(JausCommandID id, short datasize) {
  _headerSize = 6;
  _headerData = new char(_headerSize);

  _commandID = id;
  _data_size = datasize;

  // convert id to char array
  char *bId = reinterpret_cast<char *>((long)&id);
  _headerData[0] = bId[0];
  _headerData[1] = bId[1];
  _headerData[2] = bId[2];
  _headerData[3] = bId[3];

  char *bSize = reinterpret_cast<char *>(&datasize);
  _headerData[4] = bSize[0];
  _headerData[5] = bSize[1];
}

// Build header from received message data
JausMessageHeader::JausMessageHeader(char *messageData) {
  _headerSize = 6;
  _headerData = new char(_headerSize);

  _commandID = (JausCommandID) * ((long *)messageData);

  _data_size = (short)(sizeof(messageData) - _headerSize);
  // ROS_INFO("_commandID is: %d", _commandID);

  for (int i = 0; i < _headerSize; i++) {
    _headerData[i] = messageData[i];
  }
  _data_size = (short)_headerData[4];
  // ROS_INFO("_data_size is: %d", _data_size);
}

JausMessageHeader::~JausMessageHeader() {
  if (_headerData != nullptr) delete _headerData;
}

short JausMessageHeader::GetHeadersize() { return _headerSize; }

short JausMessageHeader::GetDatasize() { return _data_size; }

JausCommandID JausMessageHeader::GetId() { return _commandID; }

char *JausMessageHeader::GetHeaderData() { return _headerData; }
