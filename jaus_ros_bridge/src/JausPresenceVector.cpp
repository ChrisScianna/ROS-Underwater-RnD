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
 * JausPresenceVector.cpp
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */
//#include <stddef.h>
//#include <stdio.h>
//#include <stdlib.h>
#include "JausPresenceVector.h"
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

uint16_t read_uint16(char *p, int index = 0) { return *(uint16_t *)(p + index); }

int read_int(char *p, int index = 0) { return *(int *)(p + index); }

bool read_bool(char *p, int index = 0) { return *(bool *)(p + index); }

int read_short(char *p, int index = 0) { return *(uint16_t *)(p + index); }

JausPresenceVector::JausPresenceVector() {
  _PresenceVector = new bool[_NumberOfFields];
  memset(_PresenceVector, false, _NumberOfFields);
  for (int i = 0; i < _NumberOfFields; i++) {
    _PresenceVector[i] = false;
  }
}

JausPresenceVector::~JausPresenceVector() {
  // TODO Auto-generated destructor stub
  delete[] _PresenceVector;
}

JausPresenceVector::JausPresenceVector(uint16_t presenceVector) {
  _PresenceVector = new bool[_NumberOfFields];
  memset(_PresenceVector, false, _NumberOfFields);
  //    int size = _NumberOfFields-1;

  //    while (n != 0)
  //    {
  //            _PresenceVector[size--] = n % 2;
  //            n /= 2;
  //    }

  for (int i = 0; i < _NumberOfFields; i++) {
    _PresenceVector[i] = false;
    if ((presenceVector & (0x01 << i)) > 0) {
      _PresenceVector[i] = true;
    }
  }
}

JausPresenceVector::JausPresenceVector(char *byteArray, int index) {
  _PresenceVector = new bool[_NumberOfFields];
  memset(_PresenceVector, false, _NumberOfFields);
  if (byteArray == nullptr) {
    // throw new someexcpeion("byteArray");
  }
  if (index < 0 || index + 2 > (int)sizeof(byteArray)) {
    // throw new someexcpeion("invalid index");
  }

  uint16_t presenceVector = read_uint16(byteArray, index);
  _PresenceVector = new bool[_NumberOfFields];
  for (int i = 0; i < _NumberOfFields; i++) {
    _PresenceVector[i] = false;
    if ((presenceVector & (0x01 << i)) > 0) {
      _PresenceVector[i] = true;
    }
  }
}

void JausPresenceVector::SetFromUint16(uint16_t n) {
  int size = _NumberOfFields - 1;

  while (n != 0) {
    _PresenceVector[size--] = n % 2;
    n /= 2;
  }

  //	for (int i = 0; i < _NumberOfFields; i++)
  //	{
  //		_PresenceVector[i] = false;
  //		if ((n & (0x01 << i)) > 0)
  //		{
  //			_PresenceVector[i] = true;
  //		}
  //	}
}

bool JausPresenceVector::IsBitSet(int bitNumber) {
  if (bitNumber < 0 || bitNumber >= _NumberOfFields) {
    // throw new someexcpeion("bit number is out of range");
  }
  return _PresenceVector[bitNumber];
}

void JausPresenceVector::SetBitState(int bitNumber, bool state) {
  if (bitNumber < 0 || bitNumber >= _NumberOfFields) {
    // throw new ArgumentOutOfRangeException("bit number is out of range");
  }
  _PresenceVector[bitNumber] = state;
}

void JausPresenceVector::SetBit(int bitNumber) {
  if (bitNumber < 0 || bitNumber >= _NumberOfFields) {
    // throw new ArgumentOutOfRangeException("bit number is out of range");
  }
  _PresenceVector[bitNumber] = true;
}

void JausPresenceVector::ClearBit(int bitNumber) {
  if (bitNumber < 0 || bitNumber >= _NumberOfFields) {
    // throw new ArgumentOutOfRangeException("bit number is out of range");
  }
  _PresenceVector[bitNumber] = false;
}

uint16_t JausPresenceVector::ToUint16() {
  uint16_t presenceVector = 0;
  int base = 1;
  for (int i = _NumberOfFields - 1; i >= 0; i--) {
    presenceVector += _PresenceVector[i] * base;
    base = base * 2;
  }

  //	uint16_t presenceVector = 0;
  //	for (int i = 0; i < _NumberOfFields; i++)
  //	{
  //		if (_PresenceVector[i])
  //		{
  //			presenceVector |= (uint16_t)((0x01 << i));
  //		}
  //	}
  return presenceVector;
}

// caller need to release the memory
char *JausPresenceVector::ToByteArray() {
  //    //how to do this better way???
  //    std::string s = std::to_string(ToUint16());
  //    char const * st = s.c_str();
  //    ROS_INFO("st is %s", st);
  //    //char val = new char(sizeof(s.c_str()));
  //    char val[sizeof(st)];
  //    ROS_INFO("sizeof(st) is %d", sizeof(st));

  //    for(int i=0; i < (int)sizeof(st); i++)
  //    {
  //            val[i] = st[i];
  //    }

  //    return val;

  //
  char *pvchar;
  uint16_t val = ToUint16();
  pvchar = reinterpret_cast<char *>(&val);
  return pvchar;
}

string JausPresenceVector::ToString() {
  string st = "";

  for (int i = 0; i < _NumberOfFields; i++) {
    if (IsBitSet(i)) {
      st = st + "1";
    } else {
      st = st + "0";
    }
  }
  return st;
}
