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
 * JausPresenceVector.h
 *
 *  Created on: Jun 6, 2019
 *      Author: jsun
 */

#ifndef JAUSPRESENCEVECTOR_H_
#define JAUSPRESENCEVECTOR_H_

#include <string>
using namespace std;

class JausPresenceVector {
  short const _NumberOfFields = 16;
  // int8_t* _PresenceVector;
  bool* _PresenceVector;

 public:
  JausPresenceVector();

  JausPresenceVector(uint16_t presenceVector);
  JausPresenceVector(char* byteArray, int index);

  void SetFromUint16(uint16_t presenceVector);
  bool IsBitSet(int bitNumber);
  void SetBitState(int bitNumber, bool state);

  void SetBit(int bitNumber);

  void ClearBit(int bitNumber);
  uint16_t ToUint16();
  char* ToByteArray();
  string ToString();

  virtual ~JausPresenceVector();
  short getNumberOfFields() { return _NumberOfFields; };
};

#endif /* JAUSPRESENCEVECTOR_H_ */
