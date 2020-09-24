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

#include "Messagelist.h"
#include <ros/ros.h>
#include <mutex>

std::mutex mtx;

MessageList::MessageList() {}

MessageList::~MessageList() { Clear(); }

void MessageList::Add_message(std::string msgID, char* msg, int size) {
  mtx.lock();

  message_entry* tmp = new message_entry;
  char* newmsg = new char[size];
  for (int i = 0; i < size; i++) {
    newmsg[i] = msg[i];
  }

  tmp->data = newmsg;
  tmp->size = size;

  // If already exist, erase it first
  map_type::iterator it;
  it = _message_map.find(msgID);
  if (it != _message_map.end()) {
    delete (_message_map[msgID]);
    _message_map.erase(it);
  }

  _message_map[msgID] = tmp;
  // tmp->next = nullptr;

  mtx.unlock();
}

std::map<std::string, message_entry*> MessageList::GetAllMessages() { return _message_map; }

void MessageList::Clear() {
  mtx.lock();
  _message_map.clear();

  mtx.unlock();
}
