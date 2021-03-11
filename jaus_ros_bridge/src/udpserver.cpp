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
 * udpserver.cpp
 *
 *  Created on: May 29, 2019
 *      Author: ubuntu1604
 */

#include "udpserver.h"
#include "ros/ros.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <iostream>
#include <sstream>

#include <cstdint>
#include <cstring>
#include "JausDataManager.h"

typedef std::map<std::string, message_entry*> map_type;

udpserver::udpserver(const char* ipaddress, int port) {
  bzero(&m_servaddr, sizeof(m_servaddr));

  // Create a UDP Socket
  m_udpsocket = socket(AF_INET, SOCK_DGRAM, 0);
  // ROS_INFO("m_udpsocket is %d", m_udpsocket);

  m_servaddr.sin_family = AF_INET;
  // servaddr.sin_addr.s_addr = INADDR_ANY;
  m_servaddr.sin_addr.s_addr = inet_addr(ipaddress);  //"192.168.192.135");
  m_servaddr.sin_port = htons(port);
  // bind server address to socket descriptor
  bind(m_udpsocket, (struct sockaddr*)&m_servaddr, sizeof(m_servaddr));
  _sendingMessage = false;
  m_connected = false;
}

udpserver::~udpserver() {
  // TODO Auto-generated destructor stub
}

bool udpserver::canReceive() {
  unsigned char buf;
  int err = recv(m_udpsocket, &buf, 1, MSG_PEEK | MSG_DONTWAIT);
  return err == -1 ? false : true;
}

bool udpserver::receive() {
  // mu_receive.lock();
  bool dataReceived = false;
  // first, test if there is message to receive instead of waiting and blocking the thread
  if (!canReceive()) {
    return dataReceived;
  }

  memset(m_buffer, 0, BUFFER_SIZE * sizeof(m_buffer[0]));

  int len = sizeof(m_cliaddr);

  int n = (int)recvfrom(m_udpsocket, m_buffer, 500, 0, (struct sockaddr*)&m_cliaddr,
                        (socklen_t*)&len);  // receive message from server

  if (n != -1) {
    // ROS_INFO("From receive(): n =  %d", n);
    // m_buffer[n] = '\0';

    // puts(m_buffer);
    if (strcmp(m_buffer, "connect") == 0) {
      ROS_INFO(
          "--- Received  "
          "connect"
          " command, send it back for confirmation ---");
      sendto(m_udpsocket, m_buffer, 7, 0, (struct sockaddr*)&m_cliaddr, sizeof(m_cliaddr));
      m_connected = true;
    } else if (strcmp(m_buffer, "disconnect") == 0) {
      ROS_INFO(
          "--- Received "
          "disconnect"
          " command ---");
      m_connected = false;
      _messageList.Clear();
    }

    dataReceived = true;
  } else {
    // ROS_INFO("data not received from recvfrom()");
  }

  // mu_receive.unlock();
  return dataReceived;
}

void udpserver::SendMessage() {
  // pthread_mutex_lock( &mu_send );
  _sendingMessage = true;

  std::map<std::string, message_entry*> msg_map = _messageList.GetAllMessages();

  if (debug_mode) ROS_INFO("Sending %d messages", _messageList.GetCount());

  if (_messageList.GetCount() > 0) {
    map_type::iterator sub_iter = msg_map.begin();
    while (sub_iter != msg_map.end()) {
      sendto(m_udpsocket, sub_iter->second->data, sub_iter->second->size, 0,
             (struct sockaddr*)&m_cliaddr, sizeof(m_cliaddr));

      // ROS_INFO("Send message:  %s, with size: %d", sub_iter->first.c_str(),
      // sub_iter->second->size);

      ++sub_iter;
    }
    _messageList.Clear();
  }

  // pthread_mutex_unlock( &mu_send );
  _sendingMessage = false;
}

char* udpserver::getData() { return m_buffer; }

bool udpserver::RequestSendingMessage(std::string msgID, char* data, int datasize) {
  _messageList.Add_message(msgID, data, datasize);

  return true;
}

bool udpserver::HasMessageToSend() { return _messageList.GetCount() > 0; }

void udpserver::Disconnect() {
  if (!m_connected) return;

  char buffer[] = "disconnect";
  sendto(m_udpsocket, buffer, 10, 0, (struct sockaddr*)&m_cliaddr, sizeof(m_cliaddr));
}
