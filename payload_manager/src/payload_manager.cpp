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

// Original version: Alex Rutfield alex.rutfield@us.QinetiQ.com

/*
 * payload_manager.cpp
 */
#include <ros/ros.h>
#include "payload_manager/PayloadCommand.h"
#include <cstring>
#include <iostream>
#include <string>   

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <unistd.h>



int acceptedSocket;
void payloadMessageCallback(const payload_manager::PayloadCommand::ConstPtr& msg)
{
  ROS_INFO("Received payload message: %s", msg->command.c_str());
  sockaddr_storage client_addr;
  socklen_t client_addr_size = sizeof(client_addr);

  auto bytes_sent = send(acceptedSocket, msg->command.c_str(), msg->command.length(), 0);
  if (bytes_sent < 1) {
    ROS_WARN("Failed to send messages onto socket");
  }
  ROS_INFO("Finished sending message through socket (%ld bytes)", bytes_sent);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "payload manager");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/payload_manager/command", 1000, payloadMessageCallback);
  int portNumber = 2250;
  if (n.getParam("/payload_manager/port_number", portNumber) == 0)
    ROS_ERROR("Port not found. Using default: [%d]", portNumber);
  else
    ROS_INFO("Using port: [%d]", portNumber);
  

  ROS_INFO("Started up payload manager.");


  const unsigned int queueConnections = 8;  // number of connections allowed on the incoming queue


  addrinfo hints, *reserved, *p;
  memset(&hints, 0, sizeof(hints));

  hints.ai_family   = AF_UNSPEC; //use either IP type
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags    = AI_PASSIVE;


  // man getaddrinfo
  int getAddressInfoFlag = getaddrinfo(NULL, std::to_string(portNumber).c_str(), &hints, &reserved);
  if (getAddressInfoFlag != 0) {
      ROS_ERROR("Unable to get address info: %s", gai_strerror(getAddressInfoFlag));
      return -1;
  }

  ROS_INFO("Detecting addresses");

  unsigned int numberOfAddresses = 0;
  char ipString[INET6_ADDRSTRLEN];    // ipv6 length makes sure both ipv4/6 addresses can be stored in this variable


  void *addr;
  std::string ipVersion;
  if (reserved == NULL) {
    ROS_ERROR("No addresses found");
    return -2;
  }
  // if address is ipv4 address
  if (reserved->ai_family == AF_INET) {
      ipVersion = "IPv4";
      sockaddr_in *ipv4 = reinterpret_cast<sockaddr_in *>(reserved->ai_addr);
      addr = &(ipv4->sin_addr);
      ++numberOfAddresses;
  } else {
      // if address is ipv6 address
      ipVersion = "IPv6";
      sockaddr_in6 *ipv6 = reinterpret_cast<sockaddr_in6 *>(reserved->ai_addr);
      addr = &(ipv6->sin6_addr);
      ++numberOfAddresses;
  }

  // convert IPv4 and IPv6 addresses from binary to text form
  inet_ntop(reserved->ai_family, addr, ipString, sizeof(ipString));
  ROS_INFO("(%d) %s : %s", numberOfAddresses, ipVersion.c_str(), ipString);

  // if no addresses found
  if (!numberOfAddresses) {
      ROS_ERROR("Found no host address to use");
      return -3;
  }

  p = reserved;//use first ip address for socket

  // let's create a new socket, socketFD is returned as descriptor
  // these calls usually return -1 as reserved of some error
  int socketFD = socket(reserved->ai_family, reserved->ai_socktype, reserved->ai_protocol);
  if (socketFD == -1) {
      ROS_ERROR("Error while creating socket");
      freeaddrinfo(reserved);
      return -4;
  }


  // Let's bind address to our socket we've just created
  int bindToSocket = bind(socketFD, reserved->ai_addr, reserved->ai_addrlen);
  if (bindToSocket == -1) {
      ROS_ERROR("Error while binding socket");
        
      // if some error occurs, make sure to close socket and free resources
      close(socketFD);
      freeaddrinfo(reserved);
      return -5;
  }


  // finally start listening for connections on our socket
  int listenToSocket = listen(socketFD, queueConnections);
  if (listenToSocket == -1) {
      ROS_ERROR("Error while Listening on socket");
       // if some error occurs, make sure to close socket and free resources
      close(socketFD);
      freeaddrinfo(reserved);
      return -6;
  }

  ROS_DEBUG("finished listening on socket");
  // structure large enough to hold client's address
  sockaddr_storage client_addr;
  socklen_t client_addr_size = sizeof(client_addr);

  acceptedSocket = accept(socketFD, (sockaddr *) &client_addr, &client_addr_size);
  if (acceptedSocket == -1) {
      ROS_ERROR("Error while accepting socket");
      return -7;
  }

  ROS_INFO("Socket accepted.");
        
  ros::spin();
  close(acceptedSocket);
  close(socketFD);
  freeaddrinfo(reserved);


  return 0;
}
