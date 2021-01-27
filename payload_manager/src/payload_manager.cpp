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



int raspberrySocket;
void payloadMessageCallback(const payload_manager::PayloadCommand::ConstPtr& msg)
{
  ROS_INFO("Received payload message: %s", msg->command.c_str());
  sockaddr_storage client_addr;
  socklen_t client_addr_size = sizeof(client_addr);

  auto bytes_sent = send(raspberrySocket, msg->command.c_str(), msg->command.length(), 0);
  if (bytes_sent < 1) {
    ROS_WARN("Failed to send messages onto socket");
  }
  ROS_INFO("Finished sending message through socket (%ld bytes)", bytes_sent);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "payload_manager");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/payload_manager/command", 1000, payloadMessageCallback);
  int portNumber = 2250;
  std::string serverAddress = "192.168.24.189";
  if (n.getParam("/payload_manager/port_number", portNumber) == 0)
    ROS_ERROR("Port not found. Using default: [%d]", portNumber);
  else
    ROS_INFO("Using port: [%d]", portNumber);
  
  if (n.getParam("/payload_manager/server_address", serverAddress) == 0)
    ROS_ERROR("server address not found. Using default: [%s]",  serverAddress.c_str());
  else
    ROS_INFO("Using server address: [%s]", serverAddress.c_str());



  ROS_INFO("Started up payload manager.");
  try {
    struct sockaddr_in serv_addr; 
    char buffer[1024] = {0}; 
    if ((raspberrySocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        ROS_ERROR("Socket creation error"); 
        return -1; 
    } 
   
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(portNumber); 
       
    if(inet_pton(AF_INET, serverAddress.c_str(), &serv_addr.sin_addr)<=0)  
    { 
        ROS_ERROR("Invalid address/ Address not supported"); 
        return -1; 
    } 

    bool connectedSuccessfully = false;
    for (int i=0; i<10; i++) {
        if (connect(raspberrySocket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
        { 
            ROS_ERROR("Connection Failed"); 
            ros::Duration(2).sleep();
        } else {
            connectedSuccessfully = true;
            break;
        }
    }
    if (!connectedSuccessfully) {
        ROS_ERROR("Unable to connect to socket");
        return -2;
    }
    ROS_INFO("Connected successfully"); 
    ros::spin();


  } catch(...) {
    ROS_ERROR("Socket connection aborted");

  }
  close(raspberrySocket);


  return 0;
}
