#include "ros/ros.h"
#include "std_msgs/String.h"
#include "payload_manager/PayloadCommand.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<payload_manager::PayloadCommand>("/payload_manager/command", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  
  auto &ipAddress = "0.0.0.0";
  auto &portNum   = "2250";

  addrinfo hints, *reservedAddrInfo;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags    = AI_PASSIVE;

  int getAddressInfoFlag = getaddrinfo(ipAddress, portNum, &hints, &reservedAddrInfo);
  if (getAddressInfoFlag != 0) {
      ROS_ERROR("get address info error: %s", gai_strerror(getAddressInfoFlag));
      return -2;
  }

  if (reservedAddrInfo == NULL) {
      ROS_ERROR("No addresses found");
      return -3;
  }

  ROS_INFO("Found address for socket");
  
  // socket() call creates a new socket and returns it's descriptor
  int socketFD = socket(reservedAddrInfo->ai_family, reservedAddrInfo->ai_socktype, reservedAddrInfo->ai_protocol);
  if (socketFD == -1) {
      ROS_ERROR("Error while creating socket");
      return -4;
  }

  //set timeout
  struct timeval timeValue;
  timeValue.tv_sec = 1;
  timeValue.tv_usec = 0;
  setsockopt(socketFD, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeValue, sizeof timeValue);

  ROS_INFO("Socket created. Attempting to connect");
  int connectR = connect(socketFD, reservedAddrInfo->ai_addr, reservedAddrInfo->ai_addrlen);
  if (connectR == -1) {
      close(socketFD);
      ROS_ERROR("Error while connecting socket");
      return -5;
  }
  ROS_INFO("Socket connected");


  while (ros::ok())
  {
    payload_manager::PayloadCommand msg;

    std::stringstream ss;
    ss << "message received count: " << count;
    msg.command = ss.str();

    ROS_INFO("%s", msg.command.c_str());

    ros::spinOnce();

    loop_rate.sleep();

    std::string reply(150, ' ');
    auto bytes_recv = recv(socketFD, &reply.front(), reply.size(), 0);
    if (bytes_recv == -1) {
        ROS_ERROR("Error while receiving bytes");
        continue;
    }
 
    ROS_INFO("Client received: %s", reply.c_str());
    ++count;
  }

  close(socketFD);
  freeaddrinfo(reservedAddrInfo);

  return 0;
}
