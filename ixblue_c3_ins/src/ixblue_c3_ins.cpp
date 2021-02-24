/*  QinetiQ North America (QNA)
 *  350 Second Avenue
 *  Waltham, MA 02451
 *
 *  Proprietary Licensed Information.
 *  Not to be duplicated, used, or disclosed,
 *  except under terms of the license.
 *
 *  Copyright © 2018 QinetiQ North America  All rights reserved.
 */

/* Overview:
 * This node has provides navigation information to the system. It has
 * two functions. First, it "consumes" UDP data from the ixBlue C3 inerial
 * navigation system (INS). Once the UDP packet data is decoded, the information
 * is published to the other ROS nodes for their handling.
 */

/* To-Do:
 * - hook in publishing code.
 * - add "normal" and "error" logging.
 * - build unit test harness, likely using python & scapy to send simulated INS UDP packets.
 * - unit test on target with simulation.
 * - unit test on target with C3 INS unit.
 * - unit test with all other ROS nodes.
 */

#include "ixblue_c3_ins/ixblue_c3_ins.h"

#include <cmath>
#include <cstdint>
#include <string>

#include <ros/ros.h>

#include "auv_interfaces/StateStamped.h"

#include "ixblue_c3_ins/c3_protocol.h"
#include "ixblue_c3_ins/io_helpers.h"
#include "ixblue_c3_ins/ros_helpers.h"

#include "ixblue_c3_ins/NavigationLong.h"


namespace ixblue_c3_ins
{

ixBlueC3InsDriver::ixBlueC3InsDriver()
  : nh_(), pnh_("~")
{
  constexpr char default_ipaddress[] = "192.168.36.112";

  std::string iface_addr;
  /* get operating parameters */
  if (!pnh_.getParam("iface_addr", iface_addr))
  {
    ROS_INFO("No configured IPv4 interface address."
             " Using default [%s]", default_ipaddress);
    pnh_.param("iface_addr", iface_addr, std::string(default_ipaddress));
  }

  constexpr int default_port = 2255;

  int listen_port;
  if (!pnh_.getParam("listen_port", listen_port))
  {
    ROS_WARN("No configured UDP listening port."
             " Using default [%d]", default_port);
    pnh_.param("listen_port", listen_port, default_port);
  }

  // TODO(hidmic): validate IP address
  if (listen_port < 0 || listen_port > 65535)
  {
    throw std::runtime_error("Invalid UDP socket port");
  }

  fd_ = io_helpers::openUDPSocket(iface_addr.c_str(), (uint16_t)listen_port);
  ROS_INFO("Listening to UDP data on %s:%d", iface_addr.c_str(), listen_port);

  nav_long_pub_ =
    pnh_.advertise<ixblue_c3_ins::NavigationLong>("data", 0);
  state_pub_ = nh_.advertise<auv_interfaces::StateStamped>("state", 0);
}

ixBlueC3InsDriver::~ixBlueC3InsDriver()
{
  close(fd_);
}

void ixBlueC3InsDriver::publish(const c3_protocol::nav_long::nav_long_data_t& data)
{
  using ros_helpers::to_ros_message;

  auv_interfaces::StateStamped msg;
  // TODO(hidmic): set body frame
  msg.header.stamp = ros::Time::now();
  msg.state = to_ros_message<auv_interfaces::State>(data);
  state_pub_.publish(msg);

  nav_long_pub_.publish(to_ros_message<ixblue_c3_ins::NavigationLong>(data));
}

void ixBlueC3InsDriver::spin()
{
  char buffer[1024];

  struct timeval timeout;
  timeout.tv_sec = 0;
  // wait up to 10 ms before processing callbacks
  timeout.tv_usec = 10000;

  while (ros::ok())
  {
    try
    {
      size_t nbytes = io_helpers::readFor(
        fd_, buffer, sizeof(buffer), timeout);
      if (nbytes > 0)
      {
        /* Hand off the message for validation and parsing.
         * The message rate should be low enough that we can
         * do so in this same thread. If that turns out not
         * to be the case, then we'll need fork a new thread.
         * "NAVIGATION LONG' message should be 90 bytes.
         */
        c3_protocol::nav_long nav(buffer, nbytes);
        if (nav.get_parse_success() == 0)
        {
          publish(nav.get_data());
        }
      }
    }
    catch (const std::system_error& e)
    {
      ROS_ERROR("%s (%d)", e.what(), e.code().value());
    }

    ros::spinOnce();
  }
}

}  // namespace ixblue_c3_ins
