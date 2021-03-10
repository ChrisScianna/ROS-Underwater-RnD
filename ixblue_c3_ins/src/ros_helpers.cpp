/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018-2020, QinetiQ, Inc.
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

#include "ixblue_c3_ins/ros_helpers.h"

#include <cmath>

#include <ros/ros.h>

#include "auv_interfaces/State.h"
#include "ixblue_c3_ins/NavigationLong.h"


namespace ixblue_c3_ins
{
namespace ros_helpers
{

namespace
{

double wrap_to_pi(double rads)
{
  rads = std::fmod(rads, 2. * M_PI);
  if (rads >= M_PI)
  {
    rads -= 2. * M_PI;
  }
  else if (rads < -M_PI)
  {
    rads += 2. * M_PI;
  }
  return rads;
}

}  // namespace

template<>
ixblue_c3_ins::NavigationLong
to_ros_message(const c3_protocol::nav_long::nav_long_data_t& data)
{
  ixblue_c3_ins::NavigationLong msg;

  msg.user_status = data.user_status;
  msg.algorithm_status = data.algo_status[1];
  msg.algorithm_status <<= 32;
  msg.algorithm_status |= data.algo_status[0];

  msg.heading = data.heading;
  msg.roll = data.roll;
  msg.pitch = data.pitch;
  msg.north_speed = data.north_speed;
  msg.east_speed = data.east_speed;
  msg.vertical_speed = data.vertical_speed;
  msg.latitude = data.latitude *
    c3_protocol::nav_long::nav_long_data_t::latlon_resolution;
  msg.longitude = data.longitude *
    c3_protocol::nav_long::nav_long_data_t::latlon_resolution;
  msg.altitude = data.altitude;
  msg.timestamp = data.timestamp;
  msg.heading_error_stddev = data.heading_err_sd;
  msg.roll_error_stddev = data.roll_err_sd;
  msg.pitch_error_stddev = data.pitch_err_sd;
  msg.north_speed_error_stddev = data.north_speed_err_sd;
  msg.east_speed_error_stddev = data.east_speed_err_sd;
  msg.vertical_speed_error_stddev = data.vertical_speed_err_sd;
  msg.latitude_error_stddev = data.latitude_err_sd;
  msg.longitude_error_stddev = data.longitude_err_sd;
  msg.altitude_error_stddev = data.altitude_err_sd;

  return msg;
}

template<>
auv_interfaces::State
to_ros_message(const c3_protocol::nav_long::nav_long_data_t& data)
{
  auv_interfaces::State msg;

  msg.geolocation.position.latitude =
    data.latitude * c3_protocol::nav_long::nav_long_data_t::latlon_resolution;
  msg.geolocation.position.longitude =
    data.longitude * c3_protocol::nav_long::nav_long_data_t::latlon_resolution;
  msg.geolocation.position.altitude = data.altitude;
  msg.geolocation.covariance[0] = std::pow(data.latitude_err_sd, 2);
  msg.geolocation.covariance[4] = std::pow(data.longitude_err_sd, 2);
  msg.geolocation.covariance[8] = std::pow(data.altitude_err_sd, 2);

  // TODO(hidmic): add terrestrial reference frame
  msg.manoeuvring.pose.mean.orientation.x = data.roll;
  msg.manoeuvring.pose.mean.orientation.y = -data.pitch;  // flip y-axis
  msg.manoeuvring.pose.mean.orientation.z = wrap_to_pi(data.heading);
  msg.manoeuvring.pose.covariance[21] = std::pow(data.roll_err_sd, 2);
  msg.manoeuvring.pose.covariance[28] = std::pow(data.pitch_err_sd, 2);
  msg.manoeuvring.pose.covariance[35] = std::pow(data.heading_err_sd, 2);

  msg.manoeuvring.velocity.mean.linear.x = data.north_speed;
  msg.manoeuvring.velocity.mean.linear.y = data.east_speed;
  msg.manoeuvring.velocity.mean.linear.z = -data.vertical_speed;  // flip z-axis
  msg.manoeuvring.velocity.covariance[0] = std::pow(data.north_speed_err_sd, 2);
  msg.manoeuvring.velocity.covariance[7] = std::pow(data.east_speed_err_sd, 2);
  msg.manoeuvring.velocity.covariance[14] = std::pow(data.vertical_speed_err_sd, 2);

  return msg;
}

}  // namespace ros_helpers
}  // namespace ixblue_c3_ins
