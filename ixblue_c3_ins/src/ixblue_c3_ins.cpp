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
#include <limits>
#include <string>

#include <ros/ros.h>

#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>

#include <auv_interfaces/StateStamped.h>
#include <auv_interfaces/helpers.h>

#include "ixblue_c3_ins/c3_protocol.h"
#include "ixblue_c3_ins/io_helpers.h"
#include "ixblue_c3_ins/ros_helpers.h"

#include "ixblue_c3_ins/NavigationLong.h"

namespace ixblue_c3_ins
{

ixBlueC3InsDriver::ixBlueC3InsDriver()
  : pnh_("~"), diagnostics_updater_(nh_)
{
  diagnostics_updater_.setHardwareID("ins");

  std::string iface_addr;
  /* get operating parameters */
  if (!pnh_.getParam("iface_addr", iface_addr))
  {
    constexpr char default_ipaddress[] = "192.168.36.112";
    ROS_INFO("No configured IPv4 interface address."
             " Using default [%s]", default_ipaddress);
    iface_addr = default_ipaddress;
  }

  int listen_port;
  if (!pnh_.getParam("listen_port", listen_port))
  {
    constexpr int default_port = 2255;
    ROS_WARN("No configured UDP listening port."
             " Using default [%d]", default_port);
    listen_port = default_port;
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

  state_pub_ =
      qna::diagnostic_tools::create_publisher<
        auv_interfaces::StateStamped>(nh_, "state", 1);

  qna::diagnostic_tools::PeriodicMessageStatusParams state_rate_check_params;
  double min_rate, max_rate;
  if (pnh_.getParam("max_rate", max_rate))
  {
    state_rate_check_params.min_acceptable_period(1.0 / max_rate);
  }
  if (pnh_.getParam("min_rate", min_rate))
  {
    state_rate_check_params.max_acceptable_period(1.0 / min_rate);
  }
  state_rate_check_params.abnormal_diagnostic({  // NOLINT(whitespace/braces)
      qna::diagnostic_tools::Diagnostic::ERROR,
      health_monitor::ReportFault::POSE_DATA_STALE
  });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(
      state_pub_.add_check<qna::diagnostic_tools::PeriodicMessageStatus>(
          "rate check", state_rate_check_params));

  double absolute_steady_band;
  double relative_steady_band;
  pnh_.param("absolute_steady_band", absolute_steady_band, 0.);
  pnh_.param("relative_steady_band", relative_steady_band, 0.);
  qna::diagnostic_tools::MessageStagnationCheckParams state_stagnation_check_params;
  state_stagnation_check_params.stagnation_diagnostic({  // NOLINT(whitespace/braces)
      qna::diagnostic_tools::Diagnostic::ERROR,
      health_monitor::ReportFault::POSE_DATA_STAGNATED
  });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(
      state_pub_.add_check<qna::diagnostic_tools::MessageStagnationCheck>(
          "stagnation check",
          [absolute_steady_band, relative_steady_band](
              const auv_interfaces::StateStamped &a,
              const auv_interfaces::StateStamped &b)
          {
            return auv_interfaces::almost_equal(
                a.state, b.state, absolute_steady_band, relative_steady_band);
          }, state_stagnation_check_params));  // NOLINT

  constexpr double inf = std::numeric_limits<double>::infinity();
  double max_roll_angle, max_pitch_angle, max_yaw_angle;
  pnh_.param("max_roll_angle", max_roll_angle, inf);
  pnh_.param("max_pitch_angle", max_pitch_angle, inf);
  pnh_.param("max_yaw_angle", max_yaw_angle, inf);

  orientation_roll_check_ =
      qna::diagnostic_tools::create_health_check<double>(
          "Vehicle orientation - Roll angle check",
          [max_roll_angle](double roll) -> qna::diagnostic_tools::Diagnostic
          {
            using qna::diagnostic_tools::Diagnostic;
            using health_monitor::ReportFault;
            Diagnostic diagnostic{Diagnostic::OK};
            if (std::abs(roll) > max_roll_angle)
            {
              diagnostic.status(Diagnostic::ERROR)
                  .description("Roll angle - NOT OK -> |%f rad| > %f rad",
                               roll, max_roll_angle)
                  .code(ReportFault::POSE_ROLL_THRESHOLD_REACHED);
            }
            else
            {
              diagnostic.description("Roll angle - OK (%f)", roll);
            }
            return diagnostic;
          });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_roll_check_);

  orientation_pitch_check_ =
      qna::diagnostic_tools::create_health_check<double>(
          "Vehicle orientation - Pitch angle check",
          [max_pitch_angle](double pitch) -> qna::diagnostic_tools::Diagnostic
          {
            using qna::diagnostic_tools::Diagnostic;
            using health_monitor::ReportFault;
            Diagnostic diagnostic{Diagnostic::OK};
            if (std::abs(pitch) > max_pitch_angle)
            {
              diagnostic.status(Diagnostic::ERROR)
                  .description("Pitch angle - NOT OK -> |%f rad| > %f rad",
                               pitch, max_pitch_angle)
                  .code(ReportFault::POSE_PITCH_THRESHOLD_REACHED);
            }
            else
            {
              diagnostic.description("Pitch angle - OK (%f)", pitch);
            }
            return diagnostic;
          });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_pitch_check_);

  orientation_yaw_check_ =
      qna::diagnostic_tools::create_health_check<double>(
          "Vehicle orientation - Yaw angle check",
          [max_yaw_angle](double yaw) -> qna::diagnostic_tools::Diagnostic
          {
            using qna::diagnostic_tools::Diagnostic;
            using health_monitor::ReportFault;
            Diagnostic diagnostic{Diagnostic::OK};
            if (std::abs(yaw) > max_yaw_angle)
            {
              diagnostic.status(Diagnostic::ERROR)
                  .description("Yaw angle - NOT OK -> |%f rad| > %f rad",
                               yaw, max_yaw_angle)
                  .code(ReportFault::POSE_HEADING_THRESHOLD_REACHED);
            }
            else
            {
              diagnostic.description("Yaw angle - OK (%f)", yaw);
            }
            return diagnostic;
          });  // NOLINT(whitespace/braces)
  diagnostics_updater_.add(orientation_yaw_check_);
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

  orientation_roll_check_.test(msg.state.manoeuvring.pose.mean.orientation.x);
  orientation_pitch_check_.test(msg.state.manoeuvring.pose.mean.orientation.y);
  orientation_yaw_check_.test(msg.state.manoeuvring.pose.mean.orientation.z);

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

    diagnostics_updater_.update();

    ros::spinOnce();
  }
}

}  // namespace ixblue_c3_ins
