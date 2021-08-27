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

// Original version: Christopher Scianna <Christopher.Scianna@us.QinetiQ.com>

#include "autopilot/autopilot.h"

#include <algorithm>

namespace
{

double radiansToDegrees(double radians) { return (radians * (180.0 / M_PI)); }

double degreesToRadians(double degrees) { return ((degrees / 180.0) * M_PI); }

double wrapAngleTo180(double a)
{
  while (a < -180.0)
  {
    a += 360.0;
  }
  return fmod((a + 180.0), 360.0) - 180.0;
}

// Returns relative angle of B w.r.t A
double relativeAngle(double xb, double yb, double xa, double ya)
{
  if ((xa == xb) && (ya == yb))
  {
    return 0.0;
  }

  double w   = 0.0;
  double sop = 0.0;

  if (xa < xb)
  {
    if (ya == yb)
    {
      return 90.0;
    }
    else
    {
      w = 90.0;
    }
  }
  else if (xa > xb)
  {
    if (ya == yb)
    {
      return -90.0;
    }
    else
    {
      w = 270.0;
    }
  }

  if (ya < yb)
  {
    if (xa == xb)
    {
      return 0.0;
    }
    if (xb > xa)
    {
      sop = -1.0;
    }
    else
    {
      sop =  1.0;
    }
  }
  else if (yb < ya)
  {
    if (xa == xb)
    {
      return 180.0;
    }
    if (xb > xa)
    {
      sop =  1.0;
    }
    else
    {
      sop = -1.0;
    }
  }

  double ydiff = fabs(yb - ya);
  double xdiff = fabs(xb - xa);
  double avalPI = atan(ydiff / xdiff);
  double avalDG = radiansToDegrees(avalPI);
  double relAngle = (avalDG * sop) + w;

  return wrapAngleTo180(relAngle);
}

}  // namespace

AutoPilotNode::AutoPilotNode() : pnh_("~")
{
  thruster_control_pub_ = nh_.advertise<thruster_control::SetRPM>("input/autopilot/set_rpm", 1);
  fin_control_pub_ = nh_.advertise<fin_control::SetAngles>("input/autopilot/set_angles", 1);

  control_loop_rate_ = pnh_.param<double>("control_loop_rate", 25.);  // Hz
  minimal_speed_ = pnh_.param<double>("minimal_vehicle_speed", 2.0);  // knots
  rpms_per_knot_ = pnh_.param<double>("rpm_per_knot", 100.0);  // rpm

  const double roll_pgain = pnh_.param<double>("roll_p", 1.0);
  const double roll_igain = pnh_.param<double>("roll_i", 0.0);
  const double roll_dgain = pnh_.param<double>("roll_d", 0.0);
  const double roll_imax =  pnh_.param<double>("roll_imax", 0.0);
  const double roll_imin = pnh_.param<double>("roll_imin", 0.0);
  roll_pid_controller_.initPid(roll_pgain, roll_igain, roll_dgain, roll_imax, roll_imin);

  const double pitch_pgain = pnh_.param<double>("pitch_p", 1.0);
  const double pitch_igain = pnh_.param<double>("pitch_i", 0.0);
  const double pitch_dgain = pnh_.param<double>("pitch_d", 0.0);
  const double pitch_imax = pnh_.param<double>("pitch_imax", 0.0);
  const double pitch_imin = pnh_.param<double>("pitch_imin", 0.0);
  pitch_pid_controller_.initPid(pitch_pgain, pitch_igain, pitch_dgain, pitch_imax, pitch_imin);

  const double yaw_pgain = pnh_.param<double>("yaw_p", 1.0);
  const double yaw_igain = pnh_.param<double>("yaw_i", 0.0);
  const double yaw_dgain = pnh_.param<double>("yaw_d", 0.0);
  const double yaw_imax = pnh_.param<double>("yaw_imax", 0.0);
  const double yaw_imin = pnh_.param<double>("yaw_imin", 0.0);
  yaw_pid_controller_.initPid(yaw_pgain, yaw_igain, yaw_dgain, yaw_imax, yaw_imin);

  const double depth_pgain = pnh_.param<double>("depth_p", 1.0);
  const double depth_igain = pnh_.param<double>("depth_i", 0.0);
  const double depth_dgain = pnh_.param<double>("depth_d", 0.0);
  const double depth_imax = pnh_.param<double>("depth_imax", 0.0);
  const double depth_imin = pnh_.param<double>("depth_imin", 0.0);
  depth_pid_controller_.initPid(depth_pgain, depth_igain, depth_dgain, depth_imax, depth_imin);
  max_depth_command_ = pnh_.param<double>("max_depth_command", 10.0);

  const double altitude_pgain = pnh_.param<double>("altitude_p", 1.0);
  const double altitude_igain = pnh_.param<double>("altitude_i", 0.0);
  const double altitude_dgain = pnh_.param<double>("altitude_d", 0.0);
  const double altitude_imax = pnh_.param<double>("altitude_imax", 0.0);
  const double altitude_imin = pnh_.param<double>("altitude_imin", 0.0);
  altitude_pid_controller_.initPid(altitude_pgain, altitude_igain, altitude_dgain, altitude_imax, altitude_imin);
  max_altitude_command_ = pnh_.param<double>("max_altitude_command", 25.0);

  max_ctrl_fin_angle_in_radians_ =
      nh_.param<double>("/fin_control/max_ctrl_fin_angle", degreesToRadians(10.0));

  active_setpoints_ = Setpoint::Roll | Setpoint::Pitch | Setpoint::Yaw;
  desired_roll_ = pnh_.param<double>("desired_roll", 0.0);
  desired_pitch_ = pnh_.param<double>("desired_pitch", 0.0);
  desired_yaw_ = pnh_.param<double>("desired_yaw", 0.0);
  if (pnh_.param<bool>("depth_control_enabled", false))
  {
    active_setpoints_ |= Setpoint::Depth;
  }
  desired_depth_ = pnh_.param<double>("desired_depth", 0.0);

  if (pnh_.param<bool>("speed_control_enabled", false))
  {
    active_setpoints_ |= Setpoint::Speed;
  }
  desired_speed_ = pnh_.param<double>("desired_speed", 0.0);
  if (pnh_.param<bool>("fixed_rudder", true))
  {
    active_setpoints_ |= Setpoint::Rudder;
  }
  desired_rudder_ = pnh_.param<double>("desired_rudder", 0.0);

  allow_reverse_thrust_ =
      pnh_.param<bool>("allow_reverse_thruster_autopilot", false);
  thruster_enabled_ = pnh_.param<bool>("thruster_enabled", false);

  state_up_to_date_ = false;
  state_sub_ = nh_.subscribe("state", 1, &AutoPilotNode::stateCallback, this);

  mission_heartbeat_sub_ = nh_.subscribe(
      "/mngr/report_heartbeat", 10,
      &AutoPilotNode::missionHeartbeatCallback, this);

  mission_status_sub_ = nh_.subscribe(
      "/mngr/report_mission_execute_state", 1,
      &AutoPilotNode::missionStatusCallback, this);

  attitude_servo_sub_ = nh_.subscribe(
      "/mngr/attitude_servo", 1,
      &AutoPilotNode::attitudeServoCallback, this);

  altitude_heading_sub_ = nh_.subscribe(
      "/mngr/altitude_heading", 1,
      &AutoPilotNode::altitudeHeadingCallback, this);

  depth_heading_sub_ = nh_.subscribe(
      "/mngr/depth_heading", 1,
      &AutoPilotNode::depthHeadingCallback, this);

  fixed_rudder_sub_ = nh_.subscribe(
      "/mngr/fixed_rudder", 1,
      &AutoPilotNode::fixedRudderCallback, this);

  waypoint_sub_ = nh_.subscribe(
      "/mngr/waypoint", 10,
      &AutoPilotNode::waypointCallback, this);

  const double heartbeat_rate = nh_.param<double>("/mngr/heartbeat_rate", 2.0);
  const double hearbeat_timeout = 3.0 * heartbeat_rate;
  last_heartbeat_stamp_ = ros::Time::now();
  mission_heartbeat_timer_ = nh_.createTimer(
      ros::Duration(hearbeat_timeout),
      &AutoPilotNode::missionHeartbeatTimeout, this);
}

void AutoPilotNode::missionHeartbeatTimeout(const ros::TimerEvent& ev)
{
  if (last_heartbeat_stamp_ < ev.last_real)  // no heartbeat received in the past period
  {
    active_setpoints_ = Setpoint::Pitch;
    desired_pitch_ = -radiansToDegrees(max_ctrl_fin_angle_in_radians_);
    ROS_INFO_THROTTLE(5.0, "Mission control is down");
  }
}

void AutoPilotNode::missionHeartbeatCallback(const mission_control::ReportHeartbeat& msg)
{
  last_heartbeat_stamp_ = msg.header.stamp;
  ROS_DEBUG_THROTTLE(5.0, "Mission control is up");
}

void AutoPilotNode::missionStatusCallback(const mission_control::ReportExecuteMissionState& msg)
{
  if (msg.execute_mission_state == mission_control::ReportExecuteMissionState::COMPLETE)
  {
    active_setpoints_ = Setpoint::Pitch;
    desired_pitch_ = -radiansToDegrees(max_ctrl_fin_angle_in_radians_);
  }
}

void AutoPilotNode::stateCallback(const auv_interfaces::StateStamped& msg)
{
  current_depth_ = msg.state.manoeuvring.pose.mean.position.z;
  current_roll_ = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.x);
  current_pitch_ = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.y);
  current_yaw_ = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.z);
  geodesy::fromMsg(msg.state.geolocation.position, current_position_);
  state_up_to_date_ = true;
}

namespace
{

double saturate(double value, double limit)
{
  return std::copysign(std::min(std::abs(value), limit), value);
}

}  // namespace

void AutoPilotNode::mixActuators(double roll, double pitch, double yaw, double thrust_rpms)
{
  const double current_roll_in_radians = degreesToRadians(current_roll_);
  const double rotated_yaw = yaw * cos(current_roll_in_radians) -
                             pitch * sin(current_roll_in_radians);
  const double rotated_pitch = yaw * sin(current_roll_in_radians) +
                               pitch * cos(current_roll_in_radians);
  const double fin_angles[] = {                               //  NOLINT
      degreesToRadians(rotated_pitch - rotated_yaw + roll),   //  Fin 1 bottom stbd
      degreesToRadians(rotated_pitch + rotated_yaw + roll),   //  Fin 2 top stb
      degreesToRadians(-rotated_pitch - rotated_yaw + roll),  //  Fin 3 bottom port
      degreesToRadians(-rotated_pitch + rotated_yaw + roll),  //  Fin 4 top port
  };
  // Scale down and saturate fin angles if necessary
  const double absolute_fin_angles[] = {                              //  NOLINT
      fabs(fin_angles[0]), fabs(fin_angles[1]), fabs(fin_angles[2]),  //  NOLINT
      fabs(fin_angles[3]), max_ctrl_fin_angle_in_radians_};           // NOLINT
  const double max_angle_in_radians =
      *std::max_element(std::begin(absolute_fin_angles), std::end(absolute_fin_angles));
  const double scale = max_ctrl_fin_angle_in_radians_ / max_angle_in_radians;

  fin_control::SetAngles fin_control_message;
  for (int i = 0; i < 4; ++i)
  {
    fin_control_message.fin_angle_in_radians[i] =
        saturate(scale * fin_angles[i], max_ctrl_fin_angle_in_radians_);
  }

  fin_control_pub_.publish(fin_control_message);

  if (thruster_enabled_)
  {
    thruster_control::SetRPM thruster_control_message;
    thruster_control_message.commanded_rpms = thrust_rpms;
    thruster_control_pub_.publish(thruster_control_message);
  }
}

void AutoPilotNode::attitudeServoCallback(const mission_control::AttitudeServo& msg)
{
  ROS_INFO("Attitude servo command received!");
  active_setpoints_ = {};

  if (msg.ena_mask & mission_control::AttitudeServo::ROLL_ENA)
  {
    desired_roll_ = radiansToDegrees(msg.roll);
    active_setpoints_ |= Setpoint::Roll;
    ROS_INFO("Roll angle: %f deg", desired_roll_);
  }

  if (msg.ena_mask & mission_control::AttitudeServo::PITCH_ENA)
  {
    desired_pitch_ = radiansToDegrees(msg.pitch);
    active_setpoints_ |= Setpoint::Pitch;
    ROS_INFO("Pitch angle: %f deg", desired_pitch_);
  }

  if (msg.ena_mask & mission_control::AttitudeServo::YAW_ENA)
  {
    // Yaw setpoint in attitude servo is really a fixed rudder
    // NOTE(hidmic): this is strange, but I'll keep it as-is
    desired_rudder_ = radiansToDegrees(msg.yaw);
    active_setpoints_ |= Setpoint::Rudder;
    ROS_INFO("Yaw angle: %f deg", desired_rudder_);
  }

  if (msg.ena_mask & mission_control::AttitudeServo::SPEED_KNOTS_ENA)
  {
    active_setpoints_ |= Setpoint::Speed;
    desired_speed_ = msg.speed_knots;
    ROS_INFO("Speed: %f knots", desired_speed_);
  }
}

void AutoPilotNode::depthHeadingCallback(const mission_control::DepthHeading& msg)
{
  ROS_INFO("Depth & heading command received!");

  desired_roll_ = 0.;
  active_setpoints_ = Setpoint::Roll;
  ROS_INFO("Roll angle: %f deg", desired_roll_);

  if (msg.ena_mask & mission_control::DepthHeading::DEPTH_ENA)
  {
    desired_depth_ = msg.depth;
    active_setpoints_ |= Setpoint::Depth;
    ROS_INFO("Depth: %f m", desired_depth_);
  }

  if (msg.ena_mask & mission_control::DepthHeading::HEADING_ENA)
  {
    desired_yaw_ = radiansToDegrees(msg.heading);
    active_setpoints_ |= Setpoint::Yaw;
    ROS_INFO("Heading angle: %f deg", desired_yaw_);
  }

  if (msg.ena_mask & mission_control::DepthHeading::SPEED_KNOTS_ENA)
  {
    active_setpoints_ |= Setpoint::Speed;
    desired_speed_ = msg.speed_knots;
    ROS_INFO("Speed: %f knots", desired_speed_);
  }
}

void AutoPilotNode::altitudeHeadingCallback(const mission_control::AltitudeHeading& msg)
{
  ROS_INFO("Altitude & heading command received!");

  desired_roll_ = 0.;
  active_setpoints_ = Setpoint::Roll;
  ROS_INFO("Roll angle: %f deg", desired_roll_);

  if (msg.ena_mask & mission_control::AltitudeHeading::ALTITUDE_ENA)
  {
    active_setpoints_ |= Setpoint::Altitude;
    desired_position_.altitude = msg.altitude;
    ROS_INFO("Altitude: %f m", desired_position_.altitude);
  }

  if (msg.ena_mask & mission_control::AltitudeHeading::HEADING_ENA)
  {
    active_setpoints_ |= Setpoint::Yaw;
    desired_yaw_ = radiansToDegrees(msg.heading);
    ROS_INFO("Heading: %f deg", desired_yaw_);
  }

  if (msg.ena_mask & mission_control::AltitudeHeading::SPEED_KNOTS_ENA)
  {
    active_setpoints_ |= Setpoint::Speed;
    desired_speed_ = msg.speed_knots;
    ROS_INFO("Speed: %f knots", desired_speed_);
  }
}

void AutoPilotNode::fixedRudderCallback(const mission_control::FixedRudder& msg)
{
  ROS_INFO("Fixed rudder command received!");

  desired_roll_ = 0.;
  active_setpoints_ = Setpoint::Roll;
  ROS_INFO("Roll angle: %f deg", desired_roll_);

  if (msg.ena_mask & mission_control::FixedRudder::DEPTH_ENA)
  {
    active_setpoints_ |= Setpoint::Depth;
    desired_depth_ = msg.depth;
    ROS_INFO("Depth: %f m", desired_depth_);
  }

  if (msg.ena_mask & mission_control::FixedRudder::RUDDER_ENA)
  {
    active_setpoints_ |= Setpoint::Rudder;
    desired_rudder_ = radiansToDegrees(msg.rudder);
    ROS_INFO("Rudder angle: %f deg", desired_rudder_);
  }

  if (msg.ena_mask & mission_control::FixedRudder::SPEED_KNOTS_ENA)
  {
    active_setpoints_ |= Setpoint::Speed;
    desired_speed_ = msg.speed_knots;
    ROS_INFO("Speed: %f knots", desired_speed_);
  }
}

void AutoPilotNode::waypointCallback(const mission_control::Waypoint& msg)
{
  ROS_INFO("Waypoint received!");

  desired_roll_ = 0.;
  active_setpoints_ = Setpoint::Roll;
  ROS_INFO("Roll angle: %f deg", desired_roll_);

  if ((msg.ena_mask & mission_control::Waypoint::LAT_ENA) &&
      (msg.ena_mask & mission_control::Waypoint::LONG_ENA))
  {
    active_setpoints_ |= Setpoint::Position;
    geodesy::fromMsg(geodesy::toMsg(msg.latitude, msg.longitude), desired_position_);
    ROS_INFO("UTM Easting: %f m", desired_position_.easting);
    ROS_INFO("UTM Northing: %f m", desired_position_.northing);
  }

  if (msg.ena_mask & mission_control::Waypoint::ALTITUDE_ENA)
  {
    active_setpoints_ |= Setpoint::Altitude;
    // clamp altitude command absolute value
    desired_position_.altitude = msg.altitude;
    ROS_INFO("Altitude: %f m", desired_position_.altitude);
  }
  else if (msg.ena_mask & mission_control::Waypoint::DEPTH_ENA)
  {
    active_setpoints_ |= Setpoint::Depth;
    desired_depth_ = msg.depth;
    ROS_INFO("Depth: %f m", desired_depth_);
  }

  if (msg.ena_mask & mission_control::Waypoint::SPEED_KNOTS_ENA)
  {
    active_setpoints_ |= Setpoint::Speed;
    desired_speed_ = msg.speed_knots;
    ROS_INFO("Speed: %f knots", desired_speed_);
  }
}

void AutoPilotNode::spin()
{
  ros::Rate r(control_loop_rate_);
  while (ros::ok())
  {
      double roll_command = 0.;  // straight
      double pitch_command = 0.;  // straight
      double yaw_command = 0.;  // straight
      double thrust_command = 0.;  // coast

      if (state_up_to_date_)
      {
        if (active_setpoints_ & Setpoint::Roll)
        {
          roll_command = roll_pid_controller_.updatePid(
              wrapAngleTo180(current_roll_ - desired_roll_),
              r.expectedCycleTime());
        }

        if (active_setpoints_ & (Setpoint::Pitch | Setpoint::Depth | Setpoint::Altitude))
        {
          if (active_setpoints_ & Setpoint::Depth)
          {
            if (fabs(desired_depth_) > max_depth_command_)
            {
              ROS_WARN("Autopilot cannot command to a depth |d| > %f m", max_depth_command_);
              desired_depth_ = copysign(max_depth_command_, desired_depth_);
              ROS_INFO("Capping depth to %f m", desired_depth_);
            }
            // output of depth pid will be input to pitch pid
            desired_pitch_ = depth_pid_controller_.updatePid(current_depth_ - desired_depth_,
                                                             r.expectedCycleTime());
          }
          else if (active_setpoints_ & Setpoint::Altitude)
          {
            if (fabs(desired_position_.altitude) > max_altitude_command_)
            {
              ROS_WARN("Autopilot cannot command to an altitude |a| > %f m", max_altitude_command_);
              desired_position_.altitude = copysign(max_altitude_command_, desired_position_.altitude);
              ROS_INFO("Capping altitude to %f m", desired_position_.altitude);
            }
            // output of altitude pid will be input to pitch pid
            desired_pitch_ = altitude_pid_controller_.updatePid(
                current_position_.altitude - desired_position_.altitude,
                r.expectedCycleTime());
          }

          pitch_command = pitch_pid_controller_.updatePid(
            wrapAngleTo180(current_pitch_ - desired_pitch_),
            r.expectedCycleTime());
        }

        if (active_setpoints_ & Setpoint::Rudder)  // has precedence
        {
          yaw_command = desired_rudder_;
        }
        else if (active_setpoints_ & (Setpoint::Yaw | Setpoint::Position))
        {
          if (active_setpoints_ & Setpoint::Position)
          {
            desired_yaw_ = relativeAngle(
                desired_position_.easting, desired_position_.northing,
                current_position_.easting, current_position_.northing);
          }

          yaw_command = yaw_pid_controller_.updatePid(
              wrapAngleTo180(current_yaw_ - desired_yaw_),
              r.expectedCycleTime());
        }

        if (active_setpoints_ & Setpoint::Speed)
        {
          if (desired_speed_ < 0. && !allow_reverse_thrust_)
          {
            ROS_WARN("Autopilot cannot command a reverse thrust!");
            ROS_INFO("Forcing speed to 0 knots");
            desired_speed_ = 0.;
          }
          if (desired_speed_ != 0. && fabs(desired_speed_) < minimal_speed_)
          {
            ROS_WARN("Autopilot cannot command a speed |s| < %f", minimal_speed_);
            desired_speed_ = copysign(minimal_speed_, desired_speed_);
            ROS_INFO("Bumping speed to %f knots", desired_speed_);
          }
          thrust_command = desired_speed_ * rpms_per_knot_;
        }
      }

      mixActuators(roll_command, pitch_command, yaw_command, thrust_command);

    ros::spinOnce();
    r.sleep();
  }
}
