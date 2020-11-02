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

AutoPilotNode::AutoPilotNode(ros::NodeHandle& node_handle) : nh(node_handle)
{
  auto_pilot_in_control = false;
  mission_mode = false;
  autopilotEnabled = true;
  mmIsAlive = false;
  currentRoll = 0;
  currentPitch = 0;
  currentYaw = 0;
  currentDepth = 0;

  thruster_pub = nh.advertise<thruster_control::SetRPM>("thruster_control/set_rpm", 1);
  fins_control_pub = nh.advertise<fin_control::SetAngles>("fin_control/set_angles", 1);
  fins_enable_reporting_pub =
      nh.advertise<fin_control::EnableReportAngles>("/fin_control/enable_report_angles", 1);
  auto_pilot_in_control_pub =
      nh.advertise<autopilot::AutoPilotInControl>("autopilot/auto_pilot_in_control", 1);

  double mgr_report_hb_rate;
  nh.param<double>("/mission_manager_node/report_heart_beat_rate", mgr_report_hb_rate, 2.0);
  ROS_INFO("mm hb rate: [%f]", mgr_report_hb_rate);
  mmTimeout = 3.0 * mgr_report_hb_rate;

  nh.param<int>("/autopilot_node/control_loop_rate", control_loop_rate, 25);     // Hz
  nh.param<double>("/autopilot_node/minimal_vehicle_speed", minimalspeed, 2.0);  // knots
  nh.param<double>("/autopilot_node/rpm_per_knot", rpmPerKnot, 100.0);           // rpm
  nh.param<bool>("/autopilot_node/speed_control_enabled", speedControlEnabled, false);

  nh.param<double>("/autopilot_node/roll_p", roll_pgain, 1.0);
  nh.param<double>("/autopilot_node/roll_i", roll_igain, 0.0);
  nh.param<double>("/autopilot_node/roll_imax", roll_imax, 0.0);
  nh.param<double>("/autopilot_node/roll_imin", roll_imin, 0.0);
  nh.param<double>("/autopilot_node/roll_d", roll_dgain, 0.0);

  nh.param<double>("/autopilot_node/pitch_p", pitch_pgain, 1.0);
  nh.param<double>("/autopilot_node/pitch_i", pitch_igain, 0.0);
  nh.param<double>("/autopilot_node/pitch_imax", pitch_imax, 0.0);
  nh.param<double>("/autopilot_node/pitch_imin", pitch_imin, 0.0);
  nh.param<double>("/autopilot_node/pitch_d", pitch_dgain, 0.0);

  nh.param<double>("/autopilot_node/yaw_p", yaw_pgain, 1.0);
  nh.param<double>("/autopilot_node/yaw_i", yaw_igain, 0.0);
  nh.param<double>("/autopilot_node/yaw_imax", yaw_imax, 0.0);
  nh.param<double>("/autopilot_node/yaw_imin", yaw_imin, 0.0);
  nh.param<double>("/autopilot_node/yaw_d", yaw_dgain, 0.0);

  nh.param<double>("/autopilot_node/depth_p", depth_pgain, 1.0);
  nh.param<double>("/autopilot_node/depth_i", depth_igain, 0.0);
  nh.param<double>("/autopilot_node/depth_imax", depth_imax, 0.0);
  nh.param<double>("/autopilot_node/depth_imin", depth_imin, 0.0);
  nh.param<double>("/autopilot_node/depth_d", depth_dgain, 0.0);

  nh.param<double>("/autopilot_node/max_ctrl_plane_angle", maxCtrlPlaneAngle, 10.0);  // degrees
  nh.param<double>("/autopilot_node/max_depth_command", maxDepthCommand, 10.0);       // degrees

  roll_pid_controller.initPid(roll_pgain, roll_igain, roll_dgain, roll_imax, roll_imin);
  pitch_pid_controller.initPid(pitch_pgain, pitch_igain, pitch_dgain, pitch_imax, pitch_imin);
  yaw_pid_controller.initPid(yaw_pgain, yaw_igain, yaw_dgain, yaw_imax, yaw_imin);
  depth_pid_controller.initPid(depth_pgain, depth_igain, depth_dgain, depth_imax, depth_imin);

  nh.param<double>("/autopilot_node/desired_roll", desiredRoll, 0.0);
  nh.param<double>("/autopilot_node/desired_pitch", desiredPitch, 0.0);
  nh.param<double>("/autopilot_node/desired_yaw", desiredYaw, 0.0);
  nh.param<bool>("/autopilot_node/depth_control_enabled", depthControl, false);
  nh.param<double>("/autopilot_node/desired_depth", desiredDepth, 0.0);
  nh.param<bool>("/autopilot_node/fixed_rudder", fixedRudder, true);
  nh.param<double>("/autopilot_node/desired_rudder", desiredRudder, 0.0);
  nh.param<double>("/autopilot_node/desired_speed", desiredSpeed, 0.0);
  nh.param<bool>("/autopilot_node/allow_reverse_thruster_autopilot", allowReverseThrusterAutopilot,
                 false);
  nh.param<bool>("/autopilot_node/thruster_enabled", thrusterEnabled, false);
  nh.param<double>("/autopilot_node/max_allowed_thruster_rpm", maxAllowedThrusterRpm, 0);

  jaus_ros_sub = nh.subscribe("/jaus_ros_bridge/activate_manual_control", 1,
                              &AutoPilotNode::HandleActivateManualControl, this);
  correctedData_sub_ =
      nh.subscribe("pose/corrected_data", 1, &AutoPilotNode::correctedDataCallback, this);
  missionMgrHeartBeat_sub_ =
      nh.subscribe("/mngr/report_heartbeat", 10, &AutoPilotNode::missionMgrHbCallback, this);
  missionStatus_sub_ = nh.subscribe("/mngr/report_mission_execute_state", 1,
                                    &AutoPilotNode::missionStatusCallback, this);
  setAttitudeBehavior_sub_ =
      nh.subscribe("/mngr/attitude_servo", 1, &AutoPilotNode::attitudeServoCallback, this);
  setDepthHeadingBehavior_sub_ =
      nh.subscribe("/mngr/depth_heading", 1, &AutoPilotNode::depthHeadingCallback, this);
  setFixedRudderBehavior_sub_ =
      nh.subscribe("/mngr/fixed_rudder", 1, &AutoPilotNode::fixedRudderCallback, this);

  missionMgrHeartbeatTimer =
      nh.createTimer(ros::Duration(mmTimeout), &AutoPilotNode::missionMgrHeartbeatTimeout, this);
}

void AutoPilotNode::Start()
{
  assert(!m_thread);
  autopilotEnabled = true;
  m_thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&AutoPilotNode::workerFunc, this)));
}

void AutoPilotNode::Stop()
{
  assert(m_thread);
  autopilotEnabled = false;
  m_thread->join();
  fin_control::EnableReportAngles enableReportAnglesMsg;
  enableReportAnglesMsg.enable_report_angles = true;
  fins_enable_reporting_pub.publish(enableReportAnglesMsg);
}

void AutoPilotNode::missionMgrHeartbeatTimeout(const ros::TimerEvent& timer)
{
  boost::mutex::scoped_lock lock(mm_hb_mutex);

  thruster_control::SetRPM setrpm;
  ROS_ERROR("Mission Manager DIED!!!");
  mmIsAlive = false;

  missionMgrHeartbeatTimer.stop();

  if (auto_pilot_in_control)  // if mission mgr dies and we are not teleoperating shut down
                              // thruster.
  {
    setrpm.commanded_rpms = 0.0;
    thruster_pub.publish(setrpm);
  }
}

double AutoPilotNode::radiansToDegrees(double radians) { return (radians * (180.0 / M_PI)); }

double AutoPilotNode::degreesToRadians(double degrees) { return ((degrees / 180.0) * M_PI); }

void AutoPilotNode::correctedDataCallback(const pose_estimator::CorrectedData& data)
{
  boost::mutex::scoped_lock lock(m_mutex);

  currentDepth = data.depth;
  currentRoll = radiansToDegrees(data.rpy_ang.x);
  currentPitch = radiansToDegrees(data.rpy_ang.y);
  currentYaw = radiansToDegrees(data.rpy_ang.z);
  currentYaw = fmod((currentYaw + 180.0), 360.0) - 180.0;  // translate from INS 0-360 to +/-180
}

void AutoPilotNode::missionStatusCallback(const mission_manager::ReportExecuteMissionState& data)
{
  if (data.execute_mission_state == 2)  // Mission Complete or Abort
  {
    boost::mutex::scoped_lock lock(m_mutex);
    mission_mode = true;

    desiredRoll = 0;

    // surface
    desiredPitch = -maxCtrlPlaneAngle;
    depthControl = false;

    desiredRudder = 0;  // straight
    fixedRudder = true;

    desiredSpeed = 0;  // coast
  }
}

void AutoPilotNode::HandleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data)
{
  boost::mutex::scoped_lock lock(m_behavior_mutex);
  if (data.activate_manual_control)
  {
    // ROS_INFO("Jaus says its in control.");
    auto_pilot_in_control = false;
    mission_mode = false;
    // clear the following so we do not take off after manual control is released.
    desiredSpeed = 0;
    desiredRoll = 0;
    desiredYaw = 0;
    desiredPitch = 0;
    desiredDepth = 0;
    desiredRudder = 0;
    fixedRudder = true;
    depthControl = false;
  }
  else
  {
    // ROS_INFO("Jaus says its not in control.");
    auto_pilot_in_control = true;
  }
}

void AutoPilotNode::mixactuators(double roll, double pitch, double yaw)
{
  // da = roll, ds = pitch, dr = yaw

  yaw = fmod((yaw + 180.0), 360.0) - 180.0;  // translate from INS 0-360 to +/-180
  double d1;                                 // Fin 1 bottom stbd
  double d2;                                 // Fin 2 top stb
  double d3;                                 // Fin 3 bottom port
  double d4;                                 // Fin 4 top port
  // ROS_INFO("ds: [%f] dr: [%f] da: [%f]",ds,dr,da);

  double rollRadians = degreesToRadians(currentRoll);
  double newYaw = yaw * cos(rollRadians) - pitch * sin(rollRadians);
  double newPitch = yaw * sin(rollRadians) + pitch * cos(rollRadians);
  yaw = newYaw;
  pitch = newPitch;

  d1 = pitch - yaw + roll;
  d2 = pitch + yaw + roll;
  d3 = -pitch - yaw + roll;
  d4 = -pitch + yaw + roll;

  double maxOptions[] = {fabs(d1), fabs(d2), fabs(d3), fabs(d4), maxCtrlPlaneAngle};
  double maxAngle = *std::max_element(maxOptions, maxOptions + 5);
  d1 = d1 * maxCtrlPlaneAngle / maxAngle;
  d2 = d2 * maxCtrlPlaneAngle / maxAngle;
  d3 = d3 * maxCtrlPlaneAngle / maxAngle;
  d4 = d4 * maxCtrlPlaneAngle / maxAngle;

  fin_control::SetAngles setAnglesMsg;

  setAnglesMsg.f1_angle_in_radians = degreesToRadians(d1);
  setAnglesMsg.f2_angle_in_radians = degreesToRadians(d2);
  setAnglesMsg.f3_angle_in_radians = degreesToRadians(d3);
  setAnglesMsg.f4_angle_in_radians = degreesToRadians(d4);
  // ROS_INFO("F1,F2,F3,F4: [%f,%f,%f,%f]",d1,d2,d3,d4);
  fins_control_pub.publish(setAnglesMsg);
}

void AutoPilotNode::missionMgrHbCallback(const mission_manager::ReportHeartbeat& msg)
{
  boost::mutex::scoped_lock lock(mm_hb_mutex);

  mmIsAlive = true;

  missionMgrHeartbeatTimer.stop();
  missionMgrHeartbeatTimer.start();
}

void AutoPilotNode::attitudeServoCallback(const mission_manager::AttitudeServo& msg)
{
  boost::mutex::scoped_lock lock(m_behavior_mutex);
  mission_mode = true;
  fixedRudder = true;  // this equate to fixed rudder with roll and pitch commands.

  ROS_INFO("Angle for roll: [%f]", msg.roll);
  desiredRoll = msg.roll;

  ROS_INFO("Angle for pitch: [%f]", msg.pitch);
  desiredPitch = msg.pitch;
  depthControl = false;  // disable depth control setting pitch instead

  ROS_INFO("Angle for yaw: [%f]", msg.yaw);
  desiredRudder = msg.yaw;  // yaw in attitude servo is really a fixedrudder

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);

  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::depthHeadingCallback(const mission_manager::DepthHeading& msg)
{
  boost::mutex::scoped_lock lock(m_behavior_mutex);
  mission_mode = true;
  fixedRudder = false;

  ROS_INFO("Depth Heading Message Received setting Angle for roll to 0.");
  desiredRoll = 0;

  ROS_INFO("depth: [%f]", msg.depth);
  desiredDepth = msg.depth;
  depthControl = true;  // enabling depth control

  ROS_INFO("Angle for yaw: [%f]", msg.heading);
  desiredYaw = msg.heading;

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::fixedRudderCallback(const mission_manager::FixedRudder& msg)
{
  boost::mutex::scoped_lock lock(m_behavior_mutex);
  mission_mode = true;
  fixedRudder = true;
  ROS_INFO("Fixed Rudder Message Received setting Angle for roll to 0.");
  desiredRoll = 0;

  ROS_INFO("depth: [%f]", msg.depth);
  desiredDepth = msg.depth;
  depthControl = true;  // enabling depth control

  ROS_INFO("Angle for rudder: [%f]", msg.rudder);
  desiredRudder = msg.rudder;

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::workerFunc()
{
  double rollcmdpos = 0;
  double pitchcmdpos = 0;
  double yawcmdpos = 0;
  double yawError = 0;
  double yawDiff = 0;
  double desired_pitch = 0;
  double speedcmd = 0;
  thruster_control::SetRPM setrpm;
  ros::Rate r(control_loop_rate);
  autopilot::AutoPilotInControl incontrol;
  fin_control::EnableReportAngles enableReportAnglesMsg;
  bool lastcontrol;

  lastcontrol = auto_pilot_in_control;
  int i = 0;
  while (autopilotEnabled)
  {
    if (i < (control_loop_rate / 2.0))
    {
      i++;
    }
    else
    {
      i = 0;
      enableReportAnglesMsg.enable_report_angles = !auto_pilot_in_control;
      fins_enable_reporting_pub.publish(enableReportAnglesMsg);
      incontrol.auto_pilot_in_control = auto_pilot_in_control;
      auto_pilot_in_control_pub.publish(incontrol);
    }
    if (lastcontrol != auto_pilot_in_control)  // catch transition, report angles confict with auto
                                               // pilot. used only by OCU
    {
      lastcontrol = auto_pilot_in_control;
      enableReportAnglesMsg.enable_report_angles = !auto_pilot_in_control;
      fins_enable_reporting_pub.publish(enableReportAnglesMsg);
    }
    if (auto_pilot_in_control &&
        mmIsAlive)  // check to see if teleoperation not in control and mission mgr is running
    {
      // ROS_INFO("Auto Pilot in Control");
      // ROLL ELEMENT
      rollcmdpos = roll_pid_controller.updatePid(currentRoll - desiredRoll,
                                                 ros::Duration(1.0 / control_loop_rate));

      // PITCH ELEMENT
      if (depthControl)  // output of depth pid will be input to pitch pid
      {
        desired_pitch = depth_pid_controller.updatePid(currentDepth - desiredDepth,
                                                       ros::Duration(1.0 / control_loop_rate));
        if (fabs(desired_pitch) > maxDepthCommand)
          desired_pitch = desired_pitch / fabs(desired_pitch) * maxDepthCommand;
      }
      else
        desired_pitch = desiredPitch;
      pitchcmdpos = pitch_pid_controller.updatePid(currentPitch - desired_pitch,
                                                   ros::Duration(1.0 / control_loop_rate));

      // YAW ELEMENT
      if (!fixedRudder)
      {
        yawDiff = currentYaw - desiredYaw;
        yawError = fmod((yawDiff + 180.0), 360.0) - 180.0;
        yawcmdpos = yaw_pid_controller.updatePid(yawError, ros::Duration(1.0 / control_loop_rate));
      }
      else  // overiding yaw with fixed rudder
      {
        yawcmdpos = desiredRudder;
      }

      if (mission_mode)
        mixactuators(rollcmdpos, pitchcmdpos, yawcmdpos);
      else
        mixactuators(0, 0, 0);

      // SPEED
      setrpm.commanded_rpms = desiredSpeed * rpmPerKnot;
      if (fabs(setrpm.commanded_rpms) < (rpmPerKnot * minimalspeed) &&
          fabs(desiredSpeed) >= minimalspeed)
        setrpm.commanded_rpms =
            (fabs(setrpm.commanded_rpms) / setrpm.commanded_rpms) * rpmPerKnot * minimalspeed;
      else if (fabs(setrpm.commanded_rpms) > thruster_control::SetRPM::MAX_RPM)
        setrpm.commanded_rpms = (fabs(setrpm.commanded_rpms) / setrpm.commanded_rpms) *
                                thruster_control::SetRPM::MAX_RPM;
      else if (fabs(setrpm.commanded_rpms) > maxAllowedThrusterRpm)
        setrpm.commanded_rpms =
            (fabs(setrpm.commanded_rpms) / setrpm.commanded_rpms) * maxAllowedThrusterRpm;
      if (setrpm.commanded_rpms < 0.0 &&
          !allowReverseThrusterAutopilot)  // don't have thruster move in reverse.
        setrpm.commanded_rpms = 0.0;
      if (thrusterEnabled) thruster_pub.publish(setrpm);
    }
    // maintain control loop rate
    r.sleep();
  }
  auto_pilot_in_control = false;
  enableReportAnglesMsg.enable_report_angles = !auto_pilot_in_control;
  fins_enable_reporting_pub.publish(enableReportAnglesMsg);
}
