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


namespace
{

  double radiansToDegrees(double radians) { return (radians * (180.0 / M_PI)); }

  double degreesToRadians(double degrees) { return ((degrees / 180.0) * M_PI); }

}  // namespace

AutoPilotNode::AutoPilotNode(ros::NodeHandle& node_handle) : nh(node_handle)
{
  autoPilotInControl = false;
  missionMode = false;
  autopilotEnabled = true;
  mmIsAlive = false;
  currentRoll = 0;
  currentPitch = 0;
  currentYaw = 0;
  currentDepth = 0;
  maxAltitudeCommand = 25.0;

  thrusterPub = nh.advertise<thruster_control::SetRPM>("thruster_control/set_rpm", 1);
  finsControlPub = nh.advertise<fin_control::SetAngles>("fin_control/set_angles", 1);
  autoPilotInControlPub =
      nh.advertise<autopilot::AutoPilotInControl>("autopilot/auto_pilot_in_control", 1);

  double mgr_report_hb_rate;
  nh.param<double>("/mission_control_node/report_heart_beat_rate", mgr_report_hb_rate, 2.0);
  ROS_INFO("mm hb rate: [%f]", mgr_report_hb_rate);
  mmTimeout = 3.0 * mgr_report_hb_rate;

  nh.param<int>("/autopilot_node/control_loop_rate", controlLoopRate, 25);     // Hz
  nh.param<double>("/autopilot_node/minimal_vehicle_speed", minimalSpeed, 2.0);  // knots
  nh.param<double>("/autopilot_node/rpm_per_knot", rpmPerKnot, 100.0);           // rpm
  nh.param<bool>("/autopilot_node/speed_control_enabled", speedControlEnabled, false);

  nh.param<double>("/autopilot_node/roll_p", rollPGain, 1.0);
  nh.param<double>("/autopilot_node/roll_i", rollIGain, 0.0);
  nh.param<double>("/autopilot_node/roll_imax", rollIMax, 0.0);
  nh.param<double>("/autopilot_node/roll_imin", rollIMin, 0.0);
  nh.param<double>("/autopilot_node/roll_d", rollDGain, 0.0);

  nh.param<double>("/autopilot_node/pitch_p", pitchPGain, 1.0);
  nh.param<double>("/autopilot_node/pitch_i", rollIGain, 0.0);
  nh.param<double>("/autopilot_node/pitch_imax", pitchIMax, 0.0);
  nh.param<double>("/autopilot_node/pitch_imin", pitchIMin, 0.0);
  nh.param<double>("/autopilot_node/pitch_d", pitchDGain, 0.0);

  nh.param<double>("/autopilot_node/yaw_p", yawPGain, 1.0);
  nh.param<double>("/autopilot_node/yaw_i", yawIGain, 0.0);
  nh.param<double>("/autopilot_node/yaw_imax", yawIMax, 0.0);
  nh.param<double>("/autopilot_node/yaw_imin", yawIMin, 0.0);
  nh.param<double>("/autopilot_node/yaw_d", yawDGain, 0.0);

  nh.param<double>("/autopilot_node/depth_p", depthPGain, 1.0);
  nh.param<double>("/autopilot_node/depth_i", depthIGain, 0.0);
  nh.param<double>("/autopilot_node/depth_imax", depthIMax, 0.0);
  nh.param<double>("/autopilot_node/depth_imin", depthIMin, 0.0);
  nh.param<double>("/autopilot_node/depth_d", depthDGain, 0.0);
  nh.param<double>("/autopilot_node/max_depth_command", maxDepthCommand, 10.0);

  nh.param<double>("/autopilot_node/altitude_p", altitudePGain, 1.0);
  nh.param<double>("/autopilot_node/altitude_i", altitudeIGain, 0.0);
  nh.param<double>("/autopilot_node/altitude_imax", altitudeIMax, 0.0);
  nh.param<double>("/autopilot_node/altitude_imin", altitudeIMin, 0.0);
  nh.param<double>("/autopilot_node/altitude_d", altitudeDGain, 0.0);
  nh.param<double>("/autopilot_node/max_altitude_command",maxAltitudeCommand, 25.0);

  maxCtrlFinAngle = radiansToDegrees(
    nh.param<double>("/fin_control/max_ctrl_fin_angle", degreesToRadians(10.0)));

  rollPIDController.initPid(rollPGain, rollIGain, rollDGain, rollIMax, rollIMin);
  pitchPIDController.initPid(pitchPGain, rollIGain, pitchDGain, pitchIMax, pitchIMin);
  yawPidController.initPid(yawPGain, yawIGain, yawDGain, yawIMax, yawIMin);
  depthPIDController.initPid(depthPGain, depthIGain, depthDGain, depthIMax, depthIMin);
  altitudePIDController.initPid(altitudePGain,altitudeIGain,altitudeDGain,altitudeIMax,altitudeIMin);
  
  nh.param<double>("/autopilot_node/desired_roll", desiredRoll, 0.0);
  nh.param<double>("/autopilot_node/desired_pitch", desiredPitch, 0.0);
  nh.param<double>("/autopilot_node/desired_yaw", desiredYaw, 0.0);
  nh.param<bool>("/autopilot_node/depth_control_enabled", depthControl, false);
  nh.param<double>("/autopilot_node/desired_depth", desiredDepth, 0.0);
  nh.param<bool>("/autopilot_node/fixed_rudder", fixedRudder, true);
  nh.param<double>("/autopilot_node/desired_rudder", desiredRudder, 0.0);
  nh.param<double>("/autopilot_node/desired_speed", desiredSpeed, 0.0);
  nh.param<bool>("/autopilot_node/allow_reverse_thruster_autopilot",
                 allowReverseThrusterAutopilot, false);
  nh.param<bool>("/autopilot_node/thruster_enabled", thrusterEnabled, false);

  jausRosSub = nh.subscribe("/jaus_ros_bridge/activate_manual_control", 1,
                              &AutoPilotNode::HandleActivateManualControl, this);
  stateSub =
      nh.subscribe("state", 1, &AutoPilotNode::stateCallback, this);
  missionMgrHeartBeatSub =
      nh.subscribe("/mngr/report_heartbeat", 10, &AutoPilotNode::missionMgrHbCallback, this);
  missionStatusSub = nh.subscribe("/mngr/report_mission_execute_state", 1,
                                    &AutoPilotNode::missionStatusCallback, this);
  setAttitudeBehaviorSub =
      nh.subscribe("/mngr/attitude_servo", 1, &AutoPilotNode::attitudeServoCallback, this);
  setDepthHeadingBehaviorSub =
      nh.subscribe("/mngr/depth_heading", 1, &AutoPilotNode::depthHeadingCallback, this);
  setFixedRudderBehaviorSub =
      nh.subscribe("/mngr/fixed_rudder", 1, &AutoPilotNode::fixedRudderCallback, this);
  setWaypointBehaviorSub =
      nh.subscribe("/mngr/waypoint", 10, &AutoPilotNode::waypointCallback, this);

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
}

void AutoPilotNode::missionMgrHeartbeatTimeout(const ros::TimerEvent& timer)
{
  boost::mutex::scoped_lock lock(missonManagerHeartbeatMutex);

  thruster_control::SetRPM setRPM;
  ROS_ERROR("Mission Manager DIED!!!");
  mmIsAlive = false;

  missionMgrHeartbeatTimer.stop();

  if (autoPilotInControl)  // if mission mgr dies and we are not teleoperating shut down
                              // thruster.
  {
    setRPM.commanded_rpms = 0.0;
    thrusterPub.publish(setRPM);
  }
}

void AutoPilotNode::stateCallback(const auv_interfaces::StateStamped& msg)
{
  boost::mutex::scoped_lock lock(m_mutex);

  currentDepth = msg.state.manoeuvring.pose.mean.position.z;
  currentRoll = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.x);
  currentPitch = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.y);
  currentYaw = radiansToDegrees(msg.state.manoeuvring.pose.mean.orientation.z);
  geodesy::fromMsg(msg.state.geolocation.position, currentPosition);
}

void AutoPilotNode::missionStatusCallback(const mission_control::ReportExecuteMissionState& data)
{
  using mission_control::ReportExecuteMissionState;
  if (data.execute_mission_state == ReportExecuteMissionState::COMPLETE)
  {
    boost::mutex::scoped_lock lock(m_mutex);
    missionMode = true;

    desiredRoll = 0;

    // surface
    desiredPitch = -maxCtrlFinAngle;
    depthControl = false;
    altitudeControl = false;

    desiredRudder = 0;  // straight
    fixedRudder = true;

    desiredSpeed = 0;  // coast
  }
}

void AutoPilotNode::HandleActivateManualControl(const jaus_ros_bridge::ActivateManualControl& data)
{
  boost::mutex::scoped_lock lock(behaviorMutex);
  if (data.activate_manual_control)
  {
    // ROS_INFO("Jaus says its in control.");
    autoPilotInControl = false;
    missionMode = false;
    // clear the following so we do not take off after manual control is released.
    desiredSpeed = 0;
    desiredRoll = 0;
    desiredYaw = 0;
    desiredPitch = 0;
    desiredDepth = 0;
    desiredRudder = 0;
    fixedRudder = true;
    depthControl = false;
    altitudeControl = false;
  }
  else
  {
    // ROS_INFO("Jaus says its not in control.");
    autoPilotInControl = true;
  }
}

void AutoPilotNode::mixActuators(double roll, double pitch, double yaw)
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

  double maxOptions[] = {fabs(d1), fabs(d2), fabs(d3), fabs(d4), maxCtrlFinAngle};
  double maxAngle = *std::max_element(maxOptions, maxOptions + 5);
  d1 = d1 * maxCtrlFinAngle / maxAngle;
  d2 = d2 * maxCtrlFinAngle / maxAngle;
  d3 = d3 * maxCtrlFinAngle / maxAngle;
  d4 = d4 * maxCtrlFinAngle / maxAngle;

  fin_control::SetAngles setAnglesMsg;

  setAnglesMsg.f1_angle_in_radians = degreesToRadians(d1);
  setAnglesMsg.f2_angle_in_radians = degreesToRadians(d2);
  setAnglesMsg.f3_angle_in_radians = degreesToRadians(d3);
  setAnglesMsg.f4_angle_in_radians = degreesToRadians(d4);
  // ROS_INFO("F1,F2,F3,F4: [%f,%f,%f,%f]",d1,d2,d3,d4);
  finsControlPub.publish(setAnglesMsg);
}

void AutoPilotNode::missionMgrHbCallback(const mission_control::ReportHeartbeat& msg)
{
  boost::mutex::scoped_lock lock(missonManagerHeartbeatMutex);

  mmIsAlive = true;

  missionMgrHeartbeatTimer.stop();
  missionMgrHeartbeatTimer.start();
}

void AutoPilotNode::attitudeServoCallback(const mission_control::AttitudeServo& msg)
{
  boost::mutex::scoped_lock lock(behaviorMutex);
  missionMode = true;
  fixedRudder = true;  // this equate to fixed rudder with roll and pitch commands.

  ROS_INFO("Angle for roll: [%f]", msg.roll);
  desiredRoll = radiansToDegrees(msg.roll);

  ROS_INFO("Angle for pitch: [%f]", msg.pitch);
  desiredPitch = radiansToDegrees(msg.pitch);
  depthControl = false;  // disable depth control setting pitch instead
  altitudeControl = false;

  ROS_INFO("Angle for yaw: [%f]", msg.yaw);
  desiredRudder = radiansToDegrees(msg.yaw);  // yaw in attitude servo is really a fixedrudder

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);

  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::depthHeadingCallback(const mission_control::DepthHeading& msg)
{
  missionMode = true;
  fixedRudder = false;

  ROS_INFO("Depth Heading Message Received setting Angle for roll to 0.");
  desiredRoll = 0;

  ROS_INFO("depth: [%f]", msg.depth);
  desiredDepth = msg.depth;
  depthControl = true;  // enabling depth control
  altitudeControl = false;

  ROS_INFO("Angle for yaw: [%f]", msg.heading);
  desiredYaw = radiansToDegrees(msg.heading);

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::altitudeHeadingCallback(const mission_control::AltitudeHeading& msg)
{
  boost::mutex::scoped_lock lock(behaviorMutex);
  fixedRudder = false;
 
  ROS_INFO("Depth Heading Message Received setting Angle for roll to 0.");
  desiredRoll = 0;
 
  ROS_INFO("altitude: [%f]", msg.altitude);
 
  desiredPosition.altitude = msg.altitude;
  depthControl = false;    // disabling depth control;
  altitudeControl = true;  // enable altitude control;
 
  ROS_INFO("Angle for yaw: [%f]", msg.heading);
  desiredYaw = radiansToDegrees(msg.heading);
  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::fixedRudderCallback(const mission_control::FixedRudder& msg)
{
  boost::mutex::scoped_lock lock(behaviorMutex);
  missionMode = true;
  fixedRudder = true;
  ROS_INFO("Fixed Rudder Message Received setting Angle for roll to 0.");
  desiredRoll = 0;

  ROS_INFO("depth: [%f]", msg.depth);
  desiredDepth = msg.depth;
  depthControl = true;  // enabling depth control
  altitudeControl = false;

  ROS_INFO("Angle for rudder: [%f]", msg.rudder);
  desiredRudder = radiansToDegrees(msg.rudder);

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

void AutoPilotNode::waypointCallback(const mission_control::Waypoint& msg)
{
  boost::mutex::scoped_lock lock(behaviorMutex);
  waypointfollowing = true;
  fixedRudder = false;
  speedControlEnabled = true;
  desiredRoll = 0;

  ROS_INFO("Waypoint received lat: [%f] long: [%f]", msg.latitude, msg.longitude);

  geodesy::fromMsg(geodesy::toMsg(msg.latitude, msg.longitude), desiredPosition);
  ROS_INFO("UTM desired Easting: [%f]", desiredPosition.easting);
  ROS_INFO("UTM desired Nothing: [%f]", desiredPosition.northing);

  if (msg.ena_mask | mission_control::Waypoint::ALTITUDE_ENA)
  {
    desiredPosition.altitude = msg.altitude;
    altitudeControl = true;  // enabling altitude control
    depthControl = false;
  }
  else // default is depth control
  {
    desiredDepth = msg.depth;
    altitudeControl = false;
    depthControl = true;  // enabling depth control
  }

  ROS_INFO("speed_knots: [%f]", msg.speed_knots);
  desiredSpeed = msg.speed_knots;
}

namespace
{

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
    if(xb > xa)
    {
      sop = -1.0;
    }
    else
    {
      sop =  1.0;
    }
  }
  else if(yb < ya)
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

  double ydiff = std::abs(yb - ya);
  double xdiff = std::abs(xb - xa);
  double avalPI = atan(ydiff / xdiff);
  double avalDG = radiansToDegrees(avalPI);
  double relAngle = (avalDG * sop) + w;

  return fmod((relAngle + 180.0), 360.0) - 180.0;
}

}  // namespace

void AutoPilotNode::workerFunc()
{
  double rollCmdPos = 0;
  double pitchCmdPos = 0;
  double yawCmdPos = 0;
  double yawError = 0;
  double yawDiff = 0;
  double desiredPitch = 0;
  thruster_control::SetRPM setRPM;
  ros::Rate r(controlLoopRate);
  autopilot::AutoPilotInControl incontrol;
  bool lastControl;

  lastControl = autoPilotInControl;
  int i = 0;
  while (autopilotEnabled)
  {
    if (i < (controlLoopRate / 2.0))
    {
      i++;
    }
    else
    {
      i = 0;
      incontrol.auto_pilot_in_control = autoPilotInControl;
      autoPilotInControlPub.publish(incontrol);
    }
    if (lastControl != autoPilotInControl)  // catch transition, report angles confict with auto
                                               // pilot. used only by OCU
    {
      lastControl = autoPilotInControl;
    }
    if (autoPilotInControl &&
        mmIsAlive)  // check to see if teleoperation not in control and mission mgr is running
    {
      // ROS_INFO("Auto Pilot in Control");
      // ROLL ELEMENT
      rollCmdPos = rollPIDController.updatePid(currentRoll - desiredRoll,
                                                 ros::Duration(1.0 / controlLoopRate));

      // PITCH ELEMENT
      if (depthControl)  // output of depth pid will be input to pitch pid
      {
        desiredPitch = depthPIDController.updatePid(currentDepth - desiredDepth,
                                                       ros::Duration(1.0 / controlLoopRate));
        if (fabs(desiredPitch) > maxDepthCommand)
          desiredPitch = desiredPitch / fabs(desiredPitch) * maxDepthCommand;
      }
      else if (altitudeControl)  // out put of altitude pid will be input to pitch pid
      {
        desiredPitch = altitudePIDController.updatePid(
          currentPosition.altitude - desiredPosition.altitude,
          ros::Duration(1.0 / controlLoopRate));
        if (desiredPitch < (-1.0 * maxAltitudeCommand))
          desiredPitch = -1.0 * maxAltitudeCommand;
        else if (pitchCmdPos > maxAltitudeCommand)
          desiredPitch = maxAltitudeCommand;
      }
      else
        desiredPitch = desiredPitch;
      pitchCmdPos = pitchPIDController.updatePid(currentPitch - desiredPitch,
                                                   ros::Duration(1.0 / controlLoopRate));

      // YAW ELEMENT
      if (!fixedRudder)
      {
        if (waypointfollowing)
        {
          desiredYaw = relativeAngle(
            desiredPosition.easting, desiredPosition.northing,
            currentPosition.easting, currentPosition.northing);
        }
        yawDiff = currentYaw - desiredYaw;
        yawError = fmod((yawDiff + 180.0), 360.0) - 180.0;
        yawCmdPos = yawPidController.updatePid(yawError, ros::Duration(1.0 / controlLoopRate));
      }
      else  // overiding yaw with fixed rudder
      {
        yawCmdPos = desiredRudder;
      }

      if (missionMode)
        mixActuators(rollCmdPos, pitchCmdPos, yawCmdPos);
      else
        mixActuators(0, -maxCtrlFinAngle, 0);    //fin to surface

      // SPEED
      setRPM.commanded_rpms = desiredSpeed * rpmPerKnot;
      if (fabs(setRPM.commanded_rpms) < (rpmPerKnot * minimalSpeed) &&
          fabs(desiredSpeed) >= minimalSpeed)
        setRPM.commanded_rpms =
            (fabs(setRPM.commanded_rpms) / setRPM.commanded_rpms) * rpmPerKnot * minimalSpeed;
      if (setRPM.commanded_rpms < 0.0 &&
          !allowReverseThrusterAutopilot)  // don't have thruster move in reverse.
        setRPM.commanded_rpms = 0.0;
      if (thrusterEnabled) thrusterPub.publish(setRPM);
    }
    // maintain control loop rate
    r.sleep();
  }
  autoPilotInControl = false;
}
