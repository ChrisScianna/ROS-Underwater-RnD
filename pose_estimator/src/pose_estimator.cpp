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

#include "pose_estimator/pose_estimator.h"

using health_monitor::ReportFault;
using qna::robot::PoseEstimatorNode;
using pose_estimator::CorrectedData;

// assumes that the pressure is in absolute pascal and a true flag indicates saltwater
PoseEstimatorNode::PoseEstimatorNode(ros::NodeHandle h)
    : node_handle(h),
      private_node_handle("~"),
      running(false),
      data_valid(false),
      pubRate(20.0),
      ahrs_ok(false),
      pressure_ok(false),
      diagnosticsUpdater(h)
{
  diagnosticsUpdater.setHardwareID("pose_estimator");

  ros::NodeHandle pose_node_handle(node_handle, "pose");

  minRate = pubRate / 2;
  maxRate = pubRate * 2;
  double poseDataSteadyBand = 0.0;
  // Get runtime parameters
  private_node_handle.getParam("rate", pubRate);
  private_node_handle.getParam("min_rate", minRate);
  private_node_handle.getParam("max_rate", maxRate);
  private_node_handle.getParam("pose_data_steady_band", poseDataSteadyBand);
  private_node_handle.getParam("saltwater_flag", saltwater_flag);
  private_node_handle.getParam("max_depth", maxDepth);
  private_node_handle.getParam("max_roll_ang", maxRollAng);
  private_node_handle.getParam("max_pitch_ang", maxPitchAng);
  private_node_handle.getParam("max_yaw_ang", maxYawAng);
  pose_node_handle.param<double>("/autopilot_node/rpm_per_knot", rpmPerKnot, 303.0);  // knots

  // Subscribe to all topics
  sub_pressure_data = pose_node_handle.subscribe("/pressure_sensor/pressure", 1,
                                                 &PoseEstimatorNode::pressureDataCallback, this);
  sub_ahrs_data =
      pose_node_handle.subscribe("/vectornav/IMU", 1, &PoseEstimatorNode::ahrsDataCallback, this);

  // Advertise all topics and services
  pub_corrected_data = diagnostic_tools::create_publisher<pose_estimator::CorrectedData>(
      pose_node_handle, "corrected_data", 1);
  sub_rpm_data = pose_node_handle.subscribe("/thruster_control/report_rpm", 1,
                                            &PoseEstimatorNode::rpmDataCallback, this);
  sub_gps_data = pose_node_handle.subscribe("/fix", 1, &PoseEstimatorNode::gpsDataCallback, this);

  diagnostic_tools::Diagnostic diagnosticCorrectedDataInfoStale
  {
    diagnostic_tools::Diagnostic::ERROR, ReportFault::POSE_DATA_STALE
  };
  diagnostic_tools::PeriodicMessageStatusParams paramsCorrectedDataCheckPeriod;
  paramsCorrectedDataCheckPeriod.min_acceptable_period(1.0 / minRate);
  paramsCorrectedDataCheckPeriod.max_acceptable_period(1.0 / maxRate);
  paramsCorrectedDataCheckPeriod.abnormal_diagnostic(diagnosticCorrectedDataInfoStale);
  diagnosticsUpdater.add(pub_corrected_data.add_check<diagnostic_tools::PeriodicMessageStatus>(
      "rate check", paramsCorrectedDataCheckPeriod));

  diagnostic_tools::Diagnostic diagnosticCorrectedDataInfoStagnate
  {
      diagnostic_tools::Diagnostic::ERROR, ReportFault::POSE_DATA_STAGNATED
  };
  diagnostic_tools::MessageStagnationCheckParams paramsCorrectedDataCheckStagnation;
  paramsCorrectedDataCheckStagnation.stagnation_diagnostic(diagnosticCorrectedDataInfoStagnate);
  diagnosticsUpdater.add(pub_corrected_data.add_check<diagnostic_tools::MessageStagnationCheck>(
      "stagnation check",
      [poseDataSteadyBand](const pose_estimator::CorrectedData &a,
                           const pose_estimator::CorrectedData &b)
      {
        return (std::fabs((a.depth - b.depth) < poseDataSteadyBand) &&
                std::fabs((a.rpy_ang.x - b.rpy_ang.x) < poseDataSteadyBand) &&
                std::fabs((a.rpy_ang.y - b.rpy_ang.y) < poseDataSteadyBand) &&
                std::fabs((a.rpy_ang.z - b.rpy_ang.z) < poseDataSteadyBand));
      }
      , paramsCorrectedDataCheckStagnation));

  // Report if message data is out of range
  depthCheck = diagnostic_tools::create_health_check<double>(
      "Vehicle depth within range", [this](double depth) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        if (depth < 0. || depth > maxDepth)
        {
          return Diagnostic(Diagnostic::ERROR, ReportFault::POSE_DEPTH_THRESHOLD_REACHED)
              .description("%f m outside [%f m, %m m] range", depth, maxDepth);
        }
        return Diagnostic::OK;
      }
);
  diagnosticsUpdater.add(depthCheck);

  orientationRollCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
      "Vehicle orientation - Roll Angle check",
      [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(rpy.x) > maxRollAng)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Roll Angle - NOT OK -> |%f rad| > %f rad", rpy.x, maxRollAng)
              .code(ReportFault::POSE_ROLL_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Roll Angle - OK (%f)", rpy.x);
        }
        return diagnostic;
      }
);
  diagnosticsUpdater.add(orientationRollCheck);

  orientationPitchCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
      "Vehicle orientation - Pitch Angle check",
      [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(rpy.y) > maxPitchAng)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Pitch Angle - NOT OK -> |%f rad| > %f rad", rpy.y, maxPitchAng)
              .code(ReportFault::POSE_PITCH_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Pitch Angle - OK (%f)", rpy.y);
        }
        return diagnostic;
      }
);
  diagnosticsUpdater.add(orientationPitchCheck);

  orientationYawCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
      "Vehicle orientation - Roll Yaw check",
      [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic
      {
        using diagnostic_tools::Diagnostic;
        Diagnostic diagnostic{Diagnostic::OK};
        if (std::abs(rpy.z) > maxYawAng)
        {
          diagnostic.status(Diagnostic::ERROR)
              .description("Yaw Angle - NOT OK -> |%f rad| > %f rad", rpy.z, maxYawAng)
              .code(ReportFault::POSE_HEADING_THRESHOLD_REACHED);
        }
        else
        {
          diagnostic.description("Yaw Angle - OK (%f)", rpy.z);
        }
        return diagnostic;
      }
);

  diagnosticsUpdater.add(orientationYawCheck);

  // Initialize corrected data header
  m_correctedData.header.frame_id = "corrected_data";
}

PoseEstimatorNode::~PoseEstimatorNode()
{
  stop();
}

double PoseEstimatorNode::calculateDepth(double pressure, bool flag)
{
  double depth;
  pressure = pressure - ABSOLUTE_PASCAL_TO_GAUGE_PASCAL;
  // Calculate Depth
  if (flag)  // if saltwater
  {
    depth = pressure / GRAVITY / SALTWATER_DENSITY;
  }
  else
  {
    depth = pressure / GRAVITY / FRESHWATER_DENSITY;
  }
  return depth;
}

int PoseEstimatorNode::start()
{
  stop();
  running = true;
  // Start data publishing timer
  ROS_INFO("Starting data publishing timer");
  m_timerPub = node_handle.createTimer(ros::Duration(1.0 / pubRate),
                                       boost::bind(&PoseEstimatorNode::publishData, this));
  return 0;
}

int PoseEstimatorNode::stop()
{
  if (running)
  {
    running = false;
    m_timerPub.stop();
  }
  return 0;
}

bool PoseEstimatorNode::spin()
{
  while (!ros::isShuttingDown())
  {
    if (start() == 0)
    {
      while (node_handle.ok())
      {
        ros::spin();
      }
    }
  }
  stop();
  return true;
}

void PoseEstimatorNode::ahrsDataCallback(const sensor_msgs::Imu &data)
{
  boost::mutex::scoped_lock lock(m_mutDataLock);
  tf::Quaternion quat(data.orientation.x, data.orientation.y, data.orientation.z,
                      data.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  cur_rpy_ang[CorrectedData::ROLL] = roll * -1.0;
  cur_rpy_ang[CorrectedData::PITCH] = pitch * -1.0;
  cur_rpy_ang[CorrectedData::YAW] = yaw;
  ahrs_ok = true;
}

void PoseEstimatorNode::rpmDataCallback(const thruster_control::ReportRPM &data)
{
  boost::mutex::scoped_lock lock(m_mutDataLock);
  cur_speed = static_cast<double>(data.rpms) / rpmPerKnot;
}

void PoseEstimatorNode::gpsDataCallback(const sensor_msgs::NavSatFix &data)
{
  if (data.status.status >= 0)
  {
    boost::mutex::scoped_lock lock(m_mutDataLock);
    cur_latitude = data.latitude;
    cur_longitude = data.longitude;
    cur_gps_time = data.header.stamp.toSec();
  }
}

void PoseEstimatorNode::pressureDataCallback(const sensor_msgs::FluidPressure &data)
{
  boost::mutex::scoped_lock lock(m_mutDataLock);
  cur_pressure = data.fluid_pressure;
  pressure_ok = true;
}

void PoseEstimatorNode::processDynPos()
{
  boost::mutex::scoped_lock data_lock(m_mutDataLock);
  m_correctedData.rpy_ang.x = cur_rpy_ang[CorrectedData::ROLL];
  m_correctedData.rpy_ang.y = cur_rpy_ang[CorrectedData::PITCH];
  m_correctedData.rpy_ang.z = cur_rpy_ang[CorrectedData::YAW];
}

void PoseEstimatorNode::processLatLong()
{
  boost::mutex::scoped_lock data_lock(m_mutDataLock);
  m_correctedData.position.latitude = cur_latitude;
  m_correctedData.position.longitude = cur_longitude;
}

void PoseEstimatorNode::processDepth()
{
  boost::mutex::scoped_lock data_lock(m_mutDataLock);
  cur_depth = calculateDepth(cur_pressure, saltwater_flag);
  m_correctedData.depth = cur_depth;
}

void PoseEstimatorNode::processSpeed()
{
  boost::mutex::scoped_lock data_lock(m_mutDataLock);
  m_correctedData.speed = cur_speed;
}

void PoseEstimatorNode::publishData()
{
  // Ensure that we have started getting data from all critical sensors
  // before we start publishing data
  if (!data_valid)
  {
    if (ahrs_ok && pressure_ok)
    {
      data_valid = true;
    }
  }
  else
  {
    // these next 3 methods just copy the
    // current data to the message structure
    processDepth();
    processSpeed();
    processLatLong();
    processDynPos();
    // Publish the latest set of corrected data
    boost::mutex::scoped_lock data_lock(m_mutDataLock);
    m_correctedData.header.stamp = ros::Time::now();

    depthCheck.test(m_correctedData.depth);
    orientationRollCheck.test(m_correctedData.rpy_ang);
    orientationPitchCheck.test(m_correctedData.rpy_ang);
    orientationYawCheck.test(m_correctedData.rpy_ang);
    pub_corrected_data.publish(m_correctedData);
    diagnosticsUpdater.update();
  }
}
