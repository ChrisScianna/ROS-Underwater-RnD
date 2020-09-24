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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com


#include <boost/thread/mutex.hpp>

#include "pose_estimator/CorrectedData.h"
#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_datatypes.h"
#include "thruster_control/ReportRPM.h"

#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>

#include <health_monitor/ReportFault.h>

namespace qna {
namespace robot {

#define saltwater_density 1023.6     // kg/m^3
#define freshwater_density 997.0474  // kg/m^3
#define gravity 9.80665              // m/s^2

#define NODE_VERSION "2.01x"
// Version log
// 2.0 Initial MK-IV version
// 2.01 Updating the roll positive direction
using namespace std;
using namespace pose_estimator;

class PoseEstimatorNode  //: public LogBase
{
 public:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;
  ros::Subscriber sub_ahrs_data;
  ros::Subscriber sub_pressure_data;
  ros::Subscriber sub_gps_data;
  ros::Subscriber sub_rpm_data;

 private:
  bool running, data_valid;
  // Vars holding runtime params
  double pub_rate;

  double maxDepth;
  double maxRollAng;
  double maxPitchAng;
  double maxYawAng;
  double rpmPerKnot;

  // Vars for holding current sensor/calculated values
  double cur_latitude, cur_longitude, cur_gps_time;
  double cur_rpy_ang[3];
  double cur_pressure;
  double cur_depth;
  double cur_speed = 0;

  // Flags for signaling if we have received essential data
  bool pressure_ok;
  bool ahrs_ok;

  // Enviornmental Flags
  bool saltwater_flag;

  CorrectedData m_correctedData{};

  ros::Timer m_timerPub;
  boost::mutex m_mutDataLock;

  diagnostic_tools::DiagnosedPublisher<pose_estimator::CorrectedData> pub_corrected_data;
  diagnostic_tools::HealthCheck<double> depthCheck;
  diagnostic_tools::HealthCheck<geometry_msgs::Vector3> orientationRollCheck;
  diagnostic_tools::HealthCheck<geometry_msgs::Vector3> orientationPitchCheck;
  diagnostic_tools::HealthCheck<geometry_msgs::Vector3> orientationYawCheck;
  diagnostic_updater::Updater diagnosticsUpdater;

 public:
  // assumes that the pressure is in absolute pascal and a true flag indicates saltwater
  double calculateDepth(double pressure, bool flag) {
    double depth;
    pressure = pressure - 101325;  // Absolute Pascal to Gauge Pascal
    // Calculate Depth
    if (flag)  // if saltwater
    {
      depth = pressure / gravity / saltwater_density;
    } else {
      depth = pressure / gravity / freshwater_density;
    }
    return depth;
  }

  PoseEstimatorNode(ros::NodeHandle h)
      : node_handle(h),
        private_node_handle("~"),
        running(false),
        data_valid(false),
        pub_rate(20.0),
        ahrs_ok(false),
        pressure_ok(false),
        diagnosticsUpdater(h) {
    diagnosticsUpdater.setHardwareID("pose_estimator");

    ros::NodeHandle pose_node_handle(node_handle, "pose");
    // Get runtime parameters
    private_node_handle.getParam("rate", pub_rate);
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
    sub_gps_data = pose_node_handle.subscribe("/fix", 1, &PoseEstimatorNode::gpsDataCallback, this);

    // Advertise all topics and services
    pub_corrected_data = diagnostic_tools::create_publisher<pose_estimator::CorrectedData>(
        pose_node_handle, "corrected_data", 1);
    sub_rpm_data = pose_node_handle.subscribe("/thruster_control/report_rpm", 1,
                                              &PoseEstimatorNode::rpmDataCallback, this);

    diagnosticsUpdater.add(pub_corrected_data.add_check<diagnostic_tools::PeriodicMessageStatus>(
        "rate check", diagnostic_tools::PeriodicMessageStatusParams{}
                          .min_acceptable_period((1.0 / pub_rate) / 2)
                          .max_acceptable_period((1.0 / pub_rate) * 2)
                          .abnormal_diagnostic({diagnostic_tools::Diagnostic::ERROR,
                                                health_monitor::ReportFault::POSE_DATA_STALE})));

    // we verify that depth, RPY angles are equal.
    diagnosticsUpdater.add(pub_corrected_data.add_check<diagnostic_tools::MessageStagnationCheck>(
        "stagnation check",
        [](const pose_estimator::CorrectedData &a, const pose_estimator::CorrectedData &b) {
          return ((a.depth == b.depth) && (a.rpy_ang.x == b.rpy_ang.x) &&
                  (a.rpy_ang.y == b.rpy_ang.y) && (a.rpy_ang.z == b.rpy_ang.z));
        },
        diagnostic_tools::MessageStagnationCheckParams{}.stagnation_diagnostic(
            {diagnostic_tools::Diagnostic::ERROR,
             health_monitor::ReportFault::BATTERY_INFO_STAGNATED})));

    // Report if message data is out of range
    depthCheck = diagnostic_tools::create_health_check<double>(
        "Vehicle depth within range", [this](double depth) -> diagnostic_tools::Diagnostic {
          using diagnostic_tools::Diagnostic;
          if (depth < 0. || depth > maxDepth) {
            return Diagnostic{Diagnostic::ERROR,
                              health_monitor::ReportFault::POSE_DEPTH_THRESHOLD_REACHED}
                .description("%f m outside [%f m, %m m] range", depth, maxDepth);
          }
          return Diagnostic::OK;
        });
    diagnosticsUpdater.add(depthCheck);

    orientationRollCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
        "Vehicle orientation - Roll Angle check",
        [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic {
          using diagnostic_tools::Diagnostic;
          Diagnostic diagnostic{Diagnostic::OK};
          if (std::abs(rpy.x) > maxRollAng) {
            diagnostic.status(Diagnostic::ERROR)
                .description("Roll Angle - NOT OK -> |%f rad| > %f rad", rpy.x, maxRollAng)
                .code(health_monitor::ReportFault::POSE_ROLL_THRESHOLD_REACHED);
          } else {
            diagnostic.description("Roll Angle - OK (%f)", rpy.x);
          }
          return diagnostic;
        });
    diagnosticsUpdater.add(orientationRollCheck);

    orientationPitchCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
        "Vehicle orientation - Pitch Angle check",
        [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic {
          using diagnostic_tools::Diagnostic;
          Diagnostic diagnostic{Diagnostic::OK};
          if (std::abs(rpy.y) > maxPitchAng) {
            diagnostic.status(Diagnostic::ERROR)
                .description("Pitch Angle - NOT OK -> |%f rad| > %f rad", rpy.y, maxPitchAng)
                .code(health_monitor::ReportFault::POSE_PITCH_THRESHOLD_REACHED);
          } else {
            diagnostic.description("Pitch Angle - OK (%f)", rpy.y);
          }
          return diagnostic;
        });
    diagnosticsUpdater.add(orientationPitchCheck);

    orientationYawCheck = diagnostic_tools::create_health_check<geometry_msgs::Vector3>(
        "Vehicle orientation - Roll Yaw check",
        [this](const geometry_msgs::Vector3 &rpy) -> diagnostic_tools::Diagnostic {
          using diagnostic_tools::Diagnostic;
          Diagnostic diagnostic{Diagnostic::OK};
          if (std::abs(rpy.z) > maxYawAng) {
            diagnostic.status(Diagnostic::ERROR)
                .description("Yaw Angle - NOT OK -> |%f rad| > %f rad", rpy.z, maxYawAng)
                .code(health_monitor::ReportFault::POSE_HEADING_THRESHOLD_REACHED);
          } else {
            diagnostic.description("Yaw Angle - OK (%f)", rpy.z);
          }
          return diagnostic;
        });

    diagnosticsUpdater.add(orientationYawCheck);

    // Initialize corrected data header
    m_correctedData.header.frame_id = "corrected_data";
  }

  ~PoseEstimatorNode() { stop(); }

  int start() {
    stop();
    running = true;
    // Start data publishing timer
    ROS_INFO("Starting data publishing timer");
    m_timerPub = node_handle.createTimer(ros::Duration(1.0 / pub_rate),
                                         boost::bind(&PoseEstimatorNode::publishData, this));
    return 0;
  }

  int stop() {
    if (running) {
      running = false;
      m_timerPub.stop();
    }
    return 0;
  }

  bool spin() {
    while (!ros::isShuttingDown()) {
      if (start() == 0) {
        while (node_handle.ok()) {
          ros::spin();
        }
      }
    }
    stop();
    return true;
  }

  void ahrsDataCallback(const sensor_msgs::Imu data) {
    boost::mutex::scoped_lock lock(m_mutDataLock);
    tf::Quaternion quat(data.orientation.x, data.orientation.y, data.orientation.z,
                        data.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    cur_rpy_ang[CorrectedData::ROLL] = roll * -1.0;
    cur_rpy_ang[CorrectedData::PITCH] = pitch * -1.0;
    cur_rpy_ang[CorrectedData::YAW] = yaw;
    // ROS_INFO("RPY: [%f,%f,%f]",roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
    ahrs_ok = true;
  }

  void gpsDataCallback(const sensor_msgs::NavSatFix data) {
    if (data.status.status >= 0) {
      boost::mutex::scoped_lock lock(m_mutDataLock);
      cur_latitude = data.latitude;
      cur_longitude = data.longitude;
      cur_gps_time = data.header.stamp.toSec();
    }
  }

  void rpmDataCallback(const thruster_control::ReportRPM data) {
    boost::mutex::scoped_lock lock(m_mutDataLock);
    cur_speed = (double)data.rpms / rpmPerKnot;
  }

  void pressureDataCallback(const sensor_msgs::FluidPressure data) {
    boost::mutex::scoped_lock lock(m_mutDataLock);
    cur_pressure = data.fluid_pressure;
    pressure_ok = true;
  }

  void processDynPos() {
    boost::mutex::scoped_lock data_lock(m_mutDataLock);
    m_correctedData.rpy_ang.x = cur_rpy_ang[CorrectedData::ROLL];
    m_correctedData.rpy_ang.y = cur_rpy_ang[CorrectedData::PITCH];
    m_correctedData.rpy_ang.z = cur_rpy_ang[CorrectedData::YAW];
  }

  void processLatLong() {
    boost::mutex::scoped_lock data_lock(m_mutDataLock);
    m_correctedData.position.latitude = cur_latitude;
    m_correctedData.position.longitude = cur_longitude;
  }

  void processDepth() {
    boost::mutex::scoped_lock data_lock(m_mutDataLock);
    cur_depth = calculateDepth(cur_pressure, saltwater_flag);
    m_correctedData.depth = cur_depth;
  }

  void processSpeed() {
    boost::mutex::scoped_lock data_lock(m_mutDataLock);
    m_correctedData.speed = cur_speed;
  }

  void publishData() {
    // Ensure that we have started getting data from all critical sensors
    // before we start publishing data
    if (!data_valid) {
      if (ahrs_ok && pressure_ok) {
        data_valid = true;
      }
    } else {
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
};

}  // namespace robot
}  // namespace qna

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_estimator_node");
  ros::NodeHandle n;
  ROS_INFO("Starting Pose Estimator node Version: [%s]", NODE_VERSION);
  n.setParam("/version_numbers/pose_estimator", NODE_VERSION);
  qna::robot::PoseEstimatorNode poseObject(n);
  poseObject.spin();
  return 0;
}
