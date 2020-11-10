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
#include <cmath>
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
  double min_rate;
  double max_rate;

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
  double calculateDepth(double pressure, bool flag);
  PoseEstimatorNode(ros::NodeHandle h);
  ~PoseEstimatorNode();

  int start();
  int stop();
  bool spin();
  void ahrsDataCallback(const sensor_msgs::Imu data);
  void gpsDataCallback(const sensor_msgs::NavSatFix data);
  void rpmDataCallback(const thruster_control::ReportRPM data);
  void pressureDataCallback(const sensor_msgs::FluidPressure data);
  void processDynPos();
  void processLatLong();
  void processDepth();
  void processSpeed();
  void publishData();
};

}  // namespace robot
}  // namespace qna
