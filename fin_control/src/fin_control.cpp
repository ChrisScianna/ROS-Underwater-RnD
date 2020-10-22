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

/*
 * fin_control.cpp
 *
 */

#include "fin_control.h"
#include <diagnostic_tools/message_stagnation_check.h>
#include <diagnostic_tools/periodic_message_status.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/select.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

using namespace std;

namespace qna {
namespace robot {

FinControl::FinControl(ros::NodeHandle &nodeHandle)
    : nodeHandle(nodeHandle), diagnosticsUpdater(nodeHandle) {
  diagnosticsUpdater.setHardwareID("fin_control");

  reportAnglesEnabled = true;

  fincontrolEnabled = false;
  string serial_dev = "/dev/ttyUSB0";
  int serial_baud = 57600;
  maxCtrlFinSwing = 10.0;
  ctrlFinOffet = 0;
  ctrlFinScaleFactor = 1.0;
  servos_on = false;
  const char *log;
  currentLoggingEnabled = false;
  reportAngleRate = 0.04;
  minReportAngleRate = 0.02;
  maxReportAngleRate = 0.08;

  ros::NodeHandle nh;

  ROS_INFO("Starting fin_control node Version: [%s]", NODE_VERSION);
  nh.setParam("/version_numbers/fin_control", NODE_VERSION);

  if (nh.getParam("/fin_control/port", serial_dev) == 0)
    ROS_INFO("Parmeter Not found defaullting Serial Port: [%s]", serial_dev.c_str());

  nh.getParam("/fin_control/baud", serial_baud);

  ROS_INFO("Serial Port: [%s]", serial_dev.c_str());
  ROS_INFO("baud: [%d]", serial_baud);

  nh.getParam("/fin_control/max_ctrl_fin_swing", maxCtrlFinSwing);
  ROS_INFO("max cf swing: [%f]", maxCtrlFinSwing);

  nh.getParam("/fin_control/ctrl_fin_offset", ctrlFinOffet);
  ROS_INFO("cf offset: [%f]", ctrlFinOffet);

  nh.getParam("/fin_control/ctrl_fin_scale_factor", ctrlFinScaleFactor);
  ROS_INFO("cf scale: [%f]", ctrlFinScaleFactor);

  nh.getParam("/fin_control/current_logging_enabled", currentLoggingEnabled);
  if (currentLoggingEnabled) ROS_INFO("FIN CURRENT LOGGING ENABLED");

  nh.getParam("/fin_control/report_angle_rate", reportAngleRate);
  nh.getParam("/fin_control/min_report_angle_rate", minReportAngleRate);
  nh.getParam("/fin_control/max_report_angle_rate", maxReportAngleRate);

  nh.getParam("/fin_control/max_ctrl_plane_swing", maxCtrlPlaneSwing);

  ROS_INFO("fin control constructor enter");

  subscriber_setAngle =
      nodeHandle.subscribe("/fin_control/set_angle", 10, &FinControl::handle_SetAngle, this);
  subscriber_setAngles =
      nodeHandle.subscribe("/fin_control/set_angles", 10, &FinControl::handle_SetAngles, this);
  subscriber_enableReportAngles = nodeHandle.subscribe(
      "/fin_control/enable_report_angles", 10, &FinControl::handle_EnableReportAngles, this);

  publisher_reportAngle =
      nodeHandle.advertise<fin_control::ReportAngle>("/fin_control/report_angle", 1);

  finAngleCheck = diagnostic_tools::create_health_check<double>(
      "Fin swing within range", [this](double angle) -> diagnostic_tools::Diagnostic {
        using diagnostic_tools::Diagnostic;
        if (std::abs(angle) > maxCtrlPlaneSwing)
        {
          using health_monitor::ReportFault;
          Diagnostic diagnostic{Diagnostic::WARN, ReportFault::FIN_DATA_THRESHOLD_REACHED};
          return diagnostic.description(
              "Fin angle above maximum swing value: |%f rad| > %f rad", angle, maxCtrlPlaneSwing);
        }
        return Diagnostic::OK;
      });
  diagnosticsUpdater.add(finAngleCheck);

  // DynamixelWorkbench myWorkBench;

  if (myWorkBench.begin(serial_dev.c_str(), serial_baud, &log))
    ROS_INFO("Dynamixel Workbench intialized");
  else {
    ROS_ERROR("Dynamixel Workbench failed init %s", log);
    return;
  }

  num_of_ids = 0;

  if (myWorkBench.scan(ids, &num_of_ids, 1, 4, &log)) {
    ROS_INFO("num of ids found [%d]", num_of_ids);
    if (num_of_ids < NUM_FINS)
      ROS_WARN("Fin servos found: [%d] does not math number of fins [%d]", num_of_ids, NUM_FINS);
  } else {
    ROS_ERROR("Servo scan failed! %s", log);
    return;
  }
  for (int x = 0; x < num_of_ids; x++)  // power on all servos
  {
    if (!myWorkBench.torqueOn(ids[x], &log)) {
      ROS_ERROR("Could not set torque on fin sevro [%d] %s", ids[x], log);
      return;
    } else
      servos_on = true;
  }

  /*ROS_INFO("execise begin");
      for(int x = 0; x < 10; x++)
      {
       myWorkBench.goalPosition(1,(float)0.0,NULL);
      reportAngle();
      sleep(2);
       myWorkBench.goalPosition(1,(float)(1.57),NULL);
      reportAngle();
      sleep(2);
      }
  ROS_INFO("execise end");

  */
  if (!myWorkBench.initBulkWrite(&log)) {
    ROS_ERROR("Could not init bulk write");
    return;
  }

  Start();
  ros::spin();
  Stop();
}

FinControl::~FinControl() {
  const char *log;

  if (servos_on) {
    ROS_INFO("Turning off fin servos");
    for (int x = 0; x < num_of_ids; x++)  // power on all servos
    {
      if (!myWorkBench.torqueOff(ids[x], &log))
        ROS_ERROR("Could not turn off torque on fin sevro [%d] %s", ids[x], log);
    }
  }
}

void FinControl::reportAngles() {
  const char *log;
  fin_control::ReportAngle message;

  if (!reportAnglesEnabled) return;  // do not interfere with fins updating

  message.header.stamp = ros::Time::now();
  message.angle_in_radians = 0;

  boost::mutex::scoped_lock lock(m_mutex);

  for (int x = 0; x < num_of_ids; x++)  // power on all servos
  {
    reportAngle(ids[x]);
    //    message.ID = ids[x];
    //    if(myWorkBench.getRadian(ids[x],&message.angle_in_radians,&log))// get from dynamixel
    //    {
    //      message.angle_in_radians =
    //      (float)(round(((message.angle_in_radians/ctrlFinScaleFactor)-degreesToRadians(ctrlFinOffet))*100.0)/100.0);
    //      publisher_reportAngle.publish(message);
    // ROS_INFO("reportAngle published. Angle is: %d", message.angle_in_radians);
    //    }
    //    else
    //      ROS_ERROR("Could not get servo angle for ID %d %s",message.ID,log);
  }
}
void FinControl::reportAngle(uint8_t id) {
  const char *log;
  fin_control::ReportAngle message;

  if (!reportAnglesEnabled) return;  // do not interfere with fins updating

  message.header.stamp = ros::Time::now();
  message.angle_in_radians = 0;

  message.ID = id;
  if (myWorkBench.getRadian(id, &message.angle_in_radians, &log))  // get from dynamixel
  {
    //     message.angle_in_radians =
    //     ((message.angle_in_radians/ctrlFinScaleFactor)-degreesToRadians(ctrlFinOffet));
    message.angle_in_radians = (float)(round(((message.angle_in_radians / ctrlFinScaleFactor) -
                                              degreesToRadians(ctrlFinOffet)) *
                                             100.0) /
                                       100.0);
    finAngleCheck.test(message.angle_in_radians);

    publisher_reportAngle.publish(message);
    // ROS_INFO("reportAngle published. Angle is: %d", message.angle_in_radians);
  } else
    ROS_ERROR("Could not get servo angle for ID %d %s", message.ID, log);
}
float FinControl::radiansToDegrees(float radians) { return (radians * (180.0 / M_PI)); }

float FinControl::degreesToRadians(float degrees) { return ((degrees / 180.0) * M_PI); }

void FinControl::handle_SetAngles(const fin_control::SetAngles::ConstPtr &msg) {
  const char *log;
  float angle_plus_offset;

  // check for max angle the fins can mechanically handle
  float angle1_in_degrees;
  float angle2_in_degrees;
  float angle3_in_degrees;
  float angle4_in_degrees;

  angle1_in_degrees = radiansToDegrees(msg->f1_angle_in_radians);
  angle2_in_degrees = radiansToDegrees(msg->f2_angle_in_radians);
  angle3_in_degrees = radiansToDegrees(msg->f3_angle_in_radians);
  angle4_in_degrees = radiansToDegrees(msg->f4_angle_in_radians);

  if ((fabs(angle1_in_degrees) > (maxCtrlFinSwing / 2.0)) ||
      (fabs(angle2_in_degrees) > (maxCtrlFinSwing / 2.0)) ||
      (fabs(angle3_in_degrees) > (maxCtrlFinSwing / 2.0)) ||
      (fabs(angle4_in_degrees) > (maxCtrlFinSwing / 2.0))

  ) {
    ROS_ERROR("Set Angle degrees out of Range");
    return;
  }

  boost::mutex::scoped_lock lock(m_mutex);

  if (!myWorkBench.initBulkWrite(&log)) {
    ROS_ERROR("Could not init bulk write [%d]");
    return;
  }

  // fin1
  angle_plus_offset =
      ctrlFinScaleFactor * (msg->f1_angle_in_radians + degreesToRadians(ctrlFinOffet));

  if (!(myWorkBench.addBulkWriteParam(
          1, "Goal_Position", myWorkBench.convertRadian2Value(1, angle_plus_offset), &log))) {
    ROS_ERROR("Could not add bulk write param 1 %s", log);
    return;
  }

  // fin2
  angle_plus_offset =
      ctrlFinScaleFactor * (msg->f2_angle_in_radians + degreesToRadians(ctrlFinOffet));

  if (!(myWorkBench.addBulkWriteParam(
          2, "Goal_Position", myWorkBench.convertRadian2Value(2, angle_plus_offset), &log))) {
    ROS_ERROR("Could not add bulk write param 2 %s", log);
    return;
  }

  // fin3
  angle_plus_offset =
      ctrlFinScaleFactor * (msg->f3_angle_in_radians + degreesToRadians(ctrlFinOffet));

  if (!(myWorkBench.addBulkWriteParam(
          3, "Goal_Position", myWorkBench.convertRadian2Value(3, angle_plus_offset), &log))) {
    ROS_ERROR("Could not add bulk write param 3 %s", log);
    return;
  }

  // fin4
  angle_plus_offset =
      ctrlFinScaleFactor * (msg->f4_angle_in_radians + degreesToRadians(ctrlFinOffet));

  if (!(myWorkBench.addBulkWriteParam(
          4, "Goal_Position", myWorkBench.convertRadian2Value(4, angle_plus_offset), &log))) {
    ROS_ERROR("Could not add bulk write param 4 %s", log);
    return;
  }

  if (!myWorkBench.bulkWrite(&log)) {
    ROS_ERROR("Could not bulk write %s", log);
  }

  if (currentLoggingEnabled) {
    int32_t cdata1, cdata2, cdata3, cdata4;
    cdata1 = cdata2 = cdata3 = cdata4 = 0;

    if (!myWorkBench.itemRead(1, "Present_Current", &cdata1, &log)) {
      ROS_ERROR("Could Read F1 Current %s", log);
    } else if (!myWorkBench.itemRead(2, "Present_Current", &cdata2, &log)) {
      ROS_ERROR("Could Read F2 Current %s", log);
    } else if (!myWorkBench.itemRead(3, "Present_Current", &cdata3, &log)) {
      ROS_ERROR("Could Read F3 Current %s", log);
    } else if (!myWorkBench.itemRead(4, "Present_Current", &cdata4, &log)) {
      ROS_ERROR("Could Read F4 Current %s", log);
    } else {
      ROS_INFO("Current for fin 1-4 %f,%f,%f,%f", myWorkBench.convertValue2Current((int16_t)cdata1),
               myWorkBench.convertValue2Current((int16_t)cdata2),
               myWorkBench.convertValue2Current((int16_t)cdata3),
               myWorkBench.convertValue2Current((int16_t)cdata4));
    }
  }
}

void FinControl::handle_SetAngle(const fin_control::SetAngle::ConstPtr &msg) {
  const char *log;
  // check for max angle the fins can mechanically handle
  float angle_in_degrees;
  angle_in_degrees = radiansToDegrees(msg->angle_in_radians);
  if ((angle_in_degrees < (-1.0 * maxCtrlFinSwing / 2.0)) ||
      (angle_in_degrees > (maxCtrlFinSwing / 2.0))) {
    ROS_ERROR("Set Angle [%f] degrees out of Range", angle_in_degrees);
    return;
  }

  float angle_plus_offet =
      ctrlFinScaleFactor * (msg->angle_in_radians + degreesToRadians(ctrlFinOffet));

  // Call dynamixel service
  // ROS_INFO("Got fin angle command [%f]", msg->angle_in_radians);

  boost::mutex::scoped_lock lock(m_mutex);

  if (!(myWorkBench.goalPosition(msg->ID, angle_plus_offet, &log)))
    ROS_ERROR("Could not set servo angle for ID %d %s", msg->ID, log);
  // reportAngle(msg->ID);   //read actual position and publish new angle
}

void FinControl::handle_EnableReportAngles(const fin_control::EnableReportAngles::ConstPtr &msg) {
  if (msg->enable_report_angles) {
    reportAnglesEnabled = true;
    //   ROS_INFO("Enabling report fin angles");
  } else {
    reportAnglesEnabled = false;
    //   ROS_INFO("Disabling report fin angles");
  }
}

void FinControl::workerFunc() {
  while (fincontrolEnabled) {
    reportAngles();
    diagnosticsUpdater.update();
    // sleep to maintain 25Hz update period
    // if changing this number update durations above.
    usleep(static_cast<int>(reportAngleRate * 1e6));
  }
}
void FinControl::Start() {
  assert(!m_thread);
  fincontrolEnabled = true;
  m_thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&FinControl::workerFunc, this)));
}

void FinControl::Stop() {
  assert(m_thread);
  fincontrolEnabled = false;
  m_thread->join();
}

}  // namespace robot
}  // namespace qna
