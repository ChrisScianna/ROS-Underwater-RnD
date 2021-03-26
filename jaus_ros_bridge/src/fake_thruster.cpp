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

// Original version: Jie Sun <Jie.Sun@us.QinetiQ.com>

#include <thruster_control/ReportRPM.h>
#include <thruster_control/SetRPM.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thruster_control/ReportMotorTemperature.h>
//#include <keller_4lc_pressure/Pressure.h>
//#include <keller_9lx_pressure/Pressure.h>

//#include <ixblue_c3_ins/C3Ins.h>
#include <health_monitor/ReportLeakDetected.h>
//#include <sea_scan_echo_sensor/ReportAltimeterRange.h>
#include <health_monitor/ReportBatteryInfo.h>
#include <pose_estimator/CorrectedData.h>
//#include <mission_control/ReportLoadMissionState.h>
//#include <mission_control/ReportExecuteMissionState.h>

class MessageHandler {
  ros::NodeHandle* nodeHandle;
  ros::Timer report_timer;
  ros::Timer reportINS_timer;
  ros::Publisher publisher_reportRPM;
  ros::Publisher publisher_reportTemp;
  ros::Publisher publisher_reportINS;
  ros::Publisher publisher_reportPressureSensor;
  ros::Publisher publisher_reportBatteryLeak;
  ros::Publisher publisher_reportAltimeter;
  ros::Publisher publisher_reportPoseData;
  ros::Publisher publisher_reportBatteryInfo;

  //    ros::Publisher publisher_reportMissionState;
  //    ros::Publisher publisher_reportLoadState;
  //    ros::Publisher chatter_pub;

  int _newRPM;
  bool _changed = false;
  int _engineTemp = -100;
  int _batteryPosition = 0;
  int _batteryStatus = 1;  // Moving
  float insData = -200.123456789;
  bool leaked = false;

  // called every 3 second
  void reportINSSendTimeout(const ros::TimerEvent& timer) {
    if (insData >= 200)
      insData = -200.123456789;
    else
      insData = insData + 5;

    //        ixblue_c3_ins::C3Ins message;
    //        message.heading = insData/200;
    //        message.roll = insData/200;
    //        message.pitch = insData/200;
    //        message.north_speed = insData;
    //        message.east_speed = insData;
    //        message.vertical_speed = insData;
    //        message.latitude = insData;
    //        message.longitude = insData;
    //        message.altitude = insData;
    //        message.timestamp = (unsigned int) insData;//ros::Time::now();

    //        publisher_reportINS.publish(message);
    //        //ROS_INFO("Reported INS data from reportINSSendTimeout(). Data is %f", insData);

    //        keller_9lx_pressure::Pressure message1;
    //        message1.pressure = insData/200;
    //        message1.temperature = insData+2;
    //        publisher_reportPressureSensor.publish(message1);
    // ROS_INFO("Reported Pressure data from reportINSSendTimeout(). Pressure is %d", insData);

    //        health_monitor::ReportLeakDetected message3;
    //        leaked = !leaked;
    //        //ROS_ERROR("leaked = %d", leaked);
    //        message3.leakDetected = (int8_t)leaked;
    //        publisher_reportBatteryLeak.publish(message3);

    // sea_scan_echo_sensor/ReportAltimeterRange

    //        sea_scan_echo_sensor::ReportAltimeterRange message4;
    //        message4.UnfilteredRange = insData;
    //        publisher_reportAltimeter.publish(message4);

    pose_estimator::CorrectedData message5;
    // ROS_ERROR("pose_estimator::CorrectedData is published!!!");
    message5.latitude = insData;    // 123.456;
    message5.longitude = -insData;  //-123.456;
    message5.depth = insData;
    // message5.altitude = insData;
    message5.speed = insData;
    message5.rpy_ang[pose_estimator::CorrectedData::ROLL] = insData;
    message5.rpy_ang[pose_estimator::CorrectedData::PITCH] = insData;
    message5.rpy_ang[pose_estimator::CorrectedData::YAW] = insData;
    publisher_reportPoseData.publish(message5);

    //        health_monitor::ReportBatteryInfo message6;
    //        message6.batteryPackAConnected = true;
    //        message6.batteryPacksTotalCurrent = 5.01;
    //        message6.batteryPackACell1 = 1.02;
    //        message6.batteryPackACell2 = 2.02;
    //        message6.batteryPackACell3 = 3.02;
    //        message6.batteryPackACell4 = 4.02;
    //        message6.batteryPackACell5 = 5.02;
    //        message6.batteryPackACell6 = 6.02;
    //        message6.batteryPackACell7 = 7.02;
    //        message6.batteryPackACell8 = 8.02;
    //        message6.batteryPackACell9 = 9.02;
    //        message6.batteryPackACell10 = 10.02;
    //        message6.batteryCell1Thermocouple = 11;
    //        message6.batteryCell2Thermocouple = 22;
    //        message6.batteryCell3Thermocouple = 33;
    //        message6.batteryCell4Thermocouple = 44;
    //        message6.batteryCell5Thermocouple = 55;
    //        message6.batteryCell6Thermocouple = 66;
    //        message6.batteryCell7Thermocouple = 77;
    //        message6.batteryCell8Thermocouple = 88;

    //        message6.systemThermocouple1 = 99;
    //        publisher_reportBatteryInfo.publish(message6);

    // mission_control::ReportLoadMissionState message7;
    // message7.stamp = ros::Time::now();
    // message7.mission_id = 202;
    // message7.load_state = 0; //successful
    // publisher_reportLoadState.publish(message7);

    // mission_control::ReportExecuteMissionState message8;
    // message8.stamp = ros::Time::now();
    // ROS_WARN("message8.stamp in second is: %f", message8.stamp.toSec());
    // ROS_WARN("message8.stamp in mili second is: %f", message8.stamp.toSec()*1000);
    // message8.mission_id = 101;
    // message8.execute_mission_state = 4; //running
    // publisher_reportMissionState.publish(message8);

    // ROS_INFO("Report battery info message is published");
  }

  // called every 1 second
  void reportRPMSendTimeout(const ros::TimerEvent& timer) {
    thruster_control::ReportRPM message;

    message.stamp = ros::Time::now();
    _newRPM = _newRPM + 2;
    message.rpms = _newRPM;

    publisher_reportRPM.publish(message);
    // ROS_INFO("Report RPM message: rpm [%f] is published back.", message.rpms);

    thruster_control::ReportMotorTemperature message1;
    message1.motor_temp = (int)_engineTemp;

    if (_engineTemp >= 300)
      _engineTemp = -100;
    else
      _engineTemp += 20;

    publisher_reportTemp.publish(message1);
    // ROS_INFO("Report motor temperature message: motor_temp  [%f] is published back.",
    // message1.motor_temp);

    _changed = false;
  }

 public:
  void SetNodeHangdle(ros::NodeHandle* n) {
    nodeHandle = n;
    // chatter_pub = nodeHandle->advertise<std_msgs::String>("fake_thruster_topic", 1000);
    // repeat duration is set to 1 seconds
    report_timer =
        nodeHandle->createTimer(ros::Duration(2), &MessageHandler::reportRPMSendTimeout, this);
    reportINS_timer =
        nodeHandle->createTimer(ros::Duration(1), &MessageHandler::reportINSSendTimeout, this);
    publisher_reportRPM =
        nodeHandle->advertise<thruster_control::ReportRPM>("/thruster_control/report_rpm", 1);
    publisher_reportTemp = nodeHandle->advertise<thruster_control::ReportMotorTemperature>(
        "/thruster_control/report_motor_temp", 1);
    // publisher_reportINS = nodeHandle->advertise<ixblue_c3_ins::C3Ins>("/ixblue_c3_ins/C3Ins", 1);
    // publisher_reportPressureSensor =
    // nodeHandle->advertise<keller_9lx_pressure::Pressure>("/pressure/data", 1);
    // publisher_reportBatteryPosition =
    // nodeHandle->advertise<health_monitor::ReportBatteryPosition>("/battery_position_control/report_battery_position",
    // 1);
    publisher_reportBatteryLeak = nodeHandle->advertise<health_monitor::ReportLeakDetected>(
        "/health_monitor/report_leak_detected", 1);
    // publisher_reportAltimeter =
    // nodeHandle->advertise<sea_scan_echo_sensor::ReportAltimeterRange>("/sea_scan_echo_sensor/report_altimeter_range",
    // 1);
    publisher_reportPoseData =
        nodeHandle->advertise<pose_estimator::CorrectedData>("/pose/corrected_data", 1);
    publisher_reportBatteryInfo = nodeHandle->advertise<health_monitor::ReportBatteryInfo>(
        "/health_monitor/report_battery_info", 1);

    // publisher_reportLoadState =
    // nodeHandle->advertise<mission_control::ReportLoadMissionState>("/mngr/report_mission_load_state",
    // 1); publisher_reportMissionState =
    // nodeHandle->advertise<mission_control::ReportExecuteMissionState>("/mngr/report_mission_execute_state",
    // 1);
  };

  void chatterCallback(const std_msgs::String::ConstPtr& msg){
      // ROS_INFO("I heard from bridge node: [%s]", msg->data.c_str());
      // ros::Publisher chatter_pub = nodeHandle->advertise<std_msgs::String>("fake_thruster_topic",
      // 1000); chatter_pub.publish(msg);
  };

  void handleSetRPM(const thruster_control::SetRPM::ConstPtr& msg) {
    // ROS_ERROR("inside fake_thruster handleSetRPM(): rpm is [%f]", msg->commanded_rpms);
    if ((int)msg->commanded_rpms != 0) {
      // ROS_ERROR("newRPM is set to: [%d]", (int)msg->commanded_rpms);
      _newRPM = (int)msg->commanded_rpms;
      _changed = true;
    }

    //        if(0){
    //            if(_changed == true && setRpmFinished == true)
    //            {
    //                ROS_INFO("reportRPMSendTimeout is called.");
    //                thruster_control::ReportRPM message;

    //                message.stamp = ros::Time::now();
    //                message.rpms = _newRPM;

    //                publisher_reportRPM.publish(message);
    //                ROS_INFO("Report RPM message: rpm [%f] is published back.", message.rpms);

    //                thruster_control::ReportMotorTemperature message1;
    //                message1.motor_temp = (int)_engineTemp;

    //                _engineTemp+= 20;
    //                if(_engineTemp >= 300)
    //                    _engineTemp = 100;

    //                publisher_reportTemp.publish(message1);
    //                ROS_INFO("Report motor temperature message: motor_temp  [%f] is published
    //                back.", message1.motor_temp); _changed = false;
    //            }
    //        }
  };
};

int main(int argc, char** argv) {
  MessageHandler msgHandler;
  ros::init(argc, argv, "listener1");

  ros::NodeHandle n;
  msgHandler.SetNodeHangdle(&n);
  ros::Subscriber sub =
      n.subscribe("bridge_topic", 1, &MessageHandler::chatterCallback, &msgHandler);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("fake_thruster_topic", 1);

  ros::Subscriber thruster_set_rmp =
      n.subscribe("/thruster_control/set_rpm", 1, &MessageHandler::handleSetRPM, &msgHandler);

  ros::spin();

  return 0;
}
