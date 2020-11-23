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

#include "ReportBatteryInfo.h"
#include "JausDataManager.h"

void ReportBatteryInfo::init(ros::NodeHandle *nodeHandle, udpserver *udp) {
    _nodeHandle = nodeHandle;
    _udpserver = udp;
    _myID = "ReportBatteryInfo";

    _subscriber_reportFaultID =
      nodeHandle->subscribe("/health_monitor/report_fault", 1,
                            &ReportBatteryInfo::handleReportFaultID, this);
    _subscriber_reportBatteryInfo = nodeHandle->subscribe(
      "/thruster_control/report_battery_health", 1, &ReportBatteryInfo::handleReportBatteryInfo, this);

    _subscriber_reportTemperature = nodeHandle->subscribe(
      "/thruster_control/report_motor_temperature", 1, &ReportBatteryInfo::handleReportTemperature, this);

    Reset();
}

DataInfo ReportBatteryInfo::GetPackedMessageForBatteryInfo(void *data) {

    BatteryInfoData* batteryInfo = reinterpret_cast<BatteryInfoData*>(data);
    //ROS_INFO("ReportBatteryPosition::GetPackedMessageForBatteryInfo");

    //here: 2 for presence vector + 2 x 4 bytes + 1 x 2 bytes = 12 bytes
    JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportBatteryInfo, 14);

    char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
    //start from header
    int index = 0;
    char* headerData = header->GetHeaderData();

    for(index = 0; index < header->GetHeadersize(); index++)
    {
        newData[index] = headerData[index];
    }

    //jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
    newData[index++] = 0;
    newData[index++] = 0;

    //finally the data
    char  *temp;

    temp = reinterpret_cast<char *>(&batteryInfo->batteryPacksTotalCurrent);
    //ROS_INFO("3. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2], temp[3]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&batteryInfo->batteryVoltage);
    //ROS_INFO("4. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2], temp[3]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&batteryInfo->systemThermocouple1);
    //ROS_INFO("14. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];


    DataInfo info;
    info._data = newData;
    info._size = header->GetDatasize() + header->GetHeadersize();

    delete header;
    return info;

}

void ReportBatteryInfo::Reset() {

    _fault_id = 0;
    _batteryInfo.batteryPacksTotalCurrent = 0;
    _batteryInfo.batteryVoltage = 0;
    _batteryInfo.systemThermocouple1 = 0;
}

bool ReportBatteryInfo::HasNewBatteryInfoData(
        const sensor_msgs::BatteryState::ConstPtr &msg)
{
    if(_batteryInfo.batteryPacksTotalCurrent != (float)msg->current
                || _batteryInfo.batteryVoltage != (float)msg->voltage)
                //|| _batteryInfo.systemThermocouple1 != (unsigned short)msg->systemThermocouple1 )
        return true;
    else
        return false;
}

bool ReportBatteryInfo::HasNewTemperatureData(
        const thruster_control::ReportMotorTemperature::ConstPtr& msg)
{
    if(_batteryInfo.systemThermocouple1 != (unsigned short)msg->motor_temp )
        return true;
    else
        return false;
}

void ReportBatteryInfo::handleReportBatteryInfo(
    const sensor_msgs::BatteryState::ConstPtr &msg) {

    if (!_udpserver->IsConnected())
      return;

    if (!TimeToSend(_beginTime, clock()))
      return;

    if (!HasNewBatteryInfoData(msg))
      return;

    _batteryInfo.batteryPacksTotalCurrent = (float)msg->current;
    _batteryInfo.batteryVoltage = (float)msg->voltage;
    //_batteryInfo.systemThermocouple1 = (unsigned short)msg->systemThermocouple1;

    DataInfo info = GetPackedMessageForBatteryInfo(&_batteryInfo);
    _udpserver->RequestSendingMessage(_myID+"handleReportBatteryInfo", info._data, info._size);
    delete [] info._data;

    _beginTime = clock();
}

void ReportBatteryInfo::handleReportTemperature(
        const thruster_control::ReportMotorTemperature::ConstPtr& msg)
{
    if (!_udpserver->IsConnected())
      return;

    if (!TimeToSend1(_beginTime, clock()))
      return;

    if (!HasNewTemperatureData(msg))
      return;

    _batteryInfo.systemThermocouple1 = (unsigned short)msg->motor_temp;

    DataInfo info = GetPackedMessageForBatteryInfo(&_batteryInfo);
    _udpserver->RequestSendingMessage(_myID+"handleReportBatteryInfo", info._data, info._size);
    delete [] info._data;

    _beginTime = clock();
}

void ReportBatteryInfo::handleReportFaultID(
    const health_monitor::ReportFault::ConstPtr& msg) {
    if (!_udpserver->IsConnected())
        return;

    if (!TimeToSend(_beginTime1, clock()))
        return;

    _fault_id = msg->fault_id;

    DataInfo info = GetPackedMessageForFaultID(_fault_id);
    _udpserver->RequestSendingMessage(_myID + "handleReportFaultID", info._data, info._size);
    delete[] info._data;

    _beginTime1 = clock();
}

DataInfo ReportBatteryInfo::GetPackedMessageForFaultID(uint64_t faults) {
  // here 10: 2 for presence vector + 1 fields x 8 bytes = 10 bytes
  JausMessageHeader *header = new JausMessageHeader(JAUS_COMMAND_ReportFaultID, 10);

  char *newData = new char[header->GetHeadersize() + header->GetDatasize()];
  // start from header
  int index = 0;
  char *headerData = header->GetHeaderData();

  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  // jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data
  char  *temp;

  temp = reinterpret_cast<char *>(&faults);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];
  newData[index++] = temp[4];
  newData[index++] = temp[5];
  newData[index++] = temp[6];
  newData[index++] = temp[7];

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}
