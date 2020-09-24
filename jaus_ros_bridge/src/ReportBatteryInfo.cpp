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
  //_batterypack = batterypack;
  //_positionData._currentPosition = 10000;
  //_positionData._positionState = 10000;

  _subscriber_reportLeakDetected =
      nodeHandle->subscribe("/battery_monitor/report_leak_detected", 1,
                            &ReportBatteryInfo::handleReportLeakDetected, this);
  _subscriber_reportBatteryInfo = nodeHandle->subscribe(
      "/battery_monitor/report_battery_info", 1, &ReportBatteryInfo::handleReportBatteryInfo, this);
  Reset();
}

DataInfo ReportBatteryInfo::GetPackedMessageForBatteryInfo(void *data) {
  BatteryInfoData *batteryInfo = reinterpret_cast<BatteryInfoData *>(data);
  // ROS_INFO("ReportBatteryPosition::GetPackedMessageForBatteryInfo");

  // here: 2 for presence vector + 2 x 1 bytes + 11 x 4 bytes + 9 x 2 bytes = 66 bytes
  JausMessageHeader *header = new JausMessageHeader(JAUS_COMMAND_ReportBatteryInfo, 66);

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
  char *temp;
  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackAConnected);
  // ROS_INFO("1. temp[0] is:%d", temp[0]);
  newData[index++] = temp[0];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackBConnected);
  // ROS_INFO("2. temp[0] is:%d", temp[0]);
  newData[index++] = temp[0];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPacksTotalCurrent);
  // ROS_INFO("3. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell1);
  // ROS_INFO("4. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell2);
  // ROS_INFO("5. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell3);
  // ROS_INFO("6. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell4);
  // ROS_INFO("7. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell5);
  // ROS_INFO("8. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell6);
  // ROS_INFO("9. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1], temp[2],
  // temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell7);
  // ROS_INFO("10. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1],
  // temp[2], temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell8);
  // ROS_INFO("11. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1],
  // temp[2], temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell9);
  // ROS_INFO("12. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1],
  // temp[2], temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryPackCell10);
  // ROS_INFO("13. temp[0], temp[1], temp[2] and temp[3] is: %d %d %d %d", temp[0], temp[1],
  // temp[2], temp[3]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell1Thermocouple);
  // ROS_INFO("14. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell2Thermocouple);
  // ROS_INFO("15. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell3Thermocouple);
  // ROS_INFO("16. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell4Thermocouple);
  // ROS_INFO("17. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell5Thermocouple);
  // ROS_INFO("18. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell6Thermocouple);
  // ROS_INFO("19. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell7Thermocouple);
  // ROS_INFO("20. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->batteryCell8Thermocouple);
  // ROS_INFO("21. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  temp = reinterpret_cast<char *>(&batteryInfo->systemThermocouple1);
  // ROS_INFO("21. temp[0], and temp[1] is: %d %d ", temp[0], temp[1]);
  newData[index++] = temp[0];
  newData[index++] = temp[1];

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

void ReportBatteryInfo::Reset() {
  //_positionData._currentPosition = 10000;
  //_positionData._positionState = 10000;
  _leakDetected = false;

  _batteryInfo.batteryPackAConnected = false;
  _batteryInfo.batteryPackBConnected = false;

  _batteryInfo.batteryPackCell1 = -10;
  _batteryInfo.batteryPackCell2 = -10;
  _batteryInfo.batteryPackCell3 = -10;
  _batteryInfo.batteryPackCell4 = -10;
  _batteryInfo.batteryPackCell5 = -10;
  _batteryInfo.batteryPackCell6 = -10;
  _batteryInfo.batteryPackCell7 = -10;
  _batteryInfo.batteryPackCell8 = -10;
  _batteryInfo.batteryPackCell9 = -10;
  _batteryInfo.batteryPackCell10 = -10;
  _batteryInfo.batteryPacksTotalCurrent = -10;

  _batteryInfo.batteryCell1Thermocouple = 0;
  _batteryInfo.batteryCell2Thermocouple = 0;
  _batteryInfo.batteryCell3Thermocouple = 0;
  _batteryInfo.batteryCell4Thermocouple = 0;
  _batteryInfo.batteryCell5Thermocouple = 0;
  _batteryInfo.batteryCell6Thermocouple = 0;
  _batteryInfo.batteryCell7Thermocouple = 0;
  _batteryInfo.batteryCell8Thermocouple = 0;

  _batteryInfo.systemThermocouple1 = 0;
}

bool ReportBatteryInfo::HasNewBatteryInfoData(
    const battery_monitor::ReportBatteryInfo::ConstPtr &msg) {
  if (_batterypack == "A") {
    if (_batteryInfo.batteryPackAConnected != (bool)msg->batteryPackA.present ||
        _batteryInfo.batteryPackBConnected != (bool)msg->batteryPackB.present ||
        _batteryInfo.batteryPacksTotalCurrent != (float)msg->batteryPacksTotalCurrent ||
        _batteryInfo.batteryPackCell1 != (float)msg->batteryPackA.cell_voltage[0] ||
        _batteryInfo.batteryPackCell2 != (float)msg->batteryPackA.cell_voltage[1] ||
        _batteryInfo.batteryPackCell3 != (float)msg->batteryPackA.cell_voltage[2] ||
        _batteryInfo.batteryPackCell4 != (float)msg->batteryPackA.cell_voltage[3] ||
        _batteryInfo.batteryPackCell5 != (float)msg->batteryPackA.cell_voltage[4] ||
        _batteryInfo.batteryPackCell6 != (float)msg->batteryPackA.cell_voltage[5] ||
        _batteryInfo.batteryPackCell7 != (float)msg->batteryPackA.cell_voltage[6] ||
        _batteryInfo.batteryPackCell8 != (float)msg->batteryPackA.cell_voltage[7] ||
        _batteryInfo.batteryPackCell9 != (float)msg->batteryPackA.cell_voltage[8] ||
        _batteryInfo.batteryPackCell10 != (float)msg->batteryPackA.cell_voltage[9] ||
        _batteryInfo.batteryCell1Thermocouple != (unsigned short)msg->batteryCell1Thermocouple ||
        _batteryInfo.batteryCell2Thermocouple != (unsigned short)msg->batteryCell2Thermocouple ||
        _batteryInfo.batteryCell3Thermocouple != (unsigned short)msg->batteryCell3Thermocouple ||
        _batteryInfo.batteryCell4Thermocouple != (unsigned short)msg->batteryCell4Thermocouple ||
        _batteryInfo.batteryCell5Thermocouple != (unsigned short)msg->batteryCell5Thermocouple ||
        _batteryInfo.batteryCell6Thermocouple != (unsigned short)msg->batteryCell6Thermocouple ||
        _batteryInfo.batteryCell7Thermocouple != (unsigned short)msg->batteryCell7Thermocouple ||
        _batteryInfo.batteryCell8Thermocouple != (unsigned short)msg->batteryCell8Thermocouple ||
        _batteryInfo.systemThermocouple1 != (unsigned short)msg->systemThermocouple1)
      return true;
  } else {
    if (_batteryInfo.batteryPackAConnected != (bool)msg->batteryPackA.present ||
        _batteryInfo.batteryPackBConnected != (bool)msg->batteryPackB.present ||
        _batteryInfo.batteryPacksTotalCurrent != (float)msg->batteryPacksTotalCurrent ||
        _batteryInfo.batteryPackCell1 != (float)msg->batteryPackB.cell_voltage[0] ||
        _batteryInfo.batteryPackCell2 != (float)msg->batteryPackB.cell_voltage[1] ||
        _batteryInfo.batteryPackCell3 != (float)msg->batteryPackB.cell_voltage[2] ||
        _batteryInfo.batteryPackCell4 != (float)msg->batteryPackB.cell_voltage[3] ||
        _batteryInfo.batteryPackCell5 != (float)msg->batteryPackB.cell_voltage[4] ||
        _batteryInfo.batteryPackCell6 != (float)msg->batteryPackB.cell_voltage[5] ||
        _batteryInfo.batteryPackCell7 != (float)msg->batteryPackB.cell_voltage[6] ||
        _batteryInfo.batteryPackCell8 != (float)msg->batteryPackB.cell_voltage[7] ||
        _batteryInfo.batteryPackCell9 != (float)msg->batteryPackB.cell_voltage[8] ||
        _batteryInfo.batteryPackCell10 != (float)msg->batteryPackA.cell_voltage[9] ||
        _batteryInfo.batteryCell1Thermocouple != (unsigned short)msg->batteryCell1Thermocouple ||
        _batteryInfo.batteryCell2Thermocouple != (unsigned short)msg->batteryCell2Thermocouple ||
        _batteryInfo.batteryCell3Thermocouple != (unsigned short)msg->batteryCell3Thermocouple ||
        _batteryInfo.batteryCell4Thermocouple != (unsigned short)msg->batteryCell4Thermocouple ||
        _batteryInfo.batteryCell5Thermocouple != (unsigned short)msg->batteryCell5Thermocouple ||
        _batteryInfo.batteryCell6Thermocouple != (unsigned short)msg->batteryCell6Thermocouple ||
        _batteryInfo.batteryCell7Thermocouple != (unsigned short)msg->batteryCell7Thermocouple ||
        _batteryInfo.batteryCell8Thermocouple != (unsigned short)msg->batteryCell8Thermocouple ||
        _batteryInfo.systemThermocouple1 != (unsigned short)msg->systemThermocouple1)
      return true;
  }

  return false;
}

void ReportBatteryInfo::handleReportBatteryInfo(
    const battery_monitor::ReportBatteryInfo::ConstPtr &msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;

  if (!HasNewBatteryInfoData(msg)) return;

  _batteryInfo.batteryPackAConnected = (bool)msg->batteryPackA.present;
  _batteryInfo.batteryPackBConnected = (bool)msg->batteryPackB.present;
  _batteryInfo.batteryPacksTotalCurrent = (float)msg->batteryPacksTotalCurrent;

  if (_batterypack == "A") {
    _batteryInfo.batteryPackCell1 = (float)msg->batteryPackA.cell_voltage[0];
    _batteryInfo.batteryPackCell2 = (float)msg->batteryPackA.cell_voltage[1];
    _batteryInfo.batteryPackCell3 = (float)msg->batteryPackA.cell_voltage[2];
    _batteryInfo.batteryPackCell4 = (float)msg->batteryPackA.cell_voltage[3];
    _batteryInfo.batteryPackCell5 = (float)msg->batteryPackA.cell_voltage[4];
    _batteryInfo.batteryPackCell6 = (float)msg->batteryPackA.cell_voltage[5];
    _batteryInfo.batteryPackCell7 = (float)msg->batteryPackA.cell_voltage[6];
    _batteryInfo.batteryPackCell8 = (float)msg->batteryPackA.cell_voltage[7];
    _batteryInfo.batteryPackCell9 = (float)msg->batteryPackA.cell_voltage[8];
    _batteryInfo.batteryPackCell10 = (float)msg->batteryPackA.cell_voltage[9];
  } else {
    _batteryInfo.batteryPackCell1 = (float)msg->batteryPackB.cell_voltage[0];
    _batteryInfo.batteryPackCell2 = (float)msg->batteryPackB.cell_voltage[1];
    _batteryInfo.batteryPackCell3 = (float)msg->batteryPackB.cell_voltage[2];
    _batteryInfo.batteryPackCell4 = (float)msg->batteryPackB.cell_voltage[3];
    _batteryInfo.batteryPackCell5 = (float)msg->batteryPackB.cell_voltage[4];
    _batteryInfo.batteryPackCell6 = (float)msg->batteryPackB.cell_voltage[5];
    _batteryInfo.batteryPackCell7 = (float)msg->batteryPackB.cell_voltage[6];
    _batteryInfo.batteryPackCell8 = (float)msg->batteryPackB.cell_voltage[7];
    _batteryInfo.batteryPackCell9 = (float)msg->batteryPackB.cell_voltage[8];
    _batteryInfo.batteryPackCell10 = (float)msg->batteryPackB.cell_voltage[9];
  }

  _batteryInfo.batteryCell1Thermocouple = (unsigned short)msg->batteryCell1Thermocouple;
  _batteryInfo.batteryCell2Thermocouple = (unsigned short)msg->batteryCell2Thermocouple;
  _batteryInfo.batteryCell3Thermocouple = (unsigned short)msg->batteryCell3Thermocouple;
  _batteryInfo.batteryCell4Thermocouple = (unsigned short)msg->batteryCell4Thermocouple;
  _batteryInfo.batteryCell5Thermocouple = (unsigned short)msg->batteryCell5Thermocouple;
  _batteryInfo.batteryCell6Thermocouple = (unsigned short)msg->batteryCell6Thermocouple;
  _batteryInfo.batteryCell7Thermocouple = (unsigned short)msg->batteryCell7Thermocouple;
  _batteryInfo.batteryCell8Thermocouple = (unsigned short)msg->batteryCell8Thermocouple;

  _batteryInfo.systemThermocouple1 = (unsigned short)msg->systemThermocouple1;

  DataInfo info = GetPackedMessageForBatteryInfo(&_batteryInfo);
  _udpserver->RequestSendingMessage(_myID + "handleReportBatteryInfo", info._data, info._size);
  delete[] info._data;

  _beginTime = clock();
}

void ReportBatteryInfo::handleReportLeakDetected(
    const battery_monitor::ReportLeakDetected::ConstPtr &msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime1, clock())) return;

  if (_leakDetected != msg->leakDetected) {
    _leakDetected = msg->leakDetected;

    if (debug_mode)
      ROS_INFO("handleReportLeakDetected() is called! leakDetected =  %d", _leakDetected);

    DataInfo info = GetPackedMessageForLeak(_leakDetected);
    _udpserver->RequestSendingMessage(_myID + "handleReportLeakDetected", info._data, info._size);
    delete[] info._data;
  }

  _beginTime1 = clock();
}

DataInfo ReportBatteryInfo::GetPackedMessageForLeak(bool leak) {
  // here 3: 2 for presence vector + 1 fields x 1 bytes = 3 bytes
  JausMessageHeader *header = new JausMessageHeader(JAUS_COMMAND_ReportBatteryLeakDetected, 3);

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

  newData[index++] = leak;

  //    if(leak)
  //        newData[index++] = 1;//(int8_t)leak;
  //    else {
  //        newData[index++] = 0;//(int8_t)leak;
  //    }

  // newData[index++] = '/0';

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

// DataInfo ReportBatteryPosition::GetPackedMessage(void * data)
//{
//    BatteryPositionData* positionData = reinterpret_cast<BatteryPositionData*>(data);
//    //short* position = reinterpret_cast<short*>(data);
//    //ROS_ERROR("position status is: %d", positionData->_positionState);

//    //here 4: 2 for presence vector + 2 fields x 2 bytes = 6 bytes
//    JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportBatteryPosition, 6);

//    char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
//    //start from header
//    int index = 0;
//    char* headerData = header->GetHeaderData();

//    for(index = 0; index < header->GetHeadersize(); index++)
//    {
//        newData[index] = headerData[index];
//    }

//    //jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
//    newData[index++] = 0;
//    newData[index++] = 0;

//    //finally the data
//    char  *temp;
//    temp = reinterpret_cast<char *>(&positionData->_currentPosition);
//    //temp = position;
//    newData[index++] = temp[0];
//    newData[index++] = temp[1];

//    temp = reinterpret_cast<char *>(&positionData->_positionState);
//    //temp = position;
//    newData[index++] = temp[0];
//    newData[index++] = temp[1];

//    DataInfo info;
//    info._data = newData;
//    info._size = header->GetDatasize() + header->GetHeadersize();

//    //ROS_ERROR("inside ReportBatteryPosition::GetPackedMessage() - completed");

//    delete header;
//    return info;
//}
