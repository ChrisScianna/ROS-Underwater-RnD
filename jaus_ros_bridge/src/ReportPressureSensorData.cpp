/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 - 2021, QinetiQ, Inc.
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

#include "ReportPressureSensorData.h"
#include "JausDataManager.h"

void ReportPressureSensorData::init(ros::NodeHandle* nodeHandle, udpserver* udp) {
  //_pressure = 0;
  //_temperature = 0;

  _nodeHandle = nodeHandle;
  _udpserver = udp;
  _subscriber_reportPressureTemp = nodeHandle->subscribe(
      "/pressure_sensor/pressure", 1, &ReportPressureSensorData::handleReportPressureTemp, this);
  _myID = "ReportPressureSensorData";
  Reset();
}

bool ReportPressureSensorData::HasNewData(const sensor_msgs::FluidPressure::ConstPtr& msg) {
  if (_pressureSensorData.pressure != (float)msg->fluid_pressure) return true;
  return false;
}

void ReportPressureSensorData::Reset() {
  _pressureSensorData.pressure = 0;
  _pressureSensorData.temperature = 0;
}

void ReportPressureSensorData::handleReportPressureTemp(
    const sensor_msgs::FluidPressure::ConstPtr& msg) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;

  if (!HasNewData(msg)) return;

  _pressureSensorData.pressure = (float)(((msg->fluid_pressure) - 101325.0) / 100000.0);
  _pressureSensorData.temperature = (float)0;
  if (debug_mode)
    ROS_INFO("handleReportPressureTemp() is called!!! pressure is %f, temperature is: %f",
             _pressureSensorData.pressure, _pressureSensorData.temperature);

  DataInfo info = GetPackedMessage(&_pressureSensorData);
  _udpserver->RequestSendingMessage(_myID + "handleReportPressureTemp", info._data, info._size);
  delete[] info._data;

  _beginTime = clock();
}

DataInfo ReportPressureSensorData::GetPackedMessage(void* data) {
  PressureSensorData* pressureData = reinterpret_cast<PressureSensorData*>(data);
  // here 6: 2 for presence vector + 2 fields x 4 bytes = 10 bytes
  JausMessageHeader* header = new JausMessageHeader(JAUS_COMMAND_ReportPressureSensorData, 10);

  char* newData = new char[header->GetHeadersize() + header->GetDatasize()];
  // start from header
  int index = 0;
  char* headerData = header->GetHeaderData();

  for (index = 0; index < header->GetHeadersize(); index++) {
    newData[index] = headerData[index];
  }

  // jsun - PV's ToByteArray() is not working, so just pad presence vector to 0 for now
  newData[index++] = 0;
  newData[index++] = 0;

  // finally the data

  char* temp;
  temp = reinterpret_cast<char*>(&pressureData->pressure);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  temp = reinterpret_cast<char*>(&pressureData->temperature);
  newData[index++] = temp[0];
  newData[index++] = temp[1];
  newData[index++] = temp[2];
  newData[index++] = temp[3];

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}
