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

/*
 * ReportINSData.cpp
 *
 *  Created on: Jun 13, 2019
 *      Author: jsun
 */

#include "ReportINSData.h"
#include "JausDataManager.h"

void ReportINSData::init(ros::NodeHandle *nodeHandle, udpserver *udp) {
  _nodeHandle = nodeHandle;
  _udpserver = udp;

  _myID = "ReportINSData";
  _subscriber_reportINS =
      nodeHandle->subscribe("/vectornav/IMU", 1, &ReportINSData::handleReportINS, this);
  Reset();
}

DataInfo ReportINSData::GetPackedMessage(void *data) {
  INSData *insData = reinterpret_cast<INSData *>(data);
  // here 22: 2 for presence vector + 10 fields x 4 bytes = 42 bytes
  JausMessageHeader *header = new JausMessageHeader(JAUS_COMMAND_ReportINSData, 42);

  // build presence vector of 0.... as for now
  // if(_presenceVector != nullptr)
  //    delete _presenceVector;

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
  if (1) {
    char *temp;
    temp = reinterpret_cast<char *>(&insData->_heading);
    // ROS_ERROR("_heading is: %f ", insData->_heading);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_roll);
    // ROS_INFO("2. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_pitch);
    // ROS_INFO("3. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_north_speed);
    // ROS_INFO("4. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_east_speed);
    // ROS_INFO("5. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_vertical_speed);
    // ROS_INFO("6. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_latitude);
    // ROS_INFO("7. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_longitude);
    // ROS_INFO("8. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_altitude);
    // ROS_INFO("9. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    temp = reinterpret_cast<char *>(&insData->_timestamp);
    // ROS_INFO("10. temp[0] and temp[1] is: %d %d", temp[0], temp[1]);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index] = temp[3];
  } else {
    char temp[2];

    sprintf(temp, "%f", _insData._heading);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._roll);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._pitch);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._north_speed);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._east_speed);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._vertical_speed);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._latitude);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._longitude);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%f", _insData._altitude);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];

    sprintf(temp, "%lu", _insData._timestamp);
    newData[index++] = temp[0];
    newData[index++] = temp[1];
    newData[index++] = temp[2];
    newData[index++] = temp[3];
  }

  DataInfo info;
  info._data = newData;
  info._size = header->GetDatasize() + header->GetHeadersize();

  delete header;
  return info;
}

void ReportINSData::Reset() {
  _insData._heading = 0;
  _insData._roll = 0;
  _insData._pitch = 0;
  _insData._north_speed = 0;
  _insData._east_speed = 0;
  _insData._vertical_speed = 0;
  _insData._latitude = 0;
  _insData._longitude = 0;
  _insData._altitude = 0;
  _insData._timestamp = 0;
}

bool ReportINSData::HasNewData(const sensor_msgs::Imu data) {
  tf::Quaternion quat(data.orientation.x, data.orientation.y, data.orientation.z,
                      data.orientation.w);
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  if (_insData._heading != (float)yaw) return true;
  if (_insData._roll != (float)roll) return true;
  if (_insData._pitch != (float)pitch) return true;
  return false;
}

void ReportINSData::handleReportINS(const sensor_msgs::Imu data) {
  if (!_udpserver->IsConnected()) return;

  if (!TimeToSend(_beginTime, clock())) return;
  INSData insData;
  if (HasNewData(data)) {
    tf::Quaternion quat(data.orientation.x, data.orientation.y, data.orientation.z,
                        data.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // ROS_ERROR("handleReportINS() is called!!!");
    insData._heading = (float)yaw;
    insData._roll = (float)(roll * -1.0);
    insData._pitch = (float)(pitch * -1.0);
    insData._north_speed = 0;
    insData._east_speed = 0;
    insData._vertical_speed = 0;
    insData._latitude = 0;
    insData._longitude = 0;
    insData._altitude = 0;
    insData._timestamp = (unsigned short)0;
    DataInfo info = GetPackedMessage(&insData);
    _udpserver->RequestSendingMessage(_myID + "handleReportINS", info._data, info._size);
    delete[] info._data;
    _insData = insData;
  }
  _beginTime = clock();
}
