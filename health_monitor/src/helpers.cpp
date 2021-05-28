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

#include "health_monitor/helpers.h"
#include "health_monitor/ReportFault.h"

#include <cstdint>
#include <string>


namespace health_monitor
{

uint64_t fault_code(const std::string & fault)
{
  static constexpr struct {
    const char * name;
    uint64_t code;
  } fault_mapping[] = {
    {"PAYLOAD_ERROR", ReportFault::PAYLOAD_ERROR},
    {"PAYLOAD_NODE_DIED", ReportFault::PAYLOAD_NODE_DIED},
    {"PRESSURE_DATA_STALE", ReportFault::PRESSURE_DATA_STALE},
    {"PRESSURE_DATA_STAGNATED", ReportFault::PRESSURE_DATA_STAGNATED},
    {"PRESSURE_THRESHOLD_REACHED", ReportFault::PRESSURE_THRESHOLD_REACHED},
    {"PRESSURE_NODE_DIED", ReportFault::PRESSURE_NODE_DIED},
    {"AHRS_DATA_STALE", ReportFault::AHRS_DATA_STALE},
    {"AHRS_DATA_STAGNATED", ReportFault::AHRS_DATA_STAGNATED},
    {"AHRS_NODE_DIED", ReportFault::AHRS_NODE_DIED},
    {"FIN_DATA_STALE", ReportFault::FIN_DATA_STALE},
    {"FIN_DATA_THRESHOLD_REACHED", ReportFault::FIN_DATA_THRESHOLD_REACHED},
    {"FIN_NODE_DIED", ReportFault::FIN_NODE_DIED},
    {"THRUSTER_RPM_STALE", ReportFault::THRUSTER_RPM_STALE},
    {"THRUSTER_RPM_THRESHOLD_REACHED", ReportFault::THRUSTER_RPM_THRESHOLD_REACHED},
    {"THRUSTER_TEMP_STALE", ReportFault::THRUSTER_TEMP_STALE},
    {"THRUSTER_TEMP_STAGNATED", ReportFault::THRUSTER_TEMP_STAGNATED},
    {"THRUSTER_TEMP_THRESHOLD_REACHED", ReportFault::THRUSTER_TEMP_THRESHOLD_REACHED},
    {"THRUSTER_NODE_DIED", ReportFault::THRUSTER_NODE_DIED},
    {"POSE_DATA_STALE", ReportFault::POSE_DATA_STALE},
    {"POSE_DATA_STAGNATED", ReportFault::POSE_DATA_STAGNATED},
    {"POSE_ROLL_THRESHOLD_REACHED", ReportFault::POSE_ROLL_THRESHOLD_REACHED},
    {"POSE_PITCH_THRESHOLD_REACHED", ReportFault::POSE_PITCH_THRESHOLD_REACHED},
    {"POSE_HEADING_THRESHOLD_REACHED", ReportFault::POSE_HEADING_THRESHOLD_REACHED},
    {"POSE_DEPTH_THRESHOLD_REACHED", ReportFault::POSE_DEPTH_THRESHOLD_REACHED},
    {"POSE_NODE_DIED", ReportFault::POSE_NODE_DIED},
    {"AUTOPILOT_NODE_DIED", ReportFault::AUTOPILOT_NODE_DIED},
    {"JAUS_NODE_DIED", ReportFault::JAUS_NODE_DIED},
    {"BATTERY_INFO_STALE", ReportFault::BATTERY_INFO_STALE},
    {"BATTERY_INFO_STAGNATED", ReportFault::BATTERY_INFO_STAGNATED},
    {"BATTERY_CURRENT_THRESHOLD_REACHED", ReportFault::BATTERY_CURRENT_THRESHOLD_REACHED},
    {"BATTERY_VOLTAGE_THRESHOLD_REACHED", ReportFault::BATTERY_VOLTAGE_THRESHOLD_REACHED},
    {"BATTERY_TEMPERATURE_THRESHOLD_REACHED", ReportFault::BATTERY_TEMPERATURE_THRESHOLD_REACHED},
    {"BATTERY_NODE_DIED", ReportFault::BATTERY_NODE_DIED},
    {"BATTERY_LEAK_DETECTED", ReportFault::BATTERY_LEAK_DETECTED},
    {"MISSION_PARAMETERS_OUT_OF_RANGE", ReportFault::MISSION_PARAMETERS_OUT_OF_RANGE},
    {"MISSION_NODE_DIED", ReportFault::MISSION_NODE_DIED},
    {NULL, 0u}
  };

  int i = 0;
  for (; fault_mapping[i].name != nullptr &&
         fault.compare(fault_mapping[i].name) != 0;
       ++i);
  return fault_mapping[i].code;
}

};  // namespace health_monitor
