/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, QinetiQ, Inc.
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

#ifndef IXBLUE_C3_INS_IXBLUE_C3_INS_H
#define IXBLUE_C3_INS_IXBLUE_C3_INS_H

#include <ros/ros.h>

#include <auv_interfaces/StateStamped.h>
#include <diagnostic_tools/diagnosed_publisher.h>
#include <diagnostic_tools/health_check.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <health_monitor/ReportFault.h>

#include "ixblue_c3_ins/c3_protocol.h"


namespace ixblue_c3_ins
{

class ixBlueC3InsDriver
{
public:
  ixBlueC3InsDriver();
  ~ixBlueC3InsDriver();

  void spin();

private:
  void publish(const c3_protocol::nav_long::nav_long_data_t& data);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher nav_long_pub_;

  qna::diagnostic_tools::DiagnosedPublisher<
    auv_interfaces::StateStamped> state_pub_;
  qna::diagnostic_tools::HealthCheck<double> orientation_roll_check_;
  qna::diagnostic_tools::HealthCheck<double> orientation_pitch_check_;
  qna::diagnostic_tools::HealthCheck<double> orientation_yaw_check_;
  diagnostic_updater::Updater diagnostics_updater_;

  int fd_;  // Listening UDP socket to "consume" data from the INS
};

}  // namespace ixblue_c3_ins

#endif  // IXBLUE_C3_INS_IXBLUE_C3_INS_H
