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

// Original version: Christopher Scianna Christopher.Scianna@us.QinetiQ.com

/*  Behavioral podlog
  <podlog>
      <description>
          Turn on logging for the servocntl.
      </description>
      <when unit="sec">0</when>
      <log_id state="on">servocntl</log_id>
  </podlog>
*/

#ifndef MISSION_CONTROL_BEHAVIORS_PODLOG_H
#define MISSION_CONTROL_BEHAVIORS_PODLOG_H

#include <string>
#include "mission_control/behavior.h"

#undef USE_PODLOG
#define USE_LOGBASE

#ifdef USE_PODLOG
#undef USE_LOGBASE
#else
#define USE_LOGBASE
#endif

#ifdef USE_PODLOG
#include "podlog/SetLogState.h"
#else

#endif

namespace mission_control
{

class PodlogBehavior : public Behavior
{
 public:
  PodlogBehavior();
  virtual ~PodlogBehavior();

  virtual bool parseMissionFileParams();
  virtual void callService(ros::NodeHandle node_handle);

 private:
  bool m_logging;
  std::string m_logid;
};

}   //  namespace mission_control

#endif  //  MISSION_CONTROL_BEHAVIORS_PODLOG_H
