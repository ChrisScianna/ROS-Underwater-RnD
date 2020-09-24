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

#ifndef __PODLOG_H
#define __PODLOG_H

#include <string>

#include "../behavior.h"

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
//#include "logbase.h"
#endif

namespace mission_manager {

class PodlogBehavior : public Behavior {
 public:
  PodlogBehavior();
  virtual ~PodlogBehavior();

  virtual bool parseMissionFileParams();
  //	bool parseXml(xmlNodePtr node);

  virtual void callService(ros::NodeHandle node_handle);
  /*
          ros::ServiceClient createServiceClient(bool persistent) {
  #ifdef USE_PODLOG
                  return Behavior::createServiceClient<podlog::SetLogState>(m_service, persistent);
  #else
                  return ros::ServiceClient();
  #endif
          }

          void *createSrv() {
  #ifdef USE_PODLOG
                  return Behavior::createSrv<podlog::SetLogState>();
  #else
                  return Behavior::createSrv<logbase::SetLogState>();
  #endif
          }

          void destroySrv(void *srv) {
  #ifdef USE_PODLOG
                  return Behavior::destroySrv<podlog::SetLogState>(srv);
  #else
                  return Behavior::destroySrv<logbase::SetLogState>(srv);
  #endif
          }

          void populateSrv(void *srv);

          void callSrv(ros::ServiceClient srvcli, void *srv) {
  #ifdef USE_PODLOG
                  return Behavior::callSrv<podlog::SetLogState>(srvcli, srv);
  #else
                  std::string s = "/" + m_logid + "/set_log_state";
                  srvcli = Behavior::createServiceClient<logbase::SetLogState>(s, false);
                  return Behavior::callSrv<logbase::SetLogState>(srvcli, srv);
  #endif
          }
  */
 private:
  bool m_logging;
  std::string m_logid;
};

}  // namespace mission_manager

#endif
