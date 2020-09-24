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

#ifndef __BEHAVIOR_FACTORY_H
#define __BEHAVIOR_FACTORY_H

#include <map>
#include <string>

#include "behaviors/altitude_heading.h"
#include "behaviors/attitude_servo.h"
#include "behaviors/depth_heading.h"
#include "behaviors/fixed_rudder.h"
#include "behaviors/payload_command.h"
#include "behaviors/pinger.h"
#include "behaviors/podlog.h"
#include "behaviors/waypoint.h"

namespace mission_manager {

/*
typedef Behavior *create_obj_fn_t(void);
typedef void destroy_obj_fn_t(Behavior *behavior);
typedef const char *get_xml_tag_fn_t(void);
typedef behavior_type_t get_type_fn_t(void);

typedef const char *get_topic_fn_t(void);
typedef ros::Publisher create_pub_fn_t(ros::NodeHandle nh, int queue_size);
typedef ros::Message *create_msg_fn_t();
typedef void destroy_msg_fn_t(ros::Message *msg);

typedef const char *get_service_fn_t(void);
typedef ros::ServiceClient create_srv_client_fn_t(bool persistent);
*/
/*
class IFactory {
public:
        IFactory();
        ~IFactory();

        void *lib_handle;

        // Common hooks
        create_obj_fn_t *create_obj_fn;
        destroy_obj_fn_t *destroy_obj_fn;
};
*/
class BehaviorFactory {
 public:
  BehaviorFactory();
  BehaviorFactory(const std::string& behavior_dir);
  virtual ~BehaviorFactory();

  /*
          bool addSubFactory(const std::string& xml_tag);
          IFactory *findSubFactory(const std::string& xml_tag);
  */
  Behavior* createBehavior(const std::string& name);
  //	void destroyBehavior(const std::string& name, Behavior *behavior);

 private:
  //	typedef std::map<std::string, IFactory *> factory_map_t;
  //	factory_map_t m_factories;
};

}  // namespace mission_manager

#endif
