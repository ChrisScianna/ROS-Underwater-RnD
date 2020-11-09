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

#include "mission_manager/behavior_factory.h"

#include <dirent.h>
#include <dlfcn.h>
#include <regex.h>
#include <ros/console.h>
#include <string>
#include <boost/assign.hpp>

using mission_manager::BehaviorFactory;
using mission_manager::Behavior;

BehaviorFactory::BehaviorFactory() {}

BehaviorFactory::BehaviorFactory(const std::string& behavior_dir) {}

BehaviorFactory::~BehaviorFactory() {}

Behavior* BehaviorFactory::createBehavior(const std::string& name)
{
  Behavior* beh = NULL;

  if (name.compare("waypoint") == 0)
  {
    beh = new WaypointBehavior();
  }
  else if (name.compare("attitude_servo") == 0)
  {
    beh = new AttitudeServoBehavior();
  }
  else if (name.compare("depth_heading") == 0)
  {
    beh = new DepthHeadingBehavior();
  }
  else if (name.compare("altitude_heading") == 0)
  {
    beh = new AltitudeHeadingBehavior();
  }
  else if (name.compare("fixed_rudder") == 0)
  {
    beh = new FixedRudderBehavior();
  }

  else if (name.compare("payload") == 0)
  {
    beh = new PayloadCommandBehavior();
  }
  else
  {
    std::cout << "No factory for behavior " << name << std::endl;
  }

  return beh;
}
