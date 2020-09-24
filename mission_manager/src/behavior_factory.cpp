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

#include "behavior_factory.h"

#include <dirent.h>
#include <dlfcn.h>
#include <regex.h>
#include <ros/console.h>
#include <boost/assign.hpp>

using namespace mission_manager;

/*
IFactory::IFactory() : lib_handle(NULL)
{

}

IFactory::~IFactory()
{
        char *err;

        if (lib_handle != NULL) {
                dlerror();
                dlclose(lib_handle);
                if ((err = dlerror()) != NULL)
                        fprintf(stderr, "dlclose(): %s\n", err);
                lib_handle = NULL;
        }
}
*/

BehaviorFactory::BehaviorFactory() {}

BehaviorFactory::BehaviorFactory(const std::string& behavior_dir) {
  /*	DIR *dp;
          struct dirent *dirp;
          regex_t preg;
          std::string libpath = behavior_dir + '/';

          regcomp(&preg, "libPodBhvr.+\\.so", REG_EXTENDED);

          if ((dp = opendir(behavior_dir.c_str())) != NULL) {
                  while ((dirp = readdir(dp)) != NULL) {
                          if (regexec(&preg, dirp->d_name, 0, NULL, 0) == 0) {
                                  addSubFactory(libpath + dirp->d_name);
                          }
                  }

                  closedir(dp);
          }

          regfree(&preg);
  */
}

BehaviorFactory::~BehaviorFactory() {
  /*	factory_map_t::iterator iter;

          for (iter = m_factories.begin(); iter != m_factories.end(); ++iter)
                  delete iter->second;
          m_factories.clear();
  */
}
/*
bool BehaviorFactory::addSubFactory(const std::string& libpath)
{
        IFactory *factory;
        char *err;

        factory = new IFactory();

        // Open the library
        dlerror();
        if ((factory->lib_handle = dlopen(libpath.c_str(), RTLD_LOCAL | RTLD_NOW)) == NULL) {
        fprintf(stderr, "dlopen: %s\n", dlerror());
        delete factory;
        return false;
    }

        // Bind the Behavior constructor helper function
        dlerror();
        factory->create_obj_fn = (create_obj_fn_t *)dlsym(factory->lib_handle, "createObj");
        if ((err = dlerror()) != NULL) {
                fprintf(stderr, "dlsym(): %s\n", err);
                delete factory;
                return false;
        }

        // Bind the Behavior destructor helper function
        dlerror();
        factory->destroy_obj_fn = (destroy_obj_fn_t *)dlsym(factory->lib_handle, "destroyObj");
        if ((err = dlerror()) != NULL) {
                fprintf(stderr, "dlsym(): %s\n", err);
                delete factory;
                return false;
        }

        Behavior *beh = factory->create_obj_fn();
        m_factories[beh->getXmlTag()] = factory;
        factory->destroy_obj_fn(beh);

        ROS_DEBUG("Successfully added behavior library [%s]", libpath.c_str());

        return true;
}

IFactory *BehaviorFactory::findSubFactory(const std::string& name)
{
        factory_map_t::iterator iter = m_factories.find(name);
        if (iter == m_factories.end()) return NULL;
        return iter->second;
}
*/
Behavior* BehaviorFactory::createBehavior(const std::string& name) {
  /*	factory_map_t::iterator iter;

          if ((iter = m_factories.find(name)) == m_factories.end())
                  return NULL;

          Behavior *beh = iter->second->create_obj_fn();
          beh->factory = iter->second;
  */
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
  //	else if (name.compare("podlog") == 0)
  //	{
  //		beh = new PodlogBehavior();
  //	}
  else if (name.compare("fixed_rudder") == 0) 
  {
    beh = new FixedRudderBehavior();
  }
  //	else if (name.compare("pinger") == 0)
  //	{
  //		beh = new PingerBehavior();
  //	}
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
/*
void BehaviorFactory::destroyBehavior(const std::string& name, Behavior *behavior)
{
        factory_map_t::iterator iter;

        if ((iter = m_factories.find(name)) == m_factories.end())
                return;

        iter->second->destroy_obj_fn(behavior);
}
*/
