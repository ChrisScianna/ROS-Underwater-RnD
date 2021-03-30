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

#include "mission_control/behaviors/introspection.h"

#include <string>

namespace mission_control
{
namespace introspection
{
static constexpr char ACTIVE_PATH_KEY[] = "introspection/active_path";

std::string extendActivePath(BT::Blackboard* bb, const std::string& node)
{
  std::string parent_path;
  if (bb->get(ACTIVE_PATH_KEY, parent_path))
    {
      bb->set(ACTIVE_PATH_KEY, parent_path + "/" + node);
    }
  else
    {
      bb->set(ACTIVE_PATH_KEY, node);
    }
  return parent_path;
}

void setActivePath(BT::Blackboard* bb, const std::string& path)
{
  bb->set(ACTIVE_PATH_KEY, path);
}

bool getActivePath(const BT::Blackboard* bb, std::string& path)
{
  return bb->get(ACTIVE_PATH_KEY, path);
}

std::string getActivePath(const BT::Blackboard* bb)
{
  std::string path;
  if (!getActivePath(bb, path))
  {
    path = "?";
  }
  return path;
}

std::string getActivePath(const BT::Tree& tree)
{
  // NOTE(hidmic): no mutation will occur, it is safe to cast
  return getActivePath(const_cast<BT::Tree&>(tree).rootBlackboard().get());
}

}  // namespace introspection
}  // namespace mission_control
