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

#ifndef MISSION_CONTROL_BEHAVIORS_HELPERS_H
#define MISSION_CONTROL_BEHAVIORS_HELPERS_H

#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/tree_node.h>

#include <string>
#include <utility>


namespace mission_control
{
namespace internal
{

inline
bool
UpdatePortsList(
  const BT::PortsList& input,
  BT::PortsList* output)
{
  output->insert(input.begin(), input.end());
  return true;
}

inline
bool
UpdatePortsList(
  BT::PortsList::value_type&& input,  // NOLINT
  BT::PortsList* output)
{
  output->insert(input).second;
}

template<typename...Args>
void expand(Args&&...)
{
}

}  // namespace internal

template<typename...Args>
BT::PortsList
MakePortsList(Args&&... ports)  // NOLINT
{
  BT::PortsList list;
  internal::expand(internal::UpdatePortsList(
    std::forward<Args>(ports), &list)...);
  return list;
}

template<typename T>
struct HasTolerance
{
  static
  BT::PortsList::value_type
  InputPort(BT::StringView name)
  {
    return BT::InputPort<T>(
      BT::StrCat(name, "-tolerance"), T{0},
      BT::StrCat("Tolerance for ", name));
  }
};

template<typename T>
struct HasAngleUnits
{
  static
  BT::PortsList::value_type
  InputPort(BT::StringView name)
  {
    return BT::InputPort<std::string>(
      BT::StrCat(name, "-units"), "radians",
      BT::StrCat("Units for ", name, " angle ",
                 "(supported units are: radians, degrees)"));
  }

  static
  BT::Result
  applyToInputValue(
    BT::TreeNode* node,
    const std::string& key,
    T& value)
  {
    std::string units;
    BT::Result result =
      node->getInput<std::string>(
        key + "-units", units);
    if (result)
    {
      if ("degrees" == units)
      {
        value *= M_PI / 180.0;
      }
      else if ("radians" != units)
      {
        result = nonstd::make_unexpected(
          "Angle unit '" + units + "' is unknown " +
          "(supported units are: radians, degrees)");
      }
    }
    return result;
  }
};

template<typename T, template <typename> class...Traits>
BT::PortsList
InputPort(
  BT::StringView name,
  BT::StringView description)
{
  return MakePortsList(
    BT::InputPort(name, description),
    Traits<T>::InputPort(name)...);
}

template<typename T, template <typename> class...Traits>
BT::PortsList
InputPort(
  BT::StringView name,
  const T& default_value,
  BT::StringView description)
{
  return MakePortsList(
    BT::InputPort(name, default_value, description),
    Traits<T>::InputPort(name)...);
}

template <typename T>
BT::Result getInputValue(
  BT::TreeNode* node,
  const std::string& key,
  T& value)
{
  return node->getInput<T>(key, value);
}

template <
  typename T,
  template <typename> class Trait,
  template <typename> class...Others>
BT::Result getInputValue(
  BT::TreeNode* node,
  const std::string& key,
  T& value)
{
  BT::Result result = getInputValue<T, Others...>(node, key, value);
  if (result)
  {
    result = Trait<T>::applyToInputValue(node, key, value);
  }
  return result;
}

template <typename T>
BT::Result getInputTolerance(
    BT::TreeNode* node,
    const std::string& key,
    T& tolerance)
{
  return node->getInput<T>(key + "-tolerance", tolerance);
}

template <
  typename T,
  template <typename> class Trait,
  template <typename> class...Others>
BT::Result getInputTolerance(
  BT::TreeNode* node,
  const std::string& key,
  T& tolerance)
{
  BT::Result result = getInputTolerance<T, Others...>(node, key, tolerance);
  if (result)
  {
    result = Trait<T>::applyToInputValue(node, key, tolerance);
  }
  return result;
}

}  // namespace mission_control

#endif  // MISSION_CONTROL_BEHAVIORS_HELPERS_H
