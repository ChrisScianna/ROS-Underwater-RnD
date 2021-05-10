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

#include <type_traits>

/// Set of enum of values, useful as a type-safe "bitmask"
// Inspired by https://gpfault.net/posts/typesafe-bitmasks.txt.html
template<typename EnumT>
class ValueSet {
public:
  constexpr ValueSet() : mask_(0) {}

  constexpr ValueSet(EnumT value) : mask_(mask_for(value)) {}

  constexpr ValueSet operator|(ValueSet other) const
  {
    return ValueSet(mask_ | other.mask_);
  }

  ValueSet& operator|=(ValueSet other)
  {
    mask_ |= other.mask_;
    return *this;
  }

  constexpr ValueSet operator&(ValueSet other) const
  {
    return ValueSet(mask_ & other.mask_);
  }

  ValueSet& operator&=(ValueSet other)
  {
    mask_ &= other.mask_;
    return *this;
  }

  explicit constexpr operator bool() const
  {
    return mask_ != 0;
  }

private:
  static_assert(std::is_enum<EnumT>::value, "EnumT must be an enum type");
  using underlying_type = typename std::underlying_type<EnumT>::type;

  explicit constexpr ValueSet(underlying_type mask) : mask_(mask) {}

  static constexpr underlying_type mask_for(EnumT value)
  {
    return 1 << static_cast<underlying_type>(value);
  }

  underlying_type mask_;
};

template<typename EnumT, typename = typename std::enable_if<std::is_enum<EnumT>::value>::type>
constexpr ValueSet<EnumT> operator|(EnumT lhs, EnumT rhs)
{
  return ValueSet<EnumT>{lhs} | rhs;
}
