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

#include <gtest/gtest.h>

#include "auv_interfaces/helpers.h"


TEST(AuvInterfacesHelpers, compare_default_states)
{
  auv_interfaces::State state;
  auv_interfaces::State another_state;

  constexpr double zero_tol = 0.;
  EXPECT_TRUE(auv_interfaces::almost_equal(
      state, another_state, zero_tol, zero_tol));

  constexpr double abs_tol = 0.;
  constexpr double rel_tol = 0.01;  // 1%
  EXPECT_TRUE(auv_interfaces::almost_equal(
      state, another_state, abs_tol, rel_tol));
}


TEST(AuvInterfacesHelpers, compare_non_default_states)
{
  auv_interfaces::State state_at_t0;
  state_at_t0.manoeuvring.pose.mean.position.x = 0.0;
  auv_interfaces::State state_at_t1;
  state_at_t1.manoeuvring.pose.mean.position.x = 1.0;

  constexpr double abs_tol = 0.01;
  constexpr double rel_tol = 0.01;  // 1%
  EXPECT_FALSE(auv_interfaces::almost_equal(
      state_at_t0, state_at_t1, abs_tol, rel_tol));

  auv_interfaces::State state_at_t2;
  state_at_t2.manoeuvring.pose.mean.position.x = 1.0;
  state_at_t2.manoeuvring.pose.mean.orientation.z = 0.001;
  EXPECT_TRUE(auv_interfaces::almost_equal(
      state_at_t1, state_at_t2, abs_tol, rel_tol));
}


TEST(AuvInterfacesHelpers, compare_states_in_different_frames)
{
  auv_interfaces::State state_in_frame_a;
  state_in_frame_a.seakeeping.reference_frame_id = "a";
  auv_interfaces::State state_in_frame_b;
  state_in_frame_b.seakeeping.reference_frame_id = "b";

  constexpr double zero_tol = 0.;
  constexpr double rel_tol = 1.;  // 100%
  EXPECT_FALSE(auv_interfaces::almost_equal(
      state_in_frame_a, state_in_frame_b, zero_tol, rel_tol));
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
