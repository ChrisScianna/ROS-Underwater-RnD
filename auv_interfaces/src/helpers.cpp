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

#include <cmath>

#include "auv_interfaces/CartesianPose.h"
#include "auv_interfaces/GeneralizedVector.h"
#include "auv_interfaces/GeoLocation.h"
#include "auv_interfaces/ManoeuvringState.h"
#include "auv_interfaces/MotionPerturbation.h"
#include "auv_interfaces/SeakeepingState.h"
#include "auv_interfaces/State.h"


namespace auv_interfaces
{

bool almost_equal(double a, double b, double abs_tol, double rel_tol)
{
  return std::abs(a - b) <= abs_tol + rel_tol * std::abs(b);
}

static bool almost_equal(
    const geometry_msgs::Point& a,
    const geometry_msgs::Point& b,
    double abs_tol, double rel_tol)
{
  return (almost_equal(a.x, b.x, abs_tol, rel_tol) &&
          almost_equal(a.y, b.y, abs_tol, rel_tol) &&
          almost_equal(a.z, b.z, abs_tol, rel_tol));
}

static bool almost_equal(
    const geometry_msgs::Vector3& a,
    const geometry_msgs::Vector3& b,
    double abs_tol, double rel_tol)
{
  return (almost_equal(a.x, b.x, abs_tol, rel_tol) &&
          almost_equal(a.y, b.y, abs_tol, rel_tol) &&
          almost_equal(a.z, b.z, abs_tol, rel_tol));
}

bool almost_equal(
    const CartesianPose& a,
    const CartesianPose& b,
    double abs_tol, double rel_tol)
{
  // TODO(hidmic): handle orientation singularities
  return (almost_equal(a.position, b.position, abs_tol, rel_tol) &&
          almost_equal(a.orientation, b.orientation, abs_tol, rel_tol));
}

bool almost_equal(
    const GeneralizedVector& a,
    const GeneralizedVector& b,
    double abs_tol, double rel_tol)
{
  return (almost_equal(a.linear, b.linear, abs_tol, rel_tol) &&
          almost_equal(a.angular, b.angular, abs_tol, rel_tol));
}

bool almost_equal(
    const MotionPerturbation& a,
    const MotionPerturbation& b,
    double abs_tol, double rel_tol)
{
  return (almost_equal(a.displacement.mean, b.displacement.mean, abs_tol, rel_tol) &&
          almost_equal(a.rate.mean, b.rate.mean, abs_tol, rel_tol));
}

bool almost_equal(
    const SeakeepingState& a,
    const SeakeepingState& b,
    double abs_tol, double rel_tol)
{
  return ((a.reference_frame_id == b.reference_frame_id) &&
          almost_equal(a.perturbation, b.perturbation, abs_tol, rel_tol));
}

bool almost_equal(
    const ManoeuvringState& a,
    const ManoeuvringState& b,
    double abs_tol, double rel_tol)
{
  return ((a.reference_frame_id == b.reference_frame_id) &&
          almost_equal(a.pose.mean, b.pose.mean, abs_tol, rel_tol) &&
          almost_equal(a.velocity.mean, b.velocity.mean, abs_tol, rel_tol));
}

static bool almost_equal(const geographic_msgs::GeoPoint& a,
                  const geographic_msgs::GeoPoint& b,
                  double abs_tol, double rel_tol)
{
  return (almost_equal(a.latitude, b.latitude, abs_tol, rel_tol) &&
          almost_equal(a.longitude, b.longitude, abs_tol, rel_tol) &&
          almost_equal(a.altitude, b.altitude, abs_tol, rel_tol));
}

bool almost_equal(const GeoLocation& a, const GeoLocation& b,
                  double abs_tol, double rel_tol)
{
  return almost_equal(a.position, b.position, abs_tol, rel_tol);
}

bool almost_equal(const State& a, const State& b,
                  double abs_tol, double rel_tol)
{
  return (almost_equal(a.geolocation, b.geolocation, abs_tol, rel_tol) &&
          almost_equal(a.manoeuvring, b.manoeuvring, abs_tol, rel_tol) &&
          almost_equal(a.seakeeping, b.seakeeping, abs_tol, rel_tol));
}

}  // namespace auv_interfaces
