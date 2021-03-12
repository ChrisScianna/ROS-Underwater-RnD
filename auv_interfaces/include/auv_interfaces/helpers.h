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

#ifndef AUV_INTERFACES_HELPERS_H
#define AUV_INTERFACES_HELPERS_H

#include "auv_interfaces/CartesianPose.h"
#include "auv_interfaces/GeneralizedVector.h"
#include "auv_interfaces/GeoLocation.h"
#include "auv_interfaces/ManoeuvringState.h"
#include "auv_interfaces/MotionPerturbation.h"
#include "auv_interfaces/SeakeepingState.h"
#include "auv_interfaces/State.h"


namespace auv_interfaces
{

/// Compare numbers `a` and `b` for approximate equality.
/**
 * True if the following equation holds True:
 *
 *   \f[ |a - b| <= \left(abs_tol + rel_tol |b|\right) \f]
 *
 * Note that comparison is not symmetric in `a` and `b`.
 */
bool almost_equal(
    double a, double b,
    double abs_tol,
    double rel_tol);

/// Compare poses `a` and `b` for approximate equality.
/**
 * Note that the same tolerances apply to both position and orientation.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const CartesianPose& a,
    const CartesianPose& b,
    double abs_tol,
    double rel_tol);

/// Compare vectors `a` and `b` for approximate equality.
/**
 * Note that the same tolerances apply to both linear and angular components.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const GeneralizedVector& a,
    const GeneralizedVector& b,
    double abs_tol,
    double rel_tol);

/// Compare pertubations `a` and `b` for approximate equality.
/**
 * Note that the same tolerances apply to both linear and angular components.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const MotionPerturbation& a,
    const MotionPerturbation& b,
    double abs_tol,
    double rel_tol);

/// Compare seakeeping states `a` and `b` for approximate equality.
/**
 * Reference frames are checked for equality.
 *
 * Note that the same tolerances apply to both linear and angular components.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const SeakeepingState& a,
    const SeakeepingState& b,
    double abs_tol,
    double rel_tol);

/// Compare manoeuvring states `a` and `b` for approximate equality.
/**
 * Reference frames are checked for equality.
 *
 * Note that the same tolerances apply to both linear and angular components.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const ManoeuvringState& a,
    const ManoeuvringState& b,
    double abs_tol,
    double rel_tol);


/// Compare global positions `a` and `b` for approximate equality.
/**
 * Note that the same tolerances apply to latitude, longitude, and altitude.
 * Use a non-zero absolute tolerance only to override the relative tolerance
 * when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const GeoLocation& a,
    const GeoLocation& b,
    double abs_tol,
    double rel_tol);

/// Compare state `a` and `b` for approximate equality.
/**
 * Reference frames are checked for equality.
 *
 * Note that the same tolerances apply to all quantities.
 * Use a non-zero absolute tolerance only to override the
 * relative tolerance when magnitudes are close to 0.
 *
 * \see almost_equal(double,double,double,double)
 */
bool almost_equal(
    const State& a,
    const State& b,
    double abs_tol,
    double rel_tol);

}  // namespace auv_interfaces

#endif  // AUV_INTERFACES_HELPERS_H
