// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file trajectory_generator.cpp
 *
 * @brief Trajectory Generator implementation in C++
 *
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 */

#include "trajectory_generator.hpp"
#include <stdexcept>
#include <iostream>

namespace spline
{

TrajectoryGenerator::TrajectoryGenerator(
  const std::vector<Setpoint> & setpoints,
  int num_wp,
  int spline_samples)
: setpoints_(setpoints), num_wp_(num_wp), spline_samples_(spline_samples)
{
  path_index_ = 0;
  used_setpoints_.reserve(static_cast<size_t>(num_wp_));
  for (int i = 0; i < num_wp_; ++i) {
    used_setpoints_.push_back(setpoints_[i]);
  }
  spline_ = std::make_unique<HermiteSpline>(used_setpoints_);
  reparametrization_ =
    computeArcLengthReparametrization(*spline_, spline_samples_);
}

std::pair<const ArcLengthReparametrizationResult, double> TrajectoryGenerator::evaluateSpline(
  double s,
  Eigen::Vector3d & position,
  Eigen::Vector3d & derivative)
{
  // Check if we need to update the spline segment
  double t = reparametrizeArcLengthToT(s, reparametrization_);
  std::cout << "Evaluating spline at arc length s = " << s << ", corresponding to t = " << t <<
    std::endl;

  if (t >= 1.0) {
    // Advance path index if possible
    if (path_index_ + num_wp_ < setpoints_.size()) {
      path_index_++;
      used_setpoints_.clear();
      for (int i = 0; i < num_wp_; ++i) {
        used_setpoints_.push_back(setpoints_[path_index_ + i]);
      }
      spline_ = std::make_unique<HermiteSpline>(used_setpoints_);
      reparametrization_ =
        computeArcLengthReparametrization(*spline_, spline_samples_);
      // TODO(RPS98): More accurate s update
      s = 0.0;    // Reset arc length parameter (heuristic)
      t = reparametrizeArcLengthToT(s, reparametrization_);

      std::cout << "Spline segment updated. New path index: " << path_index_ << std::endl;
    }
  }

  // Evaluate position and derivative at arc length s
  std::tie(position, derivative) = evaluateArcLengthSplineWithDerivative(
    s, reparametrization_, *spline_);

  return std::make_pair(reparametrization_, s);
}

bool TrajectoryGenerator::isFinished(double s) const
{
  if (!spline_) {
    return true;
  }

  if (path_index_ + static_cast<size_t>(num_wp_) < setpoints_.size()) {
    return false;
  }

  double t = reparametrizeArcLengthToT(s, reparametrization_);
  return t >= spline_->getTmax();
}

}  // namespace spline
