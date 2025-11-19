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
 * @file spline_trajectory_generator.cpp
 * @brief Spline trajectory generator implementation
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 * @copyright Copyright (c) 2025 Universidad Politécnica de Madrid
 * @license BSD-3-Clause
 */

#include "spline_trajectory_generator.hpp"
#include <cmath>

namespace trajectory
{

// ============================================================================
// Path Implementation
// ============================================================================

Path::Path(const std::vector<TrajectorySetpoint> & setpoints)
: setpoints_(setpoints), current_index_(0)
{
  if (setpoints_.empty()) {
    throw std::invalid_argument("Path requires at least one setpoint");
  }
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
Path::getWaypoints(int num) const
{
  std::vector<Eigen::Vector3d> waypoints;
  std::vector<Eigen::Vector3d> tangents;

  for (int i = 0; i < num; ++i) {
    size_t idx = current_index_ + i;
    if (idx >= setpoints_.size()) {
      break;
    }
    waypoints.push_back(setpoints_[idx].position);
    tangents.push_back(setpoints_[idx].tangent);
  }

  return std::make_tuple(waypoints, tangents);
}

bool Path::advance()
{
  if (current_index_ + 1 < setpoints_.size()) {
    current_index_++;
    return true;
  }
  return false;
}

const TrajectorySetpoint * Path::getCurrentSetpoint() const
{
  if (current_index_ >= setpoints_.size()) {
    return nullptr;
  }
  return &setpoints_[current_index_];
}

size_t Path::remainingWaypoints() const
{
  return setpoints_.size() - current_index_;
}

// ============================================================================
// SplineTrajectoryGenerator Implementation
// ============================================================================

SplineTrajectoryGenerator::SplineTrajectoryGenerator(
  const std::vector<TrajectorySetpoint> & setpoints,
  int num_wp,
  int spline_samples,
  int spline_degree)
: num_wp_(num_wp),
  spline_samples_(spline_samples),
  spline_degree_(spline_degree),
  regeneration_count_(0),
  path_(setpoints)
{
  if (num_wp < 2) {
    throw std::invalid_argument("num_wp must be at least 2");
  }

  // Generate initial spline
  generateSpline();
}

void SplineTrajectoryGenerator::generateSpline()
{
  // Get next num_wp waypoints from Path
  auto [waypoints, tangents] = path_.getWaypoints(num_wp_);

  if (waypoints.size() < 2) {
    throw std::runtime_error("Not enough waypoints remaining to generate spline");
  }

  // Create new spline
  spline_ = std::make_unique<spline::HermiteSpline>(waypoints, tangents);

  // Compute arc length reparametrization
  spline_reparametrization_ = spline::computeArcLengthReparametrization(
    *spline_, spline_samples_, spline_degree_);

  regeneration_count_++;
}

SplineEvaluationResult SplineTrajectoryGenerator::evaluateArcLengthSpline(double s) const
{
  if (!spline_) {
    throw std::runtime_error("Spline not initialized");
  }

  // Normalize s to [0, 1]
  double s_norm = std::clamp(
    s / spline_reparametrization_.total_length, 0.0, 1.0);

  // Polynomial approximation: t(s_norm)
  double t = 0.0;
  int n_coeffs = spline_reparametrization_.poly_coeffs.size();
  for (int i = 0; i < n_coeffs; ++i) {
    t += spline_reparametrization_.poly_coeffs[i] *
      std::pow(s_norm, n_coeffs - 1 - i);
  }

  // Evaluate spline at parameter t
  Eigen::Vector3d position = spline_->evaluate(t);
  Eigen::Vector3d derivative = spline_->evaluateDerivative(t);

  SplineEvaluationResult result;
  result.position = position;
  result.derivative = derivative;
  result.theta = s_norm;
  result.reparametrization = spline_reparametrization_;

  return result;
}

std::tuple<double, SplineEvaluationResult>
SplineTrajectoryGenerator::evaluateSpline(double s)
{
  // Evaluate spline at current arc length s
  auto result = evaluateArcLengthSpline(s);

  double new_s = s;

  // Check if we've passed the end of current spline
  if (result.theta >= 1.0) {
    // Advance to next waypoint
    if (path_.advance()) {
      // Regenerate spline with new waypoints
      try {
        generateSpline();
        // Reset s to 0 for the new spline
        new_s = 0.0;
        // Re-evaluate at s=0
        result = evaluateArcLengthSpline(new_s);
      } catch (const std::runtime_error & e) {
        // Not enough waypoints remaining, keep current spline
        // Keep s as is (will be clamped to spline length)
      }
    }
    // else: No more waypoints available, keep current spline
  }

  return std::make_tuple(new_s, result);
}

}  // namespace trajectory
