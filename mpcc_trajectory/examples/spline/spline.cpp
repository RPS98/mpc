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
 * @file spline.cpp
 *
 * @brief Hermite Spline implementation in C++
 *
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 */

#include "spline.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

namespace spline
{

HermiteSpline::HermiteSpline(
  const std::vector<Setpoint> & setpoints)
: setpoints_(setpoints)
{
  // Validate input
  if (setpoints_.size() < 2) {
    throw std::invalid_argument("Need at least 2 setpoints");
  }

  // Extract points and tangents from setpoints
  setpoints_ = setpoints;
}

std::tuple<double, double, double, double> HermiteSpline::hermiteBasis(double t) const
{
  double t2 = t * t;
  double t3 = t2 * t;

  double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  double h10 = t3 - 2.0 * t2 + t;
  double h01 = -2.0 * t3 + 3.0 * t2;
  double h11 = t3 - t2;

  return std::make_tuple(h00, h10, h01, h11);
}

std::tuple<double, double, double, double> HermiteSpline::hermiteBasisDerivative(double t) const
{
  double t2 = t * t;

  double h00_d = 6.0 * t2 - 6.0 * t;
  double h10_d = 3.0 * t2 - 4.0 * t + 1.0;
  double h01_d = -6.0 * t2 + 6.0 * t;
  double h11_d = 3.0 * t2 - 2.0 * t;

  return std::make_tuple(h00_d, h10_d, h01_d, h11_d);
}

int HermiteSpline::findSegment(double t) const
{
  if (t < 0.0 || t > static_cast<double>(setpoints_.size() - 1)) {
    throw std::out_of_range("Parameter t is out of bounds");
  }
  // Check if t is in [0, 1, 2, ..., N-1]
  // For t in [0,1) -> idx=0, [1,2) -> idx=1, etc.
  return static_cast<int>(std::floor(t));
}

Eigen::Vector3d HermiteSpline::evaluate(double t) const
{
  // Clamp to valid range
  t = std::clamp(t, 0.0, static_cast<double>(setpoints_.size() - 1));

  // Find segment
  int idx = findSegment(t);

  // Get segment data
  const Setpoint & sp0 = setpoints_[idx];
  const Setpoint & sp1 = setpoints_[idx + 1];

  const Eigen::Vector3d & p0 = sp0.position;
  const Eigen::Vector3d & p1 = sp1.position;
  const Eigen::Vector3d & m0 = sp0.tangent;
  const Eigen::Vector3d & m1 = sp1.tangent;

  // Normalize to [0, 1] within the segment
  double t_segment = t - static_cast<double>(idx);

  // Evaluate Hermite polynomial
  const double dt = 1.0;  // Since segments are normally spaced by 1.0
  auto [h00, h10, h01, h11] = hermiteBasis(t_segment);

  return h00 * p0 + h10 * m0 * dt + h01 * p1 + h11 * m1 * dt;
}

Eigen::Vector3d HermiteSpline::evaluateDerivative(double t) const
{
  // Clamp to valid range
  t = std::clamp(t, 0.0, static_cast<double>(setpoints_.size() - 1));

  // Find segment
  int idx = findSegment(t);

  // Get segment data
  const Setpoint & sp0 = setpoints_[idx];
  const Setpoint & sp1 = setpoints_[idx + 1];

  const Eigen::Vector3d & p0 = sp0.position;
  const Eigen::Vector3d & p1 = sp1.position;
  const Eigen::Vector3d & m0 = sp0.tangent;
  const Eigen::Vector3d & m1 = sp1.tangent;

  // Normalize to [0, 1] within the segment
  double t_segment = t - static_cast<double>(idx);

  // Evaluate Hermite polynomial
  const double dt = 1.0;  // Since segments are normally spaced by 1.0
  auto [h00_d, h10_d, h01_d, h11_d] = hermiteBasisDerivative(t_segment);
  Eigen::Vector3d dp_ds = h00_d * p0 + h10_d * m0 * dt + h01_d * p1 + h11_d * m1 * dt;

  // Chain rule: dp/dt = dp/ds * ds/dt
  double ds_dt = 1.0 / dt;
  return dp_ds * ds_dt;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> HermiteSpline::evaluateWithDerivative(double t) const
{
  Eigen::Vector3d position = evaluate(t);
  Eigen::Vector3d derivative = evaluateDerivative(t);
  return std::make_pair(position, derivative);
}

ArcLengthReparametrizationResult computeArcLengthReparametrization(
  const HermiteSpline & spline,
  int n_samples)
{
  // Validate input parameters
  if (n_samples < 2) {
    throw std::invalid_argument(
            "computeArcLengthReparametrization: n_samples must be >= 2");
  }

  ArcLengthReparametrizationResult result;

  // Reserve memory
  result.arc_lengths.reserve(n_samples);
  result.t_values.reserve(n_samples);

  // Define parameter range and sampling interval
  const double t_min = 0.0;
  const double t_max = spline.getTmax();
  const double dt = (t_max - t_min) / static_cast<double>(n_samples - 1);

  // Initialize first point
  result.arc_lengths.push_back(0.0);
  result.t_values.push_back(t_min);

  double arc_length = 0.0;
  double norm_prev = spline.evaluateDerivative(t_min).norm();

  // Compute cumulative arc length using trapezoidal integration
  for (int i = 1; i < n_samples; ++i) {
    const double t_curr = t_min + static_cast<double>(i) * dt;
    const double norm_curr = spline.evaluateDerivative(t_curr).norm();

    // Trapezoidal rule: area = 0.5 * (f(x_i) + f(x_i+1)) * dx
    arc_length += 0.5 * (norm_prev + norm_curr) * dt;

    result.arc_lengths.push_back(arc_length);
    result.t_values.push_back(t_curr);

    norm_prev = norm_curr;
  }

  result.total_length = arc_length;
  result.setpoints = spline.getSetpoints();

  if (result.total_length <= 0.0) {
    throw std::runtime_error("Total arc length is non-positive");
  }

  return result;
}

double reparametrizeArcLengthToT(
  double s,
  const ArcLengthReparametrizationResult & reparametrization)
{
  // Clamp s to valid range
  double s_clamped = std::clamp(s, 0.0, reparametrization.total_length);

  // Binary search to find the interval containing s
  auto it = std::lower_bound(
    reparametrization.arc_lengths.begin(),
    reparametrization.arc_lengths.end(),
    s_clamped);

  // Handle edge cases
  if (it == reparametrization.arc_lengths.begin()) {
    // s is at or before the first point
    return 0.0;
  }

  if (it == reparametrization.arc_lengths.end()) {
    // s is at or after the last point
    return reparametrization.t_values.back();
  }

  // Linear interpolation between two points
  size_t idx_high = std::distance(reparametrization.arc_lengths.begin(), it);
  size_t idx_low = idx_high - 1;

  double s_low = reparametrization.arc_lengths[idx_low];
  double s_high = reparametrization.arc_lengths[idx_high];
  double t_low = reparametrization.t_values[idx_low];
  double t_high = reparametrization.t_values[idx_high];

  // Interpolation factor
  double alpha = (s_clamped - s_low) / (s_high - s_low);

  // Interpolate t
  double t = t_low + alpha * (t_high - t_low);

  return t;
}


Eigen::Vector3d evaluateArcLengthSpline(
  double s,
  const ArcLengthReparametrizationResult & reparametrization,
  const HermiteSpline & spline)
{
  // Compute corresponding t for given s
  double t = reparametrizeArcLengthToT(s, reparametrization);

  // Evaluate spline at interpolated t
  return spline.evaluate(t);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> evaluateArcLengthSplineWithDerivative(
  double s,
  const ArcLengthReparametrizationResult & reparametrization,
  const HermiteSpline & spline)
{
  // Compute corresponding t for given s
  double t = reparametrizeArcLengthToT(s, reparametrization);

  // Evaluate position and derivative wrt t
  Eigen::Vector3d p = spline.evaluate(t);
  Eigen::Vector3d dp_dt = spline.evaluateDerivative(t);

  // Clamp s to valid range
  double s_clamped = std::clamp(s, 0.0, reparametrization.total_length);

  // Binary search to find the interval containing s for dt/ds calculation
  auto it = std::lower_bound(
    reparametrization.arc_lengths.begin(),
    reparametrization.arc_lengths.end(),
    s_clamped);

  // Convert dp/dt to dp/ds using chain rule: dp/ds = dp/dt * dt/ds
  double dt_ds = 0.0;
  const double eps = 1e-12;

  // Handle edge cases
  if (it == reparametrization.arc_lengths.begin() ||
    it == reparametrization.arc_lengths.end())
  {
    // Fallback: ds/dt ~= ||dp/dt|| => dt/ds = 1/||dp/dt||
    double ds_dt = dp_dt.norm();
    if (ds_dt > eps) {
      dt_ds = 1.0 / ds_dt;
    }
  } else {
    // Linear approximation of dt/ds from neighboring points
    size_t idx_high = std::distance(reparametrization.arc_lengths.begin(), it);
    size_t idx_low = idx_high - 1;

    double s_low = reparametrization.arc_lengths[idx_low];
    double s_high = reparametrization.arc_lengths[idx_high];
    double t_low = reparametrization.t_values[idx_low];
    double t_high = reparametrization.t_values[idx_high];

    if (std::abs(s_high - s_low) > eps) {
      dt_ds = (t_high - t_low) / (s_high - s_low);
    } else {
      // Fallback if interval is too small
      double ds_dt = dp_dt.norm();
      if (ds_dt > eps) {
        dt_ds = 1.0 / ds_dt;
      }
    }
  }

  Eigen::Vector3d dp_ds = dp_dt * dt_ds;

  return std::make_pair(p, dp_ds);
}


}  // namespace spline
