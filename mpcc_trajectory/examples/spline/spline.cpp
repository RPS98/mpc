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
 * @brief Hermite Spline implementation in C++
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 * @copyright Copyright (c) 2025 Universidad Politécnica de Madrid
 * @license BSD-3-Clause
 */

#include "spline.hpp"
#include <cmath>
#include <sstream>

namespace spline
{

HermiteSpline::HermiteSpline(
  const std::vector<Setpoint> & setpoints,
  const std::vector<double> & ti)
: setpoints_(setpoints)
{
  // Validate input
  if (setpoints_.size() < 2) {
    throw std::invalid_argument("Need at least 2 setpoints");
  }

  // Extract points and tangents from setpoints
  points_.reserve(setpoints_.size());
  tangents_.reserve(setpoints_.size());

  for (const auto & sp : setpoints_) {
    points_.push_back(sp.position);
    tangents_.push_back(sp.tangent);
  }

  // Set parameter values
  if (ti.empty()) {
    // Default: [0, 1, 2, ..., N-1]
    ti_.resize(setpoints_.size());
    for (size_t i = 0; i < setpoints_.size(); ++i) {
      ti_[i] = static_cast<double>(i);
    }
  } else {
    if (ti.size() != setpoints_.size()) {
      throw std::invalid_argument("Length of ti must match number of setpoints");
    }
    ti_ = ti;
  }
}

HermiteSpline::HermiteSpline(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<Eigen::Vector3d> & tangents,
  const std::vector<double> & ti)
: points_(points), tangents_(tangents)
{
  // Validate input
  if (points_.size() != tangents_.size()) {
    throw std::invalid_argument("Number of points and tangents must match");
  }

  if (points_.size() < 2) {
    throw std::invalid_argument("Need at least 2 points");
  }

  // Create setpoints from points and tangents
  setpoints_.reserve(points_.size());
  for (size_t i = 0; i < points_.size(); ++i) {
    setpoints_.emplace_back(std::to_string(i), points_[i], tangents_[i]);
  }

  // Set parameter values
  if (ti.empty()) {
    // Default: [0, 1, 2, ..., N-1]
    ti_.resize(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
      ti_[i] = static_cast<double>(i);
    }
  } else {
    if (ti.size() != points_.size()) {
      throw std::invalid_argument("Length of ti must match number of points");
    }
    ti_ = ti;
  }
}

double HermiteSpline::clamp(double value, double min, double max)
{
  return std::max(min, std::min(max, value));
}

std::tuple<double, double, double, double> HermiteSpline::hermiteBasis(double s) const
{
  double s2 = s * s;
  double s3 = s2 * s;

  double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  double h10 = s3 - 2.0 * s2 + s;
  double h01 = -2.0 * s3 + 3.0 * s2;
  double h11 = s3 - s2;

  return std::make_tuple(h00, h10, h01, h11);
}

std::tuple<double, double, double, double> HermiteSpline::hermiteBasisDerivative(double s) const
{
  double s2 = s * s;

  double h00_d = 6.0 * s2 - 6.0 * s;
  double h10_d = 3.0 * s2 - 4.0 * s + 1.0;
  double h01_d = -6.0 * s2 + 6.0 * s;
  double h11_d = 3.0 * s2 - 2.0 * s;

  return std::make_tuple(h00_d, h10_d, h01_d, h11_d);
}

int HermiteSpline::findSegment(double t) const
{
  for (size_t i = 0; i < ti_.size() - 1; ++i) {
    if (t <= ti_[i + 1]) {
      return static_cast<int>(i);
    }
  }
  // Last segment
  return static_cast<int>(ti_.size() - 2);
}

Eigen::Vector3d HermiteSpline::evaluate(double t) const
{
  // Clamp to valid range
  t = clamp(t, ti_.front(), ti_.back());

  // Find segment
  int idx = findSegment(t);

  // Get segment data
  const Eigen::Vector3d & p0 = points_[idx];
  const Eigen::Vector3d & p1 = points_[idx + 1];
  const Eigen::Vector3d & m0 = tangents_[idx];
  const Eigen::Vector3d & m1 = tangents_[idx + 1];
  double t0 = ti_[idx];
  double t1 = ti_[idx + 1];

  // Normalize to [0, 1]
  double dt = t1 - t0;
  double s = (t - t0) / dt;
  s = clamp(s, 0.0, 1.0);

  // Evaluate Hermite polynomial
  auto [h00, h10, h01, h11] = hermiteBasis(s);

  return h00 * p0 + h10 * m0 * dt + h01 * p1 + h11 * m1 * dt;
}

Eigen::Vector3d HermiteSpline::evaluateDerivative(double t) const
{
  // Clamp to valid range
  t = clamp(t, ti_.front(), ti_.back());

  // Find segment
  int idx = findSegment(t);

  // Get segment data
  const Eigen::Vector3d & p0 = points_[idx];
  const Eigen::Vector3d & p1 = points_[idx + 1];
  const Eigen::Vector3d & m0 = tangents_[idx];
  const Eigen::Vector3d & m1 = tangents_[idx + 1];
  double t0 = ti_[idx];
  double t1 = ti_[idx + 1];

  // Normalize to [0, 1]
  double dt = t1 - t0;
  double s = (t - t0) / dt;
  s = clamp(s, 0.0, 1.0);

  // Evaluate derivative
  auto [h00_d, h10_d, h01_d, h11_d] = hermiteBasisDerivative(s);
  Eigen::Vector3d dp_ds = h00_d * p0 + h10_d * m0 * dt + h01_d * p1 + h11_d * m1 * dt;

  // Chain rule: dp/dt = dp/ds * ds/dt
  double ds_dt = 1.0 / dt;
  return dp_ds * ds_dt;
}

ArcLengthReparametrizationResult computeArcLengthReparametrization(
  const HermiteSpline & spline,
  int n_samples,
  int poly_degree)
{
  // Basic sanity checks
  if (n_samples < 2) {
    throw std::invalid_argument("computeArcLengthReparametrization: n_samples must be >= 2");
  }

  if (poly_degree < 1) {
    throw std::invalid_argument("computeArcLengthReparametrization: poly_degree must be >= 1");
  }

  // Get original parameter vector ti
  const std::vector<double> & ti = spline.getTi();
  if (ti.empty()) {
    throw std::runtime_error("computeArcLengthReparametrization: spline has no parameter values");
  }

  const double t_min = ti.front();
  const double t_max = ti.back();

  // Sample parameter t uniformly in [t_min, t_max]
  Eigen::VectorXd t_samples(n_samples);
  const double dt = (t_max - t_min) / static_cast<double>(n_samples - 1);

  for (int i = 0; i < n_samples; ++i) {
    t_samples[i] = t_min + static_cast<double>(i) * dt;
  }

  // Evaluate ||dp/dt|| at each sample point
  Eigen::VectorXd norms(n_samples);
  for (int i = 0; i < n_samples; ++i) {
    Eigen::Vector3d deriv = spline.evaluateDerivative(t_samples[i]);
    norms[i] = deriv.norm();  // Euclidean norm of the derivative (speed)
  }

  // Trapezoidal integration:
  // s(t) = ∫ ||dp/dt|| dt
  // Approximated cumulative integral:
  // s_{k} = s_{k-1} + 0.5 * (norms_{k-1} + norms_{k}) * dt
  Eigen::VectorXd arc_lengths(n_samples);
  arc_lengths.setZero();

  for (int i = 1; i < n_samples; ++i) {
    const double trapezoid_area =
      0.5 * (norms[i - 1] + norms[i]) * dt;
    arc_lengths[i] = arc_lengths[i - 1] + trapezoid_area;
  }

  // Total arc length L = s(t_max)
  const double total_length = arc_lengths[n_samples - 1];
  if (total_length <= 0.0) {
    throw std::runtime_error("computeArcLengthReparametrization: non-positive total length");
  }

  // Normalize arc length: s_normalized = s / L ∈ [0, 1]
  Eigen::VectorXd s_normalized = arc_lengths / total_length;

  // Fit polynomial t(s_normalized) of degree poly_degree using least squares.
  //
  // We construct a Vandermonde matrix A where each row is:
  // [s^d, s^(d-1), ..., s^1, s^0]
  // so that:
  // A * c = t_samples,
  // where c are the polynomial coefficients (highest degree first),
  // consistent with NumPy's np.polyfit convention.
  const int num_coeffs = poly_degree + 1;
  Eigen::MatrixXd A(n_samples, num_coeffs);

  for (int i = 0; i < n_samples; ++i) {
    const double s = s_normalized[i];

    // Compute powers s^0, s^1, ..., s^poly_degree
    Eigen::VectorXd powers(num_coeffs);
    powers[0] = 1.0;
    for (int k = 1; k < num_coeffs; ++k) {
      powers[k] = powers[k - 1] * s;
    }

    // Fill row i with [s^poly_degree, ..., s^1, s^0]
    for (int j = 0; j < num_coeffs; ++j) {
      A(i, j) = powers[poly_degree - j];
    }
  }

  // Solve least-squares problem A * c = t_samples
  Eigen::VectorXd poly_coeffs = A.colPivHouseholderQr().solve(t_samples);

  // Fill result structure
  ArcLengthReparametrizationResult result;
  result.points = spline.getPoints();      // Copy control points
  result.tangents = spline.getTangents();  // Copy tangents
  result.ti = spline.getTi();              // Copy original parameter values
  result.poly_coeffs = poly_coeffs;        // Polynomial coefficients for t(s_normalized)
  result.total_length = total_length;      // Total arc length

  return result;
}

}  // namespace spline
