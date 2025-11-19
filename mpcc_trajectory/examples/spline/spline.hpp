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
 * @file spline.hpp
 * @brief Hermite Spline implementation in C++
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 * @copyright Copyright (c) 2025 Universidad Politécnica de Madrid
 * @license BSD-3-Clause
 */

#ifndef HERMITE_SPLINE_HPP
#define HERMITE_SPLINE_HPP

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <tuple>
#include <algorithm>
#include <string>

namespace spline
{

/**
 * @brief Setpoint structure with id, position and tangent
 */
struct Setpoint
{
  std::string id;                ///< Identifier for the setpoint
  Eigen::Vector3d position;      ///< 3D position
  Eigen::Vector3d tangent;       ///< 3D tangent vector

  /**
   * @brief Constructor for Setpoint
   *
   * @param id Identifier string
   * @param pos 3D position vector
   * @param tan 3D tangent vector
   */
  Setpoint(const std::string & id, const Eigen::Vector3d & pos, const Eigen::Vector3d & tan)
  : id(id), position(pos), tangent(tan) {}
};

/**
 * @brief Piecewise cubic Hermite spline in 3D
 *
 * Given N setpoints with positions and tangent vectors, creates a smooth curve
 * through the points with specified tangents.
 */
class HermiteSpline
{
public:
  /**
   * @brief Constructor for HermiteSpline from setpoints
   *
   * @param setpoints Vector of Setpoint objects (N setpoints)
   * @param ti Optional vector of parameter values. If empty, uses [0, 1, 2, ..., N-1]
   *
   * @throws std::invalid_argument if less than 2 setpoints provided
   * @throws std::invalid_argument if ti size doesn't match setpoints size
   */
  HermiteSpline(
    const std::vector<Setpoint> & setpoints,
    const std::vector<double> & ti = {});

  /**
   * @brief Constructor for HermiteSpline from separate vectors (legacy)
   *
   * @param points Vector of control points (N x 3D vectors)
   * @param tangents Vector of tangent vectors at control points (N x 3D vectors)
   * @param ti Optional vector of parameter values. If empty, uses [0, 1, 2, ..., N-1]
   *
   * @throws std::invalid_argument if points and tangents don't match in size
   * @throws std::invalid_argument if less than 2 points provided
   * @throws std::invalid_argument if ti size doesn't match points size
   */
  HermiteSpline(
    const std::vector<Eigen::Vector3d> & points,
    const std::vector<Eigen::Vector3d> & tangents,
    const std::vector<double> & ti = {});

  /**
   * @brief Evaluate spline position at parameter t
   *
   * @param t Parameter value
   * @return Position as Eigen::Vector3d
   */
  Eigen::Vector3d evaluate(double t) const;

  /**
   * @brief Evaluate spline derivative dp/dt at parameter t
   *
   * @param t Parameter value
   * @return Derivative as Eigen::Vector3d
   */
  Eigen::Vector3d evaluateDerivative(double t) const;

  /**
   * @brief Get setpoints
   *
   * @return Vector of setpoints
   */
  const std::vector<Setpoint> & getSetpoints() const {return setpoints_;}

  /**
   * @brief Get spline control points
   *
   * @return Vector of control points
   */
  const std::vector<Eigen::Vector3d> & getPoints() const {return points_;}

  /**
   * @brief Get spline tangent vectors
   *
   * @return Vector of tangent vectors
   */
  const std::vector<Eigen::Vector3d> & getTangents() const {return tangents_;}

  /**
   * @brief Get parameter values at control points
   *
   * @return Vector of parameter values
   */
  const std::vector<double> & getTi() const {return ti_;}

  /**
   * @brief Get number of control points
   *
   * @return Number of control points
   */
  size_t size() const {return setpoints_.size();}

private:
  /**
   * @brief Compute Hermite basis functions at s ∈ [0, 1]
   *
   * @param s Normalized parameter in [0, 1]
   * @return Tuple of four basis functions (h00, h10, h01, h11)
   */
  std::tuple<double, double, double, double> hermiteBasis(double s) const;

  /**
   * @brief Compute derivatives of Hermite basis functions at s ∈ [0, 1]
   *
   * @param s Normalized parameter in [0, 1]
   * @return Tuple of four basis function derivatives (h00', h10', h01', h11')
   */
  std::tuple<double, double, double, double> hermiteBasisDerivative(double s) const;

  /**
   * @brief Find which segment contains parameter t
   *
   * @param t Parameter value to search for
   * @return Index of the segment containing t
   */
  int findSegment(double t) const;

  /**
   * @brief Clamp value to range [min, max]
   *
   * @param value Value to clamp
   * @param min Minimum value
   * @param max Maximum value
   * @return Clamped value
   */
  static double clamp(double value, double min, double max);

  // Member variables
  std::vector<Setpoint> setpoints_;        // Setpoints with id, position and tangent
  std::vector<Eigen::Vector3d> points_;    // Control points (N, 3) - extracted from setpoints
  std::vector<Eigen::Vector3d> tangents_;  // Tangent vectors at control points (N, 3) - extracted from setpoints
  std::vector<double> ti_;                 // Parameter values at control points (N,)
};

/**
 * @brief Result of arc length reparametrization computation
 *
 * This structure contains all the data required for symbolic evaluation
 * of the reparametrized Hermite spline.
 */
struct ArcLengthReparametrizationResult
{
  std::vector<Eigen::Vector3d> points;    // Control points (N, 3)
  std::vector<Eigen::Vector3d> tangents;  // Tangent vectors at control points (N, 3)
  std::vector<double> ti;                 // Original parameter values at control points (N,)
  Eigen::VectorXd poly_coeffs;            // Polynomial coefficients for t(s_normalized), highest degree first
  double total_length;                    // Total arc length of the spline
};

/**
 * @brief Compute arc length reparametrization parameters for a Hermite spline.
 *
 * This function computes an approximate arc length parametrization s(t) and
 * fits a polynomial approximation of the inverse mapping t(s_normalized),
 * where s_normalized = s / L ∈ [0, 1] and L is the total arc length.
 *
 * Steps:
 * 1. Sample parameter t uniformly in [t_min, t_max].
 * 2. Evaluate ||dp/dt|| at each sample using HermiteSpline::evaluateDerivative().
 * 3. Integrate using the trapezoidal rule to obtain the cumulative arc length s(t).
 * 4. Normalize s by the total length L to get s_normalized ∈ [0, 1].
 * 5. Fit a polynomial t(s_normalized) of degree poly_degree using least squares.
 *
 * @param spline HermiteSpline object to reparametrize
 * @param n_samples Number of samples for numerical integration (default 200)
 * @param poly_degree Degree of polynomial to approximate t(s_normalized) (default 5)
 * @return ArcLengthReparametrizationResult with points, tangents, ti, poly_coeffs and total_length
 *
 * @throws std::invalid_argument if n_samples < 2 or poly_degree < 1
 * @throws std::runtime_error if the spline has no parameter values or total length is non-positive
 */
ArcLengthReparametrizationResult computeArcLengthReparametrization(
  const HermiteSpline & spline,
  int n_samples = 200,
  int poly_degree = 5);


}  // namespace spline

#endif  // HERMITE_SPLINE_HPP
