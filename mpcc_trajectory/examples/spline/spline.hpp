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
 *
 * @brief Hermite Spline header in C++
 *
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 */

#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <stdexcept>
#include <tuple>
#include <utility>

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
   * @param id Identifier
   * @param pos 3D position vector
   * @param tan 3D tangent vector
   */
  Setpoint(const std::string & id, const Eigen::Vector3d & pos, const Eigen::Vector3d & tan)
  : id(id), position(pos), tangent(tan) {}
};

/**
 * @brief Hermite Spline class
 *
 * Represents a piecewise cubic Hermite spline interpolating through setpoints.
 */
class HermiteSpline
{
public:
  /**
   * @brief Constructor for HermiteSpline
   *
   * @param setpoints Vector of Setpoint objects with positions and tangents
   */
  explicit HermiteSpline(const std::vector<Setpoint> & setpoints);

  /**
   * @brief Evaluate spline position at parameter t
   *
   * @param t Parameter value (0 to getTmax())
   * @return 3D position vector
   */
  Eigen::Vector3d evaluate(double t) const;

  /**
   * @brief Evaluate spline derivative at parameter t
   *
   * @param t Parameter value (0 to getTmax())
   * @return 3D derivative/velocity vector
   */
  Eigen::Vector3d evaluateDerivative(double t) const;

  /**
   * @brief Evaluate both position and derivative at parameter t
   *
   * @param t Parameter value (0 to getTmax())
   * @return Pair of (position, derivative)
   */
  std::pair<Eigen::Vector3d, Eigen::Vector3d> evaluateWithDerivative(double t) const;

  /**
   * @brief Get maximum parameter value
   *
   * For N setpoints, t ranges from 0 to N-1
   *
   * @return Maximum t value
   */
  double getTmax() const {return static_cast<double>(setpoints_.size() - 1);}

  /**
   * @brief Get the setpoints
   *
   * @return Vector of setpoints
   */
  const std::vector<Setpoint> & getSetpoints() const {return setpoints_;}

private:
  /**
   * @brief Hermite basis functions for cubic interpolation
   *
   * @param t Parameter in [0, 1]
   * @return Tuple of (h00, h10, h01, h11)
   */
  std::tuple<double, double, double, double> hermiteBasis(double t) const;

  /**
   * @brief Derivatives of Hermite basis functions
   *
   * @param t Parameter in [0, 1]
   * @return Tuple of (h00', h10', h01', h11')
   */
  std::tuple<double, double, double, double> hermiteBasisDerivative(double t) const;

  /**
   * @brief Find which segment contains parameter t
   *
   * @param t Parameter value
   * @return Index of the segment containing t
   */
  int findSegment(double t) const;

  // Member variables
  std::vector<Setpoint> setpoints_;        // Setpoints with id, position and tangent
};

/**
 * @brief Result of arc length reparametrization computation
 */
struct ArcLengthReparametrizationResult
{
  std::vector<Setpoint> setpoints;  // Original setpoints
  std::vector<double> arc_lengths;  // Arc length values for lookup
  std::vector<double> t_values;     // Corresponding t parameter values
  double total_length;              // Total arc length in meters
};

/**
 * @brief Compute arc-length reparametrization of a Hermite spline
 *
 * Creates a lookup table mapping arc length to spline parameter t using
 * binary search and linear interpolation. This method provides the most
 * accurate arc-length parametrization at the cost of memory usage.
 *
 * @param spline HermiteSpline object to reparametrize
 * @param n_samples Number of samples for arc length computation (default 200)
 * @return LookupTableArcLengthReparametrization with lookup table data
 *
 * @throws std::invalid_argument if n_samples < 2
 * @throws std::runtime_error if total length is non-positive
 */
ArcLengthReparametrizationResult computeArcLengthReparametrization(
  const HermiteSpline & spline,
  int n_samples = 200);

/**
 * @brief Reparametrize arc length s to spline parameter t
 *
 * Uses the lookup table from ArcLengthReparametrizationResult to find
 * the corresponding spline parameter t for a given arc length s.
 *
 * @param s Arc length parameter in meters
 * @param reparametrization Result from ArcLengthReparametrizationResult
 * @return Corresponding spline parameter t
 */
double reparametrizeArcLengthToT(
  double s,
  const ArcLengthReparametrizationResult & reparametrization);

/**
 * @brief Evaluate position on arc-length parametrized spline
 *
 * Returns the position p(s) where s is the arc length parameter.
 * Uses binary search O(log n) and linear interpolation.
 *
 * @param s Arc length parameter in meters (0 to total_length)
 * @param reparametrization Result from ArcLengthReparametrizationResult
 * @param spline Original HermiteSpline object
 * @return 3D position at arc length s
 */
Eigen::Vector3d evaluateArcLengthSpline(
  double s,
  const ArcLengthReparametrizationResult & reparametrization,
  const HermiteSpline & spline);

/**
   * @brief Evaluate position and derivative dp/ds on arc-length parametrized spline
   *
   * Uses binary search O(log n) and linear interpolation.
   *
   * @param s Arc length parameter in meters (0 to total_length)
   * @param reparametrization Result from ArcLengthReparametrizationResult
   * @param spline Original HermiteSpline object
   * @return Pair of (position, derivative dp/ds) at arc length s
   */
std::pair<Eigen::Vector3d, Eigen::Vector3d> evaluateArcLengthSplineWithDerivative(
  double s,
  const ArcLengthReparametrizationResult & reparametrization,
  const HermiteSpline & spline);

}  // namespace spline

#endif  // SPLINE_HPP
