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
 * @file spline_trajectory_generator.hpp
 * @brief Spline trajectory generator for managing waypoints and spline regeneration
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 * @copyright Copyright (c) 2025 Universidad Politécnica de Madrid
 * @license BSD-3-Clause
 */

#ifndef SPLINE_TRAJECTORY_GENERATOR_HPP
#define SPLINE_TRAJECTORY_GENERATOR_HPP

#include "spline.hpp"
#include <vector>
#include <memory>
#include <tuple>

namespace trajectory
{

/**
 * @brief Setpoint structure with id, position and tangent
 */
struct TrajectorySetpoint
{
  int id;                        ///< Identifier for the setpoint
  Eigen::Vector3d position;      ///< 3D position
  Eigen::Vector3d tangent;       ///< 3D tangent vector

  /**
   * @brief Constructor for TrajectorySetpoint
   *
   * @param id Identifier
   * @param pos 3D position vector
   * @param tan 3D tangent vector
   */
  TrajectorySetpoint(int id, const Eigen::Vector3d & pos, const Eigen::Vector3d & tan)
  : id(id), position(pos), tangent(tan) {}
};

/**
 * @brief Path class to manage waypoints with an index
 */
class Path
{
public:
  /**
   * @brief Constructor for Path
   *
   * @param setpoints Vector of TrajectorySetpoint objects
   */
  explicit Path(const std::vector<TrajectorySetpoint> & setpoints);

  /**
   * @brief Get the next num waypoints and tangents starting from current index
   *
   * @param num Number of waypoints to retrieve
   * @return Tuple of (waypoints vector, tangents vector)
   */
  std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
  getWaypoints(int num) const;

  /**
   * @brief Advance the current index by one
   *
   * @return True if advanced successfully, False if at the end
   */
  bool advance();

  /**
   * @brief Get the current setpoint
   *
   * @return Current setpoint or nullptr if at the end
   */
  const TrajectorySetpoint * getCurrentSetpoint() const;

  /**
   * @brief Get total number of setpoints
   *
   * @return Total number of setpoints
   */
  size_t size() const {return setpoints_.size();}

  /**
   * @brief Get number of waypoints remaining from current index
   *
   * @return Number of remaining waypoints
   */
  size_t remainingWaypoints() const;

  /**
   * @brief Get current index
   *
   * @return Current index
   */
  size_t getCurrentIndex() const {return current_index_;}

private:
  std::vector<TrajectorySetpoint> setpoints_;
  size_t current_index_;
};

/**
 * @brief Spline evaluation result
 */
struct SplineEvaluationResult
{
  Eigen::Vector3d position;                             ///< Position at arc length s
  Eigen::Vector3d derivative;                           ///< Tangent vector at arc length s
  double theta;                                         ///< Normalized arc length [0, 1]
  spline::ArcLengthReparametrizationResult reparametrization;  ///< Current spline params
};

/**
 * @brief Spline trajectory generator class
 *
 * Manages a sequence of waypoints and generates Hermite splines through them.
 * Automatically regenerates the spline when the trajectory progresses past waypoints.
 */
class SplineTrajectoryGenerator
{
public:
  /**
   * @brief Constructor for SplineTrajectoryGenerator
   *
   * @param setpoints Vector of TrajectorySetpoint objects
   * @param num_wp Number of waypoints to use for each spline generation
   * @param spline_samples Number of samples for arc length reparametrization (default 200)
   * @param spline_degree Degree of polynomial for arc length reparametrization (default 5)
   */
  SplineTrajectoryGenerator(
    const std::vector<TrajectorySetpoint> & setpoints,
    int num_wp,
    int spline_samples = 200,
    int spline_degree = 5);

  /**
   * @brief Evaluate the spline at arc length s
   *
   * @param s Arc length value to evaluate (must be >= 0)
   * @return SplineEvaluationResult with position, derivative, theta and reparametrization
   */
  SplineEvaluationResult evaluateArcLengthSpline(double s) const;

  /**
   * @brief Evaluate the spline at arc length s and regenerate if needed
   *
   * This method evaluates the spline at the given arc length s. If s has passed
   * the spline (theta >= 1.0), the spline is regenerated starting from the next
   * waypoint in the path. The arc length s is reset to 0 when regeneration occurs.
   *
   * @param s Arc length value to evaluate (must be >= 0)
   * @return Tuple of (new_s, evaluation_result) where new_s is the adjusted arc length
   */
  std::tuple<double, SplineEvaluationResult> evaluateSpline(double s);

  /**
   * @brief Get the current spline reparametrization parameters
   *
   * @return Current reparametrization parameters
   */
  const spline::ArcLengthReparametrizationResult & getReparametrization() const
  {
    return spline_reparametrization_;
  }

  /**
   * @brief Get the current path
   *
   * @return Current path
   */
  const Path & getPath() const {return path_;}

  /**
   * @brief Get regeneration count
   *
   * @return Number of times the spline has been regenerated
   */
  int getRegenerationCount() const {return regeneration_count_;}

private:
  /**
   * @brief Generate a new spline from the current path position
   */
  void generateSpline();

  // Member variables
  int num_wp_;                                          ///< Number of waypoints per spline
  int spline_samples_;                                  ///< Samples for reparametrization
  int spline_degree_;                                   ///< Polynomial degree
  int regeneration_count_;                              ///< Number of regenerations

  Path path_;                                           ///< Path manager
  std::unique_ptr<spline::HermiteSpline> spline_;       ///< Current spline
  spline::ArcLengthReparametrizationResult spline_reparametrization_;  ///< Spline params
};

}  // namespace trajectory

#endif  // SPLINE_TRAJECTORY_GENERATOR_HPP
