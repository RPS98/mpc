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
 * @file trajectory_generator.hpp
 *
 * @brief Trajectory Generator header in C++
 *
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 */

#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <stdexcept>
#include <utility>

#include "spline.hpp"

namespace spline
{

/**
 * @brief Trajectory generator based on Hermite splines
 */
class TrajectoryGenerator
{
public:
  /**
   * @brief Construct a trajectory generator
   *
   * @param setpoints Sequence of setpoints (must contain at least 1)
   * @param num_wp Number of waypoints used to build each local spline
   * @param spline_samples Number of samples used when computing arc-length reparametrization
   */
  explicit TrajectoryGenerator(
    const std::vector<Setpoint> & setpoints,
    int num_wp,
    int spline_samples = 200);

  ~TrajectoryGenerator() = default;

  /**
   * @brief Generate or regenerate the internal spline from the current path index
   *
   * @param s Arc length parameter in meters
   * @param position Output parameter for position at arc length s
   * @param derivative Output parameter for derivative dp/ds at arc length s
   * @return ArcLengthReparametrizationResult with current spline data and parameter s
   * updated (if spline was regenerated, s is set to 0)
   */
  std::pair<const ArcLengthReparametrizationResult, double> evaluateSpline(
    double s,
    Eigen::Vector3d & position,
    Eigen::Vector3d & derivative);

  /**
   * @brief Check if the trajectory has finished for a given arc length parameter s
   *
   * @param s Arc length parameter in meters
   * @return true if the trajectory has finished, false otherwise
   */
  bool isFinished(double s) const;

private:
  std::vector<Setpoint> setpoints_;
  std::vector<Setpoint> used_setpoints_;
  int num_wp_ = 3;
  int spline_samples_ = 200;
  std::unique_ptr<HermiteSpline> spline_;        // currently active local spline
  ArcLengthReparametrizationResult reparametrization_;
  size_t path_index_ = 0;        // index of first waypoint used by current spline
};

}  // namespace spline

#endif  // TRAJECTORY_GENERATOR_HPP
