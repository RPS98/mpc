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
 * @file spline_example.cpp
 * @brief Example using SplineTrajectoryGenerator to manage waypoints and spline regeneration
 * @authors Rafael Pérez Seguí, Carmen De Rojas Pita-Romero
 * @copyright Copyright (c) 2025 Universidad Politécnica de Madrid
 * @license BSD-3-Clause
 */

#include "spline/spline.hpp"
#include "spline/spline_trajectory_generator.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

int main(int argc, char ** argv)
{
  std::cout << "=== Spline Trajectory Generator Example ===" << std::endl;

  try {
    // Step 1: Define setpoints (waypoints with tangents)
    std::cout << "\n1. Creating trajectory setpoints" << std::endl;
    std::vector<trajectory::TrajectorySetpoint> setpoints;

    // Define a series of waypoints similar to a race track
    double speed = 6.0;  // m/s - defines the magnitude of tangent vectors

    setpoints.emplace_back(
      0, Eigen::Vector3d(0.0, 0.0, 1.0),
      speed * Eigen::Vector3d(1.0, 0.0, 0.0));
    setpoints.emplace_back(
      1, Eigen::Vector3d(5.0, 0.0, 1.0),
      speed * Eigen::Vector3d(1.0, 0.0, 0.0));
    setpoints.emplace_back(
      2, Eigen::Vector3d(10.0, 5.0, 1.0),
      speed * Eigen::Vector3d(0.0, 1.0, 0.0));
    setpoints.emplace_back(
      3, Eigen::Vector3d(10.0, 10.0, 1.5),
      speed * Eigen::Vector3d(-1.0, 0.0, 0.0));
    setpoints.emplace_back(
      4, Eigen::Vector3d(5.0, 10.0, 1.0),
      speed * Eigen::Vector3d(-1.0, 0.0, 0.0));
    setpoints.emplace_back(
      5, Eigen::Vector3d(0.0, 5.0, 1.0),
      speed * Eigen::Vector3d(0.0, -1.0, 0.0));

    std::cout << "   Created " << setpoints.size() << " setpoints" << std::endl;

    // Step 2: Create SplineTrajectoryGenerator
    std::cout << "\n2. Creating SplineTrajectoryGenerator" << std::endl;
    const int num_waypoints_per_spline = 3;
    trajectory::SplineTrajectoryGenerator spline_gen(
      setpoints, num_waypoints_per_spline, 200, 5);

    std::cout << "   Using " << num_waypoints_per_spline
              << " waypoints per spline" << std::endl;
    std::cout << "   Initial spline length: " << std::fixed << std::setprecision(4)
              << spline_gen.getReparametrization().total_length << " m" << std::endl;

    // Step 3: Evaluate trajectory by incrementing arc length
    std::cout << "\n3. Evaluating trajectory along arc length" << std::endl;
    std::cout << std::setw(10) << "s [m]" << std::setw(10) << "theta"
              << std::setw(12) << "x [m]" << std::setw(12) << "y [m]"
              << std::setw(12) << "z [m]" << std::setw(15) << "regenerations"
              << std::endl;
    std::cout << std::string(71, '-') << std::endl;

    double s = 0.0;
    const double ds = 0.5;  // Arc length increment
    int last_regen_count = 0;
    int iterations = 0;
    const int max_iterations = 100;  // Safety limit

    while (iterations < max_iterations) {
      // Evaluate spline at current arc length
      auto [new_s, result] = spline_gen.evaluateSpline(s);

      // Check if spline was regenerated
      int current_regen_count = spline_gen.getRegenerationCount();
      bool regenerated = (current_regen_count > last_regen_count);
      last_regen_count = current_regen_count;

      // Print current state
      std::cout << std::setw(10) << std::setprecision(3) << s
                << std::setw(10) << std::setprecision(3) << result.theta
                << std::setw(12) << std::setprecision(4) << result.position.x()
                << std::setw(12) << std::setprecision(4) << result.position.y()
                << std::setw(12) << std::setprecision(4) << result.position.z()
                << std::setw(15) << current_regen_count;

      if (regenerated) {
        std::cout << "  <- REGENERATED";
      }
      std::cout << std::endl;

      // Update s
      s = new_s + ds;

      // Check if we should stop (no more waypoints and reached end)
      if (result.theta >= 0.99 && spline_gen.getPath().remainingWaypoints() <= 1) {
        std::cout << "\n   Reached end of trajectory" << std::endl;
        break;
      }

      iterations++;
    }

    // Step 4: Summary
    std::cout << "\n4. Summary" << std::endl;
    std::cout << "   Total iterations: " << iterations << std::endl;
    std::cout << "   Total regenerations: " << spline_gen.getRegenerationCount() << std::endl;
    std::cout << "   Remaining waypoints: "
              << spline_gen.getPath().remainingWaypoints() << std::endl;

    std::cout << "\n=== Done! ===" << std::endl;
    return 0;

  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
