// Copyright 2024 Universidad Politécnica de Madrid
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
 * @file trajectory_generator_util.hpp
 *
 * Trajectory-generator specific utils for the MPC example.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef EXAMPLE_TRAJECTORY_GENERATOR_UTIL_HPP_
#define EXAMPLE_TRAJECTORY_GENERATOR_UTIL_HPP_

#include <Eigen/Dense>

#include <iostream>
#include <memory>
#include <vector>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

inline DynamicWaypoint::Vector eigenVectorToDynamicWaypointVector(
    const std::vector<Eigen::Vector3d>& vector_waypoints) {
  DynamicWaypoint::Vector vector_dynamic_waypoints;
  for (const auto& waypoint : vector_waypoints) {
    DynamicWaypoint dynamic_waypoint;
    dynamic_waypoint.resetWaypoint(waypoint);
    vector_dynamic_waypoints.push_back(dynamic_waypoint);
  }
  return vector_dynamic_waypoints;
}

inline std::unique_ptr<DynamicTrajectory> get_trajectory_generator(
    const Eigen::Vector3d& initial_position,
    const std::vector<Eigen::Vector3d>& waypoints,
    const double speed) {
  std::unique_ptr<DynamicTrajectory> trajectory_generator = std::make_unique<DynamicTrajectory>();

  trajectory_generator->updateVehiclePosition(initial_position);
  trajectory_generator->setSpeed(speed);
  trajectory_generator->setWaypoints(eigenVectorToDynamicWaypointVector(waypoints));

  const double max_time = trajectory_generator->getMaxTime();
  std::cout << "Trajectory generated with max time: " << max_time << std::endl;

  return trajectory_generator;
}

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

#endif  // EXAMPLE_TRAJECTORY_GENERATOR_UTIL_HPP_
