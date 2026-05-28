// Copyright 2025 Universidad Politécnica de Madrid
// Licensed under the BSD-3-Clause license.

/**
 * @file run_example.cpp
 *
 * Trajectory-tracking MPC example using DynamicTrajectory per-stage references.
 *
 * Standalone smoke test for the ``mpc_acados_trajectory`` controller: closed
 * loop tracking of a minimum-jerk polynomial trajectory produced by the
 * ``dynamic_trajectory_generator`` library against the Acados sim solver
 * (no motor dynamics). DTG is fetched automatically by the example's
 * top-level CMakeLists.txt; no system install is required.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

#include "mpc_acados_core/logging/csv_logger.hpp"
#include "mpc_acados_trajectory/acados_mpc.hpp"
#include "mpc_acados_trajectory/acados_mpc_yaml.hpp"
#include "mpc_acados_trajectory/acados_sim_solver.hpp"

#include "sim_yaml.hpp"

namespace mpc_acados_trajectory_examples {

using mpc_acados_core::logging::computeMean;
using mpc_acados_core::logging::CsvLogger;
using mpc_acados_core::logging::printProgress;

constexpr double kHoverTime                = 2.0;
constexpr double kMinHorizontalSpeedForYaw = 0.3;

struct ExampleArgs {
  std::string config_path     = "examples/simulation_config.yaml";
  std::string file_name       = "mpc_log.csv";
  std::string mpc_config_path = "examples/mpc_config.yaml";
};

inline Eigen::Vector3d toEigenVec3(const std::array<double, 3>& v) { return {v[0], v[1], v[2]}; }

inline Eigen::Quaterniond toEigenQuat(const std::array<double, 4>& q) {
  return {q[0], q[1], q[2], q[3]};
}

inline Eigen::Vector3d getStatePosition(const acados_mpc::MPCData& d) {
  return toEigenVec3(d.state.getPosition());
}

inline Eigen::Quaterniond getStateOrientation(const acados_mpc::MPCData& d) {
  return toEigenQuat(d.state.getOrientation());
}

inline Eigen::Vector3d getStateVelocity(const acados_mpc::MPCData& d) {
  return toEigenVec3(d.state.getLinearVelocity());
}

inline Eigen::Vector3d getActuationAngularVelocity(const acados_mpc::MPCData& d) {
  return toEigenVec3(d.actuation.getAngularVelocity());
}

int simulatorStep(acados_mpc::MPCSimSolver& simulator, acados_mpc::MPCData* mpc_data) {
  const int status = simulator.solve(mpc_data);
  if (status != 0) {
    throw std::runtime_error("acados integrator returned status " + std::to_string(status) +
                             ". Exiting.");
  }
  return status;
}

/// Build the DynamicWaypoint vector consumed by
/// `dynamic_traj_generator::DynamicTrajectory::setWaypoints` from a sequence
/// of plain Eigen waypoints. IDs are auto-numbered so the vector preserves
/// the YAML ordering.
dynamic_traj_generator::DynamicWaypoint::Vector buildDynamicWaypoints(
    const std::vector<Eigen::Vector3d>& waypoints) {
  dynamic_traj_generator::DynamicWaypoint::Vector result;
  result.reserve(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    dynamic_traj_generator::DynamicWaypoint wp;
    wp.resetWaypoint(waypoints[i]);
    wp.setName("wp_" + std::to_string(i));
    result.emplace_back(std::move(wp));
  }
  return result;
}

/// Quaternion that aligns the body +X axis with @p velocity (path-facing).
/// Falls back to the @p current orientation when the horizontal speed is
/// below ``kMinHorizontalSpeedForYaw`` so the yaw reference does not jitter
/// at start / stop transitions. Matches `compute_path_facing` in the Python
/// driver.
Eigen::Quaterniond computePathFacing(const Eigen::Vector3d& velocity,
                                     const Eigen::Quaterniond& current) {
  if (velocity.head<2>().norm() < kMinHorizontalSpeedForYaw) {
    return current;
  }
  const double yaw = std::atan2(velocity.y(), velocity.x());
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

/// Per-stage reference filling from DynamicTrajectory. Mirrors the Python
/// helper `set_trajectory_references`: samples the trajectory at
/// ``t_now + k*dt_horizon`` (clamped to [t_min, t_max]) for every stage
/// k = 0..N. Past t_max the references freeze at the last known sample so
/// the drone hovers instead of extrapolating.
void setTrajectoryReferencesFromDtg(acados_mpc::MPCData* mpc_data,
                                    dynamic_traj_generator::DynamicTrajectory& trajectory,
                                    const Eigen::Quaterniond& current_orientation,
                                    const double t_now,
                                    const double dt_horizon,
                                    const int prediction_steps,
                                    const double t_min,
                                    const double t_max,
                                    const bool path_facing,
                                    Eigen::Vector3d& last_position,
                                    Eigen::Quaterniond& last_orientation) {
  const bool hover = t_now > t_max;

  dynamic_traj_generator::References refs;
  for (int stage = 0; stage <= prediction_steps; ++stage) {
    Eigen::Vector3d p_k;
    Eigen::Vector3d v_k = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_k = Eigen::Vector3d::Zero();

    if (hover) {
      p_k = last_position;
    } else {
      const double t_eval = std::min(std::max(t_now + stage * dt_horizon, t_min), t_max);
      if (!trajectory.evaluateTrajectory(static_cast<float>(t_eval), refs)) {
        // Optimiser still warming up — fall back to last known sample.
        p_k = last_position;
      } else {
        p_k = refs.position;
        v_k = refs.velocity;
        a_k = refs.acceleration;
      }
    }

    Eigen::Quaterniond q_k = current_orientation;
    if (path_facing && !hover) {
      q_k = computePathFacing(v_k, current_orientation);
    }

    mpc_data->p_params.setDesiredPosition({p_k.x(), p_k.y(), p_k.z()}, stage);
    mpc_data->p_params.setDesiredVelocity({v_k.x(), v_k.y(), v_k.z()}, stage);
    mpc_data->p_params.setDesiredAcceleration({a_k.x(), a_k.y(), a_k.z()}, stage);
    mpc_data->p_params.setDesiredOrientation({q_k.w(), q_k.x(), q_k.y(), q_k.z()}, stage);

    if (stage == 0) {
      if (!hover) {
        last_position = p_k;
      }
      last_orientation = q_k;
    }
  }
}

ExampleArgs parseArguments(int argc, char** argv) {
  ExampleArgs args;
  for (int index = 1; index < argc; ++index) {
    const std::string a = argv[index];
    if ((a == "-c" || a == "--config_path") && index + 1 < argc) {
      args.config_path = argv[++index];
    } else if ((a == "-f" || a == "--file_name") && index + 1 < argc) {
      args.file_name = argv[++index];
    } else if ((a == "-m" || a == "--mpc_config_path") && index + 1 < argc) {
      args.mpc_config_path = argv[++index];
    } else if (a == "-h" || a == "--help") {
      std::cout << "Usage: " << argv[0] << " [-c|--config_path <yaml>] [-f|--file_name <csv>]"
                << " [-m|--mpc_config_path <yaml>]" << std::endl;
      std::exit(0);
    }
  }
  return args;
}

void testMpcController(acados_mpc::MPC& mpc,
                       acados_mpc::MPCSimSolver& simulator,
                       const YamlSimConfig& sim_config,
                       CsvLogger& logger) {
  if (sim_config.waypoints.empty()) {
    throw std::invalid_argument(
        "simulation_config.yaml must list at least one target waypoint "
        "(sim_config.initial_position is the start; waypoints are the targets).");
  }

  acados_mpc::MPCData* mpc_data = mpc.getData();
  const int prediction_steps    = mpc.getPredictionSteps();
  const double dt               = mpc.getPredictionTimeStep();
  const double dt_horizon       = dt;
  const double v_max            = sim_config.max_speed;
  const bool path_facing        = sim_config.path_facing;

  // ---- Build the DynamicTrajectory (canonical setWaypoints flow).
  // Order taken from the validated mav_examples adapter:
  //   setSpeed -> updateVehiclePosition -> setWaypoints. The first
  //   getMinTime/getMaxTime call blocks until the optimiser publishes.
  dynamic_traj_generator::DynamicTrajectory trajectory;
  trajectory.setSpeed(v_max);
  const Eigen::Vector3d & initial_position = sim_config.initial_position;
  trajectory.updateVehiclePosition(initial_position);
  std::vector<Eigen::Vector3d> knots;
  knots.reserve(1U + sim_config.waypoints.size());
  knots.push_back(initial_position);
  knots.insert(knots.end(), sim_config.waypoints.begin(), sim_config.waypoints.end());
  trajectory.setWaypoints(buildDynamicWaypoints(knots));

  // Start the drone at the configured initial pose so the closed loop
  // begins matched to the trajectory's anchor point.
  mpc_data->state.setData(0, initial_position.x());
  mpc_data->state.setData(1, initial_position.y());
  mpc_data->state.setData(2, initial_position.z());

  const double t_min = trajectory.getMinTime();
  const double t_max = trajectory.getMaxTime();
  const double total_time = t_max + kHoverTime;

  // Initial reference at t=0 (frozen at the initial position until the
  // optimiser has produced a sample).
  Eigen::Vector3d last_position    = initial_position;
  Eigen::Quaterniond last_orientation = Eigen::Quaterniond::Identity();

  const Eigen::Matrix<double, 4, 1> zero_motor = Eigen::Matrix<double, 4, 1>::Zero();
  // First log entry at t=0
  logger.save(0.0, getStatePosition(*mpc_data), getStateOrientation(*mpc_data),
              getStateVelocity(*mpc_data), Eigen::Vector3d::Zero(),
              last_position, last_orientation, 0.0,
              Eigen::Vector3d::Zero(), zero_motor, 0.0, 0, false, v_max);

  std::vector<double> mpc_times;
  std::vector<double> sim_times;
  std::vector<double> total_times;
  const std::size_t reserve = static_cast<std::size_t>(total_time / dt) + 1U;
  mpc_times.reserve(reserve);
  sim_times.reserve(reserve);
  total_times.reserve(reserve);

  std::cout << "Starting MPC simulation..." << std::endl;
  std::cout << "Trajectory time window: [" << t_min << ", " << t_max << "] s" << std::endl;
  std::cout << "Hover time after trajectory: " << kHoverTime << " s" << std::endl;
  std::cout << "Total time: " << total_time << " s" << std::endl;
  std::cout << "MPC dt: " << dt << " s  |  Prediction steps: " << prediction_steps
            << "  |  Horizon dt: " << dt_horizon << " s" << std::endl;

  double t = 0.0;
  while (t < total_time + 1e-9) {
    t += dt;
    const auto iter_start = std::chrono::high_resolution_clock::now();

    const Eigen::Quaterniond current_orientation = getStateOrientation(*mpc_data);
    setTrajectoryReferencesFromDtg(mpc_data, trajectory, current_orientation, t, dt_horizon,
                                   prediction_steps, t_min, t_max, path_facing,
                                   last_position, last_orientation);

    const auto mpc_start = std::chrono::high_resolution_clock::now();
    const int mpc_status = mpc.solve();
    const auto mpc_end   = std::chrono::high_resolution_clock::now();
    if (mpc_status != 0) {
      std::cerr << "\nMPC solver failed with status " << mpc_status << " at time " << t << " s"
                << std::endl;
    }

    const auto sim_start = std::chrono::high_resolution_clock::now();
    simulatorStep(simulator, mpc_data);
    const auto sim_end = std::chrono::high_resolution_clock::now();

    const std::chrono::duration<double> mpc_duration   = mpc_end - mpc_start;
    const std::chrono::duration<double> sim_duration   = sim_end - sim_start;
    const std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());

    const bool hover_active = (t > t_max);
    const double controller_solve_time_us = mpc_duration.count() * 1e6;
    logger.save(t, getStatePosition(*mpc_data), getStateOrientation(*mpc_data),
                getStateVelocity(*mpc_data), Eigen::Vector3d::Zero(), last_position,
                last_orientation, mpc_data->actuation.getThrust(),
                getActuationAngularVelocity(*mpc_data), zero_motor, controller_solve_time_us,
                0, hover_active, v_max);

    printProgress(t / total_time);
  }

  std::cout << "\n";
  logger.close();

  std::cout << "\n=== Time Statistics ===" << std::endl;
  std::cout << "MPC average time: " << computeMean(mpc_times) * 1000.0 << " ms" << std::endl;
  std::cout << "Simulator average time: " << computeMean(sim_times) * 1000.0 << " ms" << std::endl;
  const double total_avg = computeMean(total_times);
  std::cout << "Total average time: " << total_avg * 1000.0 << " ms" << std::endl;
  if (total_avg > 0.0) {
    std::cout << "Real-time factor: " << dt / total_avg << std::endl;
  }
}

}  // namespace mpc_acados_trajectory_examples

int main(int argc, char** argv) {
  const auto args = mpc_acados_trajectory_examples::parseArguments(argc, argv);

  mpc_acados_trajectory_examples::YamlSimConfig sim_config;
  mpc_acados_trajectory_examples::readSimYaml(args.config_path, sim_config);

  acados_mpc::MPC mpc;
  acados_mpc::configureMpcFromYaml(mpc, args.mpc_config_path);

  acados_mpc::MPCSimSolver simulator;
  mpc_acados_core::logging::CsvLogger logger(args.file_name);

  mpc_acados_trajectory_examples::testMpcController(mpc, simulator, sim_config, logger);
  return 0;
}
