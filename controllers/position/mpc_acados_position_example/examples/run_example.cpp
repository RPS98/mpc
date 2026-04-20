// Copyright 2025 Universidad Politécnica de Madrid
// Licensed under the BSD-3-Clause license.

/**
 * @file run_example.cpp
 *
 * Position-reference MPC example using the acados sim solver as the plant.
 *
 * Loads a sequence of waypoints from simulation_config.yaml, runs the MPC in
 * closed loop against the acados integrator, and writes a CSV log compatible
 * with ``mpc_acados_core.plotting.plot_results``.
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

#include "mpc_acados_core/logging/csv_logger.hpp"
#include "mpc_acados_position/acados_mpc.hpp"
#include "mpc_acados_position/acados_mpc_yaml.hpp"
#include "mpc_acados_position/acados_sim_solver.hpp"

#include "sim_yaml.hpp"

namespace mpc_acados_position_examples {

using mpc_acados_core::logging::computeMean;
using mpc_acados_core::logging::CsvLogger;
using mpc_acados_core::logging::getDesiredOrientation;
using mpc_acados_core::logging::printProgress;

constexpr double kHoverTime = 2.0;
constexpr double kWaypointReachedTolerance = 0.1;

struct ExampleArgs {
  std::string config_path = "examples/simulation_config.yaml";
  std::string file_name = "mpc_log.csv";
  std::string mpc_config_path = "examples/mpc_config.yaml";
};

inline Eigen::Vector3d toEigenVec3(const std::array<double, 3>& v) {
  return {v[0], v[1], v[2]};
}

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

inline Eigen::Vector3d getActuationAngularVelocity(
    const acados_mpc::MPCData& d) {
  return toEigenVec3(d.actuation.getAngularVelocity());
}

int simulatorStep(acados_mpc::MPCSimSolver& simulator,
                  acados_mpc::MPCData* mpc_data) {
  const int status = simulator.solve(mpc_data);
  if (status != 0) {
    throw std::runtime_error("acados integrator returned status " +
                             std::to_string(status) + ". Exiting.");
  }
  return status;
}

void setProgressiveReferences(acados_mpc::MPCData* mpc_data,
                              const Eigen::Vector3d& current_position,
                              const Eigen::Vector3d& goal_position,
                              const Eigen::Quaterniond& desired_orientation,
                              const double v_ref,
                              const double dt_horizon,
                              const int prediction_steps) {
  const Eigen::Vector3d delta = goal_position - current_position;
  const double distance = delta.norm();

  if (distance < 1e-9) {
    mpc_data->p_params.setDesiredPosition(
        {goal_position.x(), goal_position.y(), goal_position.z()});
  } else {
    const Eigen::Vector3d direction = delta / distance;
    for (int stage = 0; stage <= prediction_steps; ++stage) {
      const double s_k = std::min((stage + 1) * v_ref * dt_horizon, distance);
      const Eigen::Vector3d stage_position = current_position + s_k * direction;
      mpc_data->p_params.setDesiredPosition(
          {stage_position.x(), stage_position.y(), stage_position.z()}, stage);
    }
  }

  mpc_data->p_params.setDesiredOrientation(
      {desired_orientation.w(), desired_orientation.x(),
       desired_orientation.y(), desired_orientation.z()});
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
      std::cout << "Usage: " << argv[0]
                << " [-c|--config_path <yaml>] [-f|--file_name <csv>]"
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
    throw std::invalid_argument("simulation_config.yaml must have at least one waypoint.");
  }

  acados_mpc::MPCData* mpc_data = mpc.getData();
  const int prediction_steps = mpc.getPredictionSteps();
  const double dt = mpc.getPredictionTimeStep();
  const double dt_horizon = dt;

  const double total_time = sim_config.sim_time + kHoverTime;
  const auto& waypoints = sim_config.waypoints;
  std::size_t pos_index = 0;
  const double v_max = sim_config.max_speed;

  mpc.getNonlinearConstraintBounds()->setLh({0.0});
  mpc.getNonlinearConstraintBounds()->setUh({v_max * v_max});
  mpc.updateNonlinearConstraintBounds();

  const Eigen::Matrix<double, 4, 1> zero_motor = Eigen::Matrix<double, 4, 1>::Zero();
  logger.save(0.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), waypoints[0],
              Eigen::Quaterniond::Identity(), 0.0, Eigen::Vector3d::Zero(),
              zero_motor, 0.0, 0, false, v_max);

  std::vector<double> mpc_times;
  std::vector<double> sim_times;
  std::vector<double> total_times;
  const std::size_t reserve = static_cast<std::size_t>(total_time / dt) + 1U;
  mpc_times.reserve(reserve);
  sim_times.reserve(reserve);
  total_times.reserve(reserve);

  std::cout << "Starting MPC simulation..." << std::endl;
  std::cout << "Total time: " << total_time << " s" << std::endl;
  std::cout << "Time step: " << dt << " s" << std::endl;
  std::cout << "Prediction steps: " << prediction_steps << std::endl;

  double t = 0.0;
  while (t < total_time + 1e-9) {
    t += dt;
    const auto iter_start = std::chrono::high_resolution_clock::now();

    const Eigen::Vector3d current_position = getStatePosition(*mpc_data);
    const Eigen::Quaterniond current_orientation = getStateOrientation(*mpc_data);
    const Eigen::Vector3d desired_position = waypoints[pos_index];
    const Eigen::Quaterniond desired_orientation = getDesiredOrientation(
        desired_position, current_position, current_orientation,
        sim_config.path_facing);
    setProgressiveReferences(mpc_data, current_position, desired_position,
                             desired_orientation, v_max, dt_horizon,
                             prediction_steps);

    const auto mpc_start = std::chrono::high_resolution_clock::now();
    const int mpc_status = mpc.solve();
    const auto mpc_end = std::chrono::high_resolution_clock::now();
    if (mpc_status != 0) {
      std::cerr << "\nMPC solver failed with status " << mpc_status
                << " at time " << t << " s" << std::endl;
    }

    const auto sim_start = std::chrono::high_resolution_clock::now();
    simulatorStep(simulator, mpc_data);
    const auto sim_end = std::chrono::high_resolution_clock::now();

    const double error = (getStatePosition(*mpc_data) - desired_position).norm();
    const bool hover_active =
        (pos_index == waypoints.size() - 1U) && (error < kWaypointReachedTolerance);
    if (error < kWaypointReachedTolerance && pos_index < waypoints.size() - 1U) {
      ++pos_index;
    }

    const std::chrono::duration<double> mpc_duration = mpc_end - mpc_start;
    const std::chrono::duration<double> sim_duration = sim_end - sim_start;
    const std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());

    const double controller_solve_time_us = mpc_duration.count() * 1e6;
    logger.save(t, getStatePosition(*mpc_data), getStateOrientation(*mpc_data),
                getStateVelocity(*mpc_data), Eigen::Vector3d::Zero(),
                desired_position, desired_orientation,
                mpc_data->actuation.getThrust(),
                getActuationAngularVelocity(*mpc_data), zero_motor,
                controller_solve_time_us, static_cast<int>(pos_index),
                hover_active, v_max);

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

}  // namespace mpc_acados_position_examples

int main(int argc, char** argv) {
  const auto args = mpc_acados_position_examples::parseArguments(argc, argv);

  mpc_acados_position_examples::YamlSimConfig sim_config;
  mpc_acados_position_examples::readSimYaml(args.config_path, sim_config);

  acados_mpc::MPC mpc;
  acados_mpc::configureMpcFromYaml(mpc, args.mpc_config_path);

  acados_mpc::MPCSimSolver simulator;
  mpc_acados_core::logging::CsvLogger logger(args.file_name);

  mpc_acados_position_examples::testMpcController(mpc, simulator, sim_config,
                                                  logger);
  return 0;
}
