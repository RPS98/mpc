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
 * @file integrator_example.cpp
 *
 * Acados MPC examples using Acados Sim solver.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <dynamic_trajectory_generator/dynamic_trajectory.hpp>
#include <dynamic_trajectory_generator/dynamic_waypoint.hpp>

#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_sim_solver.hpp"

#include "utils/example_utils.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

void traj_generator_ref_to_mpc_ref(const dynamic_traj_generator::References& references,
                                   MPCData* mpc_data,
                                   int index) {
  State state;
  state.data[0] = references.position.x();
  state.data[1] = references.position.y();
  state.data[2] = references.position.z();
  state.data[3] = 1.0;
  state.data[4] = 0.0;
  state.data[5] = 0.0;
  state.data[6] = 0.0;
  state.data[7] = references.velocity.x();
  state.data[8] = references.velocity.y();
  state.data[9] = references.velocity.z();

  if (index == MPC_N) {
    mpc_data->reference_end = state;
    return;
  } else if (index > MPC_N) {
    throw std::out_of_range("Index out of range.");
  }
  mpc_data->reference.set_state(index, state);
}

void test_mpc_controller(CsvLogger& logger,
                         MPC& mpc,
                         MPCSimSolver& simulator,
                         std::unique_ptr<DynamicTrajectory>& trajectory_generator,
                         YamlData& yaml_data) {
  // Initialize dynamic trajectory generator
  dynamic_traj_generator::References references;
  double tg_max_time = trajectory_generator->getMaxTime();

  // MPC Parameters
  MPCData* mpc_data       = mpc.get_data();
  double prediction_steps = mpc.get_prediction_steps();
  // double prediction_horizon = mpc.get_prediction_horizon();
  double tf = mpc.get_prediction_time_step();

  // Simulation
  double min_time = trajectory_generator->getMinTime();  // seconds
  double t        = 0.0;                                 // seconds
  logger.save(t, mpc_data);

  // Time measurement
  const std::size_t n_iterations = static_cast<std::size_t>(yaml_data.sim_time / yaml_data.dt);
  std::vector<double> mpc_times;
  mpc_times.reserve(n_iterations);
  std::vector<double> sim_times;
  sim_times.reserve(n_iterations);
  std::vector<double> total_times;
  total_times.reserve(n_iterations);

  for (double t = 0; t < yaml_data.sim_time; t += yaml_data.dt) {
    auto iter_start = std::chrono::high_resolution_clock::now();

    double t_eval = t;

    // yref from 0 to N-1 (N steps) and yref_N from N to N
    for (int i = 0; i < prediction_steps + 1; i++) {
      if (t_eval >= tg_max_time) {
        t_eval = tg_max_time - yaml_data.dt;
      } else if (t_eval <= min_time) {
        t_eval = min_time;
      }
      trajectory_generator->evaluateTrajectory(t_eval, references);
      traj_generator_ref_to_mpc_ref(references, mpc_data, i);
      t_eval += tf;
    }

    // Solve MPC
    auto mpc_start = std::chrono::high_resolution_clock::now();
    mpc.solve();
    auto mpc_end = std::chrono::high_resolution_clock::now();

    // Simulate
    auto sim_start = std::chrono::high_resolution_clock::now();
    simulator.solve(mpc_data);
    auto sim_end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> mpc_duration   = mpc_end - mpc_start;
    std::chrono::duration<double> sim_duration   = sim_end - sim_start;
    std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());

    logger.save(t, mpc_data);
  }
  logger.close();

  // Print time measurements and its average
  double mpc_avg_time = std::accumulate(mpc_times.begin(), mpc_times.end(), 0.0) / mpc_times.size();
  double sim_avg_time = std::accumulate(sim_times.begin(), sim_times.end(), 0.0) / sim_times.size();
  double total_avg_time =
      std::accumulate(total_times.begin(), total_times.end(), 0.0) / total_times.size();
  std::cout << "MPC average time: " << mpc_avg_time << " s" << std::endl;
  std::cout << "Simulator average time: " << sim_avg_time << " s" << std::endl;
  std::cout << "Total average time: " << total_avg_time << " s" << std::endl;
}

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

int main(int argc, char** argv) {
  // Logger
  std::string file_name = "mpc_log.csv";
  acados_mpc::acados_mpc_examples::CsvLogger logger(file_name);

  // Params
  acados_mpc::acados_mpc_examples::YamlData yaml_data;
  acados_mpc::acados_mpc_examples::read_yaml_params(
      "/home/rafa/mpc/examples/simulation_config.yaml", yaml_data);

  // Initialize MPC
  acados_mpc::MPC mpc = acados_mpc::MPC();

  // Initialize integrator
  acados_mpc::MPCSimSolver simulator = acados_mpc::MPCSimSolver();

  // Initialize trajectory generator
  auto trajectory_generator = acados_mpc::acados_mpc_examples::get_trajectory_generator(
      Eigen::Vector3d::Zero(), yaml_data.waypoints, yaml_data.trajectory_generator_max_speed);

  acados_mpc::acados_mpc_examples::test_mpc_controller(logger, mpc, simulator, trajectory_generator,
                                                       yaml_data);
  return 0;
}
