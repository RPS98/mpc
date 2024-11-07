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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file run_example.cpp
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

#include "utils/utils.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

void traj_generator_ref_to_mpc_ref(const dynamic_traj_generator::References& references,
                                   acados_mpc::MPCData* mpc_data,
                                   int index,
                                   bool path_facing = false) {
  std::array<double, 4> q = {1.0, 0.0, 0.0, 0.0};
  if (path_facing) {
    q = acados_mpc_examples::compute_path_facing(references.velocity);
  }

  if (index == MPC_N) {
    // Position
    mpc_data->reference_end.set_data(0, references.position.x());
    mpc_data->reference_end.set_data(1, references.position.y());
    mpc_data->reference_end.set_data(2, references.position.z());

    // Velocity
    mpc_data->reference_end.set_data(6, references.velocity.x());
    mpc_data->reference_end.set_data(7, references.velocity.y());
    mpc_data->reference_end.set_data(8, references.velocity.z());

    // Orientation
    mpc_data->p_params.set_data(MPC_N, 1, q[0]);
    mpc_data->p_params.set_data(MPC_N, 2, q[1]);
    mpc_data->p_params.set_data(MPC_N, 3, q[2]);
    mpc_data->p_params.set_data(MPC_N, 4, q[3]);

    return;
  } else if (index > MPC_N) {
    throw std::out_of_range("Index out of range.");
  }
  // Position
  mpc_data->reference.set_data(index, 0, references.position.x());
  mpc_data->reference.set_data(index, 1, references.position.y());
  mpc_data->reference.set_data(index, 2, references.position.z());

  // Velocity
  mpc_data->reference.set_data(index, 6, references.velocity.x());
  mpc_data->reference.set_data(index, 7, references.velocity.y());
  mpc_data->reference.set_data(index, 8, references.velocity.z());

  // Control
  mpc_data->reference.set_data(index, 9, mpc_data->p_params.data[0] * 9.81);  // Thrust

  // Orientation
  mpc_data->p_params.set_data(index, 1, q[0]);
  mpc_data->p_params.set_data(index, 2, q[1]);
  mpc_data->p_params.set_data(index, 3, q[2]);
  mpc_data->p_params.set_data(index, 4, q[3]);
}

void print_progress_bar(float progress) {
  int bar_width = 70;
  std::cout << "[";
  int pos = bar_width * progress;
  for (int i = 0; i < bar_width; ++i) {
    if (i < pos) {
      std::cout << "=";
    } else if (i == pos) {
      std::cout << ">";
    } else {
      std::cout << " ";
    }
  }
  std::cout << "] " << int(progress * 100.0) << " %\r";
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
  const std::size_t n_iterations = static_cast<std::size_t>(tg_max_time / tf);
  std::vector<double> mpc_times;
  mpc_times.reserve(n_iterations);
  std::vector<double> sim_times;
  sim_times.reserve(n_iterations);
  std::vector<double> total_times;
  total_times.reserve(n_iterations);

  for (double t = 0; t < tg_max_time + 100; t += tf) {
    print_progress_bar(t / (tg_max_time + 100));
    auto iter_start = std::chrono::high_resolution_clock::now();

    double t_eval = t;

    // yref from 0 to N-1 (N steps) and yref_N from N to N
    for (int i = 0; i < prediction_steps + 1; i++) {
      if (t_eval >= tg_max_time) {
        t_eval = tg_max_time - tf;
      } else if (t_eval <= min_time) {
        t_eval = min_time;
      }
      trajectory_generator->evaluateTrajectory(t_eval, references);
      traj_generator_ref_to_mpc_ref(references, mpc_data, i, yaml_data.path_facing);
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
  // Params
  acados_mpc::acados_mpc_examples::YamlData yaml_data;
  // acados_mpc::acados_mpc_examples::read_yaml_params("examples/simulation_config.yaml",
  // yaml_data);
  acados_mpc::acados_mpc_examples::read_yaml_params(
      "/home/rafa/mpc_examples/thirdparty/mpc/examples/simulation_config.yaml", yaml_data);

  // Initialize MPC
  acados_mpc::MPC mpc = acados_mpc::MPC();

  // Update MPC gains and bounds
  mpc.get_gains()->set_Q(yaml_data.mpc_data.Q);
  mpc.get_gains()->set_Q_end(yaml_data.mpc_data.Qe);
  mpc.get_gains()->set_R(yaml_data.mpc_data.R);
  mpc.get_bounds()->set_lbu(yaml_data.mpc_data.lbu);
  mpc.get_bounds()->set_ubu(yaml_data.mpc_data.ubu);
  mpc.update_bounds();
  mpc.update_gains();

  // Update online params
  for (int i = 0; i < acados_mpc::OnlineParams::size_n; i++) {
    for (int j = 0; j < acados_mpc::OnlineParams::Np; j++) {
      mpc.get_data()->p_params.set_data(i, j, yaml_data.mpc_data.p[j]);
    }
  }

  // Initialize integrator
  acados_mpc::MPCSimSolver simulator = acados_mpc::MPCSimSolver();

  // Initialize trajectory generator
  auto trajectory_generator = acados_mpc::acados_mpc_examples::get_trajectory_generator(
      Eigen::Vector3d::Zero(), yaml_data.waypoints, yaml_data.trajectory_generator_max_speed);

  // Logger
  std::string file_name = "mpc_log.csv";
  acados_mpc::acados_mpc_examples::CsvLogger logger(file_name);

  acados_mpc::acados_mpc_examples::test_mpc_controller(logger, mpc, simulator, trajectory_generator,
                                                       yaml_data);
  return 0;
}
