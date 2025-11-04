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
 * @author Carmen De Rojas Pita-Romero <c.derojas@upm.es>
 */

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>

#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_sim_solver.hpp"

#include "utils/utils.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

Eigen::Quaterniond compute_path_facing(const Eigen::Vector3d& current_position,
                                       const Eigen::Vector3d& target_position,
                                       const Eigen::Quaterniond& current_orientation) {
  double x_diff        = target_position[0] - current_position[0];
  double y_diff        = target_position[1] - current_position[1];
  Eigen::Vector2d diff = Eigen::Vector2d(x_diff, y_diff);
  if (diff.norm() < 0.1) {
    return current_orientation;
  }
  double yaw_target = std::atan2(y_diff, x_diff);
  Eigen::Quaterniond q_target;
  q_target = Eigen::AngleAxisd(yaw_target, Eigen::Vector3d::UnitZ());
  return q_target;
}

void position_ref_to_mpc_ref(Eigen::Vector3d waypoint,
                             acados_mpc::MPCData* mpc_data,
                             int index,
                             bool path_facing = false) {
  Eigen::Quaterniond current_orientation;
  current_orientation.w()                = mpc_data->state.data[3];
  current_orientation.x()                = mpc_data->state.data[4];
  current_orientation.y()                = mpc_data->state.data[5];
  current_orientation.z()                = mpc_data->state.data[6];
  Eigen::Quaterniond desired_orientation = current_orientation;
  if (index == MPC_N) {
    // Position
    mpc_data->p_params.set_data(5, waypoint[0]);
    mpc_data->p_params.set_data(6, waypoint[1]);
    mpc_data->p_params.set_data(7, waypoint[2]);

    // Orientation
    if (path_facing) {
      Eigen::Vector3d current_position;
      current_position << mpc_data->state.data[0], mpc_data->state.data[1], mpc_data->state.data[2];
      desired_orientation = compute_path_facing(current_position, waypoint, current_orientation);
    }
    mpc_data->p_params.set_data(MPC_N, 1, desired_orientation.w());
    mpc_data->p_params.set_data(MPC_N, 2, desired_orientation.x());
    mpc_data->p_params.set_data(MPC_N, 3, desired_orientation.y());
    mpc_data->p_params.set_data(MPC_N, 4, desired_orientation.z());

    return;
  } else if (index > MPC_N) {
    throw std::out_of_range("Index out of range.");
  }
  // Position
  mpc_data->p_params.set_data(index, 5, waypoint[0]);
  mpc_data->p_params.set_data(index, 6, waypoint[1]);
  mpc_data->p_params.set_data(index, 7, waypoint[2]);

  // Control
  mpc_data->reference.set_data(index, 9, mpc_data->p_params.data[0] * 9.81);  // Thrust

  // Orientation
  if (path_facing) {
    Eigen::Vector3d current_position;
    current_position << mpc_data->state.data[0], mpc_data->state.data[1], mpc_data->state.data[2];
    desired_orientation = compute_path_facing(current_position, waypoint, current_orientation);
  }
  mpc_data->p_params.set_data(index, 1, desired_orientation.w());
  mpc_data->p_params.set_data(index, 2, desired_orientation.x());
  mpc_data->p_params.set_data(index, 3, desired_orientation.y());
  mpc_data->p_params.set_data(index, 4, desired_orientation.z());
  return;
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
                         YamlData& yaml_data) {
  double tg_max_time = yaml_data.sim_time;

  // MPC Parameters
  MPCData* mpc_data       = mpc.get_data();
  double prediction_steps = mpc.get_prediction_steps();
  double tf               = mpc.get_prediction_time_step();

  // Simulation
  double t = 0.0;  // seconds
  logger.save(t, mpc_data);

  // Time measurement
  const std::size_t n_iterations = static_cast<std::size_t>(tg_max_time / tf);
  std::vector<double> mpc_times;
  mpc_times.reserve(n_iterations);
  std::vector<double> sim_times;
  sim_times.reserve(n_iterations);
  std::vector<double> total_times;
  total_times.reserve(n_iterations);

  int pos_index     = 0;
  double hover_time = 2.0;
  for (double t = 0; t < tg_max_time + hover_time; t += tf) {
    print_progress_bar(t / (tg_max_time + hover_time));
    auto iter_start = std::chrono::high_resolution_clock::now();

    // yref from 0 to N-1 (N steps) and yref_N from N to N
    for (int i = 0; i < prediction_steps + 1; i++) {
      position_ref_to_mpc_ref(yaml_data.waypoints[pos_index], mpc_data, i, yaml_data.path_facing);
    }
    // Solve MPC
    auto mpc_start = std::chrono::high_resolution_clock::now();
    int status     = mpc.solve();
    auto mpc_end   = std::chrono::high_resolution_clock::now();
    if (status != ACADOS_SUCCESS) {
      std::cerr << "MPC solver failed with status " << status << " at time " << t << " s"
                << std::endl;
    }

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
    double error =
        std::sqrt(std::pow(mpc_data->state.data[0] - yaml_data.waypoints[pos_index][0], 2) +
                  std::pow(mpc_data->state.data[1] - yaml_data.waypoints[pos_index][1], 2) +
                  std::pow(mpc_data->state.data[2] - yaml_data.waypoints[pos_index][2], 2));
    // If the error is less than 0.1 m, go to the next waypoint
    if (error < 0.1 && pos_index < yaml_data.waypoints.size() - 1) {
      pos_index++;
    }
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
  acados_mpc::acados_mpc_examples::read_yaml_params(
      "/home/rafa/mpc/mpc_position_error/examples/simulation_config.yaml", yaml_data);

  // Initialize MPC
  acados_mpc::MPC mpc = acados_mpc::MPC();

  // Update MPC gains and bounds
  mpc.get_gains()->set_Q(yaml_data.mpc_data.Q);
  mpc.get_gains()->set_Q_end(yaml_data.mpc_data.Qe);
  mpc.get_gains()->set_R(yaml_data.mpc_data.R);
  mpc.get_actuation_bounds()->set_lbu(yaml_data.mpc_data.lbu);
  mpc.get_actuation_bounds()->set_ubu(yaml_data.mpc_data.ubu);
  mpc.get_state_bounds()->set_lbx(yaml_data.mpc_data.lbx);
  mpc.get_state_bounds()->set_ubx(yaml_data.mpc_data.ubx);
  mpc.update_actuation_bounds();
  // mpc.update_state_bounds();
  mpc.update_gains();

  // Update online params
  for (int i = 0; i < acados_mpc::OnlineParams::size_n; i++) {
    for (int j = 0; j < acados_mpc::OnlineParams::Np; j++) {
      mpc.get_data()->p_params.set_data(i, j, yaml_data.mpc_data.p[j]);
    }
  }

  // Initialize integrator
  acados_mpc::MPCSimSolver simulator = acados_mpc::MPCSimSolver();

  // Logger
  std::string file_name = "mpc_log.csv";
  acados_mpc::acados_mpc_examples::CsvLogger logger(file_name);

  acados_mpc::acados_mpc_examples::test_mpc_controller(logger, mpc, simulator, yaml_data);
  return 0;
}
