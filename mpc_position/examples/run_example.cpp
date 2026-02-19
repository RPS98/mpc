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

void set_mpc_reference_parameters(const Eigen::Vector3d& waypoint,
                                  acados_mpc::MPC& mpc,
                                  const acados_mpc::MPCData* mpc_data,
                                  bool path_facing = false) {
  // Get current orientation
  Eigen::Quaterniond current_orientation;
  current_orientation.w() = mpc_data->state.data[3];
  current_orientation.x() = mpc_data->state.data[4];
  current_orientation.y() = mpc_data->state.data[5];
  current_orientation.z() = mpc_data->state.data[6];

  // Compute desired orientation
  Eigen::Quaterniond desired_orientation = current_orientation;
  if (path_facing) {
    Eigen::Vector3d current_position;
    current_position << mpc_data->state.data[0], mpc_data->state.data[1], mpc_data->state.data[2];
    desired_orientation = compute_path_facing(current_position, waypoint, current_orientation);
  }

  // Set online parameters using the new interface methods
  mpc.setSolverOnlineDesiredPositionParams({waypoint[0], waypoint[1], waypoint[2]});
  mpc.setSolverOnlineDesiredOrientationParams({
    desired_orientation.w(),
    desired_orientation.x(),
    desired_orientation.y(),
    desired_orientation.z()
  });
  mpc.setSolverOnlineDesiredVelocityParams({0.0, 0.0, 0.0});  // Zero velocity reference
  
  // Set desired actuation (hovering thrust)
  double mass = mpc.get_data()->p_params.data[0];
  // mpc.setSolverOnlineDesiredActuationParams({mass * 9.81, 0.0, 0.0, 0.0});
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
  std::cout.flush();
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
  
  std::cout << "Starting MPC simulation..." << std::endl;
  std::cout << "Total time: " << tg_max_time + hover_time << " s" << std::endl;
  std::cout << "Time step: " << tf << " s" << std::endl;
  std::cout << "Prediction steps: " << prediction_steps << std::endl;

  for (double t = 0; t < tg_max_time + hover_time; t += tf) {
    print_progress_bar(t / (tg_max_time + hover_time));
    auto iter_start = std::chrono::high_resolution_clock::now();

    // Set reference parameters for all prediction stages
    set_mpc_reference_parameters(
      yaml_data.waypoints[pos_index], 
      mpc, 
      mpc_data, 
      yaml_data.path_facing
    );

    // Solve MPC
    auto mpc_start = std::chrono::high_resolution_clock::now();
    int status = mpc.solve();
    auto mpc_end = std::chrono::high_resolution_clock::now();

    if (status != 0) {
      std::cerr << "\nMPC solver failed with status " << status << " at time " << t << std::endl;
    }

    // Simulate
    auto sim_start = std::chrono::high_resolution_clock::now();
    simulator.solve(mpc_data);
    auto sim_end = std::chrono::high_resolution_clock::now();

    // Time measurements
    std::chrono::duration<double> mpc_duration   = mpc_end - mpc_start;
    std::chrono::duration<double> sim_duration   = sim_end - sim_start;
    std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());
    
    // Log data
    logger.save(t, mpc_data);
    
    // Compute position error
    double error =
        std::sqrt(std::pow(mpc_data->state.data[0] - yaml_data.waypoints[pos_index][0], 2) +
                  std::pow(mpc_data->state.data[1] - yaml_data.waypoints[pos_index][1], 2) +
                  std::pow(mpc_data->state.data[2] - yaml_data.waypoints[pos_index][2], 2));
    
    // Update waypoint if reached
    if (error < 0.1 && pos_index < yaml_data.waypoints.size() - 1) {
      pos_index++;
      std::cout << "\nWaypoint " << pos_index << " reached at time " << t << " s" << std::endl;
      std::cout << "Next target: [" 
                << yaml_data.waypoints[pos_index][0] << ", "
                << yaml_data.waypoints[pos_index][1] << ", "
                << yaml_data.waypoints[pos_index][2] << "]" << std::endl;
    }
  }
  
  std::cout << "\n";  // New line after progress bar
  logger.close();

  // Print time statistics
  double mpc_avg_time = std::accumulate(mpc_times.begin(), mpc_times.end(), 0.0) / mpc_times.size();
  double sim_avg_time = std::accumulate(sim_times.begin(), sim_times.end(), 0.0) / sim_times.size();
  double total_avg_time = std::accumulate(total_times.begin(), total_times.end(), 0.0) / total_times.size();
  
  std::cout << "\n=== Time Statistics ===" << std::endl;
  std::cout << "MPC average time: " << mpc_avg_time * 1000 << " ms" << std::endl;
  std::cout << "Simulator average time: " << sim_avg_time * 1000 << " ms" << std::endl;
  std::cout << "Total average time: " << total_avg_time * 1000 << " ms" << std::endl;
  std::cout << "Real-time factor: " << tf / total_avg_time << std::endl;
}

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

int main(int argc, char** argv) {
  // Params
  std::string config_path = "examples/simulation_config.yaml";
  std::string log_file = "mpc_log.csv";
  acados_mpc::acados_mpc_examples::YamlData yaml_data;
  acados_mpc::acados_mpc_examples::read_yaml_params(config_path, yaml_data);

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
  mpc.get_soft_state_bounds()->set_lsbx(yaml_data.mpc_data.lsbx);
  mpc.get_soft_state_bounds()->set_usbx(yaml_data.mpc_data.usbx);
  mpc.get_slack_weights()->set_Zl(yaml_data.mpc_data.Zl);
  mpc.get_slack_weights()->set_Zu(yaml_data.mpc_data.Zu);
  mpc.get_slack_weights()->set_zl(yaml_data.mpc_data.zl);
  mpc.get_slack_weights()->set_zu(yaml_data.mpc_data.zu);
  mpc.get_slack_weights_end()->set_Zl_e(yaml_data.mpc_data.Zl_e);
  mpc.get_slack_weights_end()->set_Zu_e(yaml_data.mpc_data.Zu_e);
  mpc.get_slack_weights_end()->set_zl_e(yaml_data.mpc_data.zl_e);
  mpc.get_slack_weights_end()->set_zu_e(yaml_data.mpc_data.zu_e);
  mpc.update_actuation_bounds();
  mpc.update_state_bounds();
  mpc.update_soft_state_bounds();
  mpc.update_slack_weights();
  mpc.update_slack_weights_end();
  mpc.update_gains();

  // Set mass parameter (first parameter in p_params)
  mpc.setSolverOnlineMassParams(yaml_data.mpc_data.p[0]);

  // Initialize integrator
  acados_mpc::MPCSimSolver simulator = acados_mpc::MPCSimSolver();

  // Logger
  std::string file_name = "mpc_log.csv";
  acados_mpc::acados_mpc_examples::CsvLogger logger(file_name);

  acados_mpc::acados_mpc_examples::test_mpc_controller(logger, mpc, simulator, yaml_data);
  return 0;
}