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

#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>

#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_sim_solver.hpp"
#include "utils/trajectory_generator_util.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

void set_stage_reference(const dynamic_traj_generator::References& references,
                         acados_mpc::MPCData* mpc_data,
                         int index,
                         double hover_thrust) {
  if (index == MPC_N) {
    mpc_data->reference_end.setData(0, references.position.x());
    mpc_data->reference_end.setData(1, references.position.y());
    mpc_data->reference_end.setData(2, references.position.z());
    mpc_data->reference_end.setData(6, references.velocity.x());
    mpc_data->reference_end.setData(7, references.velocity.y());
    mpc_data->reference_end.setData(8, references.velocity.z());
    return;
  }

  if (index > MPC_N) {
    throw std::out_of_range("Index out of range.");
  }

  mpc_data->reference.setData(index, 0, references.position.x());
  mpc_data->reference.setData(index, 1, references.position.y());
  mpc_data->reference.setData(index, 2, references.position.z());
  mpc_data->reference.setData(index, 6, references.velocity.x());
  mpc_data->reference.setData(index, 7, references.velocity.y());
  mpc_data->reference.setData(index, 8, references.velocity.z());
  mpc_data->reference.setData(index, 9, hover_thrust);
  mpc_data->reference.setData(index, 10, 0.0);
  mpc_data->reference.setData(index, 11, 0.0);
  mpc_data->reference.setData(index, 12, 0.0);
}

void print_progress_bar(float progress) {
  const int bar_width = 70;
  std::cout << "[";
  const int pos = bar_width * progress;
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

void configure_mpc(MPC& mpc, const YamlData& yaml_data) {
  mpc.getGains()->setQ(yaml_data.mpc_data.Q);
  mpc.getGains()->setQEnd(yaml_data.mpc_data.Qe);
  mpc.getGains()->setR(yaml_data.mpc_data.R);
  mpc.updateGains();

  if (yaml_data.mpc_data.has_actuation_bounds) {
    mpc.getActuationBounds()->setLbu(yaml_data.mpc_data.lbu);
    mpc.getActuationBounds()->setUbu(yaml_data.mpc_data.ubu);
    mpc.updateActuationBounds();
  }

  if (yaml_data.mpc_data.has_state_bounds) {
    mpc.getStateBounds()->setLbx(yaml_data.mpc_data.lbx);
    mpc.getStateBounds()->setUbx(yaml_data.mpc_data.ubx);
    mpc.updateStateBounds();
  }

  if (yaml_data.mpc_data.has_soft_state_bounds) {
    mpc.getSoftStateBounds()->setLsbx(yaml_data.mpc_data.lsbx);
    mpc.getSoftStateBounds()->setUsbx(yaml_data.mpc_data.usbx);
    mpc.updateSoftStateBounds();
  }

  if (yaml_data.mpc_data.has_slack_weights) {
    mpc.getSlackWeights()->setZl(yaml_data.mpc_data.Zl);
    mpc.getSlackWeights()->setZu(yaml_data.mpc_data.Zu);
    mpc.getSlackWeights()->setzl(yaml_data.mpc_data.zl);
    mpc.getSlackWeights()->setzu(yaml_data.mpc_data.zu);
    mpc.updateSlackWeights();
  }

  if (yaml_data.mpc_data.has_terminal_slack_weights) {
    mpc.getSlackWeightsEnd()->setZlE(yaml_data.mpc_data.Zl_e);
    mpc.getSlackWeightsEnd()->setZuE(yaml_data.mpc_data.Zu_e);
    mpc.getSlackWeightsEnd()->setzlE(yaml_data.mpc_data.zl_e);
    mpc.getSlackWeightsEnd()->setzuE(yaml_data.mpc_data.zu_e);
    mpc.updateSlackWeightsEnd();
  }

  acados_mpc::Parameters params;
  for (size_t i = 0; i < acados_mpc::Parameters::size; ++i) {
    params.setData(static_cast<int>(i), yaml_data.mpc_data.p[i]);
  }
  mpc.setParameters(params);
}

void test_mpc_controller(CsvLogger& logger,
                         MPC& mpc,
                         MPCSimSolver& simulator,
                         std::unique_ptr<DynamicTrajectory>& trajectory_generator,
                         const YamlData& yaml_data) {
  dynamic_traj_generator::References references;
  const double tg_max_time = trajectory_generator->getMaxTime();

  MPCData* const mpc_data               = mpc.getData();
  OnlineParameters* const online_params = mpc.getParameters();
  const int prediction_steps            = mpc.getPredictionSteps();
  const double dt                       = mpc.getPredictionTimeStep();
  const double hover_thrust             = online_params->getMass() * 9.81;

  const double min_time = trajectory_generator->getMinTime();
  dynamic_traj_generator::References logged_references;
  double initial_eval_time = 0.0;
  if (initial_eval_time <= min_time) {
    initial_eval_time = min_time;
  }
  trajectory_generator->evaluateTrajectory(initial_eval_time, logged_references);
  std::array<double, 4> desired_orientation = online_params->getDesiredOrientation(0);
  if (yaml_data.path_facing) {
    desired_orientation = acados_mpc_examples::compute_path_facing(logged_references.velocity);
  }
  Eigen::Quaterniond desired_orientation_q(desired_orientation[0], desired_orientation[1],
                                           desired_orientation[2], desired_orientation[3]);
  logger.save(0.0, mpc_data, logged_references.position, desired_orientation_q,
              logged_references.velocity);

  const std::size_t n_iterations = static_cast<std::size_t>(tg_max_time / dt) + 1u;
  std::vector<double> mpc_times;
  mpc_times.reserve(n_iterations);
  std::vector<double> sim_times;
  sim_times.reserve(n_iterations);
  std::vector<double> total_times;
  total_times.reserve(n_iterations);

  for (double t = 0.0; t <= tg_max_time; t += dt) {
    print_progress_bar(t / tg_max_time);
    auto iter_start = std::chrono::high_resolution_clock::now();

    double t_eval       = t;
    desired_orientation = mpc_data->state.getOrientation();

    for (int i = 0; i <= prediction_steps; ++i) {
      if (t_eval >= tg_max_time) {
        t_eval = tg_max_time - dt;
      } else if (t_eval <= min_time) {
        t_eval = min_time;
      }
      trajectory_generator->evaluateTrajectory(t_eval, references);

      std::array<double, 4> stage_desired_orientation = online_params->getDesiredOrientation(i);
      if (yaml_data.path_facing) {
        stage_desired_orientation = acados_mpc_examples::compute_path_facing(references.velocity);
        online_params->setDesiredOrientation(stage_desired_orientation, i);
      }

      if (i == 0) {
        logged_references   = references;
        desired_orientation = stage_desired_orientation;
      }

      set_stage_reference(references, mpc_data, i, hover_thrust);
      t_eval += dt;
    }

    desired_orientation_q = Eigen::Quaterniond(desired_orientation[0], desired_orientation[1],
                                               desired_orientation[2], desired_orientation[3]);

    auto mpc_start = std::chrono::high_resolution_clock::now();
    mpc.solve();
    auto mpc_end = std::chrono::high_resolution_clock::now();

    auto sim_start       = std::chrono::high_resolution_clock::now();
    const int sim_status = simulator.solve(mpc_data);
    auto sim_end         = std::chrono::high_resolution_clock::now();
    if (sim_status != 0) {
      throw std::runtime_error("acados integrator returned non-zero status");
    }

    std::chrono::duration<double> mpc_duration   = mpc_end - mpc_start;
    std::chrono::duration<double> sim_duration   = sim_end - sim_start;
    std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());

    logger.save(t, mpc_data, logged_references.position, desired_orientation_q,
                logged_references.velocity);
  }
  std::cout << std::endl;
  logger.close();

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
  const std::string config_path = argc > 1 ? argv[1] : "examples/simulation_config.yaml";
  const std::string file_name   = argc > 2 ? argv[2] : "mpc_log.csv";

  acados_mpc::acados_mpc_examples::YamlData yaml_data;
  acados_mpc::acados_mpc_examples::read_yaml_params(config_path, yaml_data);

  acados_mpc::MPC mpc;
  acados_mpc::acados_mpc_examples::configure_mpc(mpc, yaml_data);

  acados_mpc::MPCSimSolver simulator;

  auto trajectory_generator = acados_mpc::acados_mpc_examples::get_trajectory_generator(
      Eigen::Vector3d::Zero(), yaml_data.waypoints, yaml_data.trajectory_generator_max_speed);

  acados_mpc::acados_mpc_examples::CsvLogger logger(file_name);

  acados_mpc::acados_mpc_examples::test_mpc_controller(logger, mpc, simulator, trajectory_generator,
                                                       yaml_data);
  return 0;
}
