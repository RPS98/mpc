// Copyright 2024 Universidad Politecnica de Madrid
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
//    * Neither the name of the Universidad Politecnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_sim_solver.hpp"

#include "utils/trajectory_generator_util.hpp"
#include "utils/utils.hpp"
#include "utils/yaml_utils.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

struct ExampleArgs {
  std::string config_path = "examples/simulation_config.yaml";
  std::string file_name   = "mpc_log.csv";
};

inline Eigen::Vector3d toEigenVector3(const std::array<double, 3>& value) {
  return {value[0], value[1], value[2]};
}

inline Eigen::Quaterniond toEigenQuaternion(const std::array<double, 4>& value) {
  return {value[0], value[1], value[2], value[3]};
}

inline Eigen::Vector3d getStatePosition(const MPCData& mpc_data) {
  return toEigenVector3(mpc_data.state.getPosition());
}

inline Eigen::Quaterniond getStateOrientation(const MPCData& mpc_data) {
  return toEigenQuaternion(mpc_data.state.getOrientation());
}

inline Eigen::Vector3d getStateVelocity(const MPCData& mpc_data) {
  return toEigenVector3(mpc_data.state.getLinearVelocity());
}

inline Eigen::Vector3d getActuationAngularVelocity(const MPCData& mpc_data) {
  return toEigenVector3(mpc_data.actuation.getAngularVelocity());
}

int simulatorStep(MPCSimSolver& simulator, MPCData* mpc_data) {
  const int status = simulator.solve(mpc_data);
  if (status != 0) {
    throw std::runtime_error("acados integrator returned status " + std::to_string(status) +
                             ". Exiting.");
  }
  return status;
}

std::unique_ptr<DynamicTrajectory> createTrajectoryGenerator(const YamlData& yaml_data) {
  const double desired_speed = yaml_data.max_speed;
  const auto& waypoints      = yaml_data.waypoints;
  std::unique_ptr<DynamicTrajectory> trajectory_generator =
      get_trajectory_generator(Eigen::Vector3d::Zero(), waypoints, desired_speed);
  std::cout << "Created trajectory generator with " << waypoints.size()
            << " and waypoints and desired speed " << desired_speed << " m/s." << std::endl;
  return trajectory_generator;
}

void setStageReference(MPCData* mpc_data,
                       const Eigen::Vector3d& desired_position,
                       const Eigen::Quaterniond& desired_orientation,
                       const Eigen::Vector3d& desired_velocity,
                       const double hover_thrust,
                       const int prediction_steps,
                       const int stage) {
  if (stage < prediction_steps) {
    const std::array<double, Reference::Nyref> y_ref = {desired_position.x(),
                                                        desired_position.y(),
                                                        desired_position.z(),
                                                        0.0,
                                                        0.0,
                                                        0.0,
                                                        desired_velocity.x(),
                                                        desired_velocity.y(),
                                                        desired_velocity.z(),
                                                        hover_thrust,
                                                        0.0,
                                                        0.0,
                                                        0.0};
    std::copy(y_ref.begin(), y_ref.end(),
              mpc_data->reference.data.begin() + stage * Reference::Nyref);
    mpc_data->p_params.setDesiredOrientation({desired_orientation.w(), desired_orientation.x(),
                                              desired_orientation.y(), desired_orientation.z()},
                                             stage);
    return;
  }

  const std::array<double, ReferenceEnd::size> y_ref_e = {
      desired_position.x(), desired_position.y(), desired_position.z(), 0.0, 0.0, 0.0,
      desired_velocity.x(), desired_velocity.y(), desired_velocity.z()};
  std::copy(y_ref_e.begin(), y_ref_e.end(), mpc_data->reference_end.data.begin());
  mpc_data->p_params.setDesiredOrientation({desired_orientation.w(), desired_orientation.x(),
                                            desired_orientation.y(), desired_orientation.z()},
                                           stage);
}

void configureMpc(MPC& mpc, const YamlData& yaml_data) {
  // Parameters
  mpc.getParameters()->setMass(yaml_data.mpc_data.mass[0]);
  mpc.getParameters()->setDesiredOrientation(yaml_data.mpc_data.desired_orientation);
  mpc.getParameters()->setExternalForce(yaml_data.mpc_data.external_force);

  // Gains
  mpc.getGains()->setQ(yaml_data.mpc_data.Q);
  mpc.getGains()->setQEnd(yaml_data.mpc_data.Qe);
  mpc.getGains()->setR(yaml_data.mpc_data.R);

  // Bounds
  if (yaml_data.mpc_data.has_actuation_bounds) {
    mpc.getActuationBounds()->setLbu(yaml_data.mpc_data.lbu);
    mpc.getActuationBounds()->setUbu(yaml_data.mpc_data.ubu);
  }

  if (yaml_data.mpc_data.has_state_bounds) {
    mpc.getStateBounds()->setLbx(yaml_data.mpc_data.lbx);
    mpc.getStateBounds()->setUbx(yaml_data.mpc_data.ubx);
  }

  if (yaml_data.mpc_data.has_soft_state_bounds) {
    mpc.getSoftStateBounds()->setLsbx(yaml_data.mpc_data.lsbx);
    mpc.getSoftStateBounds()->setUsbx(yaml_data.mpc_data.usbx);
  }

  if (yaml_data.mpc_data.has_slack_weights) {
    mpc.getSlackWeights()->setZl(yaml_data.mpc_data.Zl);
    mpc.getSlackWeights()->setZu(yaml_data.mpc_data.Zu);
    mpc.getSlackWeights()->setzl(yaml_data.mpc_data.zl);
    mpc.getSlackWeights()->setzu(yaml_data.mpc_data.zu);
  }

  if (yaml_data.mpc_data.has_terminal_slack_weights) {
    mpc.getSlackWeightsEnd()->setZlE(yaml_data.mpc_data.Zl_e);
    mpc.getSlackWeightsEnd()->setZuE(yaml_data.mpc_data.Zu_e);
    mpc.getSlackWeightsEnd()->setzlE(yaml_data.mpc_data.zl_e);
    mpc.getSlackWeightsEnd()->setzuE(yaml_data.mpc_data.zu_e);
  }

  // Update MPC with the new values
  if (yaml_data.mpc_data.has_actuation_bounds) {
    mpc.updateActuationBounds();
  }
  if (yaml_data.mpc_data.has_state_bounds) {
    mpc.updateStateBounds();
  }
  if (yaml_data.mpc_data.has_soft_state_bounds) {
    mpc.updateSoftStateBounds();
  }
  if (yaml_data.mpc_data.has_slack_weights) {
    mpc.updateSlackWeights();
  }
  if (yaml_data.mpc_data.has_terminal_slack_weights) {
    mpc.updateSlackWeightsEnd();
  }
  mpc.updateGains();
}

ExampleArgs parseArguments(int argc, char** argv) {
  ExampleArgs args;
  std::vector<std::string> positional_args;

  for (int index = 1; index < argc; ++index) {
    const std::string current_arg = argv[index];
    if ((current_arg == "-c" || current_arg == "--config_path") && index + 1 < argc) {
      args.config_path = argv[++index];
      continue;
    }
    if ((current_arg == "-f" || current_arg == "--file_name") && index + 1 < argc) {
      args.file_name = argv[++index];
      continue;
    }
    if (current_arg == "-h" || current_arg == "--help") {
      std::cout << "Usage: " << argv[0] << " [-c|--config_path <yaml>] [-f|--file_name <csv>]"
                << std::endl;
      std::exit(0);
    }
    positional_args.push_back(current_arg);
  }

  if (!positional_args.empty()) {
    args.config_path = positional_args[0];
  }
  if (positional_args.size() > 1) {
    args.file_name = positional_args[1];
  }
  if (positional_args.size() > 2) {
    throw std::invalid_argument("Too many positional arguments.");
  }

  return args;
}

namespace {

void printProgressBar(const double progress) {
  constexpr int kBarWidth       = 70;
  const double clamped_progress = std::clamp(progress, 0.0, 1.0);
  const int pos                 = static_cast<int>(kBarWidth * clamped_progress);

  std::cout << "[";
  for (int index = 0; index < kBarWidth; ++index) {
    if (index < pos) {
      std::cout << "=";
    } else if (index == pos) {
      std::cout << ">";
    } else {
      std::cout << " ";
    }
  }
  std::cout << "] " << static_cast<int>(clamped_progress * 100.0) << " %\r";
  std::cout.flush();
}

double computeAverage(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

}  // namespace

void testMpcController(MPC& mpc,
                       MPCSimSolver& simulator,
                       const YamlData& yaml_data,
                       CsvLogger& logger,
                       DynamicTrajectory& trajectory_generator) {
  MPCData* mpc_data          = mpc.getData();
  const int prediction_steps = mpc.getPredictionSteps();
  const double dt            = mpc.getPredictionTimeStep();
  const double hover_thrust  = mpc_data->p_params.getMass() * 9.81;

  // Sim time
  const double min_time = trajectory_generator.getMinTime();
  const double max_time = trajectory_generator.getMaxTime();

  // First log
  logger.save(0.0, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0,
              Eigen::Vector3d::Zero());

  // Arrays to store timing data
  std::vector<double> mpc_times;
  std::vector<double> sim_times;
  std::vector<double> total_times;
  mpc_times.reserve(static_cast<std::size_t>(max_time / dt) + 1U);
  sim_times.reserve(static_cast<std::size_t>(max_time / dt) + 1U);
  total_times.reserve(static_cast<std::size_t>(max_time / dt) + 1U);

  std::cout << "Starting MPC simulation..." << std::endl;
  std::cout << "Total time: " << max_time << " s" << std::endl;
  std::cout << "Time step: " << dt << " s" << std::endl;
  std::cout << "Prediction steps: " << prediction_steps << std::endl;

  double t = 0.0;
  while (t <= max_time) {
    t += dt;
    const auto iter_start = std::chrono::high_resolution_clock::now();

    // Set references
    Eigen::Quaterniond desired_orientation = getStateOrientation(*mpc_data);
    Eigen::Vector3d logged_position        = Eigen::Vector3d::Zero();
    Eigen::Vector3d logged_velocity        = Eigen::Vector3d::Zero();
    Eigen::Quaterniond logged_orientation  = desired_orientation;
    double t_eval                          = t;
    dynamic_traj_generator::References references;
    for (int stage = 0; stage < prediction_steps + 1; ++stage) {
      if (t_eval >= max_time) {
        t_eval = max_time - dt;
      } else if (t_eval <= min_time) {
        t_eval = min_time;
      }

      trajectory_generator.evaluateTrajectory(t_eval, references);
      const Eigen::Vector3d desired_position = references.position;
      const Eigen::Vector3d desired_velocity = references.velocity;

      if (yaml_data.path_facing) {
        desired_orientation = computePathFacing(desired_velocity);
      }

      setStageReference(mpc_data, desired_position, desired_orientation, desired_velocity,
                        hover_thrust, prediction_steps, stage);

      if (stage == 0) {
        logged_position    = desired_position;
        logged_velocity    = desired_velocity;
        logged_orientation = desired_orientation;
      }

      t_eval += dt;
    }

    // Solve MPC
    const auto mpc_start = std::chrono::high_resolution_clock::now();
    const int mpc_status = mpc.solve();
    const auto mpc_end   = std::chrono::high_resolution_clock::now();
    if (mpc_status != 0) {
      std::cerr << "\nMPC solver failed with status " << mpc_status << " at time " << t << " s"
                << std::endl;
    }

    // Simulate
    const auto sim_start = std::chrono::high_resolution_clock::now();
    simulatorStep(simulator, mpc_data);
    const auto sim_end = std::chrono::high_resolution_clock::now();

    // Log times
    const std::chrono::duration<double> mpc_duration   = mpc_end - mpc_start;
    const std::chrono::duration<double> sim_duration   = sim_end - sim_start;
    const std::chrono::duration<double> total_duration = sim_end - iter_start;
    mpc_times.push_back(mpc_duration.count());
    sim_times.push_back(sim_duration.count());
    total_times.push_back(total_duration.count());

    // Log data
    logger.save(t, getStatePosition(*mpc_data), getStateOrientation(*mpc_data),
                getStateVelocity(*mpc_data), logged_position, logged_orientation, logged_velocity,
                mpc_data->actuation.getThrust(), getActuationAngularVelocity(*mpc_data));

    // Update progress bar
    if (max_time > 0.0) {
      printProgressBar(t / max_time);
    }
  }

  std::cout << "\n";
  logger.close();

  const double mpc_avg_time   = computeAverage(mpc_times);
  const double sim_avg_time   = computeAverage(sim_times);
  const double total_avg_time = computeAverage(total_times);

  std::cout << "\n=== Time Statistics ===" << std::endl;
  std::cout << "MPC average time: " << mpc_avg_time * 1000.0 << " ms" << std::endl;
  std::cout << "Simulator average time: " << sim_avg_time * 1000.0 << " ms" << std::endl;
  std::cout << "Total average time: " << total_avg_time * 1000.0 << " ms" << std::endl;
  if (total_avg_time > 0.0) {
    std::cout << "Real-time factor: " << dt / total_avg_time << std::endl;
  }
}

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

int main(int argc, char** argv) {
  const auto args = acados_mpc::acados_mpc_examples::parseArguments(argc, argv);

  acados_mpc::acados_mpc_examples::YamlData yaml_data;
  acados_mpc::acados_mpc_examples::readYamlParams(args.config_path, yaml_data);

  acados_mpc::MPC mpc;
  acados_mpc::acados_mpc_examples::configureMpc(mpc, yaml_data);

  acados_mpc::MPCSimSolver simulator;
  acados_mpc::acados_mpc_examples::CsvLogger logger(args.file_name);
  std::unique_ptr<acados_mpc::acados_mpc_examples::DynamicTrajectory> trajectory_generator =
      acados_mpc::acados_mpc_examples::createTrajectoryGenerator(yaml_data);

  acados_mpc::acados_mpc_examples::testMpcController(mpc, simulator, yaml_data, logger,
                                                     *trajectory_generator);
  return 0;
}
