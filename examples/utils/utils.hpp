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
 * @file example_utils.hpp
 *
 * Acados MPC examples using Acados Sim solver utility functions implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef EXAMPLE_UTILS_HPP_
#define EXAMPLE_UTILS_HPP_

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "acados_mpc/acados_mpc.hpp"
#include "acados_mpc/acados_sim_solver.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
using DynamicWaypoint   = dynamic_traj_generator::DynamicWaypoint;

Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw) {
  // Calculate half angles
  double roll_half  = roll * 0.5;
  double pitch_half = pitch * 0.5;
  double yaw_half   = yaw * 0.5;

  // Calculate the sine and cosine of the half angles
  double sr = sin(roll_half);
  double cr = cos(roll_half);
  double sp = sin(pitch_half);
  double cp = cos(pitch_half);
  double sy = sin(yaw_half);
  double cy = cos(yaw_half);

  // Calculate the quaternion components
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  // Create the Quaternion object
  return Eigen::Quaterniond(w, x, y, z).normalized();
}

std::array<double, 4> compute_path_facing(const Eigen::Vector3d velocity) {
  double yaw = atan2(velocity.y(), velocity.x());
  double pitch, roll = 0.0;

  Eigen::Quaterniond q = euler_to_quaternion(roll, pitch, yaw);
  return {q.w(), q.x(), q.y(), q.z()};
}

struct YamlMPCData {
  std::array<double, acados_mpc::Gains::Nq> Q;
  std::array<double, acados_mpc::Gains::Nqe> Qe;
  std::array<double, acados_mpc::Gains::Nr> R;
  std::array<double, acados_mpc::Bounds::Nu> lbu;
  std::array<double, acados_mpc::Bounds::Nu> ubu;
  std::array<double, acados_mpc::OnlineParams::Np> p;
};

struct YamlData {
  double trajectory_generator_max_speed;
  std::vector<Eigen::Vector3d> waypoints;
  bool path_facing;
  YamlMPCData mpc_data;
};

void read_yaml_params(const std::string& file_path, YamlData& data) {
  // Check if file exists
  std::ifstream f(file_path.c_str());
  if (!f.good()) {
    std::string absolute_simulation_config_path = std::filesystem::absolute(file_path).string();
    std::cout << "File " << absolute_simulation_config_path << " does not exist." << std::endl;
    f.close();
    throw std::invalid_argument("File does not exist");
  }
  f.close();
  YAML::Node config = YAML::LoadFile(file_path);

  // Read params
  data.trajectory_generator_max_speed =
      config["sim_config"]["trajectory_generator_max_speed"].as<double>();

  for (auto waypoint : config["sim_config"]["trajectory_generator_waypoints"]) {
    data.waypoints.push_back(Eigen::Vector3d(waypoint[0].as<double>(), waypoint[1].as<double>(),
                                             waypoint[2].as<double>()));
  }

  data.path_facing = config["sim_config"]["path_facing"].as<bool>();

  // Read MPC params
  std::vector<double> Q   = config["controller"]["mpc"]["Q"].as<std::vector<double>>();
  std::vector<double> Qe  = config["controller"]["mpc"]["Qe"].as<std::vector<double>>();
  std::vector<double> R   = config["controller"]["mpc"]["R"].as<std::vector<double>>();
  std::vector<double> lbu = config["controller"]["mpc"]["lbu"].as<std::vector<double>>();
  std::vector<double> ubu = config["controller"]["mpc"]["ubu"].as<std::vector<double>>();
  std::vector<double> p   = config["controller"]["mpc"]["p"].as<std::vector<double>>();

  for (int i = 0; i < acados_mpc::Gains::Nq; i++) {
    data.mpc_data.Q[i] = Q[i];
  }
  for (int i = 0; i < acados_mpc::Gains::Nqe; i++) {
    data.mpc_data.Qe[i] = Qe[i];
  }
  for (int i = 0; i < acados_mpc::Gains::Nr; i++) {
    data.mpc_data.R[i] = R[i];
  }
  for (int i = 0; i < acados_mpc::Bounds::Nu; i++) {
    data.mpc_data.lbu[i] = lbu[i];
    data.mpc_data.ubu[i] = ubu[i];
  }
  for (int i = 0; i < acados_mpc::OnlineParams::Np; i++) {
    data.mpc_data.p[i] = p[i];
  }
}

DynamicWaypoint::Vector eigen_vector_to_dynamic_waypoint_vector(
    const std::vector<Eigen::Vector3d>& vector_waypoints) {
  DynamicWaypoint::Vector vector_dynamic_waypoints;
  for (auto waypoint : vector_waypoints) {
    DynamicWaypoint dynamic_waypoint;
    dynamic_waypoint.resetWaypoint(waypoint);
    vector_dynamic_waypoints.push_back(dynamic_waypoint);
  }
  return vector_dynamic_waypoints;
}

std::unique_ptr<DynamicTrajectory> get_trajectory_generator(
    const Eigen::Vector3d initial_position,
    const std::vector<Eigen::Vector3d>& waypoints,
    const double speed) {
  // Initialize dynamic trajectory generator
  std::unique_ptr<DynamicTrajectory> trajectory_generator = std::make_unique<DynamicTrajectory>();

  trajectory_generator->updateVehiclePosition(initial_position);
  trajectory_generator->setSpeed(speed);

  // Set waypoints
  DynamicWaypoint::Vector waypoints_to_set = eigen_vector_to_dynamic_waypoint_vector(waypoints);

  // Generate trajectory
  trajectory_generator->setWaypoints(waypoints_to_set);
  double max_time = trajectory_generator->getMaxTime();  // Block until trajectory is generated

  std::cout << "Trajectory generated with max time: " << max_time << std::endl;

  return trajectory_generator;
}

class CsvLogger {
public:
  explicit CsvLogger(const std::string& file_name) : file_name_(file_name) {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,"
             "x,y,z,qw,qx,qy,qz,vx,vy,vz,"
             "x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,vx_ref,vy_ref,vz_ref"
             "thrust_ref,wx_ref,wy_ref,wz_ref"
          << std::endl;
  }

  ~CsvLogger() { file_.close(); }

  void add_double(const double data, const bool add_final_comma = true) {
    // Check if data is nan
    if (std::isnan(data)) {
      // Throw exception
      std::invalid_argument("Data is nan");
      return;
    }
    file_ << data;
    if (add_final_comma) {
      file_ << ",";
    }
  }

  void add_string(const std::string& data, const bool add_final_comma = true) {
    file_ << data;
    if (add_final_comma) {
      file_ << ",";
    }
  }

  void save(const double time, const MPCData* mpc_data) {
    // Time
    add_double(time);

    // State
    for (int i = 0; i < MPC_NX; i++) {
      add_double(mpc_data->state.data[i]);
    }

    // Reference position
    for (int i = 0; i < 3; i++) {
      add_double(mpc_data->reference.data[i]);
    }

    // Reference orientation
    for (int i = 1; i < 5; i++) {
      add_double(mpc_data->p_params.data[i]);
    }

    // Reference velocity
    for (int i = 7; i < 10; i++) {
      add_double(mpc_data->reference.data[i]);
    }

    // Actuation
    for (int i = 0; i < MPC_NU; i++) {
      bool add_final_comma = true;
      if (i == MPC_NU - 1) {
        add_final_comma = false;
      }
      add_double(mpc_data->control.data[i], add_final_comma);
    }

    // End line
    file_ << std::endl;
  }

  void close() { file_.close(); }

private:
  std::string file_name_;
  std::ofstream file_;
};

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

#endif  // EXAMPLE_UTILS_HPP_
