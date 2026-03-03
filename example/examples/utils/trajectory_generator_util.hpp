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

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <cmath>
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

void quaternion_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  // Extract the quaternion components
  double w = q.w();
  double x = q.x();
  double y = q.y();
  double z = q.z();

  // Calculate roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  roll             = std::atan2(sinr_cosp, cosr_cosp);

  // Calculate pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1) {
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  } else {
    pitch = std::asin(sinp);
  }

  // Calculate yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw              = std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 4> compute_path_facing(const Eigen::Vector3d velocity) {
  double yaw   = atan2(velocity.y(), velocity.x());
  double pitch = 0.0;
  double roll  = 0.0;

  Eigen::Quaterniond q = euler_to_quaternion(roll, pitch, yaw);
  return {q.w(), q.x(), q.y(), q.z()};
}

struct YamlMPCData {
  double dt;
  std::array<double, acados_mpc::Gains::Nq> Q;
  std::array<double, acados_mpc::Gains::Nqe> Qe;
  std::array<double, acados_mpc::Gains::Nr> R;
  std::array<double, acados_mpc::ActuationBounds::Nu> lbu;
  std::array<double, acados_mpc::ActuationBounds::Nu> ubu;
  std::array<double, acados_mpc::StateBounds::Nx> lbx;
  std::array<double, acados_mpc::StateBounds::Nx> ubx;
  std::array<double, acados_mpc::SoftStateBounds::Nsbx> lsbx;
  std::array<double, acados_mpc::SoftStateBounds::Nsbx> usbx;
  std::array<double, acados_mpc::SlackWeights::Nsbx> Zl;
  std::array<double, acados_mpc::SlackWeights::Nsbx> Zu;
  std::array<double, acados_mpc::SlackWeights::Nsbx> zl;
  std::array<double, acados_mpc::SlackWeights::Nsbx> zu;
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> Zl_e;
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> Zu_e;
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> zl_e;
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> zu_e;
  std::array<double, acados_mpc::Parameters::Np> p;
  bool has_actuation_bounds       = false;
  bool has_state_bounds           = false;
  bool has_soft_state_bounds      = false;
  bool has_slack_weights          = false;
  bool has_terminal_slack_weights = false;
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

  YAML::Node sim_config     = config["sim_config"];
  YAML::Node max_speed_node = sim_config["max_speed"]
                                  ? sim_config["max_speed"]
                                  : sim_config["trajectory_generator_max_speed"];
  YAML::Node waypoints_node = sim_config["waypoints"]
                                  ? sim_config["waypoints"]
                                  : sim_config["trajectory_generator_waypoints"];

  // Read params
  data.trajectory_generator_max_speed = max_speed_node.as<double>();

  for (auto waypoint : waypoints_node) {
    data.waypoints.push_back(Eigen::Vector3d(waypoint[0].as<double>(), waypoint[1].as<double>(),
                                             waypoint[2].as<double>()));
  }

  data.path_facing = config["sim_config"]["path_facing"].as<bool>();

  auto read_optional_vector = [](const YAML::Node& node) -> std::vector<double> {
    if (!node || !node.IsDefined() || node.IsNull()) {
      return {};
    }
    return node.as<std::vector<double>>();
  };

  YAML::Node mpc_config  = config["controller"]["mpc"];
  YAML::Node cost_config = mpc_config["cost"] ? mpc_config["cost"] : mpc_config;
  YAML::Node constraints_config =
      mpc_config["constraints"] ? mpc_config["constraints"] : mpc_config;
  YAML::Node parameters_config = mpc_config["parameters"] ? mpc_config["parameters"] : mpc_config;

  // Read MPC params
  double dt                = mpc_config["dt"].as<double>();
  std::vector<double> Q    = read_optional_vector(cost_config["Q"]);
  std::vector<double> Qe   = read_optional_vector(cost_config["Qe"]);
  std::vector<double> R    = read_optional_vector(cost_config["R"]);
  std::vector<double> lbu  = read_optional_vector(constraints_config["lbu"]);
  std::vector<double> ubu  = read_optional_vector(constraints_config["ubu"]);
  std::vector<double> lbx  = read_optional_vector(constraints_config["lbx"]);
  std::vector<double> ubx  = read_optional_vector(constraints_config["ubx"]);
  std::vector<double> p    = read_optional_vector(parameters_config["p"]);
  std::vector<double> mass = read_optional_vector(parameters_config["mass"]);
  std::vector<double> desired_orientation =
      read_optional_vector(parameters_config["desired_orientation"]);
  std::vector<double> external_force = read_optional_vector(parameters_config["external_force"]);
  std::vector<double> lsbx           = read_optional_vector(constraints_config["lsbx"]);
  std::vector<double> usbx           = read_optional_vector(constraints_config["usbx"]);
  std::vector<double> Zl             = read_optional_vector(constraints_config["Zl"]);
  std::vector<double> Zu             = read_optional_vector(constraints_config["Zu"]);
  std::vector<double> zl             = read_optional_vector(constraints_config["zl"]);
  std::vector<double> zu             = read_optional_vector(constraints_config["zu"]);
  std::vector<double> Zl_e           = read_optional_vector(constraints_config["Zl_e"]);
  std::vector<double> Zu_e           = read_optional_vector(constraints_config["Zu_e"]);
  std::vector<double> zl_e           = read_optional_vector(constraints_config["zl_e"]);
  std::vector<double> zu_e           = read_optional_vector(constraints_config["zu_e"]);

  data.mpc_data.dt = dt;
  for (int i = 0; i < acados_mpc::Gains::Nq; i++) {
    data.mpc_data.Q[i] = Q[i];
  }
  for (int i = 0; i < acados_mpc::Gains::Nqe; i++) {
    data.mpc_data.Qe[i] = Qe[i];
  }
  for (int i = 0; i < acados_mpc::Gains::Nr; i++) {
    data.mpc_data.R[i] = R[i];
  }
  if (acados_mpc::ActuationBounds::Nu > 0 && lbu.size() == acados_mpc::ActuationBounds::Nu &&
      ubu.size() == acados_mpc::ActuationBounds::Nu) {
    for (int i = 0; i < acados_mpc::ActuationBounds::Nu; i++) {
      data.mpc_data.lbu[i] = lbu[i];
      data.mpc_data.ubu[i] = ubu[i];
    }
    data.mpc_data.has_actuation_bounds = true;
  }

  if (acados_mpc::StateBounds::Nx > 0 && lbx.size() == acados_mpc::StateBounds::Nx &&
      ubx.size() == acados_mpc::StateBounds::Nx) {
    for (int i = 0; i < acados_mpc::StateBounds::Nx; i++) {
      data.mpc_data.lbx[i] = lbx[i];
      data.mpc_data.ubx[i] = ubx[i];
    }
    data.mpc_data.has_state_bounds = true;
  }

  if (acados_mpc::SoftStateBounds::Nsbx > 0 && lsbx.size() == acados_mpc::SoftStateBounds::Nsbx &&
      usbx.size() == acados_mpc::SoftStateBounds::Nsbx) {
    for (int i = 0; i < acados_mpc::SoftStateBounds::Nsbx; i++) {
      data.mpc_data.lsbx[i] = lsbx[i];
      data.mpc_data.usbx[i] = usbx[i];
    }
    data.mpc_data.has_soft_state_bounds = true;
  }

  if (acados_mpc::SlackWeights::Nsbx > 0 && Zl.size() == acados_mpc::SlackWeights::Nsbx &&
      Zu.size() == acados_mpc::SlackWeights::Nsbx && zl.size() == acados_mpc::SlackWeights::Nsbx &&
      zu.size() == acados_mpc::SlackWeights::Nsbx) {
    for (int i = 0; i < acados_mpc::SlackWeights::Nsbx; i++) {
      data.mpc_data.Zl[i] = Zl[i];
      data.mpc_data.Zu[i] = Zu[i];
      data.mpc_data.zl[i] = zl[i];
      data.mpc_data.zu[i] = zu[i];
    }
    data.mpc_data.has_slack_weights = true;
  }

  if (acados_mpc::SlackWeightsEnd::Nsbx_e > 0 &&
      Zl_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      Zu_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      zl_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      zu_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e) {
    for (int i = 0; i < acados_mpc::SlackWeightsEnd::Nsbx_e; i++) {
      data.mpc_data.Zl_e[i] = Zl_e[i];
      data.mpc_data.Zu_e[i] = Zu_e[i];
      data.mpc_data.zl_e[i] = zl_e[i];
      data.mpc_data.zu_e[i] = zu_e[i];
    }
    data.mpc_data.has_terminal_slack_weights = true;
  }

  if (!p.empty()) {
    for (int i = 0; i < acados_mpc::Parameters::Np && i < static_cast<int>(p.size()); i++) {
      data.mpc_data.p[i] = p[i];
    }
  } else {
    if (!mass.empty()) {
      data.mpc_data.p[0] = mass[0];
    }
    if (desired_orientation.size() == acados_mpc::Parameters::desired_orientation_length) {
      for (int i = 0; i < acados_mpc::Parameters::desired_orientation_length; i++) {
        data.mpc_data.p[acados_mpc::Parameters::desired_orientation_offset + i] =
            desired_orientation[i];
      }
    }
    if (external_force.size() == acados_mpc::Parameters::external_force_length) {
      for (int i = 0; i < acados_mpc::Parameters::external_force_length; i++) {
        data.mpc_data.p[acados_mpc::Parameters::external_force_offset + i] = external_force[i];
      }
    }
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
             "x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,"
             "x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,"
             "vx_ref,vy_ref,vz_ref,thrust,wx,wy,wz"
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

  void save(const double time,
            const MPCData* mpc_data,
            const Eigen::Vector3d& desired_position,
            const Eigen::Quaterniond& desired_orientation,
            const Eigen::Vector3d& desired_velocity = Eigen::Vector3d::Zero()) {
    // Time
    add_double(time);

    // State position
    add_double(mpc_data->state.data[0]);
    add_double(mpc_data->state.data[1]);
    add_double(mpc_data->state.data[2]);

    // State orientation q
    Eigen::Quaterniond q(mpc_data->state.data[3], mpc_data->state.data[4], mpc_data->state.data[5],
                         mpc_data->state.data[6]);
    add_double(q.w());
    add_double(q.x());
    add_double(q.y());
    add_double(q.z());

    // State orientation euler
    double roll, pitch, yaw;
    quaternion_to_euler(q, roll, pitch, yaw);
    add_double(roll);
    add_double(pitch);
    add_double(yaw);

    // State velocity
    add_double(mpc_data->state.data[7]);
    add_double(mpc_data->state.data[8]);
    add_double(mpc_data->state.data[9]);

    // Reference position
    add_double(desired_position.x());
    add_double(desired_position.y());
    add_double(desired_position.z());

    // Reference orientation q
    add_double(desired_orientation.w());
    add_double(desired_orientation.x());
    add_double(desired_orientation.y());
    add_double(desired_orientation.z());

    // Reference orientation euler
    double roll_ref, pitch_ref, yaw_ref;
    quaternion_to_euler(desired_orientation, roll_ref, pitch_ref, yaw_ref);
    add_double(roll_ref);
    add_double(pitch_ref);
    add_double(yaw_ref);

    // Reference velocity
    add_double(desired_velocity.x());
    add_double(desired_velocity.y());
    add_double(desired_velocity.z());

    // Actuation
    for (int i = 0; i < MPC_NU; i++) {
      bool add_final_comma = true;
      if (i == MPC_NU - 1) {
        add_final_comma = false;
      }
      add_double(mpc_data->actuation.data[i], add_final_comma);
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
