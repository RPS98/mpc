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

namespace acados_mpc {
namespace acados_mpc_examples {

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
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  // Calculate yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  yaw              = std::atan2(siny_cosp, cosy_cosp);
}

std::array<double, 4> compute_path_facing(const Eigen::Vector3d velocity) {
  double yaw = atan2(velocity.y(), velocity.x());
  double pitch, roll = 0.0;

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
  std::array<double, acados_mpc::OnlineParams::Np> p;
};

struct YamlData {
  double sim_time;
  double max_speed;
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
  data.sim_time  = config["sim_config"]["sim_time"].as<double>();
  data.max_speed = config["sim_config"]["max_speed"].as<double>();

  for (auto waypoint : config["sim_config"]["waypoints"]) {
    data.waypoints.push_back(Eigen::Vector3d(waypoint[0].as<double>(), waypoint[1].as<double>(),
                                             waypoint[2].as<double>()));
  }

  data.path_facing = config["sim_config"]["path_facing"].as<bool>();

  // Read MPC params
  double dt                = config["controller"]["mpc"]["dt"].as<double>();
  std::vector<double> Q    = config["controller"]["mpc"]["Q"].as<std::vector<double>>();
  std::vector<double> Qe   = config["controller"]["mpc"]["Qe"].as<std::vector<double>>();
  std::vector<double> R    = config["controller"]["mpc"]["R"].as<std::vector<double>>();
  std::vector<double> lbu  = config["controller"]["mpc"]["lbu"].as<std::vector<double>>();
  std::vector<double> ubu  = config["controller"]["mpc"]["ubu"].as<std::vector<double>>();
  std::vector<double> lbx  = config["controller"]["mpc"]["lbx"].as<std::vector<double>>();
  std::vector<double> ubx  = config["controller"]["mpc"]["ubx"].as<std::vector<double>>();
  std::vector<double> lsbx = config["controller"]["mpc"]["lsbx"].as<std::vector<double>>();
  std::vector<double> usbx = config["controller"]["mpc"]["usbx"].as<std::vector<double>>();
  std::vector<double> Zl   = config["controller"]["mpc"]["Zl"].as<std::vector<double>>();
  std::vector<double> Zu   = config["controller"]["mpc"]["Zu"].as<std::vector<double>>();
  std::vector<double> zl   = config["controller"]["mpc"]["zl"].as<std::vector<double>>();
  std::vector<double> zu   = config["controller"]["mpc"]["zu"].as<std::vector<double>>();
  std::vector<double> Zl_e = config["controller"]["mpc"]["Zl_e"].as<std::vector<double>>();
  std::vector<double> Zu_e = config["controller"]["mpc"]["Zu_e"].as<std::vector<double>>();
  std::vector<double> zl_e = config["controller"]["mpc"]["zl_e"].as<std::vector<double>>();
  std::vector<double> zu_e = config["controller"]["mpc"]["zu_e"].as<std::vector<double>>();
  std::vector<double> p    = config["controller"]["mpc"]["p"].as<std::vector<double>>();

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
  if (lbu.size() == acados_mpc::ActuationBounds::Nu &&
      ubu.size() == acados_mpc::ActuationBounds::Nu) {
    for (int i = 0; i < acados_mpc::ActuationBounds::Nu; i++) {
      data.mpc_data.lbu[i] = lbu[i];
      data.mpc_data.ubu[i] = ubu[i];
    }
  }
  if (lbx.size() == acados_mpc::StateBounds::Nx && ubx.size() == acados_mpc::StateBounds::Nx) {
    for (int i = 0; i < acados_mpc::StateBounds::Nx; i++) {
      data.mpc_data.lbx[i] = lbx[i];
      data.mpc_data.ubx[i] = ubx[i];
    }
  } else {
    std::cout << "Warning: State bounds size mismatch. Skipping state bounds update." << std::endl;
  }
  if (lsbx.size() == acados_mpc::SoftStateBounds::Nsbx &&
      usbx.size() == acados_mpc::SoftStateBounds::Nsbx) {
    for (int i = 0; i < acados_mpc::SoftStateBounds::Nsbx; i++) {
      data.mpc_data.lsbx[i] = lsbx[i];
      data.mpc_data.usbx[i] = usbx[i];
    }
  } else {
    std::cout << "Warning: Soft state bounds size mismatch. Skipping soft state bounds update."
              << std::endl;
  }
  if (Zl.size() == acados_mpc::SlackWeights::Nsbx && Zu.size() == acados_mpc::SlackWeights::Nsbx &&
      zl.size() == acados_mpc::SlackWeights::Nsbx && zu.size() == acados_mpc::SlackWeights::Nsbx) {
    for (int i = 0; i < acados_mpc::SlackWeights::Nsbx; i++) {
      data.mpc_data.Zl[i] = Zl[i];
      data.mpc_data.Zu[i] = Zu[i];
      data.mpc_data.zl[i] = zl[i];
      data.mpc_data.zu[i] = zu[i];
    }
  } else {
    std::cout << "Warning: Slack weights size mismatch. Skipping slack weights update."
              << std::endl;
  }
  if (Zl_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      Zu_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      zl_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e &&
      zu_e.size() == acados_mpc::SlackWeightsEnd::Nsbx_e) {
    for (int i = 0; i < acados_mpc::SlackWeightsEnd::Nsbx_e; i++) {
      data.mpc_data.Zl_e[i] = Zl_e[i];
      data.mpc_data.Zu_e[i] = Zu_e[i];
      data.mpc_data.zl_e[i] = zl_e[i];
      data.mpc_data.zu_e[i] = zu_e[i];
    }
  } else {
    std::cout << "Warning: Slack weights end size mismatch. Skipping slack weights end update."
              << std::endl;
  }
  for (int i = 0; i < acados_mpc::OnlineParams::Np; i++) {
    data.mpc_data.p[i] = p[i];
  }
}

class CsvLogger {
public:
  explicit CsvLogger(const std::string& file_name) : file_name_(file_name) {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,"
             "x,y,z,qw,qx,qy,qz,roll,pitch,yaw,vx,vy,vz,"
             "x_ref,y_ref,z_ref,qw_ref,qx_ref,qy_ref,qz_ref,roll_ref,pitch_ref,yaw_ref,"
             "vx_ref,vy_ref,vz_ref,thrust_ref,wx_ref,wy_ref,wz_ref"
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

    // // Reference position
    // add_double(mpc_data->reference.data[0]);
    // add_double(mpc_data->reference.data[1]);
    // add_double(mpc_data->reference.data[2]);

    // Reference orientation q
    Eigen::Quaterniond q_ref(mpc_data->p_params.data[1], mpc_data->p_params.data[2],
                             mpc_data->p_params.data[3], mpc_data->p_params.data[4]);
    add_double(q_ref.w());
    add_double(q_ref.x());
    add_double(q_ref.y());
    add_double(q_ref.z());

    // Reference orientation euler
    double roll_ref, pitch_ref, yaw_ref;
    quaternion_to_euler(q_ref, roll_ref, pitch_ref, yaw_ref);
    add_double(roll_ref);
    add_double(pitch_ref);
    add_double(yaw_ref);

    // // Reference velocity
    // add_double(mpc_data->reference.data[6]);
    // add_double(mpc_data->reference.data[7]);
    // add_double(mpc_data->reference.data[8]);

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
