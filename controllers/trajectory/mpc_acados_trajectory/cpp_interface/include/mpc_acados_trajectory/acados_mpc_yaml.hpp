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
 * @file acados_mpc_yaml.hpp
 *
 * YAML-based configuration utilities for the Acados MPC library.
 * Auto-generated from templ_acados_mpc_yaml.hpp.j2 — do not edit directly.
 * Regenerate by running model_definition_generation.py.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef ACADOS_MPC_ACADOS_MPC_YAML_HPP_
#define ACADOS_MPC_ACADOS_MPC_YAML_HPP_

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mpc_acados_trajectory/acados_mpc.hpp"

namespace acados_mpc {

/**
 * @brief MPC configuration parameters loaded from a YAML file.
 *
 * Optional entries are stored as vectors:
 * - key absent in YAML  -> empty vector
 * - key present in YAML -> vector with exact expected size
 *
 * During configuration, only non-empty optional vectors are applied to MPC.
 */
struct MpcYamlConfig {
  // --- Online parameters (generated from model_definition.yaml) ---
  /// Mass of the MAV (kg)
  std::vector<double> mass{};
  /// Desired position in world frame [x, y, z] (m)
  std::vector<double> desired_position{};
  /// Desired orientation as a quaternion [qw, qx, qy, qz]
  std::vector<double> desired_orientation{};
  /// Desired linear velocity in world frame [vx, vy, vz] (m/s)
  std::vector<double> desired_velocity{};
  /// Desired linear acceleration in world frame [ax, ay, az] (m/s^2)
  std::vector<double> desired_acceleration{};
  /// External force acting on the MAV in base frame [fx, fy, fz] (N)
  std::vector<double> external_force{};
  /// Stage gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
  std::vector<double> Q{};
  /// Terminal gains for [x, y, z, roll, pitch, yaw, vx, vy, vz]
  std::vector<double> Qe{};
  /// Stage gains for control inputs [thrust, wx, wy, wz]
  std::vector<double> R{};

  // --- Actuation bounds (required: must be present in the YAML) ---
  std::array<double, ActuationBounds::Nu> lbu{};
  std::array<double, ActuationBounds::Nu> ubu{};

  // --- State bounds (optional) ---
  std::vector<double> lbx{};
  std::vector<double> ubx{};
  std::vector<double> lsbx{};
  std::vector<double> usbx{};

  // --- Nonlinear constraint bounds (optional) ---
  std::vector<double> lh{};
  std::vector<double> uh{};
  std::vector<double> lsh{};
  std::vector<double> ush{};

  // --- Slack weights for intermediate stages (optional) ---
  std::vector<double> Zl{};
  std::vector<double> Zu{};
  std::vector<double> zl{};
  std::vector<double> zu{};

  // --- Slack weights for the terminal stage (optional) ---
  std::vector<double> Zl_e{};
  std::vector<double> Zu_e{};
  std::vector<double> zl_e{};
  std::vector<double> zu_e{};
};

namespace detail {

template <std::size_t N>
std::array<double, N> vectorToArray(const std::vector<double>& values, const std::string& name) {
  if (values.size() != N) {
    throw std::invalid_argument(name + " must have " + std::to_string(N) + " elements.");
  }
  std::array<double, N> data{};
  std::copy_n(values.begin(), N, data.begin());
  return data;
}

template <std::size_t N>
std::array<double, N> yamlArrayRequired(const YAML::Node& node, const std::string& name) {
  if (!node || node.IsNull()) {
    throw std::invalid_argument("Missing required YAML entry: " + name);
  }
  return vectorToArray<N>(node.as<std::vector<double>>(), name);
}

template <std::size_t N>
std::vector<double> yamlVectorOptional(const YAML::Node& node, const std::string& name) {
  if (!node || node.IsNull()) {
    return {};
  }
  const std::vector<double> values = node.as<std::vector<double>>();
  if (values.size() != N) {
    throw std::invalid_argument(name + " must have " + std::to_string(N) + " elements.");
  }
  return values;
}

}  // namespace detail

/**
 * @brief Read MPC configuration from a YAML file into an MpcYamlConfig struct.
 *
 * Expected YAML structure:
 * @code{.yaml}
 * mpc:
 *   parameters:
 *     <param_name>: [...]   # optional
 *   constraints:
 *     lbu: [...]            # required
 *     ubu: [...]            # required
 *     lbx: [...]            # optional
 *     ubx: [...]            # optional
 *     lh:  [...]            # optional
 *     uh:  [...]            # optional
 *     Zl:  [...]            # optional
 *     ...
 * @endcode
 *
 * @param file_path Path to the YAML file.
 * @param config    Output configuration struct.
 * @throws std::invalid_argument if the file does not exist or a required field is missing.
 */
inline void readMpcYaml(const std::string& file_path, MpcYamlConfig& config) {
  std::ifstream file(file_path.c_str());
  if (!file.good()) {
    const std::string absolute_path = std::filesystem::absolute(file_path).string();
    std::cout << "File " << absolute_path << " does not exist." << std::endl;
    throw std::invalid_argument("File does not exist: " + absolute_path);
  }
  file.close();

  const YAML::Node root            = YAML::LoadFile(file_path);
  const YAML::Node mpc_cfg         = root["mpc"];
  const YAML::Node parameters_cfg  = mpc_cfg["parameters"];
  const YAML::Node constraints_cfg = mpc_cfg["constraints"];

  // Online parameters (optional, strict size validation when present)
  config.mass = detail::yamlVectorOptional<Parameters::mass_length>(parameters_cfg["mass"],
                                                                    "mpc.parameters.mass");
  config.desired_position = detail::yamlVectorOptional<Parameters::desired_position_length>(
      parameters_cfg["desired_position"], "mpc.parameters.desired_position");
  config.desired_orientation = detail::yamlVectorOptional<Parameters::desired_orientation_length>(
      parameters_cfg["desired_orientation"], "mpc.parameters.desired_orientation");
  config.desired_velocity = detail::yamlVectorOptional<Parameters::desired_velocity_length>(
      parameters_cfg["desired_velocity"], "mpc.parameters.desired_velocity");
  config.desired_acceleration = detail::yamlVectorOptional<Parameters::desired_acceleration_length>(
      parameters_cfg["desired_acceleration"], "mpc.parameters.desired_acceleration");
  config.external_force = detail::yamlVectorOptional<Parameters::external_force_length>(
      parameters_cfg["external_force"], "mpc.parameters.external_force");
  config.Q =
      detail::yamlVectorOptional<Parameters::Q_length>(parameters_cfg["Q"], "mpc.parameters.Q");
  config.Qe =
      detail::yamlVectorOptional<Parameters::Qe_length>(parameters_cfg["Qe"], "mpc.parameters.Qe");
  config.R =
      detail::yamlVectorOptional<Parameters::R_length>(parameters_cfg["R"], "mpc.parameters.R");

  // Actuation bounds — required
  config.lbu =
      detail::yamlArrayRequired<ActuationBounds::Nu>(constraints_cfg["lbu"], "mpc.constraints.lbu");
  config.ubu =
      detail::yamlArrayRequired<ActuationBounds::Nu>(constraints_cfg["ubu"], "mpc.constraints.ubu");

  // Optional bounds and slack weights
  config.lbx =
      detail::yamlVectorOptional<StateBounds::Nx>(constraints_cfg["lbx"], "mpc.constraints.lbx");
  config.ubx =
      detail::yamlVectorOptional<StateBounds::Nx>(constraints_cfg["ubx"], "mpc.constraints.ubx");
  config.lsbx = detail::yamlVectorOptional<SoftStateBounds::Nsbx>(constraints_cfg["lsbx"],
                                                                  "mpc.constraints.lsbx");
  config.usbx = detail::yamlVectorOptional<SoftStateBounds::Nsbx>(constraints_cfg["usbx"],
                                                                  "mpc.constraints.usbx");
  config.lh   = detail::yamlVectorOptional<NonlinearConstraintBounds::Nh>(constraints_cfg["lh"],
                                                                        "mpc.constraints.lh");
  config.uh   = detail::yamlVectorOptional<NonlinearConstraintBounds::Nh>(constraints_cfg["uh"],
                                                                        "mpc.constraints.uh");
  config.lsh  = detail::yamlVectorOptional<SoftNonlinearConstraintBounds::Nsh>(
      constraints_cfg["lsh"], "mpc.constraints.lsh");
  config.ush = detail::yamlVectorOptional<SoftNonlinearConstraintBounds::Nsh>(
      constraints_cfg["ush"], "mpc.constraints.ush");
  config.Zl =
      detail::yamlVectorOptional<SlackWeights::Ns>(constraints_cfg["Zl"], "mpc.constraints.Zl");
  config.Zu =
      detail::yamlVectorOptional<SlackWeights::Ns>(constraints_cfg["Zu"], "mpc.constraints.Zu");
  config.zl =
      detail::yamlVectorOptional<SlackWeights::Ns>(constraints_cfg["zl"], "mpc.constraints.zl");
  config.zu =
      detail::yamlVectorOptional<SlackWeights::Ns>(constraints_cfg["zu"], "mpc.constraints.zu");
  config.Zl_e = detail::yamlVectorOptional<SlackWeightsEnd::Ns_e>(constraints_cfg["Zl_e"],
                                                                  "mpc.constraints.Zl_e");
  config.Zu_e = detail::yamlVectorOptional<SlackWeightsEnd::Ns_e>(constraints_cfg["Zu_e"],
                                                                  "mpc.constraints.Zu_e");
  config.zl_e = detail::yamlVectorOptional<SlackWeightsEnd::Ns_e>(constraints_cfg["zl_e"],
                                                                  "mpc.constraints.zl_e");
  config.zu_e = detail::yamlVectorOptional<SlackWeightsEnd::Ns_e>(constraints_cfg["zu_e"],
                                                                  "mpc.constraints.zu_e");
}

/**
 * @brief Configure an MPC instance from a YAML file.
 *
 * @details
 * This helper is the entry point to apply runtime MPC tuning from `mpc_config.yaml`.
 * It performs all required steps in one call:
 * 1. Parse the YAML file with readMpcYaml().
 * 2. Apply online parameters to mpc.getParameters().
 * 3. Apply configured bounds and slack weights.
 * 4. Call all required update*() methods so values are pushed to the acados solver.
 *
 * YAML requirements:
 * - Required fields: `mpc.constraints.lbu`, `mpc.constraints.ubu`.
 * - Optional fields: all online parameters and remaining constraint/slack entries.
 *
 * Optional-field behavior:
 * - If omitted, the current value already stored inside `mpc` is preserved.
 * - If provided, vector sizes must match the generated model/solver dimensions.
 *
 * @note Dynamic references (e.g. desired_position, desired_orientation) are only
 *       initial values and are usually overwritten by the control loop at runtime.
 *
 * @param mpc       Initialized MPC instance to configure.
 * @param file_path Path to the YAML configuration file.
 *
 * @throws std::invalid_argument If the YAML file does not exist, a required key is
 *         missing, or any provided vector has an invalid size.
 */
inline void configureMpcFromYaml(MPC& mpc, const std::string& file_path) {
  MpcYamlConfig config;
  readMpcYaml(file_path, config);

  // Online parameters
  if (!config.mass.empty()) {
    const auto mass =
        detail::vectorToArray<Parameters::mass_length>(config.mass, "mpc.parameters.mass");
    mpc.getParameters()->setMass(mass[0]);
  }
  if (!config.desired_position.empty()) {
    mpc.getParameters()->setDesiredPosition(
        detail::vectorToArray<Parameters::desired_position_length>(
            config.desired_position, "mpc.parameters.desired_position"));
  }
  if (!config.desired_orientation.empty()) {
    mpc.getParameters()->setDesiredOrientation(
        detail::vectorToArray<Parameters::desired_orientation_length>(
            config.desired_orientation, "mpc.parameters.desired_orientation"));
  }
  if (!config.desired_velocity.empty()) {
    mpc.getParameters()->setDesiredVelocity(
        detail::vectorToArray<Parameters::desired_velocity_length>(
            config.desired_velocity, "mpc.parameters.desired_velocity"));
  }
  if (!config.desired_acceleration.empty()) {
    mpc.getParameters()->setDesiredAcceleration(
        detail::vectorToArray<Parameters::desired_acceleration_length>(
            config.desired_acceleration, "mpc.parameters.desired_acceleration"));
  }
  if (!config.external_force.empty()) {
    mpc.getParameters()->setExternalForce(detail::vectorToArray<Parameters::external_force_length>(
        config.external_force, "mpc.parameters.external_force"));
  }
  if (!config.Q.empty()) {
    mpc.getParameters()->setQ(
        detail::vectorToArray<Parameters::Q_length>(config.Q, "mpc.parameters.Q"));
  }
  if (!config.Qe.empty()) {
    mpc.getParameters()->setQe(
        detail::vectorToArray<Parameters::Qe_length>(config.Qe, "mpc.parameters.Qe"));
  }
  if (!config.R.empty()) {
    mpc.getParameters()->setR(
        detail::vectorToArray<Parameters::R_length>(config.R, "mpc.parameters.R"));
  }

  // Actuation bounds
  mpc.getActuationBounds()->setLbu(config.lbu);
  mpc.getActuationBounds()->setUbu(config.ubu);

  // State bounds
  if constexpr (StateBounds::Nx > 0) {
    if (!config.lbx.empty()) {
      mpc.getStateBounds()->setLbx(
          detail::vectorToArray<StateBounds::Nx>(config.lbx, "mpc.constraints.lbx"));
    }
    if (!config.ubx.empty()) {
      mpc.getStateBounds()->setUbx(
          detail::vectorToArray<StateBounds::Nx>(config.ubx, "mpc.constraints.ubx"));
    }
  }

  // Soft state bounds
  if constexpr (SoftStateBounds::Nsbx > 0) {
    if (!config.lsbx.empty()) {
      mpc.getSoftStateBounds()->setLsbx(
          detail::vectorToArray<SoftStateBounds::Nsbx>(config.lsbx, "mpc.constraints.lsbx"));
    }
    if (!config.usbx.empty()) {
      mpc.getSoftStateBounds()->setUsbx(
          detail::vectorToArray<SoftStateBounds::Nsbx>(config.usbx, "mpc.constraints.usbx"));
    }
  }

  // Slack weights
  if constexpr (SlackWeights::Ns > 0) {
    if (!config.Zl.empty()) {
      mpc.getSlackWeights()->setZl(
          detail::vectorToArray<SlackWeights::Ns>(config.Zl, "mpc.constraints.Zl"));
    }
    if (!config.Zu.empty()) {
      mpc.getSlackWeights()->setZu(
          detail::vectorToArray<SlackWeights::Ns>(config.Zu, "mpc.constraints.Zu"));
    }
    if (!config.zl.empty()) {
      mpc.getSlackWeights()->setzl(
          detail::vectorToArray<SlackWeights::Ns>(config.zl, "mpc.constraints.zl"));
    }
    if (!config.zu.empty()) {
      mpc.getSlackWeights()->setzu(
          detail::vectorToArray<SlackWeights::Ns>(config.zu, "mpc.constraints.zu"));
    }
    if (!config.Zl_e.empty()) {
      mpc.getSlackWeightsEnd()->setZlE(
          detail::vectorToArray<SlackWeightsEnd::Ns_e>(config.Zl_e, "mpc.constraints.Zl_e"));
    }
    if (!config.Zu_e.empty()) {
      mpc.getSlackWeightsEnd()->setZuE(
          detail::vectorToArray<SlackWeightsEnd::Ns_e>(config.Zu_e, "mpc.constraints.Zu_e"));
    }
    if (!config.zl_e.empty()) {
      mpc.getSlackWeightsEnd()->setzlE(
          detail::vectorToArray<SlackWeightsEnd::Ns_e>(config.zl_e, "mpc.constraints.zl_e"));
    }
    if (!config.zu_e.empty()) {
      mpc.getSlackWeightsEnd()->setzuE(
          detail::vectorToArray<SlackWeightsEnd::Ns_e>(config.zu_e, "mpc.constraints.zu_e"));
    }
  }

  // Nonlinear constraint bounds
  if constexpr (NonlinearConstraintBounds::Nh > 0) {
    if (!config.lh.empty()) {
      mpc.getNonlinearConstraintBounds()->setLh(
          detail::vectorToArray<NonlinearConstraintBounds::Nh>(config.lh, "mpc.constraints.lh"));
    }
    if (!config.uh.empty()) {
      mpc.getNonlinearConstraintBounds()->setUh(
          detail::vectorToArray<NonlinearConstraintBounds::Nh>(config.uh, "mpc.constraints.uh"));
    }
  }

  // Soft nonlinear constraint bounds
  if constexpr (SoftNonlinearConstraintBounds::Nsh > 0) {
    if (!config.lsh.empty()) {
      mpc.getSoftNonlinearConstraintBounds()->setLsh(
          detail::vectorToArray<SoftNonlinearConstraintBounds::Nsh>(config.lsh,
                                                                    "mpc.constraints.lsh"));
    }
    if (!config.ush.empty()) {
      mpc.getSoftNonlinearConstraintBounds()->setUsh(
          detail::vectorToArray<SoftNonlinearConstraintBounds::Nsh>(config.ush,
                                                                    "mpc.constraints.ush"));
    }
  }

  // Push bound changes to the solver
  mpc.updateActuationBounds();
  if constexpr (StateBounds::Nx > 0) {
    if (!config.lbx.empty() || !config.ubx.empty()) {
      mpc.updateStateBounds();
    }
  }
  if constexpr (SoftStateBounds::Nsbx > 0) {
    if (!config.lsbx.empty() || !config.usbx.empty()) {
      mpc.updateSoftStateBounds();
    }
  }
  if constexpr (SlackWeights::Ns > 0) {
    if (!config.Zl.empty() || !config.Zu.empty() || !config.zl.empty() || !config.zu.empty()) {
      mpc.updateSlackWeights();
    }
    if (!config.Zl_e.empty() || !config.Zu_e.empty() || !config.zl_e.empty() ||
        !config.zu_e.empty()) {
      mpc.updateSlackWeightsEnd();
    }
  }
  if constexpr (NonlinearConstraintBounds::Nh > 0) {
    if (!config.lh.empty() || !config.uh.empty()) {
      mpc.updateNonlinearConstraintBounds();
    }
  }
  if constexpr (SoftNonlinearConstraintBounds::Nsh > 0) {
    if (!config.lsh.empty() || !config.ush.empty()) {
      mpc.updateSoftNonlinearConstraintBounds();
    }
  }
}

}  // namespace acados_mpc

#endif  // ACADOS_MPC_ACADOS_MPC_YAML_HPP_
