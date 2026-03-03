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
 * @file yaml_utils.hpp
 *
 * YAML utilities shared across MPC examples.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef EXAMPLE_YAML_UTILS_HPP_
#define EXAMPLE_YAML_UTILS_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "acados_mpc/acados_mpc.hpp"

namespace acados_mpc {
namespace acados_mpc_examples {

struct YamlMPCData {
  std::array<double, acados_mpc::Gains::Nq> Q{};
  std::array<double, acados_mpc::Gains::Nqe> Qe{};
  std::array<double, acados_mpc::Gains::Nr> R{};
  std::array<double, acados_mpc::ActuationBounds::Nu> lbu{};
  std::array<double, acados_mpc::ActuationBounds::Nu> ubu{};
  std::array<double, acados_mpc::StateBounds::Nx> lbx{};
  std::array<double, acados_mpc::StateBounds::Nx> ubx{};
  std::array<double, acados_mpc::SoftStateBounds::Nsbx> lsbx{};
  std::array<double, acados_mpc::SoftStateBounds::Nsbx> usbx{};
  std::array<double, acados_mpc::SlackWeights::Nsbx> Zl{};
  std::array<double, acados_mpc::SlackWeights::Nsbx> Zu{};
  std::array<double, acados_mpc::SlackWeights::Nsbx> zl{};
  std::array<double, acados_mpc::SlackWeights::Nsbx> zu{};
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> Zl_e{};
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> Zu_e{};
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> zl_e{};
  std::array<double, acados_mpc::SlackWeightsEnd::Nsbx_e> zu_e{};
  std::array<double, acados_mpc::Parameters::mass_length> mass{{1.0}};
  std::array<double, acados_mpc::Parameters::desired_orientation_length> desired_orientation{
      {1.0, 0.0, 0.0, 0.0}};
  std::array<double, acados_mpc::Parameters::external_force_length> external_force{};
  bool has_actuation_bounds       = false;
  bool has_state_bounds           = false;
  bool has_soft_state_bounds      = false;
  bool has_slack_weights          = false;
  bool has_terminal_slack_weights = false;
};

struct YamlData {
  double max_speed = 0.0;
  std::vector<Eigen::Vector3d> waypoints;
  bool path_facing = false;
  std::string solver_definition_path;
  std::string ocp_json_file_path;
  YamlMPCData mpc_data;
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
bool yamlArrayIfPresent(const YAML::Node& node,
                        std::array<double, N>& output,
                        const std::string& name) {
  if (!node || node.IsNull()) {
    return false;
  }
  output = vectorToArray<N>(node.as<std::vector<double>>(), name);
  return true;
}

template <std::size_t N>
std::array<double, N> yamlArrayOrDefault(const YAML::Node& node,
                                         const std::array<double, N>& default_values,
                                         const std::string& name) {
  if (!node || node.IsNull()) {
    return default_values;
  }
  return vectorToArray<N>(node.as<std::vector<double>>(), name);
}

template <std::size_t N>
std::array<double, N> yamlArrayRequired(const YAML::Node& node, const std::string& name) {
  if (!node || node.IsNull()) {
    throw std::invalid_argument("Missing required YAML entry: " + name);
  }
  return vectorToArray<N>(node.as<std::vector<double>>(), name);
}

inline Eigen::Vector3d waypointFromNode(const YAML::Node& node, const std::string& name) {
  const std::vector<double> values = node.as<std::vector<double>>();
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must have 3 elements.");
  }
  return {values[0], values[1], values[2]};
}

}  // namespace detail

inline void readYamlParams(const std::string& file_path, YamlData& data) {
  std::ifstream file(file_path.c_str());
  if (!file.good()) {
    const std::string absolute_path = std::filesystem::absolute(file_path).string();
    std::cout << "File " << absolute_path << " does not exist." << std::endl;
    throw std::invalid_argument("File does not exist");
  }
  file.close();

  const YAML::Node config          = YAML::LoadFile(file_path);
  const YAML::Node sim_config      = config["sim_config"];
  const YAML::Node controller_cfg  = config["controller"];
  const YAML::Node mpc_cfg         = controller_cfg["mpc"];
  const YAML::Node cost_cfg        = mpc_cfg["cost"] ? mpc_cfg["cost"] : mpc_cfg;
  const YAML::Node constraints_cfg = mpc_cfg["constraints"] ? mpc_cfg["constraints"] : mpc_cfg;
  const YAML::Node parameters_cfg  = mpc_cfg["parameters"] ? mpc_cfg["parameters"] : mpc_cfg;

  const YAML::Node max_speed_node = sim_config["max_speed"]
                                        ? sim_config["max_speed"]
                                        : sim_config["trajectory_generator_max_speed"];
  const YAML::Node waypoints_node = sim_config["waypoints"]
                                        ? sim_config["waypoints"]
                                        : sim_config["trajectory_generator_waypoints"];

  if (!max_speed_node || !waypoints_node) {
    throw std::invalid_argument("Missing sim_config.max_speed/waypoints entries.");
  }

  data.max_speed              = max_speed_node.as<double>();
  data.path_facing            = sim_config["path_facing"].as<bool>();
  data.solver_definition_path = controller_cfg["solver_definition_path"].as<std::string>();
  data.ocp_json_file_path     = controller_cfg["ocp_json_file_path"].as<std::string>();

  data.waypoints.clear();
  for (std::size_t index = 0; index < waypoints_node.size(); ++index) {
    data.waypoints.push_back(detail::waypointFromNode(
        waypoints_node[index], "sim_config.waypoints[" + std::to_string(index) + "]"));
  }

  data.mpc_data.Q =
      detail::yamlArrayRequired<acados_mpc::Gains::Nq>(cost_cfg["Q"], "controller.mpc.cost.Q");
  data.mpc_data.Qe =
      detail::yamlArrayRequired<acados_mpc::Gains::Nqe>(cost_cfg["Qe"], "controller.mpc.cost.Qe");
  data.mpc_data.R =
      detail::yamlArrayRequired<acados_mpc::Gains::Nr>(cost_cfg["R"], "controller.mpc.cost.R");

  data.mpc_data.has_actuation_bounds = detail::yamlArrayIfPresent<acados_mpc::ActuationBounds::Nu>(
      constraints_cfg["lbu"], data.mpc_data.lbu, "controller.mpc.constraints.lbu");
  if (data.mpc_data.has_actuation_bounds) {
    data.mpc_data.ubu = detail::yamlArrayRequired<acados_mpc::ActuationBounds::Nu>(
        constraints_cfg["ubu"], "controller.mpc.constraints.ubu");
  }

  data.mpc_data.has_state_bounds = detail::yamlArrayIfPresent<acados_mpc::StateBounds::Nx>(
      constraints_cfg["lbx"], data.mpc_data.lbx, "controller.mpc.constraints.lbx");
  if (data.mpc_data.has_state_bounds) {
    data.mpc_data.ubx = detail::yamlArrayRequired<acados_mpc::StateBounds::Nx>(
        constraints_cfg["ubx"], "controller.mpc.constraints.ubx");
  }

  data.mpc_data.has_soft_state_bounds =
      detail::yamlArrayIfPresent<acados_mpc::SoftStateBounds::Nsbx>(
          constraints_cfg["lsbx"], data.mpc_data.lsbx, "controller.mpc.constraints.lsbx");
  if (data.mpc_data.has_soft_state_bounds) {
    data.mpc_data.usbx = detail::yamlArrayRequired<acados_mpc::SoftStateBounds::Nsbx>(
        constraints_cfg["usbx"], "controller.mpc.constraints.usbx");
  }

  data.mpc_data.has_slack_weights = detail::yamlArrayIfPresent<acados_mpc::SlackWeights::Nsbx>(
      constraints_cfg["Zl"], data.mpc_data.Zl, "controller.mpc.constraints.Zl");
  if (data.mpc_data.has_slack_weights) {
    data.mpc_data.Zu = detail::yamlArrayRequired<acados_mpc::SlackWeights::Nsbx>(
        constraints_cfg["Zu"], "controller.mpc.constraints.Zu");
    data.mpc_data.zl = detail::yamlArrayRequired<acados_mpc::SlackWeights::Nsbx>(
        constraints_cfg["zl"], "controller.mpc.constraints.zl");
    data.mpc_data.zu = detail::yamlArrayRequired<acados_mpc::SlackWeights::Nsbx>(
        constraints_cfg["zu"], "controller.mpc.constraints.zu");
  }

  data.mpc_data.has_terminal_slack_weights =
      detail::yamlArrayIfPresent<acados_mpc::SlackWeightsEnd::Nsbx_e>(
          constraints_cfg["Zl_e"], data.mpc_data.Zl_e, "controller.mpc.constraints.Zl_e");
  if (data.mpc_data.has_terminal_slack_weights) {
    data.mpc_data.Zu_e = detail::yamlArrayRequired<acados_mpc::SlackWeightsEnd::Nsbx_e>(
        constraints_cfg["Zu_e"], "controller.mpc.constraints.Zu_e");
    data.mpc_data.zl_e = detail::yamlArrayRequired<acados_mpc::SlackWeightsEnd::Nsbx_e>(
        constraints_cfg["zl_e"], "controller.mpc.constraints.zl_e");
    data.mpc_data.zu_e = detail::yamlArrayRequired<acados_mpc::SlackWeightsEnd::Nsbx_e>(
        constraints_cfg["zu_e"], "controller.mpc.constraints.zu_e");
  }

  data.mpc_data.mass = detail::yamlArrayOrDefault<acados_mpc::Parameters::mass_length>(
      parameters_cfg["mass"], data.mpc_data.mass, "controller.mpc.parameters.mass");
  data.mpc_data.desired_orientation =
      detail::yamlArrayOrDefault<acados_mpc::Parameters::desired_orientation_length>(
          parameters_cfg["desired_orientation"], data.mpc_data.desired_orientation,
          "controller.mpc.parameters.desired_orientation");
  data.mpc_data.external_force =
      detail::yamlArrayOrDefault<acados_mpc::Parameters::external_force_length>(
          parameters_cfg["external_force"], data.mpc_data.external_force,
          "controller.mpc.parameters.external_force");
}

}  // namespace acados_mpc_examples
}  // namespace acados_mpc

#endif  // EXAMPLE_YAML_UTILS_HPP_
