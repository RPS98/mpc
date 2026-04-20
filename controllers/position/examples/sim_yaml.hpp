// Copyright 2025 Universidad Politécnica de Madrid
// Licensed under the BSD-3-Clause license.

#ifndef MPC_ACADOS_POSITION_EXAMPLES_SIM_YAML_HPP_
#define MPC_ACADOS_POSITION_EXAMPLES_SIM_YAML_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace mpc_acados_position_examples {

struct YamlSimConfig {
  double sim_time = 0.0;
  double max_speed = 0.0;
  std::vector<Eigen::Vector3d> waypoints;
  bool path_facing = false;
  std::string solver_definition_path;
  std::string ocp_json_file_path;
};

namespace detail {

inline Eigen::Vector3d waypointFromNode(const YAML::Node& node,
                                        const std::string& name) {
  const std::vector<double> values = node.as<std::vector<double>>();
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must have 3 elements.");
  }
  return {values[0], values[1], values[2]};
}

}  // namespace detail

inline void readSimYaml(const std::string& file_path, YamlSimConfig& config) {
  std::ifstream file(file_path.c_str());
  if (!file.good()) {
    const std::string absolute_path = std::filesystem::absolute(file_path).string();
    std::cout << "File " << absolute_path << " does not exist." << std::endl;
    throw std::invalid_argument("File does not exist");
  }
  file.close();

  const YAML::Node root = YAML::LoadFile(file_path);
  const YAML::Node sim_cfg = root["sim_config"];
  const YAML::Node controller_cfg = root["controller"];

  config.sim_time = sim_cfg["sim_time"].as<double>();
  config.max_speed = sim_cfg["max_speed"].as<double>();
  config.path_facing = sim_cfg["path_facing"].as<bool>();
  if (controller_cfg) {
    if (controller_cfg["solver_definition_path"]) {
      config.solver_definition_path =
          controller_cfg["solver_definition_path"].as<std::string>();
    }
    if (controller_cfg["ocp_json_file_path"]) {
      config.ocp_json_file_path =
          controller_cfg["ocp_json_file_path"].as<std::string>();
    }
  }

  config.waypoints.clear();
  for (std::size_t index = 0; index < sim_cfg["waypoints"].size(); ++index) {
    config.waypoints.push_back(detail::waypointFromNode(
        sim_cfg["waypoints"][index],
        "sim_config.waypoints[" + std::to_string(index) + "]"));
  }
}

}  // namespace mpc_acados_position_examples

#endif  // MPC_ACADOS_POSITION_EXAMPLES_SIM_YAML_HPP_
