#!/bin/bash
# Regenerate the controller library template from model_definition.yaml.
#
# Produces the Python datatypes under mpc_acados_trajectory/model_definition/
# and the C++ template under mpc_acados_trajectory/cpp_interface/ (headers,
# sources, tests and its own CMakeLists.txt). Run this whenever
# model_definition.yaml changes.
#
# Requires mpc_acados_core to be installed or reachable via PYTHONPATH.
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config "${SCRIPT_DIR}/model_definition.yaml"
