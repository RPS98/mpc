#!/bin/bash
# Regenerate Python and C++ datatypes from model_definition.yaml.
# Requires mpc_acados_core to be installed or on PYTHONPATH.
#
# Usage:
#   ./generate.sh
#   PYTHONPATH=/path/to/mpc_acados ./generate.sh
set -euo pipefail
CTRL_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
python3 -m mpc_acados_core.generate.model_definition_generation \
  --config "${CTRL_DIR}/generate_model_definition/model_definition.yaml"
