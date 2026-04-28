#!/bin/bash
# Generate mpc_interface/ next to this script by delegating to the
# mpc_acados_position Python CLI. Works from any CWD.
#
# Requirements: ``mpc_acados_core`` and ``mpc_acados_position`` must be
# importable by the active Python interpreter (PYTHONPATH or editable
# install).
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

python3 -m mpc_acados_position.acados_solver \
  -c "${SCRIPT_DIR}/solver_definition_mpc_position.yaml" \
  -o "${SCRIPT_DIR}"
