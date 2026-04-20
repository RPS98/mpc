#!/bin/bash
# Generate mpc_interface/: a consumable C++ subproject.
#
# The content of mpc_interface/ is produced by:
#   1. Copying the library template at ../mpc_acados_trajectory/cpp_interface/.
#   2. Exporting the acados C code under mpc_interface/mpc_generated_code/
#      (the path is set in solver_definition_mpc_trajectory.yaml::solver.export_dir).
#
# External repos that want to consume the controller can copy this example
# directory, adjust LIB_CPP_DIR to point at their local mpc_acados_trajectory
# installation, and run this script to regenerate mpc_interface/.
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

LIB_CPP_DIR="${SCRIPT_DIR}/../mpc_acados_trajectory/cpp_interface"
if [ ! -d "${LIB_CPP_DIR}" ]; then
  echo "error: library template not found at ${LIB_CPP_DIR}" >&2
  echo "       run controllers/trajectory/generate_datatypes.sh first." >&2
  exit 1
fi

rm -rf mpc_interface
cp -R "${LIB_CPP_DIR}" mpc_interface

python3 - <<'PY'
from pathlib import Path

from mpc_acados_trajectory import AcadosMPCSolver

AcadosMPCSolver(
    solver_definition_path=str(Path('solver_definition_mpc_trajectory.yaml').resolve()),
    generate_acados_solver=True,
    generate_acados_simulator=True,
    generate_code=True,
)
PY

echo "mpc_interface/ ready at: ${SCRIPT_DIR}/mpc_interface"
