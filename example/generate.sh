#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

python3 -m mpc.acados_solver -c solver_definition_mpc.yaml

# Compile the generated code
rm -rf build
mkdir build && cd build
cmake ..
make