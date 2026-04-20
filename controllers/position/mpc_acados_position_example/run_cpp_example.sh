#!/bin/bash
# Run the C++ closed-loop example for mpc_acados_position and plot the log.
# Requires the project to have been built (cmake -S . -B build && cmake --build build -j).
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

rm -f simulator_logs/mpc_log.csv
./build/examples/mpc_acados_position_run_example \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
