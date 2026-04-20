#!/bin/bash
# Run the Python closed-loop example for mpc_acados_trajectory and plot the log.
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

rm -f simulator_logs/mpc_log.csv
python3 examples/run_example.py \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
