#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

rm mpc_log.csv > /dev/null 2>&1
./build/examples/acados_mpc_run_example examples/simulation_config.yaml mpc_log.csv
python3 examples/utils/plot_results.py -f mpc_log.csv
