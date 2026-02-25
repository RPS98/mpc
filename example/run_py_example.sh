#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

rm mpc_log.csv > /dev/null 2>&1
python3 examples/run_example.py -c examples/simulation_config.yaml -f mpc_log.csv
python3 examples/utils/plot_results.py -f mpc_log.csv