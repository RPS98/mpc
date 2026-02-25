#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

rm mpc_log.csv > /dev/null 2>&1
./build/examples/mpc_position_example_run_example 
python3 examples/utils/plot_results.py -f mpc_log.csv