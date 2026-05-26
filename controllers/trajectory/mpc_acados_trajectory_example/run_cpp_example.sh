#!/bin/bash
# Run the C++ closed-loop example for mpc_acados_trajectory and plot the log.
# Requires the project to have been built (cmake -S . -B build && cmake --build build -j).
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

LOCAL_LIB_DIRS=(
  "${SCRIPT_DIR}/mpc_interface/mpc_generated_code/mpc_generated_code"
  "${SCRIPT_DIR}/build/mpc_interface"
  "${SCRIPT_DIR}/build/_deps/dynamic_trajectory_generator-build"
  "${SCRIPT_DIR}/build/_deps/dynamic_trajectory_generator-build/subpackages/mav_trajectory_generation"
  "${SCRIPT_DIR}/build/_deps/nlopt-build"
)
LOCAL_LD=""
for d in "${LOCAL_LIB_DIRS[@]}"; do
  [ -d "$d" ] && LOCAL_LD="${LOCAL_LD:+${LOCAL_LD}:}${d}"
done
export LD_LIBRARY_PATH="${LOCAL_LD}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

rm -f simulator_logs/mpc_log.csv
./build/examples/mpc_acados_trajectory_run_example \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
