#!/bin/bash
# Run the Python closed-loop example for mpc_acados_trajectory and plot the log.
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Expose the dynamic_trajectory_generator package fetched + built by
# `bash build.sh` (CMakeLists.txt::FetchContent). The wrapper python
# package lives at build/python/dynamic_trajectory_generator_py (symlink
# to the upstream source) and the compiled .so lives next to it in the
# dtg_pybind build subdir. Skip if the caller already supplies the
# package on PYTHONPATH (e.g. an editable install).
if ! python3 -c 'import dynamic_trajectory_generator_py' 2>/dev/null; then
  BUILD_PY_DIR="${SCRIPT_DIR}/build/python"
  BUILD_DTG_DIR="${SCRIPT_DIR}/build/dtg_pybind"
  if [ -d "${BUILD_PY_DIR}" ] && [ -d "${BUILD_DTG_DIR}" ]; then
    export PYTHONPATH="${BUILD_PY_DIR}:${BUILD_DTG_DIR}:${PYTHONPATH:-}"
  else
    echo "[run_py_example] WARN: dynamic_trajectory_generator_py not importable"
    echo "                  and build/python or build/dtg_pybind missing."
    echo "                  Did you run 'bash build.sh'?"
  fi
fi

rm -f simulator_logs/mpc_log.csv
python3 examples/run_example.py \
  -c examples/simulation_config.yaml \
  -m examples/mpc_config.yaml \
  -f mpc_log.csv
python3 -m mpc_acados_core.plotting.plot_results -f simulator_logs/mpc_log.csv
