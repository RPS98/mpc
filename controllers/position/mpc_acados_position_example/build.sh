#!/bin/bash
# Clean build + (re)generate mpc_interface/ if missing + compile the example.
#
# Usage:
#   ./build.sh              # incremental build (keeps previous build/)
#   ./build.sh --clean      # wipe build/ first
#   ./build.sh --regen      # also wipe mpc_interface/ before (re)generating
#
# Generation of mpc_interface/ is only triggered when it is missing or when
# --regen is passed; otherwise the existing acados C code is reused.
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

CLEAN=0
REGEN=0
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --regen) REGEN=1 ;;
    -h|--help)
      sed -n '2,9p' "$0"
      exit 0
      ;;
    *)
      echo "error: unknown argument '$arg'" >&2
      exit 2
      ;;
  esac
done

if [ "${REGEN}" -eq 1 ]; then
  rm -rf mpc_interface
fi

SOLVER_LIB="mpc_interface/mpc_generated_code/mpc_generated_code/libacados_ocp_solver_mpc.so"
if [ ! -f "${SOLVER_LIB}" ]; then
  echo "[build] mpc_interface/ missing, running generate_mpc_interface.sh..."
  ./generate_mpc_interface.sh
fi

if [ "${CLEAN}" -eq 1 ]; then
  rm -rf build
fi

cmake -S . -B build
cmake --build build -j

echo "[build] done. Run ./run_cpp_example.sh or ./run_py_example.sh"
