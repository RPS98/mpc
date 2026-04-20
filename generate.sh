#!/bin/bash
# Regenerate Python modules and C++ headers/sources for one or all controllers.
# Usage: ./generate.sh {position|trajectory|all}
set -euo pipefail
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

target="${1:-all}"

generate_one() {
  local ctrl="$1"
  echo "==> Regenerating controller: ${ctrl}"
  python3 -m mpc_acados_core.generate.model_definition_generation --controller "${ctrl}"
}

case "${target}" in
  position|trajectory)
    generate_one "${target}"
    ;;
  all)
    generate_one position
    generate_one trajectory
    ;;
  *)
    echo "Unknown target: ${target}. Expected {position|trajectory|all}." >&2
    exit 1
    ;;
esac
