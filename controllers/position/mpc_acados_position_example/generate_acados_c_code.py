#!/usr/bin/env python3
# Copyright 2025 Universidad Politécnica de Madrid
# Licensed under the BSD-3-Clause license.

"""Entry point for regenerating acados C code for the position controller.

Run from this directory so that the configured ``export_dir`` in the solver
definition yaml resolves to ``../mpc_acados_position/acados_generated/mpc_generated_code``:

    python3 generate_acados_c_code.py

or from anywhere by passing the yaml path:

    python3 generate_acados_c_code.py --yaml /path/to/solver_definition_mpc_position.yaml
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from mpc_acados_position import AcadosMPCSolver


_EXAMPLE_DIR = Path(__file__).resolve().parent
CONTROLLER_ROOT = _EXAMPLE_DIR.parent
DEFAULT_YAML = _EXAMPLE_DIR / 'examples' / 'solver_definition_mpc_position.yaml'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '-c', '--yaml',
        type=Path,
        default=DEFAULT_YAML,
        help='Path to the solver definition yaml (default: %(default)s)',
    )
    parser.add_argument(
        '--workdir',
        type=Path,
        default=CONTROLLER_ROOT,
        help='Working directory used to resolve the yaml export_dir '
             '(default: the controller root).',
    )
    args = parser.parse_args()

    os.chdir(args.workdir)
    AcadosMPCSolver(
        solver_definition_path=str(args.yaml.resolve()),
        generate_acados_solver=True,
        generate_acados_simulator=True,
        generate_code=True,
    )
    print('Acados C code generated under:',
          CONTROLLER_ROOT / 'mpc_acados_position' / 'acados_generated' / 'mpc_generated_code')
    return 0


if __name__ == '__main__':
    sys.exit(main())
