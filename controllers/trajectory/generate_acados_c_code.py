#!/usr/bin/env python3
# Copyright 2025 Universidad Politécnica de Madrid
# Licensed under the BSD-3-Clause license.

"""Entry point for regenerating acados C code for the trajectory controller.

Run from the controller root so that the configured ``export_dir`` in the
solver definition yaml resolves to ``./acados_trajectory_mpc/mpc_generated_code``.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from mpc_acados_trajectory import AcadosMPCSolver


CONTROLLER_ROOT = Path(__file__).resolve().parent
DEFAULT_YAML = CONTROLLER_ROOT / 'examples' / 'solver_definition_mpc_trajectory.yaml'


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
          args.workdir / 'acados_trajectory_mpc' / 'mpc_generated_code')
    return 0


if __name__ == '__main__':
    sys.exit(main())
