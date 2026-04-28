# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Shared mpc_interface/ generation logic used by per-controller CLIs.

A controller package (e.g. ``mpc_acados_position``) wraps these helpers in a
tiny ``acados_solver.py`` module so the end user can run, from any directory::

    python3 -m mpc_acados_position.acados_solver -c <path>/solver.yaml

The helper:

1. Reads the YAML, picks the target directory name from ``solver.export_dir``
   (e.g. ``mpc_interface/`` -> ``mpc_interface``, ``mpcc_interface/`` ->
   ``mpcc_interface``), and copies the controller's ``cpp_interface/`` template
   next to the YAML (or to an explicit ``--output-dir``) under that name.
2. Changes the current working directory to the output root — acados
   interprets ``solver.export_dir`` relative to the CWD during code
   generation, so the C code lands under
   ``<output_dir>/<export_dir>/mpc_generated_code/mpc_generated_code/``.
3. Instantiates the bound ``AcadosMPCSolver`` subclass provided by the
   controller, which triggers the acados C code / shared library generation.
4. Restores the original CWD.

Side effects: writes/overwrites ``<output_dir>/<export_dir>/`` and creates
acados intermediate files below it.
"""

from __future__ import annotations

import argparse
import os
import shutil
from pathlib import Path
from typing import Optional, Type

from mpc_acados_core.utils.solver_config import SolverDefinition

__all__ = ['generate_mpc_interface', 'run_cli']


_DEFAULT_INTERFACE_DIR = 'mpc_interface'


def _resolve_interface_dir_name(yaml_path: Path) -> str:
    """Return the directory name to use for the generated mpc_interface tree.

    Derived from ``solver.export_dir`` in the YAML: a value such as
    ``'mpcc_interface/'`` becomes the directory name ``'mpcc_interface'``.
    Falls back to ``'mpc_interface'`` when the field is missing or empty so
    pre-existing variants without this configuration keep their behaviour.
    """
    try:
        solver_definition = SolverDefinition.from_yaml(str(yaml_path))
    except Exception:
        return _DEFAULT_INTERFACE_DIR
    export_dir = (solver_definition.solver.export_dir or '').strip()
    if not export_dir:
        return _DEFAULT_INTERFACE_DIR
    # Trim trailing slashes and any leading path noise; use only the basename
    # so the directory is created next to the YAML, not nested arbitrarily.
    name = Path(export_dir.rstrip('/').rstrip('\\')).name
    return name or _DEFAULT_INTERFACE_DIR


def generate_mpc_interface(
    *,
    solver_cls: Type,
    cpp_template_dir: Path,
    solver_definition_path: Path,
    output_dir: Optional[Path] = None,
) -> Path:
    """Generate ``mpc_interface/`` ready for ``add_subdirectory``.

    :param solver_cls: Concrete ``AcadosMPCSolverBase`` subclass with the
        controller-specific types already bound (e.g.
        ``mpc_acados_position.AcadosMPCSolver``).
    :param cpp_template_dir: Path to the controller's ``cpp_interface/``
        template directory. Typically
        ``Path(<controller_pkg>.__file__).parent / 'cpp_interface'``.
    :param solver_definition_path: Path to the solver definition YAML.
        May be relative; it is resolved to an absolute path here.
    :param output_dir: Directory where ``mpc_interface/`` will be created.
        Defaults to the parent directory of ``solver_definition_path``.
    :return: Absolute path to the generated ``mpc_interface/`` directory.
    :raises FileNotFoundError: If the YAML or the template directory is
        missing.
    """
    yaml_path = Path(solver_definition_path).expanduser().resolve(strict=True)
    out_dir = (
        Path(output_dir).expanduser().resolve()
        if output_dir is not None
        else yaml_path.parent
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    template = Path(cpp_template_dir)
    if not template.is_dir():
        raise FileNotFoundError(
            f'cpp_interface template not found at {template}. '
            f'Has the controller package been installed correctly?'
        )

    target = out_dir / _resolve_interface_dir_name(yaml_path)
    if target.exists():
        shutil.rmtree(target)
    shutil.copytree(template, target, symlinks=True)

    old_cwd = os.getcwd()
    try:
        os.chdir(out_dir)
        solver_cls(
            solver_definition_path=str(yaml_path),
            generate_acados_solver=True,
            generate_acados_simulator=True,
            generate_code=True,
        )
    finally:
        os.chdir(old_cwd)

    return target


def run_cli(
    *,
    solver_cls: Type,
    cpp_template_dir: Path,
    prog: str,
) -> None:
    """Argparse-based CLI wrapper around :func:`generate_mpc_interface`.

    Intended to be called from each controller's ``acados_solver.py``::

        from mpc_acados_core.generate.mpc_interface_generation import run_cli
        from mpc_acados_position import AcadosMPCSolver

        def main() -> None:
            run_cli(
                solver_cls=AcadosMPCSolver,
                cpp_template_dir=Path(__file__).parent / 'cpp_interface',
                prog='python -m mpc_acados_position.acados_solver',
            )

    :param solver_cls: See :func:`generate_mpc_interface`.
    :param cpp_template_dir: See :func:`generate_mpc_interface`.
    :param prog: Program name shown in ``--help``.
    """
    parser = argparse.ArgumentParser(
        prog=prog,
        description='Generate mpc_interface/ ready for add_subdirectory.',
    )
    parser.add_argument(
        '-c', '--config',
        required=True,
        help='Path to the solver definition YAML.',
    )
    parser.add_argument(
        '-o', '--output-dir',
        default=None,
        help=(
            "Where mpc_interface/ is written. "
            "Defaults to the YAML's parent directory."
        ),
    )
    args = parser.parse_args()

    target = generate_mpc_interface(
        solver_cls=solver_cls,
        cpp_template_dir=cpp_template_dir,
        solver_definition_path=Path(args.config),
        output_dir=Path(args.output_dir) if args.output_dir else None,
    )
    print(f'mpc_interface/ ready at: {target}')
