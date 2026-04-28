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

"""CLI entry point to generate the trajectory-tracking ``mpc_interface/``.

Usage (from any directory, with ``mpc_acados_core`` and
``mpc_acados_trajectory`` on ``PYTHONPATH``)::

    python3 -m mpc_acados_trajectory.acados_solver \\
        -c /path/to/solver_definition_mpc_trajectory.yaml \\
        [-o /path/to/output_dir]

Without ``-o``, output is written next to the YAML file.
"""

from pathlib import Path

from mpc_acados_core.generate.mpc_interface_generation import run_cli

from mpc_acados_trajectory import AcadosMPCSolver


def main() -> None:
    run_cli(
        solver_cls=AcadosMPCSolver,
        cpp_template_dir=Path(__file__).parent / 'cpp_interface',
        prog='python -m mpc_acados_trajectory.acados_solver',
    )


if __name__ == '__main__':
    main()
