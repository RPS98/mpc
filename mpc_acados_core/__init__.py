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


"""Shared core for acados-based MPC controllers.

Exposes the generic base classes and datatypes consumed by controller
variants (``mpc_acados_position``, ``mpc_acados_trajectory``, ...). The core
itself is controller-agnostic and contains no generated artifacts.
"""

__version__ = '0.1.0'

from mpc_acados_core.acados_solver import AcadosMPCSolverBase
from mpc_acados_core.drone_model import DroneModelBase
from mpc_acados_core.mpc_controller import MPCBase, MPCDataBase
from mpc_acados_core.mpc_datatype import (
    ActuationBounds,
    Gains,
    NonlinearConstraintBounds,
    Reference,
    ReferenceEnd,
    SlackWeights,
    SlackWeightsEnd,
    SoftNonlinearConstraintBounds,
    SoftStateBounds,
    StateBounds,
)

__all__ = [
    'AcadosMPCSolverBase',
    'ActuationBounds',
    'DroneModelBase',
    'Gains',
    'MPCBase',
    'MPCDataBase',
    'NonlinearConstraintBounds',
    'Reference',
    'ReferenceEnd',
    'SlackWeights',
    'SlackWeightsEnd',
    'SoftNonlinearConstraintBounds',
    'SoftStateBounds',
    'StateBounds',
]
