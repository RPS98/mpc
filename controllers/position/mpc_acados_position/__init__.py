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


"""Position-tracking MPC controller built on mpc_acados_core.

Bundles the generated datatypes (state, actuation, parameters, dynamics),
the hand-written drone model, and the concrete ``MPC``/``AcadosMPCSolver``
subclasses that inject those variant-specific types into the core base
classes.
"""

__version__ = '0.1.0'

from mpc_acados_core import (
    AcadosMPCSolverBase,
    ActuationBounds,
    Gains,
    MPCBase,
    MPCDataBase,
    NonlinearConstraintBounds,
    Reference,
    ReferenceEnd,
    SlackWeights,
    SlackWeightsEnd,
    SoftNonlinearConstraintBounds,
    SoftStateBounds,
    StateBounds,
)

from mpc_acados_position.model_definition import (
    Actuation,
    CaActuation,
    CaDynamics,
    CaParameters,
    CaState,
    Dynamics,
    OnlineParameters,
    Parameters,
    State,
)
from mpc_acados_position.drone_model import DroneModel, get_acados_model


class AcadosMPCSolver(AcadosMPCSolverBase):
    """Acados MPC solver bound to the position-tracking controller types."""

    State = State
    Actuation = Actuation
    Parameters = Parameters
    get_acados_model = staticmethod(get_acados_model)


class MPC(MPCBase):
    """MPC controller bound to the position-tracking controller types."""

    State = State
    Actuation = Actuation
    OnlineParameters = OnlineParameters
    Parameters = Parameters


__all__ = [
    'AcadosMPCSolver',
    'Actuation',
    'ActuationBounds',
    'CaActuation',
    'CaDynamics',
    'CaParameters',
    'CaState',
    'DroneModel',
    'Dynamics',
    'Gains',
    'MPC',
    'MPCBase',
    'MPCDataBase',
    'NonlinearConstraintBounds',
    'OnlineParameters',
    'Parameters',
    'Reference',
    'ReferenceEnd',
    'SlackWeights',
    'SlackWeightsEnd',
    'SoftNonlinearConstraintBounds',
    'SoftStateBounds',
    'State',
    'StateBounds',
    'get_acados_model',
]
