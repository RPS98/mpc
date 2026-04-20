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

from mpc_acados_position.actuation import Actuation, CaActuation
from mpc_acados_position.dynamics import CaDynamics, Dynamics
from mpc_acados_position.parameters import (
    CaParameters,
    OnlineParameters,
    Parameters,
)
from mpc_acados_position.state import CaState, State
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
