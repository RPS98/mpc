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
