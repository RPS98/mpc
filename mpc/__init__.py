"""MPC Library for MAVs using acados."""

__version__ = "0.1.0"

from .mpc_controller import MPC, MPCData
from .mpc_datatype import (
    Reference, ReferenceEnd, Gains, ActuationBounds, StateBounds,
    SoftStateBounds, SlackWeights, SlackWeightsEnd
)
from .model_definition.actuation import Actuation
from .model_definition.parameters import Parameters
from .model_definition.state import State
from .acados_solver import AcadosMPCSolver

__all__ = [
    'MPC',
    'MPCData',
    'Reference',
    'ReferenceEnd',
    'Gains',
    'ActuationBounds',
    'StateBounds',
    'SoftStateBounds',
    'SlackWeights',
    'SlackWeightsEnd',
    'Actuation',
    'Parameters',
    'State',
    'AcadosMPCSolver'
]
