"""MPC Library for MAVs using acados."""

__version__ = "0.1.0"

from .mpc_controller import MPC, MPCParameters
from .acados_solver import AcadosMPCSolver
from .drone_model import get_acados_model

__all__ = [
    'MPC',
    'MPCParameters',
    'AcadosMPCSolver',
    'get_acados_model'
]
