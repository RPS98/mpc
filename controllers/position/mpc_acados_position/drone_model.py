#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
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
"""Position-tracking drone model for the Acados MPC."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from acados_template import AcadosModel
import casadi as ca

from mpc_acados_core.drone_model import DroneModelBase
from mpc_acados_core.utils import quaternion_utils as q_utils

from mpc_acados_position.model_definition.actuation import CaActuation
from mpc_acados_position.model_definition.dynamics import CaDynamics
from mpc_acados_position.model_definition.parameters import CaParameters
from mpc_acados_position.model_definition.state import CaState


class DroneModel(CaDynamics, DroneModelBase):
    """Position-reference drone model. Regulates to ``p.desired_position``."""

    def __init__(
            self,
            x: CaState = None,
            u: CaActuation = None,
            p: CaParameters = None) -> None:
        """
        Multirotor dynamics ``x_dot = f(x, u, p)`` for position tracking.

        :param x: Multirotor state.
        :type x: CaState
        :param u: Multirotor control signals.
        :type u: CaActuation
        :param p: Multirotor parameters.
        :type p: CaParameters
        """
        super().__init__()

        self._con_h_expr = []
        self._cost_y_expr = []
        self._cost_y_expr_e = []
        self._cost_expr_ext = []
        self._cost_expr_ext_e = []

        self._x = x if x is not None else CaState()
        self._u = u if u is not None else CaActuation()
        self._p = p if p is not None else CaParameters()

        # MODEL DYNAMICS

        position_dot = self._x.linear_velocity

        orientation_dot = q_utils.quaternion_derivate(
            self._x.orientation,
            self._u.angular_velocity,
        )

        gravity = ca.DM(9.81)

        linear_velocity_dot = self.velocity_derivate(
            self._x.orientation,
            self._u.thrust,
            gravity,
            self._p.mass,
            self._p.external_force,
        )

        self._f_expl = ca.vertcat(
            position_dot,
            orientation_dot,
            linear_velocity_dot,
        )

        # GAINS
        gains_position_state = ca.diag(self._p.Q[0:3])
        gains_attitude_state = ca.diag(self._p.Q[3:6])
        gains_velocity_state = ca.diag(self._p.Q[6:9])
        gains_position_state_e = ca.diag(self._p.Qe[0:3])
        gains_attitude_state_e = ca.diag(self._p.Qe[3:6])
        gains_velocity_state_e = ca.diag(self._p.Qe[6:9])
        gains_actuation = ca.diag(self._p.R)

        # Total gains for the custom Hessian
        self._gains_total_stage = ca.diag(ca.vertcat(
            self._p.Q,
            self._p.R,
        ))
        self._gains_total_terminal = ca.diag(self._p.Qe)

        # STATE ERRORS
        error_position = self._x.position - self._p.desired_position
        error_attitude = q_utils.quaternion_error(
            self._x.orientation, self._p.desired_orientation)
        error_velocity = self._x.linear_velocity

        hover_actuation = ca.vertcat(self._p.mass * gravity, ca.SX.zeros(3))
        error_u = self._u.vector - hover_actuation

        # Total error for the custom Hessian
        self._error_total_stage = ca.vertcat(
            error_position,
            error_attitude,
            error_velocity,
            error_u,
        )
        self._error_total_terminal = ca.vertcat(
            error_position,
            error_attitude,
            error_velocity,
        )

        # COSTS
        cost_position_state = self._compute_quadratic_cost(error_position, gains_position_state)
        cost_position_state_e = self._compute_quadratic_cost(error_position, gains_position_state_e)

        cost_attitude_state = self._compute_quadratic_cost(error_attitude, gains_attitude_state)
        cost_attitude_state_e = self._compute_quadratic_cost(error_attitude, gains_attitude_state_e)

        cost_velocity_state = self._compute_quadratic_cost(error_velocity, gains_velocity_state)
        cost_velocity_state_e = self._compute_quadratic_cost(error_velocity, gains_velocity_state_e)

        cost_actuation = self._compute_quadratic_cost(error_u, gains_actuation)

        self._cost_expr_ext = (
            cost_position_state
            + cost_attitude_state
            + cost_velocity_state
            + cost_actuation
        )
        self._cost_expr_ext_e = (
            cost_position_state_e
            + cost_attitude_state_e
            + cost_velocity_state_e
        )

        # NONLINEAR CONSTRAINT: Speed limit
        self._con_h_expr = ca.vertcat(
            ca.dot(self._x.linear_velocity, self._x.linear_velocity),
        )


def get_acados_model() -> tuple[AcadosModel, DroneModel]:
    """Initialize the Acados multirotor model for the position controller."""
    model_name = 'mpc'

    drone_model = DroneModel()

    f_expl = drone_model.f_expl
    f_impl = drone_model.xdot - drone_model.f_expl

    z = []

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = drone_model.x
    model.xdot = drone_model.xdot
    model.u = drone_model.u
    model.z = z
    model.p = drone_model.p
    model.name = model_name
    model.cost_y_expr = drone_model.cost_y_expr
    model.cost_y_expr_e = drone_model.cost_y_expr_e
    model.cost_expr_ext_cost = drone_model.cost_expr_ext
    model.cost_expr_ext_cost_e = drone_model.cost_expr_ext_e
    model.con_h_expr = drone_model.con_h_expr
    model.con_h_expr_e = drone_model.con_h_expr

    return model, drone_model


if __name__ == '__main__':
    pass
