#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
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

"""Shared drone-model primitives for Acados MPC controllers.

This module defines the base class ``DroneModelBase`` exposing the MAV dynamics
and cost helpers that are common across all controller variants (position,
trajectory, ...). Variant-specific controllers subclass ``DroneModelBase`` (and
their own generated ``CaDynamics``) to build the dynamics, cost expressions,
and constraints that are specific to that variant.
"""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import casadi as ca

from mpc_acados_core.utils import quaternion_utils as q_utils


class DroneModelBase:
    """
    Mixin that exposes shared helpers and properties for MAV drone models.

    Subclasses are expected to populate the following attributes in their own
    ``__init__``:

    - ``self._x``: CasADi state object (exposes ``vector``, ``position``,
      ``orientation``, ``linear_velocity``, ...).
    - ``self._u``: CasADi actuation object (exposes ``vector``, ``thrust``,
      ``angular_velocity``, ...).
    - ``self._p``: CasADi parameters object (exposes ``vector``, ``mass``,
      ``Q``, ``R``, ``Qe``, ``external_force`` and any variant-specific
      parameters).
    - ``self._f_expl``: Explicit dynamics expression ``x_dot = f(x, u, p)``.
    - ``self._cost_expr_ext``: Stage external cost expression (scalar).
    - ``self._cost_expr_ext_e``: Terminal external cost expression (scalar).
    - ``self._cost_y_expr``: Stage least-squares cost expression (optional).
    - ``self._cost_y_expr_e``: Terminal least-squares cost expression (optional).
    - ``self._con_h_expr``: Nonlinear constraint expression ``h(x, u)``.
    - ``self._gains_total_stage`` / ``self._gains_total_terminal``: Weight
      matrices used by the custom Hessian helpers below.
    - ``self._error_total_stage`` / ``self._error_total_terminal``: Error
      vectors used by the custom Hessian helpers below.
    """

    @staticmethod
    def velocity_derivate(
            quaternion: ca.SX,
            thrust: ca.SX,
            gravity: ca.SX,
            mass: ca.SX,
            external_force: ca.SX = ca.vertcat(0, 0, 0)) -> ca.SX:
        """
        Compute the linear velocity derivative.

        v_dot = R(q) * (thrust/mass + f_ext/mass) - [0, 0, g].

        :param quaternion: Quaternion [qw, qx, qy, qz] rotation from world
            to body frame.
        :type quaternion: ca.SX
        :param thrust: Thrust in Newton, aligned with the body z-axis.
        :type thrust: ca.SX
        :param gravity: Gravity acceleration (m/s^2), world frame.
        :type gravity: ca.SX
        :param mass: Mass of the vehicle (kg).
        :type mass: ca.SX
        :param external_force: External force acting on the vehicle in body
            frame [fx, fy, fz] (N).
        :type external_force: ca.SX
        :return: Linear velocity derivative [vx_dot, vy_dot, vz_dot] in
            world frame.
        :rtype: ca.SX
        """
        acceleration_body_frame = ca.vertcat(0, 0, thrust / mass) + external_force / mass
        acceleration_world = q_utils.apply_rotation(
            q_utils.normalize_quaternion(quaternion),
            acceleration_body_frame
        )
        return acceleration_world - ca.vertcat(0, 0, gravity)

    @staticmethod
    def _compute_quadratic_cost(error: ca.SX, gains: ca.SX) -> ca.SX:
        """
        Compute a quadratic cost ``0.5 * e^T * W * e``.

        :param error: Error vector.
        :type error: ca.SX
        :param gains: Weight matrix.
        :type gains: ca.SX
        :return: Scalar quadratic cost.
        :rtype: ca.SX
        """
        return 0.5 * ca.mtimes([error.T, gains, error])

    def _build_stage_custom_hessian(self) -> ca.SX:
        """Build the stage Hessian that reproduces the Gauss-Newton curvature."""
        ux = ca.vertcat(self._u.vector, self._x.vector)
        stage_jacobian = ca.jacobian(self._error_total_stage, ux)
        return ca.mtimes([stage_jacobian.T, self._gains_total_stage, stage_jacobian])

    def _build_terminal_custom_hessian(self) -> ca.SX:
        """Build the terminal Hessian that reproduces the Gauss-Newton curvature."""
        terminal_jacobian = ca.jacobian(self._error_total_terminal, self._x.vector)
        return ca.mtimes([terminal_jacobian.T, self._gains_total_terminal, terminal_jacobian])

    @property
    def f_expl(self) -> ca.SX:
        """Explicit dynamics expression ``f(x, u, p)``."""
        return self._f_expl

    @property
    def cost_expr_ext(self) -> ca.SX:
        """Stage external cost expression."""
        return self._cost_expr_ext

    @property
    def cost_expr_ext_e(self) -> ca.SX:
        """Terminal external cost expression."""
        return self._cost_expr_ext_e

    @property
    def cost_expr_ext_custom_hess(self) -> ca.SX:
        """Custom Hessian for the stage external cost in (u, x) coordinates."""
        return self._build_stage_custom_hessian()

    @property
    def cost_expr_ext_custom_hess_e(self) -> ca.SX:
        """Custom Hessian for the terminal external cost in x coordinates."""
        return self._build_terminal_custom_hessian()

    @property
    def cost_y_expr(self) -> ca.SX:
        """Stage least-squares cost expression."""
        return self._cost_y_expr

    @property
    def cost_y_expr_e(self) -> ca.SX:
        """Terminal least-squares cost expression."""
        return self._cost_y_expr_e

    @property
    def con_h_expr(self) -> ca.SX:
        """Nonlinear constraint expression ``h(x, u)``."""
        return self._con_h_expr

    @property
    def xdot(self) -> ca.SX:
        """State derivative ``x_dot`` as exposed by the CasADi dynamics."""
        return self.vector

    @property
    def x(self) -> ca.SX:
        """State vector ``x``."""
        return self._x.vector

    @property
    def state(self):
        """CasADi state object."""
        return self._x

    @property
    def u(self) -> ca.SX:
        """Actuation vector ``u``."""
        return self._u.vector

    @property
    def actuation(self):
        """CasADi actuation object."""
        return self._u

    @property
    def p(self) -> ca.SX:
        """Parameter vector ``p``."""
        return self._p.vector

    @property
    def parameters(self):
        """CasADi parameters object."""
        return self._p


if __name__ == '__main__':
    pass
