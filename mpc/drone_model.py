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

"""Acados Model definition."""

__authors__ = 'Rafael Pérez Seguí, Carmen De Rojas Pita-Romero'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from acados_template import AcadosModel
import casadi as ca
from mpc.utils import quaternion_utils as q_utils
from mpc.model_definition.state import CaState
from mpc.model_definition.actuation import CaActuation
from mpc.model_definition.parameters import CaParameters
from mpc.model_definition.dynamics import CaDynamics
from mpc.spline.spline_evaluation import evaluate_arc_length_spline


class DroneModel(CaDynamics):
    """Dynamics for MAV."""

    def __init__(
            self,
            x: CaState = CaState(),
            u: CaActuation = CaActuation(),
            p: CaParameters = CaParameters()):
        """
        Multirotor dynamics x_dot = f(x, u).

        :param x (CaState): Multirotor state.
        :param u (CaActuation): Multirotor control signals.
        :param p (CaParameters): Multirotor parameters.
        """
        super().__init__()
        self._x = x
        self._u = u
        self._p = p

        # Model equations
        position_dot = x.linear_velocity
        orientation_dot = q_utils.quaternion_derivate(
            x.orientation, u.angular_velocity
        )
        gravity = ca.DM(9.81)
        linear_velocity_dot = self.velocity_derivate(
            x.orientation,
            u.thrust,
            gravity,
            p.mass)
        theta_dot = u.theta_velocity        

        # Full dynamics
        self._f_expl = ca.vertcat(
            position_dot,
            orientation_dot,
            linear_velocity_dot,
            theta_dot)

        self._q_att = q_utils.quaternion_error(
            self._x.orientation,
            p.desired_orientation
        )
        
        # Spline parameters from state and parameters
        # Note: p0 is current drone position, not from parameters
        # m0 is current velocity (approximation for tangent at current position)
        # p0 = x.position
        # m0 = x.linear_velocity
        p1 = p.s1_p
        m1 = p.s1_m
        p2 = p.s2_p
        m2 = p.s2_m
        p3 = p.s3_p
        m3 = p.s3_m
        s_length = p.s_length
        poly_coeffs = p.s_poly_coeffs
        
        position_ref, tangent_vec = evaluate_arc_length_spline(
            x.theta,
            [p1, p2, p3],
            [m1, m2, m3],
            [0.0, 1.0, 2.0],
            s_length,
            poly_coeffs,
        )
        
        # Compute contouring and lag errors
        delta_pos = x.position - position_ref
        e_lag = ca.dot(delta_pos, tangent_vec)
        e_lag_vec = e_lag * tangent_vec
        e_contour_vec = delta_pos - e_lag_vec
        e_contour = ca.norm_2(e_contour_vec)

        # MPCC External Cost Function
        # J_contouring + J_lag + J_attitude + J_control + J_progress
        J_contour = e_contour_vec.T @ ca.diag(p.gains_contour_error) @ e_contour_vec
        J_lag = p.gain_lag_error * e_lag**2
        J_attitude = self._q_att.T @ ca.diag(p.gains_orientation) @ self._q_att
        u_ref = ca.vertcat(p.mass * gravity, 0.0, 0.0, 0.0, 0.0)
        u_error = u.vector - u_ref
        J_control = u_error.T @ ca.diag(ca.vertcat(
            p.gains_actuation,
            p.gains_actuation_theta_velocity
        )) @ u_error
        J_progress = -p.gain_progress * u.theta_velocity    

        # Total stage cost
        self._cost_expr_ext = J_contour + J_lag + J_attitude + J_control + J_progress
        # Total terminal cost
        self._cost_expr_ext_e = J_contour + J_lag+ J_attitude

    @staticmethod
    def velocity_derivate(
            quaternion: ca.SX,
            thrust: ca.SX,
            gravity: ca.SX,
            mass: ca.SX) -> ca.SX:
        """
        Compute the linear velocity derivative.

        v_dot = 2 * q_v x t = 2 * [qw, qx, qy, qz] x t

        :param quaternion (ca.SX): The quaternion [qw, qx, qy, qz]
        rotation from world to body frame.
        :param thrust (ca.SX): The thrust (N) in body frame.
        :param gravity (ca.SX): The gravity (m/s^2) in world frame.
        :param mass (ca.SX): The mass (kg).

        :return (ca.SX): The linear velocity derivative [vx_dot, vy_dot, vz_dot]
        in world frame.
        """
        # Compute inverse rotation
        acceleration_body_frame = ca.vertcat(0, 0, thrust / mass)
        acceleration_world = q_utils.apply_rotation(
            q_utils.normalize_quaternion(quaternion),
            acceleration_body_frame
        )

        v_dot = acceleration_world - ca.vertcat(0, 0, gravity)

        return v_dot

    @property
    def f_expl(self) -> ca.SX:
        """
        Get the explicit dynamics f(x, u).

        :return (ca.SX): The explicit dynamics.
        """
        return self._f_expl

    @property
    def cost_expr_ext(self) -> ca.SX:
        """
        Get the cost external expression.

        :return (ca.SX): The cost external expression.
        """
        return self._cost_expr_ext

    @property
    def cost_expr_ext_e(self) -> ca.SX:
        """
        Get the cost external end expression.

        :return (ca.SX): The cost external end expression.
        """
        return self._cost_expr_ext_e

    @property
    def xdot(self) -> ca.SX:
        """
        Get the state derivative x_dot.

        :return (ca.SX): The state derivative x_dot.
        """
        return self.vector

    @property
    def x(self) -> ca.SX:
        """
        Get the state x.

        :return (ca.SX): The state x.
        """
        return self._x.vector

    @property
    def state(self) -> ca.SX:
        """
        Get the state.

        :return (CaState): The state.
        """
        return self._x

    @property
    def u(self) -> ca.SX:
        """
        Get the control u.

        :return (ca.SX): The control u.
        """
        return self._u.vector

    @property
    def actuation(self) -> ca.SX:
        """
        Get the actuation.

        :return (CaControl): The actuation.
        """
        return self._u

    @property
    def p(self) -> ca.SX:
        """
        Get the parameters.

        :return (ca.SX): The parameters.
        """
        return self._p.vector

    @property
    def parameters(self) -> ca.SX:
        """
        Get the parameters.

        :return (CaParameters): The parameters.
        """
        return self._p
    
    @property
    def position_ref(self) -> ca.SX:
        """
        Get the reference position on the spline.

        :return (ca.SX): The reference position (3x1).
        """
        return self._position_ref
    
    @property
    def tangent_vec(self) -> ca.SX:
        """
        Get the tangent vector on the spline.

        :return (ca.SX): The tangent vector (3x1).
        """
        return self._tangent_vec
    
    @property
    def normal_vec(self) -> ca.SX:
        """
        Get the normal vector on the spline.

        :return (ca.SX): The normal vector (3x1).
        """
        return self._normal_vec
    
    @property
    def contouring_error(self) -> ca.SX:
        """
        Get the contouring error (perpendicular distance to path).

        :return (ca.SX): The contouring error (scalar).
        """
        return self._e_contour
    
    @property
    def lag_error(self) -> ca.SX:
        """
        Get the lag error (along-track distance).

        :return (ca.SX): The lag error (scalar).
        """
        return self._e_lag


def get_acados_model() -> AcadosModel:
    """Initialize the Acados multirotor model."""
    model_name = 'mpc'

    # System Model
    drone_model = DroneModel()

    # Explicit and Implicit functions
    f_expl = drone_model.f_expl
    f_impl = drone_model.xdot - drone_model.f_expl

    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = drone_model.x
    model.xdot = drone_model.xdot
    model.u = drone_model.u
    model.z = z
    model.p = drone_model.p
    model.name = model_name
    
    # MPCC External Cost
    model.cost_expr_ext_cost = drone_model.cost_expr_ext
    model.cost_expr_ext_cost_e = drone_model.cost_expr_ext_e

    return model


if __name__ == '__main__':
    # Create a state
    x = CaState()
    print('State:    ', x.vector)

    # Create a actuation
    u = CaActuation()
    print('Actuation:  ', u.vector)

    # Create parameters
    p = CaParameters()
    print('Parameters: ', p.vector)

    # Create drone_model
    drone_model = DroneModel(x, u, p)
    print('Model: ', drone_model)

    # Create an Acados model
    acados_model = get_acados_model()

    def format_output(label, expr):
        print(f'{label}:')
        terms = ca.vertsplit(expr, 1)
        for term in terms:
            print(f'    {term}')
        print()

    format_output('f_impl', acados_model.f_impl_expr)
    format_output('f_expl', acados_model.f_expl_expr)

    print("External cost stage shape:", acados_model.cost_expr_ext_cost.shape)
    print("External cost terminal shape:", acados_model.cost_expr_ext_cost_e.shape)

